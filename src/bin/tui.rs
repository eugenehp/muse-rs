//! Real-time EEG chart viewer for Muse headsets.
//!
//! Usage:
//!   cargo run --bin tui               # scan forever until a Muse is found, then stream
//!   cargo run --bin tui -- --simulate # use the built-in EEG simulator (no hardware needed)
//!
//! Keys (streaming view)
//! ---------------------
//!   Tab      open device picker
//!   s        trigger a fresh BLE scan right now
//!   +  / =   zoom out  (increase µV scale)
//!   -        zoom in   (decrease µV scale)
//!   a        auto-scale: fit Y axis to current peak amplitude
//!   v        toggle smooth overlay (dim raw + bright 9-pt moving-average)
//!   p        pause streaming   (sends 'h' to real device)
//!   r        resume streaming  (sends 'd' to real device)
//!   c        clear waveform buffers
//!   d        disconnect current device
//!   q / Esc  quit
//!
//! Keys (device picker overlay)
//! ----------------------------
//!   ↑ / ↓   navigate list
//!   Enter    connect to highlighted device
//!   s        rescan while picker is open
//!   Esc      close picker

use std::collections::VecDeque;
use std::f64::consts::PI;
use std::io;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::Result;
use crossterm::{
    event::{self, Event, KeyCode, KeyModifiers},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use ratatui::{
    backend::CrosstermBackend,
    layout::{Constraint, Layout, Margin, Rect},
    style::{Color, Modifier, Style},
    symbols,
    text::{Line, Span},
    widgets::{
        Axis, Block, Borders, Chart, Clear, Dataset, GraphType, List, ListItem, ListState,
        Paragraph,
    },
    Frame, Terminal,
};
use tokio::sync::{mpsc, oneshot};

use muse_rs::muse_client::{MuseClient, MuseClientConfig, MuseDevice, MuseHandle};
use muse_rs::protocol::EEG_CHANNEL_NAMES;
use muse_rs::types::MuseEvent;

// ── Constants ─────────────────────────────────────────────────────────────────

/// Width of the scrolling waveform window in seconds.
/// Increasing this shows more history but reduces per-sample horizontal resolution.
const WINDOW_SECS: f64 = 2.0;

/// EEG sample rate expected from the headset (Hz).
/// Used to convert sample indices to x-axis time values.
const EEG_HZ: f64 = 256.0;

/// Number of samples retained per channel — enough to fill exactly `WINDOW_SECS`.
const BUF_SIZE: usize = (WINDOW_SECS * EEG_HZ) as usize; // 512

/// Number of EEG channels displayed (TP9, AF7, AF8, TP10).
/// The optional AUX channel (index 4) is not shown in the TUI.
const NUM_CH: usize = 4;

/// Discrete Y-axis scale steps in µV (half the full symmetric range ±scale).
/// The user cycles through these with `+` / `-`; `a` picks the best fit automatically.
const Y_SCALES: &[f64] = &[10.0, 25.0, 50.0, 100.0, 200.0, 500.0, 1000.0, 2000.0];

/// Index into `Y_SCALES` used when the app starts with a real device.
/// ±500 µV covers typical real-world EEG amplitudes with headroom for artefacts.
const DEFAULT_SCALE: usize = 5;

/// Per-channel line colours: TP9=Cyan, AF7=Yellow, AF8=Green, TP10=Magenta.
const COLORS: [Color; 4] = [Color::Cyan, Color::Yellow, Color::Green, Color::Magenta];

/// Dimmed versions of COLORS used for the raw background trace when smooth is on.
const DIM_COLORS: [Color; 4] = [
    Color::Rgb(0, 90, 110),   // dim cyan
    Color::Rgb(110, 90, 0),   // dim yellow
    Color::Rgb(0, 110, 0),    // dim green
    Color::Rgb(110, 0, 110),  // dim magenta
];

/// Moving-average window in samples. 9 samples ≈ 35 ms at 256 Hz.
/// Passes alpha (8-13 Hz) and beta (13-30 Hz) clearly while removing HF noise.
const SMOOTH_WINDOW: usize = 9;

/// Braille spinner frames cycled at ~100 ms intervals to indicate background activity.
const SPINNER: &[&str] = &["⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏"];

/// Seconds between automatic scan retries when no Muse device is found at all.
/// Long enough to avoid flooding the BLE adapter but short enough to feel responsive.
const RETRY_SECS: u64 = 6;

/// Seconds to wait before re-scanning after an unexpected disconnect.
/// Shorter than `RETRY_SECS` — the device is almost certainly nearby and will
/// resume advertising within a second or two of losing the link.
const RECONNECT_DELAY_SECS: u64 = 2;

// ── App mode ──────────────────────────────────────────────────────────────────

#[derive(Clone)]
pub enum AppMode {
    /// BLE scan is running (initial or retry).
    Scanning,
    /// Actively establishing a connection.
    Connecting(String),
    /// Streaming live EEG. Carries the device name and its short identifier.
    Connected { name: String, id: String },
    /// `--simulate` flag: running the built-in signal generator.
    Simulated,
    /// Last scan found no Muse devices. Will retry automatically.
    NoDevices,
    /// Was connected; link was lost. Will retry automatically.
    Disconnected,
}

// ── App state (shared with the BLE event task via Arc<Mutex<_>>) ──────────────

pub struct App {
    // ── EEG data
    bufs: [VecDeque<f64>; NUM_CH],

    // ── Status
    pub mode: AppMode,
    pub battery: Option<f32>,
    pub accel: Option<(f32, f32, f32)>,
    pub gyro: Option<(f32, f32, f32)>,

    // ── Rate tracking
    total_samples: u64,
    pkt_times: VecDeque<Instant>,

    // ── UI controls
    scale_idx: usize,
    pub paused: bool,

    // ── Device picker
    pub show_picker: bool,
    pub picker_cursor: usize,
    /// Display strings for each discovered device: "Name  [short-id]"
    pub picker_entries: Vec<String>,
    /// Index into picker_entries that is currently connected.
    pub picker_connected_idx: Option<usize>,
    /// Spinner shown inside the picker while a scan is running.
    pub picker_scanning: bool,

    /// Last connection / setup error, shown in the Disconnected status line.
    pub last_error: Option<String>,

    /// When true, draw a dim raw trace + a bright smoothed overlay on each chart.
    pub smooth: bool,
}

impl App {
    /// Create a fresh `App` in [`AppMode::Scanning`] with all fields at their defaults.
    fn new() -> Self {
        Self {
            bufs: std::array::from_fn(|_| VecDeque::with_capacity(BUF_SIZE + 16)),
            mode: AppMode::Scanning,
            battery: None,
            accel: None,
            gyro: None,
            total_samples: 0,
            pkt_times: VecDeque::with_capacity(256),
            scale_idx: DEFAULT_SCALE,
            paused: false,
            show_picker: false,
            picker_cursor: 0,
            picker_entries: vec![],
            picker_connected_idx: None,
            picker_scanning: false,
            last_error: None,
            smooth: true,
        }
    }

    /// Append `samples` from electrode `ch` to the rolling ring-buffer.
    ///
    /// Drops the oldest samples to keep the buffer at exactly `BUF_SIZE` entries.
    /// Channel 0 (TP9) is also used to update the packet-rate tracker.
    ///
    /// No-ops when `paused` is `true` or `ch >= NUM_CH`.
    pub fn push(&mut self, ch: usize, samples: &[f64]) {
        if self.paused || ch >= NUM_CH {
            return;
        }
        let buf = &mut self.bufs[ch];
        for &v in samples {
            buf.push_back(v);
            while buf.len() > BUF_SIZE {
                buf.pop_front();
            }
        }
        if ch == 0 {
            self.total_samples += samples.len() as u64;
            let now = Instant::now();
            self.pkt_times.push_back(now);
            // Keep only arrival times from the last 2 seconds for rate computation.
            while self
                .pkt_times
                .front()
                .map(|t| now.duration_since(*t) > Duration::from_secs(2))
                .unwrap_or(false)
            {
                self.pkt_times.pop_front();
            }
        }
    }

    /// Wipe all EEG buffers and reset transient sensor readings.
    ///
    /// Called when disconnecting or reconnecting so the charts start blank.
    /// Does **not** change `mode`, `show_picker`, or any picker state — those
    /// are managed by the caller.
    pub fn clear(&mut self) {
        for b in &mut self.bufs {
            b.clear();
        }
        self.total_samples = 0;
        self.pkt_times.clear();
        self.battery = None;
        self.accel = None;
        self.gyro = None;
        self.last_error = None;
    }

    /// Compute the current EEG packet arrival rate in packets per second.
    ///
    /// Uses a 2-second sliding window of arrival timestamps recorded in
    /// `pkt_times`.  Returns `0.0` when fewer than 2 timestamps are available.
    fn pkt_rate(&self) -> f64 {
        let n = self.pkt_times.len();
        if n < 2 {
            return 0.0;
        }
        let span = self
            .pkt_times
            .back()
            .unwrap()
            .duration_since(self.pkt_times[0])
            .as_secs_f64();
        if span < 1e-9 {
            0.0
        } else {
            (n as f64 - 1.0) / span
        }
    }

    /// Return the current half-range of the Y axis in µV (e.g. 500 for ±500 µV).
    fn y_range(&self) -> f64 {
        Y_SCALES[self.scale_idx]
    }

    /// Increase the Y-axis scale (zoom out) by one step.  Clamped at the largest step.
    fn scale_up(&mut self) {
        if self.scale_idx + 1 < Y_SCALES.len() {
            self.scale_idx += 1;
        }
    }

    /// Decrease the Y-axis scale (zoom in) by one step.  Clamped at the smallest step.
    fn scale_down(&mut self) {
        if self.scale_idx > 0 {
            self.scale_idx -= 1;
        }
    }

    /// Choose the smallest Y-scale step that fits the current peak amplitude.
    ///
    /// Adds a 10 % headroom margin above the observed peak so the waveform
    /// doesn't immediately hit the axis.  Selects the largest scale if the
    /// peak exceeds all available steps.
    fn auto_scale(&mut self) {
        let peak = self
            .bufs
            .iter()
            .flat_map(|b| b.iter())
            .fold(0.0_f64, |acc, &v| acc.max(v.abs()));
        // Round up to the next scale step with a small headroom margin (10 %).
        let needed = peak * 1.1;
        self.scale_idx = Y_SCALES
            .iter()
            .position(|&s| s >= needed)
            .unwrap_or(Y_SCALES.len() - 1);
    }
}

// ── Helpers ───────────────────────────────────────────────────────────────────

/// Shorten a BLE identifier for compact display.
/// UUID  → last 8 hex chars, e.g. "90ABCDEF"
/// MAC   → last 8 chars, e.g.  "EE:FF"
fn short_id(id: &str) -> String {
    let trimmed = id.trim_matches(|c: char| c == '{' || c == '}');
    if trimmed.len() > 8 {
        trimmed[trimmed.len() - 8..].to_uppercase()
    } else {
        trimmed.to_uppercase()
    }
}

/// Build the display string shown in the picker list and the header.
fn device_entry(d: &MuseDevice) -> String {
    format!("{}  [{}]", d.name, short_id(&d.id))
}

/// Symmetric moving-average (boxcar) smoother.
///
/// Each output point is the mean of the `window` nearest input points (up to
/// `window/2` on each side, clamped at the buffer edges so the length is
/// preserved).  At 256 Hz a 9-sample window has a -3 dB point of ≈ 75 Hz —
/// it visibly quiets high-frequency noise while leaving alpha/beta intact.
fn smooth_signal(data: &[(f64, f64)], window: usize) -> Vec<(f64, f64)> {
    if data.len() < 3 || window < 2 {
        return data.to_vec();
    }
    let half = window / 2;
    data.iter()
        .enumerate()
        .map(|(i, &(x, _))| {
            let start = i.saturating_sub(half);
            let end = (i + half + 1).min(data.len());
            let sum: f64 = data[start..end].iter().map(|&(_, y)| y).sum();
            (x, sum / (end - start) as f64)
        })
        .collect()
}

// ── EEG simulator ─────────────────────────────────────────────────────────────

/// Generate one synthetic EEG sample at time `t` (seconds) for channel `ch`.
///
/// The signal is a superposition of three physiological-band sinusoids with
/// a deterministic pseudo-random noise floor:
///
/// | Component | Frequency | Amplitude | Notes |
/// |-----------|-----------|-----------|-------|
/// | Alpha     | 10 Hz     | ±20 µV    | phase-shifted per channel |
/// | Beta      | 22 Hz     | ±6 µV     | — |
/// | Theta     | 6 Hz      | ±10 µV    | — |
/// | Noise     | —         | ±4 µV     | deterministic hash of (t, ch) |
///
/// Peak amplitude is approximately ±40 µV, which fits comfortably within the
/// ±50 µV scale automatically selected for `--simulate` mode.
fn sim_sample(t: f64, ch: usize) -> f64 {
    let phi = ch as f64 * PI / 2.5;
    let alpha = 20.0 * (2.0 * PI * 10.0 * t + phi).sin();
    let beta = 6.0 * (2.0 * PI * 22.0 * t + phi * 1.7).sin();
    let theta = 10.0 * (2.0 * PI * 6.0 * t + phi * 0.9).sin();
    let nx = t * 1000.7 + ch as f64 * 137.508;
    let noise = ((nx.sin() * 9973.1).fract() - 0.5) * 8.0;
    alpha + beta + theta + noise
}

/// Spawn a background task that generates synthetic EEG, accelerometer, and
/// gyroscope data and writes it into `app` at the real headset rate.
///
/// * EEG: 12 samples per channel every `12 / 256 Hz ≈ 46.9 ms` (4 channels).
/// * Telemetry / IMU: updated every 21 EEG ticks (≈ 984 ms).
///
/// The task runs for the lifetime of the process and is only started when the
/// `--simulate` flag is passed.  Time is advanced even while paused so that
/// the waveform resumes from the correct phase when unpaused.
fn spawn_simulator(app: Arc<Mutex<App>>) {
    tokio::spawn(async move {
        let pkt_interval = Duration::from_secs_f64(12.0 / EEG_HZ);
        let mut ticker = tokio::time::interval(pkt_interval);
        let dt = 1.0 / EEG_HZ;
        let mut t = 0.0_f64;
        let mut seq = 0u32;
        loop {
            ticker.tick().await;
            let mut s = app.lock().unwrap();
            if s.paused {
                t += 12.0 * dt;
                continue;
            }
            for ch in 0..NUM_CH {
                let samples: Vec<f64> =
                    (0..12).map(|i| sim_sample(t + i as f64 * dt, ch)).collect();
                s.push(ch, &samples);
            }
            seq = seq.wrapping_add(1);
            if seq.is_multiple_of(21) {
                s.battery = Some((85.0 - t as f32 / 300.0).clamp(0.0, 100.0));
                s.accel = Some((
                    (0.01 * (2.0 * PI * 0.3 * t).sin()) as f32,
                    (0.02 * (2.0 * PI * 0.5 * t).cos()) as f32,
                    (-1.0 + 0.005 * (2.0 * PI * 0.1 * t).sin()) as f32,
                ));
                s.gyro = Some((
                    (0.12 * (2.0 * PI * 0.2 * t).sin()) as f32,
                    (0.08 * (2.0 * PI * 0.3 * t).cos()) as f32,
                    (0.05 * (2.0 * PI * 0.1 * t).sin()) as f32,
                ));
            }
            t += 12.0 * dt;
        }
    });
}

// ── BLE helpers ───────────────────────────────────────────────────────────────

/// Start a background scan; return a receiver for the results.
///
/// The task has a hard deadline of `scan_timeout + 10 s` to guard against
/// `Manager::new()` or `adapter.start_scan()` hanging indefinitely inside
/// btleplug when the Bluetooth stack is in a bad state.
/// Scan result delivered through the oneshot channel.
struct ScanResult {
    devices: Vec<MuseDevice>,
    /// `Some(msg)` when the scan failed or timed out.
    error: Option<String>,
}

fn start_scan(config: MuseClientConfig) -> oneshot::Receiver<ScanResult> {
    let (tx, rx) = oneshot::channel();
    let deadline = Duration::from_secs(config.scan_timeout_secs + 10);
    tokio::spawn(async move {
        let result = match tokio::time::timeout(deadline, MuseClient::new(config).scan_all()).await
        {
            Ok(Ok(devices)) => {
                log::info!("Scan completed: {} device(s) found", devices.len());
                ScanResult { devices, error: None }
            }
            Ok(Err(e)) => {
                log::error!("Scan failed: {e}");
                ScanResult { devices: vec![], error: Some(format!("{e}")) }
            }
            Err(_) => {
                log::error!("Scan timed out after {deadline:?}");
                ScanResult { devices: vec![], error: Some("scan timed out".into()) }
            }
        };
        let _ = tx.send(result);
    });
    rx
}

/// Wipe all live signal data and schedule a fresh BLE scan.
///
/// Called identically from both "unexpected disconnect" (step 2) and
/// "connection attempt failed" (step 1b), ensuring the recovery path is the
/// same in both cases:
///   • EEG buffers, battery, sensors, last error are cleared
///   • Picker overlay is closed so the UI is in a clean state
///   • Mode switches to Scanning immediately (spinner appears right away)
///   • A new scan is queued after `delay_secs` (gives BLE stack time to settle)
fn restart_scan(
    app: &Arc<Mutex<App>>,
    pending_scan: &mut Option<oneshot::Receiver<ScanResult>>,
    retry_at: &mut Option<tokio::time::Instant>,
    delay_secs: u64,
) {
    {
        let mut s = app.lock().unwrap();
        s.clear();
        s.picker_connected_idx = None;
        s.picker_entries.clear();   // remove stale device from the list immediately
        s.show_picker = false;
        s.mode = AppMode::Scanning;
        s.picker_scanning = true;
    }
    if pending_scan.is_none() {
        *retry_at = Some(tokio::time::Instant::now() + Duration::from_secs(delay_secs));
    }
}

/// Spawn a task that forwards BLE events into `app`.
/// Returns immediately; the task runs until the peripheral disconnects.
fn spawn_event_task(mut rx: tokio::sync::mpsc::Receiver<MuseEvent>, app: Arc<Mutex<App>>) {
    tokio::spawn(async move {
        while let Some(ev) = rx.recv().await {
            let mut s = app.lock().unwrap();
            match ev {
                // The Connected event from setup_peripheral is now redundant
                // (mode is set by do_connect before spawning this task), but
                // we handle it anyway as a safety net.
                MuseEvent::Connected(_) => {}
                MuseEvent::Disconnected => {
                    s.mode = AppMode::Disconnected;
                    s.picker_connected_idx = None;
                    break;
                }
                MuseEvent::Eeg(r) if r.electrode < NUM_CH => {
                    s.push(r.electrode, &r.samples);
                }
                MuseEvent::Telemetry(t) => {
                    s.battery = Some(t.battery_level);
                }
                MuseEvent::Accelerometer(a) => {
                    let v = a.samples[0];
                    s.accel = Some((v.x, v.y, v.z));
                }
                MuseEvent::Gyroscope(g) => {
                    let v = g.samples[0];
                    s.gyro = Some((v.x, v.y, v.z));
                }
                _ => {}
            }
        }
    });
}

/// Payload delivered on a successful connection.
struct ConnectOutcome {
    rx: mpsc::Receiver<MuseEvent>,
    handle: MuseHandle,
    device_idx: usize,
    name: String,
    id: String,
}

/// Kick off a background connection attempt and return immediately.
///
/// The caller receives a `oneshot` that resolves to:
/// - `Some(ConnectOutcome)` on success
/// - `None` on failure  (`app.mode` is already set to `Disconnected`)
///
/// All blocking work (`peripheral.connect()`, service discovery, startup
/// sequence) runs inside a spawned task so the main loop stays responsive.
fn start_connect(
    idx: usize,
    device: MuseDevice,
    app: Arc<Mutex<App>>,
    scan_cfg: MuseClientConfig,
) -> oneshot::Receiver<Option<ConnectOutcome>> {
    let (tx, rx) = oneshot::channel();

    // Immediately update UI to "Connecting…" without blocking.
    {
        let mut s = app.lock().unwrap();
        s.clear();
        s.mode = AppMode::Connecting(device.name.clone());
        s.picker_connected_idx = None;
        s.show_picker = false;
    }

    tokio::spawn(async move {
        let client = MuseClient::new(scan_cfg);
        match client.connect_to(device.clone()).await {
            Ok((evt_rx, h)) => {
                match h.start(false, false).await {
                    Ok(()) => {
                        let _ = tx.send(Some(ConnectOutcome {
                            rx: evt_rx,
                            handle: h,
                            device_idx: idx,
                            name: device.name.clone(),
                            id: short_id(&device.id),
                        }));
                    }
                    Err(e) => {
                        log::warn!("start() failed: {e}");
                        // Disconnect so BlueZ doesn't keep the link half-open.
                        let _ = h.disconnect().await;
                        let mut s = app.lock().unwrap();
                        s.mode = AppMode::Disconnected;
                        s.last_error = Some(format!("start() failed: {e}"));
                        let _ = tx.send(None);
                    }
                }
            }
            Err(e) => {
                log::warn!("connect_to failed: {e}");
                let mut s = app.lock().unwrap();
                s.mode = AppMode::Disconnected;
                s.last_error = Some(format!("connect_to failed: {e}"));
                let _ = tx.send(None);
            }
        }
    });

    rx
}

// ── Rendering ─────────────────────────────────────────────────────────────────

/// Top-level render callback handed to [`Terminal::draw`].
///
/// Divides the terminal into three horizontal bands (header / charts / footer)
/// and then optionally overlays the device-picker modal on top.
fn draw(frame: &mut Frame, app: &App) {
    let area = frame.area();
    let root = Layout::vertical([
        Constraint::Length(3),
        Constraint::Min(0),
        Constraint::Length(3),
    ])
    .split(area);

    draw_header(frame, root[0], app);
    draw_charts(frame, root[1], app);
    draw_footer(frame, root[2], app);

    if app.show_picker {
        draw_device_picker(frame, area, app);
    }
}

// ── Header ────────────────────────────────────────────────────────────────────

/// Return the current braille spinner frame based on wall-clock milliseconds.
///
/// The frame advances every 100 ms, producing a smooth animation at the
/// ~30 FPS render rate without requiring any additional state.
fn spinner_str() -> &'static str {
    let ms = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis();
    SPINNER[(ms / 100) as usize % SPINNER.len()]
}

/// Render the status bar at the top of the screen.
///
/// Displays (left to right): app title, connection status (with spinner when
/// scanning), battery percentage, packet rate, Y-axis scale, and total sample count.
/// The status label colour reflects the current mode: green = connected,
/// yellow = scanning/no devices, red = disconnected.
fn draw_header(frame: &mut Frame, area: Rect, app: &App) {
    let (label, color) = match &app.mode {
        AppMode::Scanning => (format!("{} Scanning…", spinner_str()), Color::Yellow),
        AppMode::Connecting(name) => (
            format!("{} Connecting to {}…", spinner_str(), name),
            Color::Yellow,
        ),
        AppMode::Connected { name, id } => {
            (format!("● {}  [{}]", name, id), Color::Green)
        }
        AppMode::Simulated => ("◆ Simulated".to_owned(), Color::Cyan),
        AppMode::NoDevices => (
            format!("{} No devices found — retrying…", spinner_str()),
            Color::Yellow,
        ),
        AppMode::Disconnected => {
            let reason = app
                .last_error
                .as_deref()
                .map(|e| format!(" ({e})"))
                .unwrap_or_default();
            (
                format!("{} Disconnected{reason} — retrying…", spinner_str()),
                Color::Red,
            )
        }
    };

    let bat = app
        .battery
        .map(|b| format!("Bat {b:.0}%"))
        .unwrap_or_else(|| "Bat N/A".into());

    let rate = format!("{:.1} pkt/s", app.pkt_rate());
    let scale = format!("±{:.0} µV", app.y_range());
    let total = format!("{}K smp", app.total_samples / 1_000);

    let line = Line::from(vec![
        Span::styled(
            " MUSE EEG Monitor ",
            Style::default().fg(Color::White).add_modifier(Modifier::BOLD),
        ),
        sep(),
        Span::styled(label, Style::default().fg(color).add_modifier(Modifier::BOLD)),
        sep(),
        Span::styled(bat, Style::default().fg(Color::White)),
        sep(),
        Span::styled(rate, Style::default().fg(Color::White)),
        sep(),
        Span::styled(
            scale,
            Style::default()
                .fg(Color::LightBlue)
                .add_modifier(Modifier::BOLD),
        ),
        sep(),
        Span::styled(total, Style::default().fg(Color::DarkGray)),
        Span::raw(" "),
    ]);

    frame.render_widget(
        Paragraph::new(line).block(Block::default().borders(Borders::ALL)),
        area,
    );
}

/// Dimmed vertical separator used between header fields.
#[inline]
fn sep<'a>() -> Span<'a> {
    Span::styled(" │ ", Style::default().fg(Color::DarkGray))
}

// ── EEG charts ────────────────────────────────────────────────────────────────

/// Render the four EEG waveform charts stacked vertically.
///
/// Converts each channel's ring-buffer into `(time_s, µV)` point pairs,
/// clamps them to the current Y window (to prevent ratatui from discarding
/// out-of-range points), and dispatches one [`draw_channel`] call per row.
fn draw_charts(frame: &mut Frame, area: Rect, app: &App) {
    let rows = Layout::vertical([
        Constraint::Ratio(1, 4),
        Constraint::Ratio(1, 4),
        Constraint::Ratio(1, 4),
        Constraint::Ratio(1, 4),
    ])
    .split(area);

    let y_range = app.y_range();
    let data: Vec<Vec<(f64, f64)>> = (0..NUM_CH)
        .map(|ch| {
            app.bufs[ch]
                .iter()
                .enumerate()
                // Clamp to the visible Y window so every sample stays on-screen.
                // Without clamping, ratatui silently drops out-of-range points and
                // the waveform breaks into isolated dots wherever the signal
                // exceeds the axis bounds (common with Athena high-amplitude data).
                .map(|(i, &v)| (i as f64 / EEG_HZ, v.clamp(-y_range, y_range)))
                .collect()
        })
        .collect();

    for ch in 0..NUM_CH {
        draw_channel(frame, rows[ch], ch, &data[ch], app);
    }
}

/// Render a single EEG channel chart into `area`.
///
/// * **Title bar** — electrode name, live min/max/RMS stats in µV,
///   and optional `[CLIP]` / `[SMOOTH]` badges.
/// * **Border** — turns red when any sample in the buffer exceeds the Y axis.
/// * **Smooth mode** — draws the raw signal in a dim colour as background
///   context, then overlays the 9-sample moving-average in the full channel colour.
/// * **Raw mode** — single bright line with no overlay.
fn draw_channel(frame: &mut Frame, area: Rect, ch: usize, data: &[(f64, f64)], app: &App) {
    let color = COLORS[ch];
    let y_range = app.y_range();
    let name = EEG_CHANNEL_NAMES[ch];

    // Compute stats from the raw (un-clamped) buffer so min/max reflect true signal.
    let (min_v, max_v, rms_v) = {
        let buf = &app.bufs[ch];
        if buf.is_empty() {
            (0.0_f64, 0.0_f64, 0.0_f64)
        } else {
            let min = buf.iter().copied().fold(f64::INFINITY, f64::min);
            let max = buf.iter().copied().fold(f64::NEG_INFINITY, f64::max);
            let rms = (buf.iter().map(|&v| v * v).sum::<f64>() / buf.len() as f64).sqrt();
            (min, max, rms)
        }
    };

    // Clipping = any sample lies outside the current Y window.
    let clipping = max_v > y_range || min_v < -y_range;

    let border_color = if clipping { Color::Red } else { color };

    let clip_tag = if clipping { " [CLIP +]" } else { "" };
    let smooth_tag = if app.smooth { " [SMOOTH]" } else { "" };
    let title = format!(
        " {name}  min:{min_v:+6.1}  max:{max_v:+6.1}  rms:{rms_v:5.1} µV{clip_tag}{smooth_tag} "
    );

    let y_labels: Vec<String> = [-1.0, -0.5, 0.0, 0.5, 1.0]
        .iter()
        .map(|&f| format!("{:+.0}", f * y_range))
        .collect();

    let x_labels = vec![
        "0s".to_string(),
        format!("{:.1}s", WINDOW_SECS / 2.0),
        format!("{:.0}s", WINDOW_SECS),
    ];

    // Build dataset(s).
    // Smooth mode: dim raw trace underneath + bright smoothed line on top.
    // Raw mode:    single bright raw trace.
    let smoothed: Vec<(f64, f64)> = if app.smooth {
        smooth_signal(data, SMOOTH_WINDOW)
    } else {
        vec![]
    };

    let datasets: Vec<Dataset> = if app.smooth {
        vec![
            // Layer 1 – raw signal, dimmed so it reads as context/background.
            Dataset::default()
                .marker(symbols::Marker::Braille)
                .graph_type(GraphType::Line)
                .style(Style::default().fg(DIM_COLORS[ch]))
                .data(data),
            // Layer 2 – smoothed overlay in the full channel colour.
            Dataset::default()
                .marker(symbols::Marker::Braille)
                .graph_type(GraphType::Line)
                .style(Style::default().fg(color))
                .data(&smoothed),
        ]
    } else {
        vec![Dataset::default()
            .marker(symbols::Marker::Braille)
            .graph_type(GraphType::Line)
            .style(Style::default().fg(color))
            .data(data)]
    };

    let chart = Chart::new(datasets)
        .block(
            Block::default()
                .title(Span::styled(
                    title,
                    Style::default().fg(color).add_modifier(Modifier::BOLD),
                ))
                .borders(Borders::ALL)
                .border_style(Style::default().fg(border_color)),
        )
        .x_axis(
            Axis::default()
                .bounds([0.0, WINDOW_SECS])
                .labels(x_labels)
                .style(Style::default().fg(Color::DarkGray)),
        )
        .y_axis(
            Axis::default()
                .bounds([-y_range, y_range])
                .labels(y_labels)
                .style(Style::default().fg(Color::DarkGray)),
        );

    frame.render_widget(chart, area);
}

// ── Footer ────────────────────────────────────────────────────────────────────

/// Render the two-line footer at the bottom of the screen.
///
/// **Line 1** — keyboard shortcut reference.  A `⏸ PAUSED` badge is appended
/// when streaming is paused.
///
/// **Line 2** — context-dependent:
/// * `NoDevices` mode: a platform-specific hint for how to grant Bluetooth access.
/// * All other modes: live accelerometer and gyroscope readings.
fn draw_footer(frame: &mut Frame, area: Rect, app: &App) {
    let pause_span = if app.paused {
        Span::styled(
            "  ⏸ PAUSED",
            Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
        )
    } else {
        Span::raw("")
    };

    let keys = Line::from(vec![
        Span::raw(" "),
        key("[Tab]"),
        Span::raw("Devices  "),
        key("[d]"),
        Span::raw("Disconnect  "),
        key("[+]"),
        Span::raw("Scale↑  "),
        key("[-]"),
        Span::raw("Scale↓  "),
        key("[a]"),
        Span::raw("Auto-scale  "),
        key("[v]"),
        Span::raw(if app.smooth { "Raw  " } else { "Smooth  " }),
        key("[p]"),
        Span::raw("Pause  "),
        key("[r]"),
        Span::raw("Resume  "),
        key("[c]"),
        Span::raw("Clear  "),
        key("[q]"),
        Span::raw("Quit"),
        pause_span,
    ]);

    // Second row: macOS permission hint when waiting for devices, else IMU.
    let second_line = match &app.mode {
        AppMode::NoDevices => {
            let base = if cfg!(target_os = "macos") {
                " No Muse found. On macOS grant Bluetooth access: System Settings → Privacy & Security → Bluetooth."
            } else {
                " No Muse found. Make sure the headset is powered on and in range."
            };
            let detail = app
                .last_error
                .as_deref()
                .map(|e| format!(" Error: {e}"))
                .unwrap_or_default();
            Line::from(Span::styled(
                format!("{base}{detail} Retrying…"),
                Style::default().fg(Color::Yellow),
            ))
        }
        _ => {
            let (ax, ay, az) = app.accel.unwrap_or((0.0, 0.0, 0.0));
            let (gx, gy, gz) = app.gyro.unwrap_or((0.0, 0.0, 0.0));
            Line::from(vec![
                Span::raw(" "),
                Span::styled("Accel ", Style::default().fg(Color::DarkGray)),
                Span::styled(
                    format!("x:{ax:+.3}g  y:{ay:+.3}g  z:{az:+.3}g"),
                    Style::default().fg(Color::Cyan),
                ),
                Span::raw("   "),
                Span::styled("Gyro ", Style::default().fg(Color::DarkGray)),
                Span::styled(
                    format!("x:{gx:+.3}°/s  y:{gy:+.3}°/s  z:{gz:+.3}°/s"),
                    Style::default().fg(Color::Magenta),
                ),
            ])
        }
    };

    frame.render_widget(
        Paragraph::new(vec![keys, second_line]).block(Block::default().borders(Borders::ALL)),
        area,
    );
}

/// Styled keybinding label (bold yellow) used in the footer hint line.
#[inline]
fn key(s: &str) -> Span<'_> {
    Span::styled(
        s,
        Style::default()
            .fg(Color::Yellow)
            .add_modifier(Modifier::BOLD),
    )
}

// ── Device picker overlay ─────────────────────────────────────────────────────

/// Render the centered device-picker modal over the main UI.
///
/// The popup is sized to 60 % of the terminal width (min 52, max full width)
/// and tall enough to show all discovered devices plus a 2-line hint area.
///
/// * Devices are shown as `"Name  [SHORT-ID]"` with the connected device
///   highlighted in green with a `← connected` suffix.
/// * While a BLE scan is running a spinner appears in the title bar.
/// * The cursor row is highlighted with a `▶` prefix; `↑` / `↓` scroll it.
fn draw_device_picker(frame: &mut Frame, area: Rect, app: &App) {
    let n = app.picker_entries.len().max(1);
    let inner_h = n as u16 + 4;
    let box_h = inner_h + 2;
    let box_w = (area.width * 60 / 100).max(52).min(area.width);
    let x = area.x + (area.width.saturating_sub(box_w)) / 2;
    let y = area.y + (area.height.saturating_sub(box_h)) / 2;
    let popup = Rect::new(x, y, box_w, box_h.min(area.height));

    frame.render_widget(Clear, popup);

    let title = if app.picker_scanning {
        format!(" {} Scanning…  ({} found) ", spinner_str(), app.picker_entries.len())
    } else {
        format!(" Select Device  ({} found) ", app.picker_entries.len())
    };

    frame.render_widget(
        Block::default()
            .title(Span::styled(
                title,
                Style::default().fg(Color::White).add_modifier(Modifier::BOLD),
            ))
            .borders(Borders::ALL)
            .border_style(Style::default().fg(Color::White)),
        popup,
    );

    let inner = popup.inner(Margin {
        horizontal: 1,
        vertical: 1,
    });

    let hint_h = 2u16;
    let [list_area, _, hint_area] = Layout::vertical([
        Constraint::Length(inner.height.saturating_sub(hint_h + 1)),
        Constraint::Length(1),
        Constraint::Length(hint_h),
    ])
    .areas(inner);

    // Build list items
    let items: Vec<ListItem> = if app.picker_entries.is_empty() {
        vec![ListItem::new(Span::styled(
            "  No devices found — press [s] to scan",
            Style::default().fg(Color::DarkGray),
        ))]
    } else {
        app.picker_entries
            .iter()
            .enumerate()
            .map(|(i, entry)| {
                let connected = app.picker_connected_idx == Some(i);
                let (bullet, color, suffix) = if connected {
                    ("● ", Color::Green, "  ← connected")
                } else {
                    ("  ", Color::White, "")
                };
                ListItem::new(Span::styled(
                    format!("{bullet}{entry}{suffix}"),
                    Style::default().fg(color),
                ))
            })
            .collect()
    };

    let mut list_state = ListState::default();
    if !app.picker_entries.is_empty() {
        list_state.select(Some(app.picker_cursor));
    }

    frame.render_stateful_widget(
        List::new(items)
            .highlight_style(
                Style::default()
                    .fg(Color::Black)
                    .bg(Color::White)
                    .add_modifier(Modifier::BOLD),
            )
            .highlight_symbol("▶ "),
        list_area,
        &mut list_state,
    );

    frame.render_widget(
        Paragraph::new(vec![
            Line::from(vec![
                key(" [↑↓]"),
                Span::raw(" Navigate  "),
                key("[↵]"),
                Span::raw(" Connect  "),
                key("[s]"),
                Span::raw(" Rescan  "),
                key("[Esc]"),
                Span::raw(" Close"),
            ]),
            Line::from(Span::styled(
                " Device list is refreshed after every scan",
                Style::default().fg(Color::DarkGray),
            )),
        ]),
        hint_area,
    );
}

// ── Entry point ───────────────────────────────────────────────────────────────

#[tokio::main]
async fn main() -> Result<()> {
    use std::io::IsTerminal as _;
    if !io::stdout().is_terminal() {
        eprintln!("Error: muse-rs tui requires a real terminal (TTY).");
        eprintln!("Run it directly in a terminal emulator, not piped or redirected.");
        std::process::exit(1);
    }

    // ── Logging ─────────────────────────────────────────────────────────────
    // Write logs to a file so they never interfere with the TUI display.
    // Set RUST_LOG=debug for verbose BLE diagnostics, e.g.:
    //   RUST_LOG=debug cargo run --bin tui
    // Logs are written to muse-tui.log in the current directory.
    {
        use std::fs::File;
        if let Ok(file) = File::create("muse-tui.log") {
            env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
                .target(env_logger::Target::Pipe(Box::new(file)))
                .init();
        }
    }

    let simulate = std::env::args().any(|a| a == "--simulate");

    // ── Shared UI state ───────────────────────────────────────────────────────
    let app = Arc::new(Mutex::new(App::new()));

    // ── Session state (owned by main task only) ───────────────────────────────
    let mut devices: Vec<MuseDevice> = vec![];
    let mut connected_idx: Option<usize> = None;
    let mut handle: Option<Arc<MuseHandle>> = None;

    // Oneshot receiver for the current background scan task.
    let mut pending_scan: Option<oneshot::Receiver<ScanResult>> = None;
    // Oneshot receiver for the current background connect task.
    let mut pending_connect: Option<oneshot::Receiver<Option<ConnectOutcome>>> = None;
    // Instant at which the next automatic retry scan should start.
    let mut retry_at: Option<tokio::time::Instant> = None;

    let scan_cfg = MuseClientConfig {
        scan_timeout_secs: 5,
        ..Default::default()
    };

    // ── Start data source ─────────────────────────────────────────────────────
    if simulate {
        let mut s = app.lock().unwrap();
        s.mode = AppMode::Simulated;
        // Simulator peak ≈ ±40 µV — start at ±50 µV so the waveform fills the chart.
        // Real devices use the DEFAULT_SCALE (±500 µV) set at App::new().
        s.scale_idx = 2; // Y_SCALES[2] = 50.0
        drop(s);
        spawn_simulator(Arc::clone(&app));
    } else {
        app.lock().unwrap().picker_scanning = true;
        pending_scan = Some(start_scan(scan_cfg.clone()));
    }

    // ── Terminal setup ────────────────────────────────────────────────────────
    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen)?;
    let mut terminal = Terminal::new(CrosstermBackend::new(stdout))?;
    let tick = Duration::from_millis(33); // ~30 FPS

    // ── Main loop ─────────────────────────────────────────────────────────────
    'main: loop {
        // ── 1. Collect finished scan results ─────────────────────────────────
        if let Some(ref mut rx) = pending_scan {
            if let Ok(scan_result) = rx.try_recv() {
                pending_scan = None;
                devices = scan_result.devices;

                {
                    let mut s = app.lock().unwrap();
                    s.picker_entries = devices.iter().map(device_entry).collect();
                    s.picker_scanning = false;
                    if devices.is_empty() {
                        s.mode = AppMode::NoDevices;
                        if let Some(err) = scan_result.error {
                            s.last_error = Some(err);
                        }
                    }
                }

                if devices.is_empty() {
                    retry_at = Some(
                        tokio::time::Instant::now() + Duration::from_secs(RETRY_SECS),
                    );
                } else if handle.is_none() && pending_connect.is_none() {
                    // Auto-connect to the first device — non-blocking.
                    pending_connect = Some(start_connect(
                        0,
                        devices[0].clone(),
                        Arc::clone(&app),
                        scan_cfg.clone(),
                    ));
                }
            }
        }

        // ── 1b. Collect finished connection attempt ───────────────────────────
        if let Some(ref mut rx) = pending_connect {
            if let Ok(result) = rx.try_recv() {
                pending_connect = None;
                if let Some(outcome) = result {
                    let h = Arc::new(outcome.handle);
                    {
                        let mut s = app.lock().unwrap();
                        s.mode = AppMode::Connected {
                            name: outcome.name,
                            id: outcome.id,
                        };
                        s.last_error = None;
                        s.picker_connected_idx = Some(outcome.device_idx);
                    }
                    connected_idx = Some(outcome.device_idx);
                    handle = Some(Arc::clone(&h));
                    spawn_event_task(outcome.rx, Arc::clone(&app));
                } else {
                    // Connection attempt failed — wipe state and restart scanning.
                    connected_idx = None;
                    devices.clear();    // remove stale device entries from local list
                    restart_scan(&app, &mut pending_scan, &mut retry_at, RECONNECT_DELAY_SECS);
                }
            }
        }

        // ── 2. Detect unexpected disconnection ───────────────────────────────
        //    The event task sets mode = Disconnected; we react here.
        {
            let is_disconnected =
                matches!(app.lock().unwrap().mode, AppMode::Disconnected);
            if is_disconnected && handle.is_some() {
                // Drop the dead handle; best-effort remote disconnect in background.
                if let Some(h) = handle.take() {
                    tokio::spawn(async move { let _ = h.disconnect().await; });
                }
                connected_idx = None;
                devices.clear();    // remove stale device entries from local list
                restart_scan(&app, &mut pending_scan, &mut retry_at, RECONNECT_DELAY_SECS);
            }
        }

        // ── 3. Fire pending retry scan ────────────────────────────────────────
        if let Some(t) = retry_at {
            if tokio::time::Instant::now() >= t && pending_scan.is_none() {
                retry_at = None;
                app.lock().unwrap().mode = AppMode::Scanning;
                app.lock().unwrap().picker_scanning = true;
                pending_scan = Some(start_scan(scan_cfg.clone()));
            }
        }

        // ── 4. Render ─────────────────────────────────────────────────────────
        {
            let s = app.lock().unwrap();
            terminal.draw(|f| draw(f, &s))?;
        }

        // ── 5. Handle keyboard ────────────────────────────────────────────────
        if !event::poll(tick)? {
            continue;
        }
        let Event::Key(key) = event::read()? else {
            continue;
        };

        // ── Global quit — fires regardless of picker or any other overlay ────
        // In raw mode Ctrl+C is not SIGINT; it arrives as a key event and must
        // be handled explicitly.  Without this check the app is impossible to
        // exit while the picker is open.
        let ctrl_c = key.modifiers.contains(KeyModifiers::CONTROL)
            && key.code == KeyCode::Char('c');
        if key.code == KeyCode::Char('q') || ctrl_c {
            break 'main;
        }

        // ── Picker overlay keys ───────────────────────────────────────────────
        if app.lock().unwrap().show_picker {
            match key.code {
                KeyCode::Esc => {
                    app.lock().unwrap().show_picker = false;
                }
                KeyCode::Char('s') => {
                    if pending_scan.is_none() {
                        retry_at = None;
                        let mut s = app.lock().unwrap();
                        s.mode = AppMode::Scanning;
                        s.picker_scanning = true;
                        drop(s);
                        pending_scan = Some(start_scan(scan_cfg.clone()));
                    }
                }
                KeyCode::Up => {
                    let mut s = app.lock().unwrap();
                    if s.picker_cursor > 0 {
                        s.picker_cursor -= 1;
                    }
                }
                KeyCode::Down => {
                    let mut s = app.lock().unwrap();
                    let max = s.picker_entries.len().saturating_sub(1);
                    if s.picker_cursor < max {
                        s.picker_cursor += 1;
                    }
                }
                KeyCode::Enter => {
                    let (cursor, n) = {
                        let s = app.lock().unwrap();
                        (s.picker_cursor, s.picker_entries.len())
                    };
                    if n > 0 && cursor < n && cursor < devices.len() {
                        retry_at = None;
                        // Disconnect old handle in the background.
                        if let Some(h) = handle.take() {
                            tokio::spawn(async move { let _ = h.disconnect().await; });
                        }
                        connected_idx = None;
                        pending_connect = Some(start_connect(
                            cursor,
                            devices[cursor].clone(),
                            Arc::clone(&app),
                            scan_cfg.clone(),
                        ));
                    }
                }
                _ => {}
            }
            continue;
        }

        // ── Normal-view keys ──────────────────────────────────────────────────
        match key.code {
            KeyCode::Char('q') | KeyCode::Esc => break 'main,

            // Open device picker
            KeyCode::Tab => {
                let mut s = app.lock().unwrap();
                s.show_picker = true;
                if let Some(ci) = connected_idx {
                    s.picker_cursor = ci;
                }
            }

            // Manual rescan
            KeyCode::Char('s') => {
                if pending_scan.is_none() {
                    retry_at = None;
                    app.lock().unwrap().mode = AppMode::Scanning;
                    app.lock().unwrap().picker_scanning = true;
                    pending_scan = Some(start_scan(scan_cfg.clone()));
                }
            }

            // Disconnect current device (keep scanning)
            KeyCode::Char('d') => {
                if let Some(h) = handle.take() {
                    // Fire-and-forget: never block the main loop on a BLE call.
                    tokio::spawn(async move { let _ = h.disconnect().await; });
                }
                pending_connect = None;
                connected_idx = None;
                app.lock().unwrap().picker_connected_idx = None;
                // Trigger a fresh scan to allow re-connecting
                if pending_scan.is_none() {
                    retry_at = None;
                    app.lock().unwrap().mode = AppMode::Scanning;
                    app.lock().unwrap().picker_scanning = true;
                    pending_scan = Some(start_scan(scan_cfg.clone()));
                }
            }

            // µV scale
            KeyCode::Char('+') | KeyCode::Char('=') => {
                app.lock().unwrap().scale_up();
            }
            KeyCode::Char('-') => {
                app.lock().unwrap().scale_down();
            }
            KeyCode::Char('a') => {
                app.lock().unwrap().auto_scale();
            }

            // Toggle smooth overlay
            KeyCode::Char('v') => {
                let mut s = app.lock().unwrap();
                s.smooth = !s.smooth;
            }

            // Pause / resume
            KeyCode::Char('p') => {
                app.lock().unwrap().paused = true;
                if let Some(h) = handle.clone() {
                    tokio::spawn(async move { let _ = h.pause().await; });
                }
            }
            KeyCode::Char('r') => {
                app.lock().unwrap().paused = false;
                if let Some(h) = handle.clone() {
                    tokio::spawn(async move { let _ = h.resume().await; });
                }
            }

            // Clear buffers (guard: Ctrl+C is already handled above as quit)
            KeyCode::Char('c') if !key.modifiers.contains(KeyModifiers::CONTROL) => {
                app.lock().unwrap().clear();
            }

            _ => {}
        }
    }

    // ── Teardown ──────────────────────────────────────────────────────────────
    if let Some(h) = handle {
        let _ = h.disconnect().await;
    }
    disable_raw_mode()?;
    execute!(terminal.backend_mut(), LeaveAlternateScreen)?;
    terminal.show_cursor()?;
    Ok(())
}
