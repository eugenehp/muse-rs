use std::collections::BTreeSet;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use anyhow::{anyhow, Result};
use btleplug::api::{
    Central, CentralEvent, Characteristic, Manager as _, Peripheral as _, ScanFilter, WriteType,
};
use btleplug::platform::{Adapter, Manager, Peripheral};
use futures::StreamExt;
use log::{debug, info, warn};
use tokio::sync::mpsc;
use uuid::Uuid;

use crate::parse::{
    decode_eeg_samples, parse_accelerometer, parse_athena_notification, parse_gyroscope,
    parse_ppg_reading, parse_telemetry, ControlAccumulator,
};
use crate::protocol::{
    decode_response, encode_command, ACCELEROMETER_CHARACTERISTIC, ATHENA_SENSOR_CHARACTERISTIC,
    CONTROL_CHARACTERISTIC, EEG_CHARACTERISTICS, EEG_FREQUENCY, EEG_SAMPLES_PER_READING,
    GYROSCOPE_CHARACTERISTIC, PPG_CHARACTERISTICS, PPG_FREQUENCY, PPG_SAMPLES_PER_READING,
    TELEMETRY_CHARACTERISTIC,
};
use crate::types::{ControlResponse, EegReading, MuseEvent};

// ── Timestamp helper ──────────────────────────────────────────────────────────

fn now_ms() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("system clock is before Unix epoch")
        .as_secs_f64()
        * 1000.0
}

// ── Per-channel timestamp tracker (mirrors JS getTimestamp) ──────────────────

/// Reconstructs a wall-clock timestamp for each BLE EEG/PPG notification
/// using the device's monotonic packet index.
///
/// The Muse headset does not embed absolute timestamps in its BLE packets.
/// Instead, each notification carries a 16-bit sequence counter that
/// increments by 1 per packet.  This tracker anchors the first packet to
/// `now()` and then extrapolates all subsequent timestamps from the index
/// delta and the known sample rate, mirroring the TypeScript `getTimestamp`
/// helper in `muse-utils.ts`.
///
/// Benefits over simply calling `now()` on every packet:
/// * Handles short bursts of late delivery (jitter) without skewing timestamps.
/// * Preserves the correct inter-packet spacing even when the event loop is
///   briefly busy.
/// * Correctly back-dates packets that arrive with an index already behind
///   the current anchor.
///
/// One `TimestampTracker` should be created per logical channel (electrode or
/// PPG channel) and reused for the lifetime of that connection.
struct TimestampTracker {
    /// The packet index seen in the most recently *forwarded* packet, or
    /// `None` before the first packet is received.
    last_index: Option<u16>,
    /// Wall-clock timestamp (ms since epoch) that corresponds to `last_index`,
    /// or `None` before the first packet is received.
    last_timestamp: Option<f64>,
}

impl TimestampTracker {
    /// Create a new tracker with no anchor.  The first call to [`get`] will
    /// anchor it to the current wall clock.
    fn new() -> Self {
        Self {
            last_index: None,
            last_timestamp: None,
        }
    }

    /// Return the wall-clock timestamp in milliseconds since Unix epoch for the
    /// notification identified by `event_index`.
    ///
    /// `samples_per_reading` and `frequency` are used to compute the duration
    /// of one reading: `reading_delta_ms = 1000 × samples_per_reading / frequency`.
    ///
    /// On the first call the anchor is set to `now() − reading_delta` so that
    /// the returned timestamp represents the *start* of the reading window
    /// rather than the moment the packet arrived.
    ///
    /// 16-bit counter wrap-around (0xFFFF → 0x0000) is handled by detecting
    /// when the raw delta would be larger than 0x1000 and adjusting accordingly.
    fn get(&mut self, event_index: u16, samples_per_reading: usize, frequency: f64) -> f64 {
        let reading_delta = 1000.0 * (1.0 / frequency) * samples_per_reading as f64;

        if self.last_index.is_none() || self.last_timestamp.is_none() {
            self.last_index = Some(event_index);
            self.last_timestamp = Some(now_ms() - reading_delta);
        }

        let mut idx = event_index as i32;
        let last = self.last_index.unwrap() as i32;

        // Handle 16-bit wrap-around: if the apparent backward delta is larger
        // than half the counter space, assume the counter has wrapped.
        while last - idx > 0x1000 {
            idx += 0x10000;
        }

        let ts = self.last_timestamp.unwrap();

        if idx == last {
            // Duplicate or re-delivered packet — return the existing anchor.
            ts
        } else if idx > last {
            // Normal forward progress: advance the anchor.
            let new_ts = ts + reading_delta * (idx - last) as f64;
            self.last_index = Some(event_index);
            self.last_timestamp = Some(new_ts);
            new_ts
        } else {
            // Packet arrived with an index behind the anchor (late delivery or
            // out-of-order).  Back-date without updating the anchor so future
            // packets continue to be stamped correctly.
            ts - reading_delta * (last - idx) as f64
        }
    }

    /// Reset the tracker.  The next call to [`get`] will re-anchor to the
    /// current wall clock, as if the tracker were freshly constructed.
    ///
    /// Call this when re-connecting or after a detected stream gap.
    fn reset(&mut self) {
        self.last_index = None;
        self.last_timestamp = None;
    }
}

// ── MuseDevice ────────────────────────────────────────────────────────────────

/// A Muse headset discovered during a BLE scan.
///
/// Returned by [`MuseClient::scan_all`]; pass to [`MuseClient::connect_to`]
/// to establish a streaming connection.
#[derive(Clone, Debug)]
pub struct MuseDevice {
    /// Advertised device name (e.g. `"Muse-AB12"`).
    pub name: String,
    /// Platform BLE identifier.
    /// • macOS / Windows — a UUID string
    /// • Linux — a Bluetooth MAC address (`AA:BB:CC:DD:EE:FF`)
    pub id: String,
    pub(crate) peripheral: Peripheral,
    /// The adapter that discovered this device.  Kept so that
    /// [`MuseClient::connect_to`] can listen for disconnect events on the
    /// correct adapter without creating a second `Manager`.
    pub(crate) adapter: Adapter,
}

// ── MuseClientConfig ──────────────────────────────────────────────────────────

/// Configuration for [`MuseClient`].
///
/// The same config is used for both Classic and Athena firmware.
/// Fields that are firmware-specific are noted below.
#[derive(Debug, Clone)]
pub struct MuseClientConfig {
    /// Subscribe to the AUX (5th) EEG channel.
    ///
    /// **Classic firmware only** — changes the startup preset from `p21` to `p20`.
    /// Has no effect on Athena firmware (`p1045` is always used).
    /// Default: `false`.
    pub enable_aux: bool,
    /// Subscribe to the three PPG (optical heart-rate) channels.
    ///
    /// **Classic firmware only** — changes the startup preset to `p50`.
    /// Has no effect on Athena firmware; Athena optical data is received but
    /// not yet decoded into [`crate::types::MuseEvent::Ppg`].
    /// Default: `false`.
    pub enable_ppg: bool,
    /// BLE scan duration in seconds before giving up. Default: `15`.
    pub scan_timeout_secs: u64,
    /// Match devices whose advertised name starts with this string.
    ///
    /// The default `"Muse"` matches all known Muse models.  Use a more
    /// specific prefix (e.g. `"Muse-AB"`) to connect to a particular headset
    /// in a multi-device environment.  Default: `"Muse"`.
    pub name_prefix: String,
}

impl Default for MuseClientConfig {
    fn default() -> Self {
        Self {
            enable_aux: false,
            enable_ppg: false,
            scan_timeout_secs: 15,
            name_prefix: "Muse".into(),
        }
    }
}

// ── MuseClient ────────────────────────────────────────────────────────────────

/// BLE client for Muse EEG headsets.
///
/// Handles scanning, connecting, GATT subscription, and notification dispatch
/// for all supported Muse models.  Firmware is detected automatically at
/// connect time by checking for the presence of the Athena universal sensor
/// characteristic (`273e0013-…`):
///
/// * **Classic** (Muse 1, Muse 2, Muse S ≤ fw 3.x) — subscribes to one
///   characteristic per sensor and dispatches typed notifications.
/// * **Athena** (Muse S fw ≥ 4.x) — subscribes to the single universal
///   characteristic and demultiplexes tag-based packets.
///
/// No configuration change is needed to switch between firmware variants;
/// the same `MuseClientConfig` and event stream work for both.
pub struct MuseClient {
    config: MuseClientConfig,
}

impl MuseClient {
    pub fn new(config: MuseClientConfig) -> Self {
        Self { config }
    }

    // ── Public: scan ─────────────────────────────────────────────────────────

    /// Scan for **all** nearby Muse devices and return them.
    ///
    /// The scan runs for `config.scan_timeout_secs` seconds so that multiple
    /// devices in range can all be discovered before the function returns.
    ///
    /// On macOS, `CBCentralManager` needs a moment to reach the *poweredOn*
    /// state after initialisation; we wait up to 2 s for that before starting
    /// the actual RF scan.
    pub async fn scan_all(&self) -> Result<Vec<MuseDevice>> {
        let manager = Manager::new().await?;
        let adapters = manager.adapters().await?;
        let adapter = adapters
            .into_iter()
            .next()
            .ok_or_else(|| anyhow!("No Bluetooth adapter found"))?;

        // ── macOS: wait for the CoreBluetooth manager to reach poweredOn ─────
        // When the binary is freshly launched (or Bluetooth was recently
        // toggled), CBCentralManager starts in an "unknown" state.
        // Calling scanForPeripherals before it is ready is a silent no-op.
        // We poll adapter_state() and only proceed once it reports PoweredOn.
        #[cfg(target_os = "macos")]
        {
            use btleplug::api::CentralState;

            let deadline = tokio::time::Instant::now() + Duration::from_secs(3);
            loop {
                match adapter.adapter_state().await {
                    Ok(CentralState::PoweredOn) => {
                        info!("macOS: adapter is PoweredOn");
                        break;
                    }
                    Ok(state) => {
                        if tokio::time::Instant::now() >= deadline {
                            warn!("macOS: adapter still in state {state:?} after 3 s — proceeding anyway");
                            break;
                        }
                        debug!("macOS: adapter state = {state:?}, waiting…");
                    }
                    Err(e) => {
                        warn!("macOS: adapter_state() error: {e}");
                        break;
                    }
                }
                tokio::time::sleep(Duration::from_millis(200)).await;
            }
            // Extra safety margin — let the delegate settle.
            tokio::time::sleep(Duration::from_millis(300)).await;
        }

        info!(
            "scan_all: scanning for {} s …",
            self.config.scan_timeout_secs
        );
        adapter.start_scan(ScanFilter::default()).await?;
        tokio::time::sleep(Duration::from_secs(self.config.scan_timeout_secs)).await;
        adapter.stop_scan().await.ok();

        let mut found = vec![];
        for p in adapter.peripherals().await? {
            if let Ok(Some(props)) = p.properties().await {
                if let Some(name) = props.local_name {
                    if name.starts_with(&self.config.name_prefix) {
                        let id = p.id().to_string();
                        info!("scan_all: found {name}  id={id}");
                        found.push(MuseDevice { name, id, peripheral: p, adapter: adapter.clone() });
                    }
                }
            }
        }
        info!("scan_all: {} device(s) found", found.len());
        Ok(found)
    }

    // ── Public: connect_to ────────────────────────────────────────────────────

    /// Connect to a specific device returned by [`MuseClient::scan_all`], subscribe to
    /// all enabled characteristics, and start the notification dispatch task.
    ///
    /// Returns an event receiver and a [`MuseHandle`] for sending commands.
    pub async fn connect_to(
        &self,
        device: MuseDevice,
    ) -> Result<(mpsc::Receiver<MuseEvent>, MuseHandle)> {
        self.setup_peripheral(device.peripheral, device.name, device.adapter)
            .await
    }

    // ── Public: connect (convenience) ────────────────────────────────────────

    /// Scan for the first Muse device, connect, and start streaming.
    ///
    /// Equivalent to calling [`MuseClient::scan_all`] and then [`MuseClient::connect_to`]
    /// on the first result. Useful when only one headset is expected.
    pub async fn connect(&self) -> Result<(mpsc::Receiver<MuseEvent>, MuseHandle)> {
        let manager = Manager::new().await?;
        let adapters = manager.adapters().await?;
        let adapter = adapters
            .into_iter()
            .next()
            .ok_or_else(|| anyhow!("No Bluetooth adapter found"))?;

        // macOS: wait for CBCentralManager to reach poweredOn (same as scan_all)
        #[cfg(target_os = "macos")]
        {
            use btleplug::api::CentralState;

            let deadline = tokio::time::Instant::now() + Duration::from_secs(3);
            loop {
                match adapter.adapter_state().await {
                    Ok(CentralState::PoweredOn) => break,
                    Ok(_) if tokio::time::Instant::now() >= deadline => break,
                    Ok(_) => {}
                    Err(_) => break,
                }
                tokio::time::sleep(Duration::from_millis(200)).await;
            }
            tokio::time::sleep(Duration::from_millis(300)).await;
        }

        info!(
            "Scanning for Muse devices (timeout: {} s) …",
            self.config.scan_timeout_secs
        );
        adapter.start_scan(ScanFilter::default()).await?;
        let peripheral = self
            .find_first(&adapter, &self.config.name_prefix, self.config.scan_timeout_secs)
            .await?;
        adapter.stop_scan().await.ok();

        let props = peripheral.properties().await?.unwrap_or_default();
        let device_name = props.local_name.unwrap_or_else(|| "Unknown".into());
        info!("Found device: {device_name}");

        self.setup_peripheral(peripheral, device_name, adapter)
            .await
    }

    // ── Private: setup_peripheral ─────────────────────────────────────────────

    /// Connect a peripheral, subscribe to all enabled GATT characteristics,
    /// spawn the notification dispatch task, and return the event channel.
    async fn setup_peripheral(
        &self,
        peripheral: Peripheral,
        device_name: String,
        adapter: Adapter,
    ) -> Result<(mpsc::Receiver<MuseEvent>, MuseHandle)> {
        // Hard timeout on connect(): BlueZ's org.bluez.Device1.Connect can block
        // forever when the device is out of range or the stack is in a bad state.
        // Ten seconds is generous for a BLE connection that typically takes <2 s.
        tokio::time::timeout(Duration::from_secs(10), peripheral.connect())
            .await
            .map_err(|_| anyhow!("BLE connect() timed out after 10 s"))??;

        // On Linux (bluez-async / D-Bus) the BLE stack signals connection
        // completion before the remote GATT service cache is populated.
        // Calling discover_services() too quickly can return an empty set,
        // causing every find_char() call to fail with "Characteristic not found".
        // A short pause lets the kernel / BlueZ finish GATT discovery first.
        #[cfg(target_os = "linux")]
        tokio::time::sleep(Duration::from_millis(600)).await;

        tokio::time::timeout(Duration::from_secs(15), peripheral.discover_services())
            .await
            .map_err(|_| anyhow!("discover_services() timed out after 15 s"))??;
        info!("Connected and services discovered: {device_name}");

        let chars: BTreeSet<Characteristic> = peripheral.characteristics();

        let find_char = |uuid: Uuid| -> Result<Characteristic> {
            chars
                .iter()
                .find(|c| c.uuid == uuid)
                .cloned()
                .ok_or_else(|| anyhow!("Characteristic {uuid} not found"))
        };

        // ── Detect Athena firmware ─────────────────────────────────────────────
        // Athena (new Muse S) exposes a universal sensor char (0x273e0013…)
        // that carries all data in tag-based packets.  Classic Muse devices do
        // not have this characteristic.
        let is_athena = chars.iter().any(|c| c.uuid == ATHENA_SENSOR_CHARACTERISTIC);
        info!(
            "{device_name}: firmware detected as {}",
            if is_athena { "Athena" } else { "Classic" }
        );

        // ── Common control characteristic ──────────────────────────────────────
        let control_char = find_char(CONTROL_CHARACTERISTIC)?;
        peripheral.subscribe(&control_char).await?;

        // ── Event channel ─────────────────────────────────────────────────────
        let (tx, rx) = mpsc::channel::<MuseEvent>(256);
        let _ = tx.send(MuseEvent::Connected(device_name.clone())).await;

        // ── Disconnect watcher ──────────────────────────────────────────────
        // Listen on the adapter's CentralEvent stream for DeviceDisconnected.
        // This fires reliably when the BLE link drops (headset powered off,
        // out of range, etc.) — often faster than waiting for the notification
        // stream to close.
        let disconnect_tx = tx.clone();
        let peripheral_id = peripheral.id();
        tokio::spawn(async move {
            match adapter.events().await {
                Ok(mut events) => {
                    while let Some(event) = events.next().await {
                        if let CentralEvent::DeviceDisconnected(id) = event {
                            if id == peripheral_id {
                                info!("Disconnect watcher: device {id:?} disconnected.");
                                let _ = disconnect_tx.send(MuseEvent::Disconnected).await;
                                break;
                            }
                        }
                    }
                }
                Err(e) => {
                    warn!("Disconnect watcher: could not subscribe to adapter events: {e}");
                }
            }
        });

        let peripheral_clone = peripheral.clone();

        if is_athena {
            // ── Athena: subscribe to the single universal sensor characteristic ─
            let sensor_char = find_char(ATHENA_SENSOR_CHARACTERISTIC)?;
            peripheral.subscribe(&sensor_char).await?;

            tokio::spawn(async move {
                let mut notifications = match peripheral_clone.notifications().await {
                    Ok(n) => n,
                    Err(e) => {
                        warn!("Athena: could not get notifications stream: {e}");
                        return;
                    }
                };
                info!("Athena: notification stream subscribed, waiting for data…");
                let mut ctrl_acc = ControlAccumulator::new();
                let mut notif_count: u64 = 0;
                let mut sensor_count: u64 = 0;
                let mut eeg_event_count: u64 = 0;

                while let Some(notif) = notifications.next().await {
                    let data = &notif.value;
                    notif_count += 1;

                    if notif.uuid == CONTROL_CHARACTERISTIC {
                        let fragment = decode_response(data);
                        debug!("Athena control fragment: {:?}", fragment);
                        if let Some(json_str) = ctrl_acc.push(&fragment) {
                            match serde_json::from_str::<serde_json::Value>(&json_str) {
                                Ok(serde_json::Value::Object(map)) => {
                                    let _ = tx
                                        .send(MuseEvent::Control(ControlResponse {
                                            raw: json_str,
                                            fields: map,
                                        }))
                                        .await;
                                }
                                Ok(_) => {}
                                Err(e) => {
                                    warn!("Athena control JSON error: {e} | raw: {json_str}")
                                }
                            }
                        }
                        continue;
                    }

                    // All sensor data arrives on ATHENA_SENSOR_CHARACTERISTIC
                    sensor_count += 1;
                    let events = parse_athena_notification(data);
                    let n_eeg = events.iter().filter(|e| matches!(e, MuseEvent::Eeg(_))).count();
                    eeg_event_count += n_eeg as u64;

                    if sensor_count <= 3 || sensor_count % 500 == 0 {
                        info!(
                            "Athena sensor: notif #{notif_count} sensor #{sensor_count} \
                             uuid={} len={} events={} eeg={} (total eeg: {eeg_event_count})",
                            notif.uuid,
                            data.len(),
                            events.len(),
                            n_eeg,
                        );
                        if sensor_count <= 3 && !data.is_empty() {
                            // Dump ALL tags found in the packet for debugging.
                            let mut tags = Vec::new();
                            let mut ti = 9usize; // skip 9-byte header
                            while ti < data.len() {
                                tags.push(format!("0x{:02x}@{}", data[ti], ti));
                                let tag_type = data[ti] & 0x0F;
                                let payload_start = ti + 1 + 4;
                                let plen = match tag_type {
                                    0x1 | 0x2 => 28, // EEG
                                    0x3 => 7,        // DRL/REF
                                    0x4 | 0x5 => 30, // Optical
                                    0x7 => 36,       // IMU
                                    0x8 => 20,       // Battery
                                    _ => 0,          // unknown → advance 1
                                };
                                if plen > 0 && payload_start + plen <= data.len() {
                                    ti = payload_start + plen;
                                } else if plen > 0 {
                                    break; // truncated
                                } else {
                                    ti += 1;
                                }
                            }
                            debug!("Athena sensor tags: [{}]", tags.join(", "));
                            debug!(
                                "Athena sensor raw (first 64 bytes): {:02x?}",
                                &data[..data.len().min(64)]
                            );
                        }
                    }

                    for event in events {
                        let _ = tx.send(event).await;
                    }
                }

                info!("Athena notification stream ended – device disconnected.");
                let _ = tx.send(MuseEvent::Disconnected).await;
            });
        } else {
            // ── Classic Muse: separate characteristic per sensor ───────────────
            let telemetry_char = find_char(TELEMETRY_CHARACTERISTIC)?;
            peripheral.subscribe(&telemetry_char).await?;

            let accel_char = find_char(ACCELEROMETER_CHARACTERISTIC)?;
            peripheral.subscribe(&accel_char).await?;

            let gyro_char = find_char(GYROSCOPE_CHARACTERISTIC)?;
            peripheral.subscribe(&gyro_char).await?;

            let num_eeg = if self.config.enable_aux { 5 } else { 4 };
            for &eeg_uuid in &EEG_CHARACTERISTICS[..num_eeg] {
                match find_char(eeg_uuid) {
                    Ok(c) => peripheral.subscribe(&c).await?,
                    Err(e) => warn!("EEG char {eeg_uuid}: {e}"),
                }
            }

            if self.config.enable_ppg {
                for &ppg_uuid in &PPG_CHARACTERISTICS {
                    match find_char(ppg_uuid) {
                        Ok(c) => peripheral.subscribe(&c).await?,
                        Err(e) => warn!("PPG char {ppg_uuid}: {e}"),
                    }
                }
            }

            let enable_ppg = self.config.enable_ppg;
            let enable_aux = self.config.enable_aux;

            tokio::spawn(async move {
                let mut notifications = match peripheral_clone.notifications().await {
                    Ok(n) => n,
                    Err(e) => {
                        warn!("Classic: could not get notifications stream: {e}");
                        return;
                    }
                };
                info!("Classic: notification stream subscribed, waiting for data…");
                let mut notif_count: u64 = 0;

                let mut eeg_ts: Vec<TimestampTracker> =
                    (0..5).map(|_| TimestampTracker::new()).collect();
                let mut ppg_ts: Vec<TimestampTracker> =
                    (0..3).map(|_| TimestampTracker::new()).collect();
                let mut ctrl_acc = ControlAccumulator::new();

                while let Some(notif) = notifications.next().await {
                    let data = &notif.value;
                    let uuid = notif.uuid;
                    notif_count += 1;
                    if notif_count <= 5 || notif_count % 500 == 0 {
                        info!(
                            "Classic: notif #{notif_count} uuid={uuid} len={}",
                            data.len()
                        );
                    }

                    if uuid == CONTROL_CHARACTERISTIC {
                        let fragment = decode_response(data);
                        debug!("Control fragment: {:?}", fragment);
                        if let Some(json_str) = ctrl_acc.push(&fragment) {
                            match serde_json::from_str::<serde_json::Value>(&json_str) {
                                Ok(serde_json::Value::Object(map)) => {
                                    let _ = tx
                                        .send(MuseEvent::Control(ControlResponse {
                                            raw: json_str,
                                            fields: map,
                                        }))
                                        .await;
                                }
                                Ok(_) => {}
                                Err(e) => {
                                    warn!("Control JSON parse error: {e} | raw: {json_str}")
                                }
                            }
                        }
                        continue;
                    }

                    if uuid == TELEMETRY_CHARACTERISTIC {
                        if let Some(t) = parse_telemetry(data) {
                            let _ = tx.send(MuseEvent::Telemetry(t)).await;
                        }
                        continue;
                    }

                    if uuid == ACCELEROMETER_CHARACTERISTIC {
                        if let Some(a) = parse_accelerometer(data) {
                            let _ = tx.send(MuseEvent::Accelerometer(a)).await;
                        }
                        continue;
                    }

                    if uuid == GYROSCOPE_CHARACTERISTIC {
                        if let Some(g) = parse_gyroscope(data) {
                            let _ = tx.send(MuseEvent::Gyroscope(g)).await;
                        }
                        continue;
                    }

                    let num_eeg_chars = if enable_aux { 5 } else { 4 };
                    if let Some(electrode) = EEG_CHARACTERISTICS[..num_eeg_chars]
                        .iter()
                        .position(|&u| u == uuid)
                    {
                        if data.len() >= 2 {
                            let index = u16::from_be_bytes([data[0], data[1]]);
                            let timestamp = eeg_ts[electrode].get(
                                index,
                                EEG_SAMPLES_PER_READING,
                                EEG_FREQUENCY,
                            );
                            let samples = decode_eeg_samples(&data[2..]);
                            let _ = tx
                                .send(MuseEvent::Eeg(EegReading {
                                    index,
                                    electrode,
                                    timestamp,
                                    samples,
                                }))
                                .await;
                        }
                        continue;
                    }

                    if enable_ppg {
                        if let Some(ppg_channel) =
                            PPG_CHARACTERISTICS.iter().position(|&u| u == uuid)
                        {
                            if data.len() >= 2 {
                                let index = u16::from_be_bytes([data[0], data[1]]);
                                let timestamp = ppg_ts[ppg_channel].get(
                                    index,
                                    PPG_SAMPLES_PER_READING,
                                    PPG_FREQUENCY,
                                );
                                if let Some(reading) =
                                    parse_ppg_reading(data, ppg_channel, timestamp)
                                {
                                    let _ = tx.send(MuseEvent::Ppg(reading)).await;
                                }
                            }
                            continue;
                        }
                    }

                    debug!("Unknown notification from {uuid}");
                }

                info!("Classic notification stream ended – device disconnected.");
                let _ = tx.send(MuseEvent::Disconnected).await;
                for t in &mut eeg_ts {
                    t.reset();
                }
                for t in &mut ppg_ts {
                    t.reset();
                }
            });
        }

        let handle = MuseHandle {
            peripheral,
            control_char,
            is_athena,
        };

        Ok((rx, handle))
    }

    // ── Private: find_first ───────────────────────────────────────────────────

    /// Poll until the first matching peripheral appears or the timeout expires.
    async fn find_first(
        &self,
        adapter: &btleplug::platform::Adapter,
        prefix: &str,
        timeout_secs: u64,
    ) -> Result<Peripheral> {
        use tokio::time::{sleep, timeout};

        let result = timeout(Duration::from_secs(timeout_secs), async {
            loop {
                let peripherals = adapter.peripherals().await.unwrap_or_default();
                for p in peripherals {
                    if let Ok(Some(props)) = p.properties().await {
                        if let Some(name) = &props.local_name {
                            if name.starts_with(prefix) {
                                return p;
                            }
                        }
                    }
                }
                sleep(Duration::from_millis(250)).await;
            }
        })
        .await;

        result.map_err(|_| anyhow!("Timed out scanning for a Muse device after {timeout_secs} s"))
    }
}

// ── MuseHandle ────────────────────────────────────────────────────────────────

/// A handle to an active Muse connection that lets you send control commands.
pub struct MuseHandle {
    peripheral: Peripheral,
    control_char: Characteristic,
    /// `true` when the connected device runs the Athena firmware (new Muse S).
    pub is_athena: bool,
}

impl MuseHandle {
    /// Send a raw command string (e.g. `"h"`, `"d"`, `"p21"`).
    pub async fn send_command(&self, cmd: &str) -> Result<()> {
        let payload = encode_command(cmd);
        self.peripheral
            .write(&self.control_char, &payload, WriteType::WithoutResponse)
            .await?;
        Ok(())
    }

    /// Pause data streaming.
    ///
    /// Sends `h` to both Classic and Athena firmware.
    pub async fn pause(&self) -> Result<()> {
        self.send_command("h").await
    }

    /// Resume data streaming after a pause.
    ///
    /// Sends both `dc001` (Athena) and `d` (Classic) when connected to an
    /// Athena device, because some Athena firmware versions (e.g. 3.x) only
    /// accept the Classic `d` command.  On Classic devices only `d` is sent.
    pub async fn resume(&self) -> Result<()> {
        if self.is_athena {
            // Try both: dc001 for newer Athena fw, d for older/transitional.
            self.send_command("dc001").await?;
            self.send_command("d").await
        } else {
            self.send_command("d").await
        }
    }

    /// Initialise the headset and begin streaming sensor data.
    ///
    /// # Classic startup sequence
    ///
    /// `h` → `s` → *preset* → `d`
    ///
    /// | `enable_ppg` | `enable_aux` | Preset |
    /// |---|---|---|
    /// | false | false | `p21` (EEG only) |
    /// | false | true  | `p20` (EEG + AUX) |
    /// | true  | —     | `p50` (EEG + PPG) |
    ///
    /// # Athena startup sequence
    ///
    /// `v4` → `s` → `h` → `p1045` → `dc001` × 2 → `L1` → **2 s wait**
    ///
    /// The `enable_ppg` and `enable_aux` flags are ignored for Athena; the
    /// firmware always uses preset `p1045`.  The 2-second wait at the end is
    /// required by the Athena firmware before packets actually start flowing.
    pub async fn start(&self, enable_ppg: bool, enable_aux: bool) -> Result<()> {
        if self.is_athena {
            // Athena startup — mirrors MuseAthenaClient.start() in muse-jsx.
            //
            // Some Athena firmware versions (e.g. Muse S fw 3.x on
            // Athena_RevE hardware) reject `dc001` (rc:69) but accept the
            // Classic `d` command.  We send both so that streaming starts
            // regardless of firmware version.  The device silently ignores
            // whichever command it does not understand.
            let delay = |ms| tokio::time::sleep(Duration::from_millis(ms));
            self.send_command("v4").await?;
            delay(100).await;
            self.send_command("s").await?;
            delay(100).await;
            self.send_command("h").await?;
            delay(100).await;
            self.send_command("p1045").await?;
            delay(100).await;
            // Athena data-start: send dc001 twice (per TypeScript reference),
            // then also send Classic `d` as fallback for fw 3.x which rejects
            // dc001 with rc:69.
            self.send_command("dc001").await?;
            delay(50).await;
            self.send_command("dc001").await?;
            delay(50).await;
            self.send_command("d").await?;
            delay(100).await;
            self.send_command("L1").await?;
            // Firmware needs ~2 s to settle before packets arrive.
            delay(2100).await;
            Ok(())
        } else {
            // Classic startup
            self.pause().await?;
            let preset = if enable_ppg {
                "p50"
            } else if enable_aux {
                "p20"
            } else {
                "p21"
            };
            self.send_command("s").await?;
            self.send_command(preset).await?;
            self.resume().await?;
            Ok(())
        }
    }

    /// Request firmware / hardware info (`v1` command).
    pub async fn request_device_info(&self) -> Result<()> {
        self.send_command("v1").await
    }

    /// Check if the peripheral is still connected at the BLE adapter level.
    /// Useful for implementing a connection watchdog — poll this periodically
    /// to detect disconnects faster than waiting for the notification stream
    /// to close.
    pub async fn is_connected(&self) -> bool {
        self.peripheral.is_connected().await.unwrap_or(false)
    }

    /// Gracefully disconnect.
    pub async fn disconnect(&self) -> Result<()> {
        self.peripheral.disconnect().await?;
        Ok(())
    }
}
