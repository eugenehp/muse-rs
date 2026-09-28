//! Command-line client for Muse EEG headsets.
//!
//! Three things a person wants from a tool like this, in order: what is in
//! range, what does it stream, and give me the data. They are `scan`, `info`
//! and `stream`.
//!
//! `stream` summarises by default rather than printing every event. A Muse S
//! on Athena firmware produces about 1,400 events a second across 24 channels,
//! which as a line each is not output, it is a denial of service — the useful
//! question at that rate is "what is arriving and how fast", which is what the
//! summary answers. `--raw` restores the per-event dump for piping into
//! something that parses it.

use std::collections::BTreeMap;
use std::io::{self, BufRead};
use std::time::{Duration, Instant};

use anyhow::{anyhow, Result};
use muse_rs::muse_client::{MuseClient, MuseClientConfig, MuseDevice, MuseHandle};
use muse_rs::protocol::{eeg_channel_name, ppg_channel_name};
use muse_rs::types::MuseEvent;
use tokio::sync::mpsc::Receiver;

// ── Arguments ─────────────────────────────────────────────────────────────────

/// What the user asked for.
struct Args {
    command: Command,
    device: Option<String>,
    prefix: String,
    timeout: u64,
    enable_ppg: bool,
    enable_aux: bool,
    preset: Option<String>,
    raw: bool,
}

#[derive(PartialEq)]
enum Command {
    Scan,
    Info,
    Stream,
}

const HELP: &str = "\
muse-rs — stream from a Muse EEG headset

USAGE
    muse-rs [COMMAND] [OPTIONS]

COMMANDS
    scan            list the Muse devices in range, then exit
    info            connect, report what the device streams, then exit
    stream          connect and stream (the default)

OPTIONS
    --device <NAME|ID>   connect to this one, by name or id (default: the first found)
    --prefix <PREFIX>    only consider names starting with this (default: Muse)
    --timeout <SECS>     how long to scan for (default: 15)
    --no-ppg             turn optical off — on Athena, the narrowest optical mode
    --aux                Classic only: add the AUX electrode
    --preset <PRESET>    send this preset after startup, e.g. p1044
    --raw                print every event instead of a summary
    -h, --help           this

WHILE STREAMING  (type and press enter)
    q                quit          p   pause          r   resume
    i                device info   <anything else>    sent as a raw command

EXAMPLES
    muse-rs scan
    muse-rs info                        what does my headset actually send?
    muse-rs stream --no-ppg
    muse-rs --raw > session.log         the full firehose, for a parser
    muse-rs --preset p1045              ask for a different mode
";

fn parse_args() -> Result<Args> {
    let mut args = Args {
        command: Command::Stream,
        device: None,
        prefix: "Muse".into(),
        timeout: 15,
        enable_ppg: true,
        enable_aux: false,
        preset: None,
        raw: false,
    };

    let mut argv = std::env::args().skip(1).peekable();
    // A bare first word that is not a flag is the command.
    if let Some(first) = argv.peek() {
        match first.as_str() {
            "scan" => {
                args.command = Command::Scan;
                argv.next();
            }
            "info" => {
                args.command = Command::Info;
                argv.next();
            }
            "stream" => {
                args.command = Command::Stream;
                argv.next();
            }
            _ => {}
        }
    }

    while let Some(arg) = argv.next() {
        let mut value = || {
            argv.next()
                .ok_or_else(|| anyhow!("{arg} needs a value — see --help"))
        };
        match arg.as_str() {
            "-h" | "--help" => {
                print!("{HELP}");
                std::process::exit(0);
            }
            "--device" => args.device = Some(value()?),
            "--prefix" => args.prefix = value()?,
            "--timeout" => {
                let given = value()?;
                args.timeout = given
                    .parse()
                    .map_err(|_| anyhow!("--timeout wants a number of seconds, got {given:?}"))?;
            }
            "--preset" => args.preset = Some(value()?),
            "--no-ppg" => args.enable_ppg = false,
            "--aux" => args.enable_aux = true,
            "--raw" => args.raw = true,
            other => {
                return Err(anyhow!("unknown argument {other:?} — see --help"));
            }
        }
    }
    Ok(args)
}

// ── What arrived ──────────────────────────────────────────────────────────────

/// Athena's range limit: 14 bits centred on 8192, scaled by 0.0885 µV/LSB,
/// is ±724.99 µV. Classic's is 12 bits at 0.48828125, or ±1000 µV. A sample at
/// the limit is the converter clipping rather than a voltage.
const RAIL_ATHENA: f64 = 724.0;
const RAIL_CLASSIC: f64 = 999.0;

/// A tally of everything seen, which is what both `info` and the streaming
/// summary are built from: the channels that appeared, how much each carried,
/// and the most recent value.
/// What one electrode delivered.
#[derive(Default, Clone, Copy)]
struct Electrode {
    samples: u64,
    /// Samples sitting at the decoder's range limit. An amplifier against the
    /// stop is not measuring anything — which is what an electrode with no
    /// skin contact does, and the most common reason a Muse "records nothing".
    railed: u64,
    last: f64,
}

/// What one optical channel delivered.
#[derive(Default, Clone, Copy)]
struct Optical {
    samples: u64,
    last: u32,
}

#[derive(Default)]
struct Seen {
    eeg: BTreeMap<usize, Electrode>,
    ppg: BTreeMap<usize, Optical>,
    accel: u64,
    gyro: u64,
    telemetry: u64,
    battery: Option<f64>,
    firmware: Option<serde_json::Map<String, serde_json::Value>>,
    is_athena: bool,
    name: Option<String>,
}

impl Seen {
    fn record(&mut self, event: &MuseEvent) {
        match event {
            MuseEvent::Eeg(r) => {
                let rail = if self.is_athena {
                    RAIL_ATHENA
                } else {
                    RAIL_CLASSIC
                };
                let e = self.eeg.entry(r.electrode).or_default();
                e.samples += r.samples.len() as u64;
                e.railed += r.samples.iter().filter(|v| v.abs() >= rail).count() as u64;
                if let Some(last) = r.samples.last() {
                    e.last = *last;
                }
            }
            MuseEvent::Ppg(r) => {
                let e = self.ppg.entry(r.ppg_channel).or_default();
                e.samples += r.samples.len() as u64;
                if let Some(last) = r.samples.last() {
                    e.last = *last;
                }
            }
            // Samples rather than packets: each IMU packet carries three.
            MuseEvent::Accelerometer(a) => self.accel += a.samples.len() as u64,
            MuseEvent::Gyroscope(g) => self.gyro += g.samples.len() as u64,
            MuseEvent::Connected(name) => self.name = Some(name.clone()),
            MuseEvent::Telemetry(t) => {
                self.telemetry += 1;
                self.battery = Some(t.battery_level as f64);
            }
            MuseEvent::Control(c) => {
                if c.fields.contains_key("fw") {
                    self.firmware = Some(c.fields.clone());
                }
                if let Some(bp) = c.fields.get("bp").and_then(|v| v.as_f64()) {
                    self.battery = Some(bp);
                }
            }
            _ => {}
        }
    }

    /// Per-channel sample rate, as a range when the channels disagree.
    ///
    /// Averaging across channels hides the case that matters: two EEG tags can
    /// arrive in the same stream, one carrying four channels and one carrying
    /// eight, which leaves the low channels sampled harder than the high ones.
    /// An average reports a rate that no channel actually has.
    fn rate(per_channel: impl Iterator<Item = u64>, elapsed: f64) -> String {
        if elapsed <= 0.0 {
            return "—".into();
        }
        let rates: Vec<f64> = per_channel.map(|n| n as f64 / elapsed).collect();
        let (Some(low), Some(high)) = (
            rates.iter().cloned().reduce(f64::min),
            rates.iter().cloned().reduce(f64::max),
        ) else {
            return "—".into();
        };
        if high - low > high * 0.1 {
            format!("{low:.0}-{high:.0} Hz")
        } else {
            format!("{high:.0} Hz")
        }
    }

    /// The inventory `info` prints: what streams, how fast, and named.
    fn report(&self, elapsed: f64) {
        println!("\n  observed over {elapsed:.1}s");
        if self.eeg.is_empty() {
            println!("    EEG         nothing");
        } else {
            println!(
                "    EEG        {:>2} ch @ {:>9}   {}",
                self.eeg.len(),
                Self::rate(self.eeg.values().map(|e| e.samples), elapsed),
                self.eeg
                    .keys()
                    .map(|c| eeg_channel_name(*c, self.is_athena))
                    .collect::<Vec<_>>()
                    .join(" ")
            );
        }
        if self.ppg.is_empty() {
            println!("    optical     nothing");
        } else {
            println!(
                "    optical    {:>2} ch @ {:>9}   {}",
                self.ppg.len(),
                Self::rate(self.ppg.values().map(|e| e.samples), elapsed),
                self.ppg
                    .keys()
                    .map(|c| ppg_channel_name(*c))
                    .collect::<Vec<_>>()
                    .join(" ")
            );
        }
        println!(
            "    IMU         accel {:.0} Hz · gyro {:.0} Hz",
            self.accel as f64 / elapsed,
            self.gyro as f64 / elapsed
        );
        println!("    telemetry   {:.1} Hz", self.telemetry as f64 / elapsed);

        if !self.eeg.is_empty() {
            let contact: String = self
                .eeg
                .iter()
                .map(|(c, e)| {
                    let pct = e.railed as f64 / e.samples.max(1) as f64 * 100.0;
                    format!(
                        "{} {} {:>3.0}%",
                        eeg_channel_name(*c, self.is_athena),
                        if pct > 50.0 { "✗" } else { "✓" },
                        pct
                    )
                })
                .collect::<Vec<_>>()
                .join("   ");
            println!("\n  contact     {contact}");
            println!(
                "              ✗ = samples pinned at the ±{:.0} µV rail, which is the\n                               converter clipping rather than a voltage — the electrode is not\n                               touching skin. All ✗ is the normal reading for a headset on a desk.",
                if self.is_athena { RAIL_ATHENA } else { RAIL_CLASSIC }
            );
        }
    }

    /// The counters as they stand, for the next tick to subtract from.
    fn snapshot(&self) -> Seen {
        Seen {
            eeg: self.eeg.clone(),
            ppg: self.ppg.clone(),
            accel: self.accel,
            gyro: self.gyro,
            telemetry: self.telemetry,
            ..Default::default()
        }
    }

    /// Samples per channel since `previous`, which is what a rate should be
    /// measured over.
    fn delta<'a, V: Copy>(
        now: &'a BTreeMap<usize, V>,
        before: &'a BTreeMap<usize, V>,
        samples: fn(&V) -> u64,
    ) -> impl Iterator<Item = u64> + 'a {
        now.iter()
            .map(move |(ch, e)| samples(e) - before.get(ch).map(samples).unwrap_or(0))
    }

    /// One line, for the streaming summary.
    fn line_since(&self, previous: &Seen, elapsed: f64, interval: f64) -> String {
        let electrodes: String = self
            .eeg
            .iter()
            .take(4)
            .map(|(c, e)| format!("{}={:+.0}", eeg_channel_name(*c, self.is_athena), e.last))
            .collect::<Vec<_>>()
            .join(" ");
        // Which electrodes spent this interval against the stop, rather than
        // measuring. Named, because "which one is loose" is the question.
        let railing: Vec<&str> = self
            .eeg
            .iter()
            .filter(|(ch, e)| {
                let before = previous.eeg.get(ch).copied().unwrap_or_default();
                let samples = e.samples - before.samples;
                let railed = e.railed - before.railed;
                samples > 0 && railed as f64 / samples as f64 > 0.5
            })
            .map(|(ch, _)| eeg_channel_name(*ch, self.is_athena))
            .collect();
        let contact = if railing.is_empty() {
            String::new()
        } else if railing.len() == self.eeg.len() {
            "  ⚠ all electrodes railing".to_string()
        } else {
            format!("  ⚠ railing: {}", railing.join(" "))
        };

        format!(
            "t={:>4.0}s  eeg {:>2}ch {:>9}  opt {:>2}ch {:>9}  imu {:>3.0}Hz  batt {:>4}  │ {electrodes} µV{contact}",
            elapsed,
            self.eeg.len(),
            Self::rate(Self::delta(&self.eeg, &previous.eeg, |e| e.samples), interval),
            self.ppg.len(),
            Self::rate(Self::delta(&self.ppg, &previous.ppg, |e| e.samples), interval),
            (self.accel - previous.accel) as f64 / interval,
            self.battery
                .map(|b| format!("{b:.0}%"))
                .unwrap_or_else(|| "—".into()),
        )
    }
}

// ── Connecting ────────────────────────────────────────────────────────────────

fn config(args: &Args) -> MuseClientConfig {
    MuseClientConfig {
        enable_aux: args.enable_aux,
        enable_ppg: args.enable_ppg,
        scan_timeout_secs: args.timeout,
        name_prefix: args.prefix.clone(),
    }
}

/// Find the device the arguments describe, or the first one going.
async fn find(args: &Args) -> Result<Option<MuseDevice>> {
    let Some(wanted) = &args.device else {
        return Ok(None);
    };
    let devices = MuseClient::new(config(args)).scan_all().await?;
    let wanted_lower = wanted.to_lowercase();
    devices
        .into_iter()
        .find(|d| d.id == *wanted || d.name.to_lowercase().contains(&wanted_lower))
        .map(Some)
        .ok_or_else(|| anyhow!("no device matching {wanted:?} — try `muse-rs scan`"))
}

async fn connect(args: &Args) -> Result<(Receiver<MuseEvent>, MuseHandle, String)> {
    let client = MuseClient::new(config(args));
    let (rx, handle) = match find(args).await? {
        Some(device) => {
            println!("connecting to {} ({})", device.name, device.id);
            let name = device.name.clone();
            let (rx, handle) = client.connect_to(device).await?;
            return Ok((rx, handle, name));
        }
        None => {
            println!("scanning for a device named {:?} …", args.prefix);
            client.connect().await?
        }
    };
    Ok((rx, handle, "the headset".into()))
}

/// Start streaming, honouring `--preset` if one was asked for.
async fn start(handle: &MuseHandle, args: &Args) -> Result<()> {
    handle.start(args.enable_ppg, args.enable_aux).await?;
    if let Some(preset) = &args.preset {
        println!("switching to preset {preset}");
        handle.pause().await?;
        handle.send_command(preset).await?;
        handle.resume().await?;
        // The firmware needs a moment before packets resume.
        tokio::time::sleep(Duration::from_millis(800)).await;
    }
    Ok(())
}

// ── Commands ──────────────────────────────────────────────────────────────────

async fn cmd_scan(args: &Args) -> Result<()> {
    println!(
        "scanning {}s for names starting {:?} …",
        args.timeout, args.prefix
    );
    let devices = MuseClient::new(config(args)).scan_all().await?;
    if devices.is_empty() {
        println!("\nnothing found.");
        println!("  · is it on, and not already connected to something else?");
        println!("  · a Muse sleeps after a few minutes idle — press its button to wake it");
        println!("  · --prefix matches the advertised name from the start, so a typo");
        println!("    finds nothing; the default \"Muse\" matches every model");
        return Ok(());
    }
    println!();
    for device in &devices {
        println!("  {:<16} {}", device.name, device.id);
    }
    println!(
        "\n{} device(s). Connect to one with: muse-rs stream --device {}",
        devices.len(),
        devices[0].name
    );
    Ok(())
}

async fn cmd_info(args: &Args) -> Result<()> {
    let (mut rx, handle, name) = connect(args).await?;
    start(&handle, args).await?;
    handle.request_device_info().await?;

    // Drain first. Packets queue while Athena's startup sequence runs — it
    // ends in a two-second wait — and counting that backlog against a short
    // window reports a rate no channel has: EEG came out at 400 Hz against a
    // true 256, and only converged when the window grew long enough to drown
    // the backlog out.
    let mut seen = Seen {
        is_athena: handle.is_athena,
        ..Default::default()
    };
    let mut live = true;

    // The settling phase still *records* — the device name, firmware and
    // battery arrive here and are wanted — but its counters are thrown away
    // before the rates are measured.
    let settle = Duration::from_secs(2);
    let settling = Instant::now();
    while settling.elapsed() < settle {
        match tokio::time::timeout(settle - settling.elapsed(), rx.recv()).await {
            Ok(Some(event)) => seen.record(&event),
            Ok(None) => {
                live = false;
                break;
            }
            Err(_) => break,
        }
    }
    seen.eeg.clear();
    seen.ppg.clear();
    seen.accel = 0;
    seen.gyro = 0;
    seen.telemetry = 0;

    let window = Duration::from_secs(4);
    let began = Instant::now();
    while live && began.elapsed() < window {
        match tokio::time::timeout(window - began.elapsed(), rx.recv()).await {
            Ok(Some(event)) => seen.record(&event),
            Ok(None) => {
                live = false;
                break;
            }
            Err(_) => break,
        }
    }

    println!("\n{}", seen.name.as_deref().unwrap_or(&name));
    if !live {
        println!("  ⚠ the link dropped before the measurement finished —");
        println!("    the rates below are short, or missing entirely");
    }
    println!(
        "  firmware    {}",
        if handle.is_athena {
            "Athena"
        } else {
            "Classic"
        }
    );
    if let Some(fw) = &seen.firmware {
        let field = |k: &str| {
            fw.get(k)
                .map(|v| v.to_string().trim_matches('"').to_string())
                .unwrap_or_else(|| "?".into())
        };
        println!(
            "  version     fw {} · hw {} · {}",
            field("fw"),
            field("hw"),
            field("sp")
        );
    }
    if let Some(battery) = seen.battery {
        println!("  battery     {battery:.1} %");
    }
    println!(
        "  preset      {}",
        args.preset.clone().unwrap_or_else(|| if handle.is_athena {
            if args.enable_ppg { "p1041" } else { "p1045" }.into()
        } else if args.enable_ppg {
            "p50".into()
        } else if args.enable_aux {
            "p20".into()
        } else {
            "p21".into()
        })
    );
    seen.report(began.elapsed().as_secs_f64());
    println!("\n  stream it with: muse-rs stream");

    handle.disconnect().await.ok();
    Ok(())
}

async fn cmd_stream(args: &Args) -> Result<()> {
    let (mut rx, handle, name) = connect(args).await?;
    let handle = std::sync::Arc::new(handle);
    start(&handle, args).await?;

    println!(
        "streaming from {name} — q to quit, p pause, r resume, i info{}",
        if args.raw { " (raw)" } else { "" }
    );
    let _ = name;

    // stdin is read on its own thread: it blocks, and the event loop must not.
    let commands = handle.clone();
    tokio::spawn(async move {
        let (tx, mut lines) = tokio::sync::mpsc::channel::<String>(8);
        std::thread::spawn(move || {
            for line in io::stdin().lock().lines().map_while(|l| l.ok()) {
                if tx.blocking_send(line).is_err() {
                    break;
                }
            }
        });
        while let Some(line) = lines.recv().await {
            let result = match line.trim() {
                "" => continue,
                "q" => {
                    let _ = commands.disconnect().await;
                    std::process::exit(0);
                }
                "p" => commands.pause().await,
                "r" => commands.resume().await,
                "i" => commands.request_device_info().await,
                other => commands.send_command(other).await,
            };
            if let Err(e) = result {
                eprintln!("command failed: {e}");
            }
        }
    });

    let mut seen = Seen {
        is_athena: handle.is_athena,
        ..Default::default()
    };
    let began = Instant::now();
    // Rates are per tick rather than cumulative: a running average from t=0
    // carries the startup backlog for a minute, and never shows the *current*
    // rate — which is the number that answers "is it still streaming" and
    // that reacts when you pause it.
    let mut previous = Seen::default();
    let mut last_tick = Instant::now();
    let mut first_tick = true;
    let mut ticker = tokio::time::interval(Duration::from_secs(1));
    ticker.tick().await; // the first tick is immediate

    loop {
        tokio::select! {
            event = rx.recv() => {
                let Some(event) = event else { break };
                seen.record(&event);
                match &event {
                    // The device's own name only arrives here, after the
                    // banner has already been printed.
                    MuseEvent::Connected(name) if !args.raw => println!("connected to {name}"),
                    MuseEvent::Disconnected if !args.raw => {
                        println!("\ndisconnected");
                        break;
                    }
                    _ => {}
                }
                if args.raw {
                    print_raw(&event, handle.is_athena);
                }
            }
            _ = ticker.tick(), if !args.raw => {
                let interval = last_tick.elapsed().as_secs_f64();
                // The first tick covers whatever queued during startup — real
                // events, but not a measure of the current rate. Counted, not
                // printed.
                if first_tick {
                    first_tick = false;
                } else {
                    println!("{}", seen.line_since(&previous, began.elapsed().as_secs_f64(), interval));
                }
                previous = seen.snapshot();
                last_tick = Instant::now();
            }
        }
    }
    Ok(())
}

/// The per-event dump, for `--raw`.
fn print_raw(event: &MuseEvent, is_athena: bool) {
    match event {
        MuseEvent::Connected(name) => println!("[CONNECTED] {name}"),
        MuseEvent::Disconnected => println!("[DISCONNECTED]"),
        MuseEvent::Eeg(r) => println!(
            "[EEG] ch={:<6} idx={:5} ts={:.0} sample[0]={:+8.3} µV",
            eeg_channel_name(r.electrode, is_athena),
            r.index,
            r.timestamp,
            r.samples.first().copied().unwrap_or(f64::NAN)
        ),
        MuseEvent::Ppg(r) => println!(
            "[PPG] ch={:<9} idx={:5} ts={:.0} sample[0]={}",
            ppg_channel_name(r.ppg_channel),
            r.index,
            r.timestamp,
            r.samples.first().copied().unwrap_or(0)
        ),
        MuseEvent::Telemetry(t) => println!(
            "[TELEMETRY] seq={:5} battery={:.1}% fuel_gauge={:.1} mV temp={}",
            t.sequence_id, t.battery_level, t.fuel_gauge_voltage, t.temperature
        ),
        MuseEvent::Accelerometer(a) => {
            let s = &a.samples[0];
            println!(
                "[ACCEL] seq={:5} x={:+.5}g y={:+.5}g z={:+.5}g",
                a.sequence_id, s.x, s.y, s.z
            );
        }
        MuseEvent::Gyroscope(g) => {
            let s = &g.samples[0];
            println!(
                "[GYRO]  seq={:5} x={:+.5}°/s y={:+.5}°/s z={:+.5}°/s",
                g.sequence_id, s.x, s.y, s.z
            );
        }
        MuseEvent::Control(c) => {
            let kind = if c.fields.contains_key("fw") {
                "DEVICE INFO"
            } else {
                "CONTROL"
            };
            println!("[{kind}] {}", c.raw);
        }
    }
}

/// Called from `main.rs`, which is the binary's entry point; this is the body.
#[tokio::main]
pub async fn run() -> Result<()> {
    // Quiet by default: the library's info logs are per-packet and this
    // program prints its own status. `RUST_LOG=muse_rs=info` brings them back.
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn")).init();

    let args = parse_args()?;
    match args.command {
        Command::Scan => cmd_scan(&args).await,
        Command::Info => cmd_info(&args).await,
        Command::Stream => cmd_stream(&args).await,
    }
}
