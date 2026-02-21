use std::io::{self, BufRead};

use anyhow::Result;
use log::{error, info};

use muse_rs::muse_client::{MuseClient, MuseClientConfig};
use muse_rs::protocol::{EEG_CHANNEL_NAMES, PPG_CHANNEL_NAMES};
use muse_rs::types::MuseEvent;

#[tokio::main]
async fn main() -> Result<()> {
    // ── Logging ───────────────────────────────────────────────────────────────
    // Set RUST_LOG=debug for verbose output, e.g.:
    //   RUST_LOG=muse_rs=debug cargo run
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    // ── Configuration ─────────────────────────────────────────────────────────
    // Flip these flags to enable additional sensor data.
    let config = MuseClientConfig {
        enable_aux: false,
        enable_ppg: false,
        scan_timeout_secs: 15,
        name_prefix: "Muse".into(),
    };

    let enable_ppg = config.enable_ppg;
    let enable_aux = config.enable_aux;

    // ── Connect ───────────────────────────────────────────────────────────────
    let client = MuseClient::new(config);

    info!("Connecting to Muse headset …");
    let (mut rx, handle) = client.connect().await?;

    // Wrap in Arc so it can be shared between the command task and the main loop.
    let handle = std::sync::Arc::new(handle);

    // ── Start streaming ───────────────────────────────────────────────────────
    handle.start(enable_ppg, enable_aux).await?;
    info!("Streaming started. Press Ctrl-C or type 'q' + Enter to quit.\n");
    info!("Commands (type + Enter):");
    info!("  q  – quit");
    info!("  p  – pause streaming");
    info!("  r  – resume streaming");
    info!("  i  – request device info");
    info!("  <any other string> – send as raw command\n");

    // ── Stdin command loop ────────────────────────────────────────────────────
    // We read lines on a dedicated OS thread (to avoid holding a non-Send
    // StdinLock across await points), then relay them to an async task.
    let (line_tx, mut line_rx) = tokio::sync::mpsc::unbounded_channel::<String>();

    std::thread::spawn(move || {
        let stdin = io::stdin();
        for line in stdin.lock().lines() {
            match line {
                Ok(l) => {
                    if line_tx.send(l.trim().to_owned()).is_err() {
                        break;
                    }
                }
                Err(_) => break,
            }
        }
    });

    // Process lines in an async task so we can await handle methods.
    let handle_cmd = std::sync::Arc::clone(&handle);
    tokio::spawn(async move {
        while let Some(line) = line_rx.recv().await {
            if line.is_empty() {
                continue;
            }
            match line.as_str() {
                "q" => {
                    info!("Quit requested.");
                    handle_cmd.disconnect().await.ok();
                    std::process::exit(0);
                }
                "p" => {
                    info!("Pausing …");
                    if let Err(e) = handle_cmd.pause().await {
                        error!("Pause error: {e}");
                    }
                }
                "r" => {
                    info!("Resuming …");
                    if let Err(e) = handle_cmd.resume().await {
                        error!("Resume error: {e}");
                    }
                }
                "i" => {
                    info!("Requesting device info …");
                    if let Err(e) = handle_cmd.request_device_info().await {
                        error!("Device info error: {e}");
                    }
                }
                cmd => {
                    info!("Sending command: '{cmd}'");
                    if let Err(e) = handle_cmd.send_command(cmd).await {
                        error!("Command error: {e}");
                    }
                }
            }
        }
    });

    // ── Main event loop ───────────────────────────────────────────────────────
    while let Some(event) = rx.recv().await {
        match event {
            MuseEvent::Connected(name) => {
                info!("✅  Connected to: {name}");
            }
            MuseEvent::Disconnected => {
                info!("❌  Disconnected from device.");
                break;
            }

            // ── EEG ──────────────────────────────────────────────────────────
            MuseEvent::Eeg(reading) => {
                let ch_name = EEG_CHANNEL_NAMES
                    .get(reading.electrode)
                    .copied()
                    .unwrap_or("?");
                // Show the first sample of each packet as a quick indicator.
                let first = reading.samples.first().copied().unwrap_or(f64::NAN);
                println!(
                    "[EEG] ch={ch_name:4} idx={:5}  ts={:.0} ms  sample[0]={first:+8.3} µV",
                    reading.index, reading.timestamp
                );
            }

            // ── PPG ──────────────────────────────────────────────────────────
            MuseEvent::Ppg(reading) => {
                let ch_name = PPG_CHANNEL_NAMES
                    .get(reading.ppg_channel)
                    .copied()
                    .unwrap_or("?");
                let first = reading.samples.first().copied().unwrap_or(0);
                println!(
                    "[PPG] ch={ch_name:8} idx={:5}  ts={:.0} ms  sample[0]={first}",
                    reading.index, reading.timestamp
                );
            }

            // ── Telemetry ─────────────────────────────────────────────────────
            MuseEvent::Telemetry(t) => {
                println!(
                    "[TELEMETRY] seq={:5}  battery={:.1}%  fuel_gauge={:.1} mV  temp={}",
                    t.sequence_id, t.battery_level, t.fuel_gauge_voltage, t.temperature
                );
            }

            // ── Accelerometer ─────────────────────────────────────────────────
            MuseEvent::Accelerometer(a) => {
                let s = &a.samples[0];
                println!(
                    "[ACCEL] seq={:5}  x={:+.5}g  y={:+.5}g  z={:+.5}g",
                    a.sequence_id, s.x, s.y, s.z
                );
            }

            // ── Gyroscope ─────────────────────────────────────────────────────
            MuseEvent::Gyroscope(g) => {
                let s = &g.samples[0];
                println!(
                    "[GYRO]  seq={:5}  x={:+.5}°/s  y={:+.5}°/s  z={:+.5}°/s",
                    g.sequence_id, s.x, s.y, s.z
                );
            }

            // ── Control / Device Info ─────────────────────────────────────────
            MuseEvent::Control(resp) => {
                if resp.fields.contains_key("fw") {
                    println!("[DEVICE INFO] {}", resp.raw);
                } else {
                    println!("[CONTROL] {}", resp.raw);
                }
            }
        }
    }

    info!("Event loop finished – exiting.");
    Ok(())
}
