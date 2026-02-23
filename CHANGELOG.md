# v0.1.0

First feature-complete release of `muse-rs` — an async Rust library and terminal UI for streaming real-time sensor data from Interaxon Muse EEG headsets over Bluetooth Low Energy.

## Highlights

- **Full Athena firmware support** — automatic protocol detection for Muse S devices running the newer Athena firmware (tag-based multiplexed packets on a single BLE characteristic)
- **PPG (optical) decoding** — both Classic (24-bit BE) and Athena (20-bit LE packed) PPG data decoded into `MuseEvent::Ppg` events with 3 channels (ambient, infrared, red) at 64 Hz
- **Real-time TUI** with EEG and PPG views, device picker, smooth overlay, and auto-reconnect
- **Cross-platform** — Linux (BlueZ), macOS (CoreBluetooth), Windows (WinRT)

## What's included

### Library (`muse-rs`)

| Sensor | Classic | Athena |
|---|---|---|
| EEG (4ch / 8ch) | ✓ 12-bit BE, 256 Hz | ✓ 14-bit LE, 256 Hz |
| PPG / Optical | ✓ 24-bit BE, 64 Hz | ✓ 20-bit LE, 64 Hz |
| Accelerometer | ✓ | ✓ |
| Gyroscope | ✓ | ✓ |
| Battery | ✓ u16 BE / 512 | ✓ u16 LE / 256 |
| Control JSON | ✓ | ✓ (fragment reassembly) |
| Disconnect detection | ✓ | ✓ (adapter event stream) |

### TUI (`cargo run --bin tui`)

- **EEG view** — 4-channel scrolling braille waveforms (TP9, AF7, AF8, TP10) with per-channel min/max/RMS stats, clipping indicator, and configurable ±µV scale
- **PPG view** — 3-channel optical waveforms (ambient, infrared, red) with auto-scaling Y axis
- **Smooth overlay** — dim raw trace + bright 9-sample moving average (toggle with `v`)
- **Device picker** — scan, select, and switch between multiple Muse devices
- **Auto-reconnect** — automatic rescan after unexpected disconnect
- **Simulator** — `--simulate` flag for UI development without hardware

### Console streamer (`cargo run`)

- Prints all decoded events (EEG, PPG, IMU, battery, control) to stdout
- Interactive commands: pause, resume, device info, raw command passthrough

## Athena protocol support

The Athena decoder handles the full tag-based packet format documented by the [OpenMuse](https://github.com/DominiqueMakowski/OpenMuse) project:

| Tag | Sensor | Payload |
|---|---|---|
| `0x11` | EEG 4ch (4 samples) | 28 B |
| `0x12` | EEG 8ch (2 samples) | 28 B |
| `0x34` | Optical 4ch (3 samples) | 30 B |
| `0x35` | Optical 8ch (2 samples) | 40 B |
| `0x36` | Optical 16ch (1 sample) | 40 B |
| `0x47` | IMU accel+gyro (3 samples) | 36 B |
| `0x53` | DRL/REF | 24 B |
| `0x88` | Battery (new fw, variable) | 188–230 B |
| `0x98` | Battery (old fw) | 20 B |

### Transitional firmware compatibility

Muse S devices with firmware 3.x on Athena hardware expose the Athena sensor characteristic but reject the `dc001` data-start command (rc:69). The library sends both `dc001` and the Classic `d` command as fallback, ensuring streaming starts regardless of firmware version.

## macOS improvements

Uses a [fork of btleplug](https://github.com/eugenehp/btleplug/tree/imrpoved_mac_version) with:

- Reliable disconnect detection via `CentralEvent::DeviceDisconnected`
- Expanded broadcast channel buffers (16 → 256)
- Null-safety improvements preventing hangs on unreachable peripherals
- Single adapter instance reuse (avoids duplicate `CBCentralManager` creation)

## Quick start

```rust
use muse_rs::prelude::*;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let client = MuseClient::new(MuseClientConfig::default());
    let (mut rx, handle) = client.connect().await?;
    handle.start(false, false).await?;

    while let Some(event) = rx.recv().await {
        match event {
            MuseEvent::Eeg(r) => println!("EEG ch{}: {:.2} µV", r.electrode, r.samples[0]),
            MuseEvent::Ppg(r) => println!("PPG ch{}: {:?}", r.ppg_channel, r.samples),
            MuseEvent::Disconnected => break,
            _ => {}
        }
    }
    Ok(())
}
```

## Install

```shell
cargo add muse-rs
```

Or as a git dependency:

```toml
[dependencies]
muse-rs = { git = "https://github.com/eugenehp/muse-rs.git", tag = "v0.1.0" }
```

## Acknowledgments

- [OpenMuse](https://github.com/DominiqueMakowski/OpenMuse) by Dominique Makowski — Python Athena decoder used as reference for packet structure, payload sizes, and battery scaling
- [muse-js](https://github.com/urish/muse-js) by Uri Shaked — original Muse Web Bluetooth implementation
- [btleplug](https://github.com/deviceplug/btleplug) — cross-platform BLE for Rust

**Full Changelog**: https://github.com/eugenehp/muse-rs/commits/v0.1.0
