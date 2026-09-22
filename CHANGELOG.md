# v0.2.0 — 2026-09-21

**If you decode Athena data, read "Fixed" first: this release changes what
arrives.** More optical channels, more IMU samples, and one electrode that was
labelled wrongly. The `MuseClient` / `MuseDevice` / `MuseHandle` API itself is
unchanged.

## Fixed

Every one of these was found by running against a Muse S Athena (fw 3.1.11) and
comparing what the device sent with what the library emitted. None were visible
from a green test suite — the data was arriving and being discarded at the last
step.

- **Optical channels were being thrown away.** The 4- and 8-channel optical
  handlers ended in `for ch in 0..n_ch.min(3)`: the count was decoded
  correctly, then everything past the third channel was dropped before an event
  was built. The clamp was sized to `PPG_CHANNEL_NAMES`, so a naming table was
  silently constraining the data. In 8-channel mode that discarded 5 of 8
  channels; all of them carry real signal, each with its own DC level.
- **The 16-channel optical mode was skipped entirely** (`0x36`), which made
  presets `p1041` and `p1042` look like they carried no optical data at all.
  They carry the most of any preset. Decoding is confirmed by the result's
  shape: de-interleaved this way each of the sixteen channels holds a steady
  level of its own, where a wrong stride smears them together.
- **Two thirds of the IMU data was discarded.** An Athena IMU packet is 36
  bytes — three accelerometer and gyroscope triples — and only the first was
  decoded; the other two slots were filled by *copying* it, so `samples`
  reported three readings and carried one. Measured rate goes from an apparent
  26 Hz to the documented 52 Hz, and the samples are now distinct.
- **Athena's fifth electrode was labelled wrongly.** `EEG_CHANNEL_NAMES` is the
  Classic mapping, where index 4 is AUX; on Athena it is FPz. Applied to Athena
  data it did not run out of names, it printed a wrong one. Use the new
  firmware-aware `protocol::eeg_channel_name`.

## Changed

- **Athena's startup preset is now `p1041`, not `p1045`.** Measured across every
  preset the firmware accepts, `p1045` selects the *narrowest* optical mode —
  four channels — while `p1041` carries sixteen at the same per-channel rate.
  `enable_ppg: false` now selects `p1045` rather than being ignored. Optical
  cannot be switched off on Athena; no preset has none.
- **BLE backend is now [webbluetooth](https://github.com/eugenehp/webbluetooth)**

 instead of the btleplug fork — a Rust port of the Web Bluetooth API, talking to CoreBluetooth, BlueZ, WinRT, Android and the browser through one surface.
  - The macOS "wait for `CBCentralManager` to reach poweredOn" polling loops are gone; `Bluetooth::availability()` waits for the adapter's first state report, and reports *why* it is unusable when it is not.
  - Disconnects come from `BluetoothDevice::watch_disconnect()` — per device, rather than filtering an adapter-wide event stream.
  - Notifications are subscribed per characteristic and merged, so a missing EEG or PPG characteristic no longer silently joins the same stream as everything else.
  - Access is scoped to the Muse vendor service `0000fe8d-…` by the Web Bluetooth grant model; the GATT blocklist is enforced.
  - The `[patch.crates-io]` override of `btleplug` is gone.
  - One adapter session per process, via `Bluetooth::shared()`. A device is
    only usable through the session that found it, and the TUI scans with one
    `MuseClient` and connects with another — so this is load-bearing rather
    than tidiness.
  - The `Uuid` constants in `protocol` are passed to webbluetooth unconverted,
    through its `uuid` feature. There is no conversion helper in this crate any
    more, and no `expect` for a value that cannot be wrong.

## Added

- **`muse-rs scan` and `muse-rs info`.** The CLI took no arguments at all: it
  connected to whichever device answered first and printed every event. `scan`
  lists what is in range; `info` connects and reports the firmware, battery,
  preset and — the point — the channels that actually arrived, by name and
  rate. On an Athena headset that is 8 EEG channels and 16 optical ones, which
  was previously discoverable only by reading 50,000 lines of output.
- **Arguments**: `--device`, `--prefix`, `--timeout`, `--no-ppg`, `--aux`,
  `--preset`, `--raw`, `--help`.
- **`stream` summarises by default**, once a second, with per-channel rates and
  live values. Athena sends ~1,400 events a second across 24 channels; a line
  each is not output, it is a denial of service. `--raw` restores the old dump.
- **Electrode contact.** `info` reports, per electrode, the share of samples
  pinned at the converter's range limit — which is clipping rather than a
  voltage, and is what an electrode not touching skin does. `stream` names the
  offenders live. The threshold comes from each firmware's own decode
  arithmetic: ±725 µV on Athena, ±1000 µV on Classic.
- `protocol::ATHENA_EEG_CHANNEL_NAMES`, `eeg_channel_name`, `ppg_channel_name`.

## Removed

- The `futures` dependency. It was one import — `stream::{self, BoxStream,
  StreamExt}` — and `webbluetooth::stream` re-exports all of it, so the
  notification-merging code is unchanged and the dependency is gone from
  `cargo tree` entirely.

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
