# muse-rs

A Rust library and terminal UI for streaming real-time sensor data from
**Interaxon Muse EEG headsets** over Bluetooth Low Energy.

Supports every Muse model — including the newer Muse S running the **Athena**
firmware — and automatically selects the correct protocol at connection time.

![demo](/docs/tui.png)

## Installation

```shell
cargo add muse-rs
```

---

## Supported hardware

| Model | Firmware | EEG ch | PPG | AUX | Detection |
|---|---|---|---|---|---|
| Muse 1 (2014) | Classic | 4 | ✗ | ✗ | default |
| Muse 2 | Classic | 4 | ✓ | ✓ | default |
| Muse S | Classic | 4 | ✓ | ✓ | default |
| Muse S | **Athena** | 8 | ✓\* | ✗ | auto-detected |

\* Athena PPG data is received but not yet decoded into `MuseEvent::Ppg`.

---

## Firmware variants: Classic vs. Athena

Interaxon ships two completely different BLE protocols depending on the device
and firmware version.  This library detects which one is in use at connect time
and switches behaviour automatically — no configuration is required.

### Detection

At connect time, the library inspects the GATT service table.  If the
**universal sensor characteristic** (`273e0013-…`) is present, the device is
running Athena firmware.  Classic devices do not expose this characteristic.

### Classic firmware (Muse 1, Muse 2, Muse S ≤ fw 3.x)

One dedicated GATT characteristic per sensor:

| Sensor | Characteristic | Rate | Format |
|---|---|---|---|
| EEG TP9 / AF7 / AF8 / TP10 | `273e0003–0006` | 256 Hz | 12-bit BE packed, 12 samples/pkt |
| EEG AUX (optional) | `273e0007` | 256 Hz | same |
| Accelerometer | `273e000a` | ~52 Hz | 3 × i16 BE XYZ samples/pkt |
| Gyroscope | `273e0009` | ~52 Hz | 3 × i16 BE XYZ samples/pkt |
| PPG ambient / IR / red | `273e000f–0011` | 64 Hz | 6 × u24 BE samples/pkt |
| Telemetry | `273e000b` | ~1 Hz | 5 × u16 BE fields |
| Control | `273e0001` | cmd/resp | length-prefixed ASCII + JSON |

**EEG scale:** `µV = 0.48828125 × (raw₁₂ − 2048)`

**Startup sequence:** `h` → `s` → preset (`p21` / `p20` / `p50`) → `d`

**Resume command:** `d`

### Athena firmware (Muse S fw ≥ 4.x)

All sensor data is **multiplexed onto one characteristic** using a tag-based
binary framing:

| Sensor | Tag(s) | Payload | Rate | Format |
|---|---|---|---|---|
| EEG (8 ch) | `0x11`, `0x12` | 28 B | 256 Hz | 14-bit LE packed, 2 samples/ch/pkt |
| IMU (accel + gyro) | `0x47` | 36 B | ~52 Hz | 3 × (i16 LE accel + i16 LE gyro) |
| Optical / PPG | `0x34`, `0x35` | 30 B | 64 Hz | not yet decoded |
| Battery | `0x88`, `0x98` | 20 B | ~1 Hz | u16 LE fuel-gauge |
| Control | `273e0001` | — | cmd/resp | same as Classic |

**EEG scale:** `µV = (raw₁₄ − 8192) × 0.0885`

**EEG channels (index order):** TP9, AF7, AF8, TP10, FPz, AUX\_R, AUX\_L, AUX
(the first 4 are the standard electrode positions; indices 4–7 are extended
channels only available on Athena hardware).

**IMU:** accelerometer uses the same scale as Classic (0.0000610352 g/LSB);
gyroscope scale is negated (−0.0074768 °/s/LSB vs. +0.0074768 for Classic).

**Startup sequence:** `v4` → `s` → `h` → `p1045` → `dc001` × 2 → `L1` → 2 s wait
(the 2-second wait is required by the firmware before data flows).

**Resume command:** `dc001` (not `d`)

### Side-by-side comparison

| Property | Classic | Athena |
|---|---|---|
| Characteristics | one per sensor | one universal (`273e0013`) |
| EEG channels | 4 (+ optional AUX) | 8 |
| EEG bit-width | 12-bit | 14-bit |
| EEG byte order | big-endian | little-endian |
| EEG samples/pkt | 12 | 2 |
| EEG µV/LSB | 0.48828125 | 0.0885 |
| IMU byte order | big-endian | little-endian |
| Gyro sign | positive | negated |
| Resume cmd | `d` | `dc001` |
| Startup | 4 steps | 7 steps + 2 s wait |
| PPG decoded | ✓ | ✗ (received, not decoded) |

---

## Features

### Library

Use `muse-rs` as a library in your own project:

```toml
# Cargo.toml

# Full build (includes TUI feature):
muse-rs = "0.0.1"

# Library only — skips ratatui / crossterm compilation:
muse-rs = { version = "0.0.1", default-features = false }
```

```rust
use muse_rs::prelude::*;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let client = MuseClient::new(MuseClientConfig::default());

    // Scan and connect
    let devices = client.scan_all().await?;
    let (mut rx, handle) = client.connect_to(devices.into_iter().next().unwrap()).await?;

    // Start streaming (works for both Classic and Athena automatically)
    handle.start(false, false).await?;

    while let Some(event) = rx.recv().await {
        match event {
            MuseEvent::Eeg(r) => println!("ch{} sample[0]: {:.2} µV", r.electrode, r.samples[0]),
            MuseEvent::Disconnected => break,
            _ => {}
        }
    }
    Ok(())
}
```

### Sensor support per model

| `MuseEvent` variant | Muse 1 | Muse 2 | Muse S Classic | Muse S Athena |
|---|---|---|---|---|
| `Eeg` (4 ch) | ✓ | ✓ | ✓ | ✓ (ch 0–3) |
| `Eeg` (ch 4–7, Athena-only) | ✗ | ✗ | ✗ | ✓ |
| `Accelerometer` | ✓ | ✓ | ✓ | ✓ |
| `Gyroscope` | ✓ | ✓ | ✓ | ✓ |
| `Telemetry` (battery) | ✓ | ✓ | ✓ | ✓ |
| `Ppg` | ✗ | ✓\* | ✓\* | ✗† |
| `Control` | ✓ | ✓ | ✓ | ✓ |

\* Requires `enable_ppg: true` in `MuseClientConfig`.  
† Athena optical data is received but the decoder is not yet implemented.

---

## Prerequisites

| Requirement | Notes |
|---|---|
| Rust ≥ 1.75 | `rustup update stable` |
| Bluetooth adapter | Any BLE-capable adapter |
| Linux | `bluez` + `dbus` (`libdbus-1-dev`) |
| macOS | Core Bluetooth — see notes below |
| Windows | WinRT Bluetooth — works out of the box |

### Linux — install system dependencies

```bash
sudo apt-get install libdbus-1-dev pkg-config
```

### macOS — Bluetooth permissions

macOS requires every binary that uses CoreBluetooth to declare
`NSBluetoothAlwaysUsageDescription` in an embedded `Info.plist`; without it
the OS silently denies all BLE operations.

`build.rs` handles this automatically: on every `cargo build` targeting macOS
it links `Info.plist` into the `__TEXT,__info_plist` section of the Mach-O
binary via the `-sectcreate` linker flag.

**First-run flow**

1. Run `cargo run --bin tui` (or `cargo run`).
2. macOS shows a one-time system dialog:
   > _"muse-rs" would like to use Bluetooth_
3. Click **Allow**.
4. The scan runs and finds your Muse headset.

If you previously clicked **Don't Allow**, re-grant access in:

> **System Settings → Privacy & Security → Bluetooth**

Add the terminal app (Terminal.app, iTerm2, …) or the compiled binary to the
allow-list, then re-run.

> **Note**: if BLE still returns no devices after granting permission, make
> sure Bluetooth is enabled (`System Settings → Bluetooth`) and the headset
> is powered on.  Press **[s]** in the TUI to trigger a fresh scan at any time.

---

## Build

```bash
cd muse-rs
cargo build --release          # builds lib + both binaries (tui feature on by default)
cargo build --no-default-features  # builds lib + headless CLI only (no ratatui/crossterm)
```

---

## TUI — real-time waveform viewer

```bash
cargo run --bin tui                # scan → auto-connect to first found device
cargo run --bin tui -- --simulate  # built-in EEG simulator (no hardware needed)
```

### What you see

```
┌──────────────────────────────────────────────────────────────────────────────┐
│  MUSE EEG Monitor  │  ● Muse-AB12  │  Bat 85%  │  21.3 pkt/s  │  ±500 µV     │
├──────────────────────────────────────────────────────────────────────────────┤
│ TP9  min: -38.2  max: +41.5  rms: 17.8 µV                    [SMOOTH]        │
│ ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿  braille waveform, rolling 2-second window                │
├──────────────────────────────────────────────────────────────────────────────┤
│ AF7  ...                                                                     │
├──────────────────────────────────────────────────────────────────────────────┤
│ AF8  ...                                                                     │
├──────────────────────────────────────────────────────────────────────────────┤
│ TP10 ...                                                                     │
├──────────────────────────────────────────────────────────────────────────────┤
│ [Tab]Devices  [d]Disconnect  [+]Scale↑  [-]Scale↓  [a]Auto-scale  [v]Smooth  │
│ Accel x:+0.010g  y:+0.020g  z:-1.000g   Gyro x:+0.120°/s  …                  │
└──────────────────────────────────────────────────────────────────────────────┘
```

Each panel shows a rolling **2-second window** rendered with Braille markers
(~4× the resolution of block characters).  The border turns **red** when any
sample in the buffer exceeds the current Y-axis scale.

**Smooth mode** (on by default, toggle with `v`): draws the raw signal in a
dim colour as background context, then overlays a 9-sample moving-average
(≈ 35 ms at 256 Hz) in the full channel colour.

> **Scale note**: the TUI starts at **±500 µV** for real devices — wide enough
> to capture typical artefacts on first connect.  The simulator starts at
> ±50 µV to match its ≈ ±40 µV peak amplitude.  Press `a` to auto-scale to
> the current signal at any time.

### Device picker (Tab)

```
┌──── Select Device  (2 found) ────────────────────────────────────────────┐
│  ● Muse-AB12  [90ABCDEF]  ← connected                                    │
│ ▶  Muse-CD34  [12345678]                                                 │
│                                                                          │
│  [↑↓] Navigate  [↵] Connect  [s] Rescan  [Esc] Close                     │
│  Device list is refreshed after every scan                               │
└──────────────────────────────────────────────────────────────────────────┘
```

If a device disconnects unexpectedly the list is cleared immediately and a
fresh BLE scan starts after a 2-second delay (giving the headset time to resume
advertising).

### TUI key reference

| Key | Context | Action |
|---|---|---|
| `Tab` | streaming | open device picker |
| `s` | streaming / picker | rescan for Muse devices |
| `d` | streaming | disconnect and rescan |
| `+` / `=` | streaming | zoom out (increase µV scale) |
| `-` | streaming | zoom in (decrease µV scale) |
| `a` | streaming | auto-scale to current peak |
| `v` | streaming | toggle smooth overlay |
| `p` | streaming | pause streaming |
| `r` | streaming | resume streaming |
| `c` | streaming | clear all waveform buffers |
| `q` / Esc | streaming | quit |
| `↑` / `↓` | picker | navigate list |
| Enter | picker | connect to selected device |
| `s` | picker | rescan |
| Esc | picker | close picker |

### Simulator

The built-in simulator (`--simulate`) generates realistic-looking EEG without
hardware and is useful for UI development or demos:

| Component | Frequency | Amplitude |
|---|---|---|
| Alpha | 10 Hz | ±20 µV |
| Beta | 22 Hz | ±6 µV |
| Theta | 6 Hz | ±10 µV |
| Noise | broadband | ±4 µV (deterministic) |

Each channel has a different phase so the waveforms are visually distinct.
Fake accelerometer, gyroscope, and battery data are updated at ~1 Hz.

---

## Console streamer (`muse-rs` binary)

```bash
cargo run --release
```

Scans up to 15 seconds, connects to the first Muse found, and streams all
decoded events to stdout.  Works with both Classic and Athena firmware.

### Interactive commands (type + Enter)

| Command | Action |
|---|---|
| `q` | Gracefully disconnect and exit |
| `p` | Pause data streaming |
| `r` | Resume data streaming |
| `i` | Request firmware / hardware info (`v1` command) |
| anything else | Forwarded as a raw control command |

### Enable verbose logging

```bash
RUST_LOG=debug cargo run
RUST_LOG=muse_rs=debug cargo run   # library logs only
```

---

## Configuration

```rust
let config = MuseClientConfig {
    enable_aux:        false,   // subscribe to EEG AUX channel (Classic only)
    enable_ppg:        false,   // subscribe to PPG channels (Classic only)
    scan_timeout_secs: 15,      // abort scan after this many seconds
    name_prefix:       "Muse".into(), // match devices whose name starts with this
};
```

| `enable_ppg` | `enable_aux` | Preset sent (Classic) | Preset sent (Athena) |
|---|---|---|---|
| false | false | `p21` (EEG only) | `p1045` |
| false | true | `p20` (EEG + AUX) | `p1045` |
| true | — | `p50` (EEG + PPG) | `p1045` (PPG not decoded) |

> Athena always uses `p1045` regardless of PPG/AUX flags; individual sensor
> enabling is not yet supported for that firmware.

---

## Project layout

```
muse-rs/
├── Cargo.toml
└── src/
    ├── lib.rs           # Crate root: module declarations + prelude
    ├── main.rs          # Headless CLI binary (cargo run)
    ├── bin/
    │   └── tui.rs       # Full-screen TUI binary (cargo run --bin tui)
    ├── muse_client.rs   # MuseClient (scan/connect) + MuseHandle (commands)
    │                    # Firmware detection + dual protocol dispatch
    ├── protocol.rs      # GATT UUIDs, sampling constants, encode/decode helpers
    ├── parse.rs         # Classic decoders (12-bit EEG, 24-bit PPG, BE IMU)
    │                    # Athena decoder (14-bit LE EEG, tag-based framing)
    └── types.rs         # EegReading, PpgReading, ImuData, MuseEvent, …
```

---

## Protocol notes

### Command encoding (both firmwares)

```
wire = [ len, body_bytes..., '\n' ]
  where body = ASCII command string
        len  = body.len() + 1          (the '\n' is included in the count)
```

### Classic EEG decoding

Each notification: 2-byte big-endian packet index + 18 bytes of 12-bit packed
samples (3 bytes → 2 samples, big-endian):

```
sample = (byte[0] << 4) | (byte[1] >> 4)         // even samples
sample = ((byte[1] & 0xF) << 8) | byte[2]         // odd samples
µV     = 0.48828125 × (sample − 2048)
```

### Athena EEG decoding

Each notification: 9-byte header + tag-based entries.  EEG payload (28 bytes):
14-bit little-endian integers packed LSB-first, channel-major layout
(ch0\_s0, ch0\_s1, ch1\_s0, … ch7\_s1 = 16 values):

```
µV = (raw₁₄ − 8192) × 0.0885
```

### PPG decoding (Classic only)

Six 24-bit big-endian integers per notification:

```
value = (b0 << 16) | (b1 << 8) | b2
```

### Timestamp reconstruction (Classic only)

Classic firmware embeds a 16-bit rolling packet index.  Timestamps are
reconstructed by anchoring the first packet to `now()` and extrapolating
subsequent ones from the index delta and the known sample rate.  16-bit
wrap-around (0xFFFF → 0x0000) is handled automatically.

Athena notifications do not carry a per-channel index; timestamps are not
reconstructed for Athena EEG packets (`timestamp` field is always `0.0`).

---

## References

* [muse-jsx](../muse-jsx) – TypeScript reference implementation (Web Bluetooth)
* [btleplug](https://github.com/deviceplug/btleplug) – Cross-platform BLE library for Rust
* [urish/muse-js](https://github.com/urish/muse-js) – Original muse-js

## License

[Apache-2.0](/LICENSE)

## Copyright 

2026, [Eugene Hauptmann](github.com/eugenehp)