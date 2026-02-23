# muse-rs

A Rust library and terminal UI for streaming real-time sensor data from
**Interaxon Muse EEG headsets** over Bluetooth Low Energy.

Supports every Muse model — including the newer Muse S running the **Athena**
firmware — and automatically selects the correct protocol at connection time.

## EEG view

![EEG view](./docs/tui.png)

## PPG view

![PPG view](./docs/ppg.png)

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
| Muse S | **Athena** | 4 or 8 | ✓ | ✗ | auto-detected |

Athena PPG (optical) data is decoded from 20-bit LE packed samples into
`MuseEvent::Ppg` — 3 samples per channel (ambient, infrared, red) at 64 Hz.

Athena EEG supports both 4-channel (tag `0x11`, 4 samples/pkt) and 8-channel
(tag `0x12`, 2 samples/pkt) modes depending on the preset.

---

## Firmware variants: Classic vs. Athena

Interaxon ships two completely different BLE protocols depending on the device
and firmware version.  This library detects which one is in use at connect time
and switches behaviour automatically — no configuration is required.

### Detection

At connect time, the library inspects the GATT service table.  If the
**universal sensor characteristic** (`273e0013-…`) is present, the device is
running Athena firmware.  Classic devices do not expose this characteristic.

> **Note:** Some transitional Muse S devices (fw 3.x) expose the Athena
> sensor characteristic but reject the Athena `dc001` data-start command
> (rc:69).  The library automatically falls back to the Classic `d` command
> in this case.

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

All sensor data is **multiplexed onto one characteristic** (`273e0013`) using
a tag-based binary framing.

#### Packet structure

Based on the [OpenMuse](https://github.com/DominiqueMakowski/OpenMuse)
project's `decode.py`:

```
byte[0]      packet length
bytes[1..9]  header (pkt_index, device clock, metadata)
byte[9]      first subpacket tag (sensor type)
bytes[10..13] 4-byte subpacket metadata
bytes[14..]  first subpacket payload, then additional [TAG][META4][PAYLOAD]…
```

The first subpacket's type is given by `byte[9]`; subsequent subpackets each
carry their own 1-byte tag + 4-byte metadata header before the payload.

#### Known tags

Tags use the full byte value — payload sizes depend on the specific tag, not
just the lower nibble.

| Tag  | Sensor       | Payload    | Channels | Samples/ch | Rate    |
|------|--------------|------------|----------|------------|---------|
| `0x11` | EEG 4ch    | 28 B       | 4        | 4          | 256 Hz  |
| `0x12` | EEG 8ch    | 28 B       | 8        | 2          | 256 Hz  |
| `0x34` | Optical 4ch | 30 B      | 4        | 3          | 64 Hz   |
| `0x35` | Optical 8ch | 40 B      | 8        | 2          | 64 Hz   |
| `0x36` | Optical 16ch| 40 B      | 16       | 1          | 64 Hz   |
| `0x47` | IMU         | 36 B      | 6        | 3          | 52 Hz   |
| `0x53` | DRL/REF     | 24 B      | –        | –          | 32 Hz   |
| `0x88` | Battery (new fw) | 188–230 B | 1   | 1          | ~0.2 Hz |
| `0x98` | Battery (old fw) | 20 B  | 1        | 1          | 1 Hz    |

**EEG scale:** `µV = (raw₁₄ − 8192) × 0.0885`

**EEG channels (index order):** TP9, AF7, AF8, TP10, FPz, AUX\_R, AUX\_L, AUX
(the first 4 are the standard electrode positions; indices 4–7 are extended
channels only available on Athena hardware).

**IMU:** accelerometer scale = 0.0000610352 g/LSB (same as Classic);
gyroscope scale = −0.0074768 °/s/LSB (negated vs. Classic).

**Battery:** first 2 bytes of payload = u16 LE, divide by 256.0 for percentage.
Confirmed by matching against the `bp` field in the Athena control JSON
response and independently verified by the
[OpenMuse](https://github.com/DominiqueMakowski/OpenMuse) project.

**Startup sequence:** `v4` → `s` → `h` → `p1045` → `dc001` × 2 → `d` (fallback) → `L1` → 2 s wait

**Resume command:** `dc001` + `d` (both sent for compatibility with fw 3.x)

### Side-by-side comparison

| Property | Classic | Athena |
|---|---|---|
| Characteristics | one per sensor | one universal (`273e0013`) |
| EEG channels | 4 (+ optional AUX) | 4 or 8 |
| EEG bit-width | 12-bit | 14-bit |
| EEG byte order | big-endian | little-endian |
| EEG samples/pkt | 12 | 2 (8ch) or 4 (4ch) |
| EEG µV/LSB | 0.48828125 | 0.0885 |
| IMU byte order | big-endian | little-endian |
| Gyro sign | positive | negated |
| PPG format | 24-bit BE, 6 samp | 20-bit LE, 3×4ch |
| Battery | u16 BE / 512 | u16 LE / 256 |
| Resume cmd | `d` | `dc001` + `d` fallback |
| Startup | 4 steps | 8 steps + 2 s wait |
| PPG decoded | ✓ | ✓ |

---

## Features

### Library

Use `muse-rs` as a library in your own project:

```toml
# Cargo.toml

# Full build (includes TUI feature):
muse-rs = "0.1.0"

# Library only — skips ratatui / crossterm compilation:
muse-rs = { version = "0.1.0", default-features = false }
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
            MuseEvent::Ppg(r) => println!("ppg ch{}: {:?}", r.ppg_channel, r.samples),
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
| `Eeg` (ch 4–7, Athena-only) | ✗ | ✗ | ✗ | ✓ (8ch mode) |
| `Accelerometer` | ✓ | ✓ | ✓ | ✓ |
| `Gyroscope` | ✓ | ✓ | ✓ | ✓ |
| `Telemetry` (battery) | ✓ | ✓ | ✓ | ✓ |
| `Ppg` | ✗ | ✓\* | ✓\* | ✓ |
| `Control` | ✓ | ✓ | ✓ | ✓ |

\* Classic requires `enable_ppg: true` in `MuseClientConfig`.
Athena always includes PPG with preset `p1045`.

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

### macOS — BLE disconnect detection

This project uses a [fork of btleplug](https://github.com/eugenehp/btleplug/tree/imrpoved_mac_version)
with improved macOS support, including reliable disconnect detection via
`CentralEvent::DeviceDisconnected`, expanded broadcast channel buffers, and
null-safety improvements that prevent hangs when a peripheral becomes
unreachable.

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

### EEG view (default)

```
┌──────────────────────────────────────────────────────────────────────────────┐
│  MUSE EEG Monitor  │  ● Muse-AB12  │  EEG  │  Bat 85%  │  21.3 pkt/s  │  ±500 µV  │
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
│ [Tab]Devices [1]EEG [2]PPG [d]Disconnect [+/-]Scale [a]Auto [v]Smooth         │
│ Accel x:+0.010g  y:+0.020g  z:-1.000g   Gyro x:+0.120°/s  …                  │
└──────────────────────────────────────────────────────────────────────────────┘
```

### PPG view (press `2`)

```
┌──────────────────────────────────────────────────────────────────────────────┐
│  MUSE EEG Monitor  │  ● Muse-AB12  │  PPG  │  Bat 85%  │  21.3 pkt/s  │  auto  │
├──────────────────────────────────────────────────────────────────────────────┤
│ Ambient   min:12400  max:13200                                [SMOOTH]        │
│ ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿  3 optical channels, auto-scaled Y axis                │
├──────────────────────────────────────────────────────────────────────────────┤
│ Infrared  ...                                                                │
├──────────────────────────────────────────────────────────────────────────────┤
│ Red       ...                                                                │
├──────────────────────────────────────────────────────────────────────────────┤
│ [Tab]Devices [1]EEG [2]PPG [d]Disconnect [+/-]Scale [a]Auto [v]Smooth         │
│ Accel x:+0.010g  y:+0.020g  z:-1.000g   Gyro x:+0.120°/s  …                  │
└──────────────────────────────────────────────────────────────────────────────┘
```

Each panel shows a rolling **2-second window** rendered with Braille markers
(~4× the resolution of block characters).  EEG borders turn **red** when any
sample exceeds the current Y-axis scale.  PPG uses auto-scaling Y axis that
adapts to the data range with 5% margin.

**Smooth mode** (on by default, toggle with `v`): draws the raw signal in a
dim colour as background context, then overlays a 9-sample moving-average
(≈ 35 ms at 256 Hz / ≈ 140 ms at 64 Hz) in the full channel colour.

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
| `1` | streaming | switch to EEG view (4 channels) |
| `2` | streaming | switch to PPG view (3 optical channels) |
| `s` | streaming / picker | rescan for Muse devices |
| `d` | streaming | disconnect and rescan |
| `+` / `=` | EEG view | zoom out (increase µV scale) |
| `-` | EEG view | zoom in (decrease µV scale) |
| `a` | EEG view | auto-scale to current peak |
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
PPG streaming is enabled by default.

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

The TUI writes logs to `muse-tui.log` in the current directory (never to
stderr, which would corrupt the alternate-screen display):

```bash
RUST_LOG=debug cargo run --bin tui
cat muse-tui.log
```

---

## Configuration

```rust
let config = MuseClientConfig {
    enable_aux:        false,   // subscribe to EEG AUX channel (Classic only)
    enable_ppg:        true,    // subscribe to PPG channels (Classic only)
    scan_timeout_secs: 15,      // abort scan after this many seconds
    name_prefix:       "Muse".into(), // match devices whose name starts with this
};
```

| `enable_ppg` | `enable_aux` | Preset sent (Classic) | Preset sent (Athena) |
|---|---|---|---|
| false | false | `p21` (EEG only) | `p1045` |
| false | true | `p20` (EEG + AUX) | `p1045` |
| true | — | `p50` (EEG + PPG) | `p1045` |

> Athena always uses `p1045` regardless of PPG/AUX flags; all sensors
> (EEG, PPG, IMU, battery) are streamed on the multiplexed characteristic.

---

## Project layout

```
muse-rs/
├── Cargo.toml
├── build.rs             # macOS Info.plist embedding for CoreBluetooth
├── Info.plist           # NSBluetoothAlwaysUsageDescription
└── src/
    ├── lib.rs           # Crate root: module declarations + prelude
    ├── main.rs          # Headless CLI binary (cargo run)
    ├── bin/
    │   └── tui.rs       # Full-screen TUI binary (cargo run --bin tui)
    │                    # EEG + PPG views, device picker, smooth overlay
    ├── muse_client.rs   # MuseClient (scan/connect) + MuseHandle (commands)
    │                    # Firmware detection + dual protocol dispatch
    │                    # BLE disconnect detection (adapter event stream)
    ├── protocol.rs      # GATT UUIDs, sampling constants, encode/decode helpers
    ├── parse.rs         # Classic decoders (12-bit EEG, 24-bit PPG, BE IMU)
    │                    # Athena decoder (tag-based: EEG, PPG, IMU, battery)
    │                    # ControlAccumulator (JSON fragment reassembly)
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

Each notification: 14-byte header + tag-based entries.  EEG payload (28 bytes):
14-bit little-endian integers packed LSB-first, sample-major layout.

For 8ch mode (tag `0x12`): 2 samples × 8 channels = 16 values.
For 4ch mode (tag `0x11`): 4 samples × 4 channels = 16 values.

```
µV = (raw₁₄ − 8192) × 0.0885
```

### PPG decoding

**Classic:** six 24-bit big-endian unsigned integers per notification:

```
value = (b0 << 16) | (b1 << 8) | b2
```

**Athena (tag `0x34`, 4-channel):** 30-byte payload, 12 × 20-bit LE unsigned
integers (3 samples × 4 channels, sample-major layout).
Channels 0–2 = ambient, infrared, red:

```
raw = parseUintLE(payload, 20)    // 12 values
ch_samples[ch][s] = raw[s * 4 + ch]
```

**Athena (tag `0x35`, 8-channel):** 40-byte payload, 16 × 20-bit LE unsigned
integers (2 samples × 8 channels).

### Battery decoding

**Classic:** `battery_level = u16_BE / 512.0`

**Athena:** First 2 bytes of battery payload: `battery_level = u16_LE / 256.0`.
Tag `0x98` (old fw) has a fixed 20-byte payload; tag `0x88` (new fw) has a
variable-length payload (188–230 bytes) — the parser consumes to the packet
boundary using `byte[0]` (pkt\_len).

### Timestamp reconstruction (Classic only)

Classic firmware embeds a 16-bit rolling packet index.  Timestamps are
reconstructed by anchoring the first packet to `now()` and extrapolating
subsequent ones from the index delta and the known sample rate.  16-bit
wrap-around (0xFFFF → 0x0000) is handled automatically.

Athena notifications do not carry a per-channel index; timestamps are not
reconstructed for Athena EEG packets (`timestamp` field is always `0.0`).

### Control JSON reassembly

Both firmwares send JSON responses as length-prefixed fragments split across
multiple BLE notifications.  The `ControlAccumulator` in `parse.rs` tracks
brace nesting depth to reassemble complete JSON objects from the fragment
stream, handling nested objects and fragments that split mid-token.

---

## Dependencies

| Crate | Purpose |
|---|---|
| [btleplug](https://github.com/eugenehp/btleplug/tree/imrpoved_mac_version) | Cross-platform BLE (forked for improved macOS support) |
| [tokio](https://tokio.rs) | Async runtime |
| [ratatui](https://ratatui.rs) | Terminal UI framework (optional, `tui` feature) |
| [crossterm](https://github.com/crossterm-rs/crossterm) | Terminal backend (optional, `tui` feature) |

---

## References

* [OpenMuse](https://github.com/DominiqueMakowski/OpenMuse) — Python Muse S / Athena decoder; used as reference for tag-based packet structure, payload sizes, battery decoding, and EEG/optical channel layouts
* [muse-jsx](https://github.com/eugenehp/muse-jsx) — TypeScript reference implementation (Web Bluetooth); basis for the Athena startup sequence and `parsePacket()` tag decoder
* [btleplug](https://github.com/eugenehp/btleplug/tree/imrpoved_mac_version) — Cross-platform BLE library for Rust (fork with improved macOS disconnect handling)
* [urish/muse-js](https://github.com/urish/muse-js) — Original muse-js library by Uri Shaked
* [Interaxon Muse](https://choosemuse.com/) — Official Muse headset manufacturer

---

## Citation

If you use `muse-rs` in academic research or published work, please cite it as:

### BibTeX

```bibtex
@software{hauptmann2026musers,
  author       = {Hauptmann, Eugene},
  title        = {muse-rs: Rust Library and TUI for Muse EEG Headsets},
  year         = {2026},
  url          = {https://github.com/eugenehp/muse-rs},
  version      = {0.1.0},
  description  = {Async Rust library and terminal UI for streaming real-time
                  EEG, PPG, IMU, and battery data from Interaxon Muse headsets
                  over Bluetooth Low Energy. Supports Classic and Athena
                  firmware with automatic protocol detection.},
}
```

### APA

> Hauptmann, E. (2026). *muse-rs: Rust Library and TUI for Muse EEG Headsets* (Version 0.1.0) [Computer software]. https://github.com/eugenehp/muse-rs

### IEEE

> E. Hauptmann, "muse-rs: Rust Library and TUI for Muse EEG Headsets," 2026. [Online]. Available: https://github.com/eugenehp/muse-rs

---

## License

[Apache-2.0](./LICENSE)

## Copyright

© 2026, [Eugene Hauptmann](https://github.com/eugenehp)
