# Muse S Athena — BLE protocol

What this crate knows about the Athena firmware's Bluetooth protocol, and how
each piece is known. Three kinds of source, marked throughout:

- **measured** — observed on a Muse S Athena (`MuseS-EAA2`, fw 3.1.19,
  hardware `Athena_RevE`/`Athena_RevF_NW`) in November 2026;
- **OpenMuse** — from [OpenMuse](https://github.com/DominiqueMakowski/OpenMuse)
  `decode.py`.

Anything that rests on one source alone says so. The Classic protocol is in the
top-level [README](../README.md#protocol-notes).

## GATT

Everything is in service `0000fe8d-0000-1000-8000-00805f9b34fb`; the
characteristics are `273eXXXX-4c4d-454d-96be-f03bac821358`.

| UUID suffix | `CharacteristicId` | Use |
|---|---|---|
| `0001` | `SERIAL` (0) | commands out, JSON replies in |
| `0013` | `UNIVERSAL` | all Athena sensor data, multiplexed |

Its presence is how the crate tells Athena from Classic. The enum also
names `MUSE_DEBUG` and `TIME_SYNC` characteristics, which this crate does not
use yet.

## Commands

A command is one write to `SERIAL`: a length byte, the ASCII command, `\n`; the
length counts the newline. 20 bytes per write
`MuseHandle::send_command` (`MAX_COMMAND_LEN`).

### Every command is answered (measured)

Each accepted command gets a JSON object back, assembled from several
notifications, carrying `rc` — `0` for success. `MuseHandle::command` waits for it.

| Command | Reply (fw 3.1.19) | Time |
|---|---|---|
| `v4` | version object, `rc:0` | ~100 ms |
| `s` | status object, `rc:0` | ~75 ms |
| `h` | `{"rc":0}` | ~30 ms |
| `p1041` | `{"rc":0}` | ~30 ms |
| `dc001` | `{"rc":0}`; data flows ~0.4 s after connecting | ~30 ms |
| `d` | `{"rc":255}` — refused | ~40 ms |
| `L1` | **nothing** | — |
| unsupported preset | **nothing**; the old preset carries on | — |

So a timeout means "not this headset's command" as often as it means trouble.

### The command set

It builds every command sends; each builder
returns a fixed string, read from the disassembly:

| Builder | Command |
|---|---|
| `halt_streaming` | `h` |
| `start_streaming(int)` | `d` + argument — `dc001` |
| `status` | `s` |
| `version(int)` | `v` + number — `v4` when streaming, `v7` in the firmware updater |
| `set_preset` | `p` + preset |
| `keep_alive` | `k` |
| `indicator_led_on` / `_off` | `L1` / `L0` |
| `licensing` | `F1` |
| `boot_to_bootloader` | `*0` |
| `boot_to_headset` | `*1` |

`L1` in the startup sequence, then, turns the indicator LED on. `*0`, `*1` and
`F1` belong to the firmware update and licensing — see
[firmware-update.md](firmware-update.md).

### The replies (measured)

Version (`v4`):

```json
{"fw":"3.1.19","bn":1,"tp":"consumer","hw":"01.1","pv":4,"ap":"headset",
 "sp":"Athena_RevE","hb":"Athena_RevF_NW","bl":"1.0.1","be":"1.5.1","rc":0}
```

`fw` firmware, `pv` protocol version, `ap` mode — `headset`, or `bootloader`
during an update — and `bl` the bootloader's version. `sp` is very
likely the board; the server's firmware request calls it `bsp`. `be` is
probably the Bluetooth chip's firmware.

Status (`s`):

```json
{"hn":"MuseS-EAA2","sn":"7011-BCNK-EAA2","ma":"00-55-da-bb-ea-a2","hs":"DP15-H2UT-1126",
 "id":"0","bp":38.90,"ps":4129,"ln":"0","ch":[0,0],"rc":0}
```

`hn` name, `sn` serial, `ma` MAC, `bp` battery percent, and **`ps` the current
preset in hex**: 4129 is `0x1021`, the preset the headset powers up in; after
`p1045` it reads 4165. `ln`, `id` and `ch` are not identified.

### Startup (measured)

`MuseHandle::start` on Athena:

```
v4 → s → h → p1041 (or p1045) → dc001 (→ d only if refused) → L1 → 2 s
```

each answered before the next, except `L1`, which never is. On fw 3.1.19 the
redundant `d` the sequence used to send was refused with `rc:255`, and data
starts before the two-second wait is over; the wait stays for older firmware.

## Presets (measured)

Every preset, switched on the headset (`h` → preset → `dc001`),
recording which tags arrived:

| Preset | EEG | Optical | IMU | DRL/REF |
|---|---|---|---|---|
| `p1025`, `p1026`, `p1041`, `p1042` | 8 ch | 16 ch (`0x36`) | ✓ | ✓ |
| `p1027`, `p1028`, `p1043`, `p1044` | 8 ch | 8 ch (`0x35`) | ✓ | ✓ |
| `p1029`, `p102A`, `p1045`, `p1046` | 8 ch | 4 ch (`0x34`) | ✓ | ✓ |
| `p1031`, `p1032` | 4 ch | 16 ch | ✓ | ✓ |
| `p1033`, `p1034` | 4 ch | 8 ch | ✓ | ✓ |
| `p1035`, `p1036` | 4 ch | 4 ch | ✓ | ✓ |
| `p1022` | 8 ch | – | – | ✓ |
| `p1023` | a trickle | – | – | – |
| `p1024` | – | – | ✓ | – |
| `p21` | 4 ch | – | ✓ | ✓ |

- No answer, and no change: `p408C`, `p407C`, and the Classic presets
  `p10`–`p55` other than `p21`. Not tried: `p60`, `p61`, `p63`, `pAB`, `pAD`.
- `p1021`, the power-up preset, was only caught mid-transition; untested on
  its own.
- Every optical mode delivers the same per-channel rate, so a wider mode is
  more data, not the same data rearranged. What separates the two presets of a
  row is not visible in which tags arrive.
- No preset produced an unknown tag: nothing streamed that the crate cannot
  decode. In particular no thermistor data.

Full list (`MusePreset`): `p10`, `p12`, `p14`, `p20`–`p23`, `p31`,
`p32`, `p50`–`p55`, `p60`, `p61`, `p63`, `pAB`, `pAD`, `p1021`–`p1029`,
`p102A`, `p1031`–`p1036`, `p1041`–`p1046`, `p408C`, `p407C`.

## Sensor data

### Notifications and packets

A notification from `UNIVERSAL` holds one or more packets back to back, each
opening with its own length (OpenMuse checks it too —
`Ble4 char=%d size mismatch: actual=%zu declared=%u`):

```
packet:
  [0]        packet length, header included
  [1]        packet counter (measured: +1 per packet, never skips)
  [2..6]     device clock, u32 LE — see Timestamps
  [6..9]     unknown
  [9]        first subpacket's tag
  [10]       first subpacket's counter
  [11..14]   unknown
  [14..]     first subpacket's payload, then [TAG][COUNTER][3 unknown][PAYLOAD]…
```

Bytes 9–13 have the same shape as every later subpacket's header, so the first
subpacket is read like the rest. An unknown tag ends its packet — without its
length nothing after it can be found — and a packet whose length is impossible
ends the notification.

The parser used to read on past a packet's end and take the next packet's
header for subpackets, which made readings out of header bytes and lost the
second packet's data. On macOS every notification so far has held exactly one
packet (1,567 of 1,567), so the bug did not show there.

### Tags

| Tag | Sensor | Payload | Layout | Rate |
|---|---|---|---|---|
| `0x11` | EEG 4 ch | 28 B | 16 × 14-bit LE, 4 samples × 4 ch | 256 Hz |
| `0x12` | EEG 8 ch | 28 B | 16 × 14-bit LE, 2 samples × 8 ch | 256 Hz |
| `0x34` | optical 4 ch | 30 B | 12 × 20-bit LE, 3 × 4 | 64 Hz |
| `0x35` | optical 8 ch | 40 B | 16 × 20-bit LE, 2 × 8 | 64 Hz |
| `0x36` | optical 16 ch | 40 B | 16 × 20-bit LE, 1 × 16 | 64 Hz |
| `0x47` | IMU | 36 B | 18 × i16 LE, 3 × (accel xyz, gyro xyz) | 52 Hz |
| `0x53` | DRL/REF | 24 B | 12 × u16 LE, 6 samples × 2 ch | 32 Hz |
| `0x88` | battery | rest of packet | u16 LE ÷ 256 = %, then unknown | ~0.2 Hz |
| `0x98` | battery (older fw) | 20 B | u16 LE ÷ 256 = % | 1 Hz |

All sample-major: every channel's first sample, then every channel's second.

- **EEG:** `µV = (raw − 8192) × 0.0885`. Channels TP9, AF7, AF8, TP10, FPz,
  AUX_R, AUX_L, AUX — index 4 is FPz here and AUX on Classic, so use
  `protocol::eeg_channel_name`, which takes the firmware.
- **Optical:** raw, unscaled. Channels 0–2 are ambient, infrared, red; the
  rest are named by index (`opt3`…`opt15`) because their roles are not
  documented. Each holds a distinct DC level of its own, measured.
- **IMU:** accelerometer 0.0000610352 g/LSB, gyroscope −0.0074768 °/s/LSB
  (negated relative to Classic). All three samples are real; they used to be
  one sample copied three times.
- **DRL/REF** (`MuseEvent::DrlRef`): the layout is `DRL_REF` packet
  maker (2 values, 6 per packet), confirmed by the data: de-interleaved this
  way each channel holds a steady level of its own, ≈ 8232 and ≈ 8272, with a
  spread of one or two counts. Raw — which is DRL and which REF, and the units,
  are not known.
- **Battery:** confirmed against the status reply's `bp` (0x26c8 / 256 = 38.8 %
  against a reported 38 %). The rest of a `0x88` payload varies in length and
  is not decoded.

### Counters and lost data (measured)

Each sensor numbers its subpackets with its own 8-bit counter (OpenMuse:
"per-sensor-type sequence counter"; checks them —
`DuplicateCounterError`, `OutOfOrderCounterError` — and fills gaps with NaN).
The counter is now `index` on EEG and optical readings and `sequence_id` on IMU
and battery.

On real hardware the counters skip: the EEG counter about once every 140
subpackets, optical a little more often, while the packet counter around them
never does — the data is lost on the headset, not over the air. In a stream
where EEG read 250 Hz instead of 256, the lost samples reported for the same
seconds made up the difference.

`parse::AthenaDecoder` keeps the last counter per tag and emits a
`MuseEvent::Gap` (sensor, packets missing, samples per channel missing) in
front of the data after a hole. A repeat is logged; a jump back of more than
half the counter's range is taken as a restart (a preset change), not a gap.
`parse_athena_notification` is the same decoder without the bookkeeping.

### Timestamps

Athena readings still carry `timestamp: 0.0`. The header's device clock is the
obvious source, but OpenMuse's reading of it — u32 LE ticks at 256 kHz —
does not fit fw 3.1.19: the value advances in steps of 256 every few packets,
and treated as 256 kHz it puts three seconds of data into a hundredth of one.
It has a separate time-sync mechanism (`clock_sync.h`, the `TIME_SYNC`
characteristic, "waiting for time sync" before streaming) which this crate
does not implement.

## Fixtures

`tests/fixtures/athena_fw3.1.19_p1041.hex` — 80 consecutive notifications
from `UNIVERSAL` in `p1041`, one hex string per line: 275 EEG subpackets, 142
optical, 38 IMU, 10 DRL/REF, 2 battery, and two gaps the headset really
produced (optical counter 107 → 110, IMU 30 → 32). `tests/athena.rs` decodes
it, and covers each case the old parser got wrong.

To record more, run with `RUST_LOG=muse_rs=trace`: every notification is
logged as `Athena raw: <hex>`.
