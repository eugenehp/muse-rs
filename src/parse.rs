//! Binary decoders for Muse BLE notification payloads.
//!
//! All public functions in this module are pure (no I/O, no allocation beyond
//! the returned collections) and are safe to call from any async or sync context.
//!
//! # Classic vs. Athena decoder split
//!
//! ## Classic firmware (Muse 1 / Muse 2 / Muse S ≤ fw 3.x)
//!
//! One GATT characteristic per sensor; each notification has a fixed layout:
//!
//! | Function | Sensor | Format |
//! |---|---|---|
//! | [`decode_eeg_samples`] | EEG | 12-bit BE packed, 12 samples, 0.48828125 µV/LSB |
//! | [`decode_ppg_samples`] | PPG | 24-bit BE unsigned, 6 samples |
//! | [`parse_telemetry`] | Battery | 5 × u16 BE fields |
//! | [`parse_accelerometer`] | Accel | 3 × i16 BE XYZ, 0.0000610352 g/LSB |
//! | [`parse_gyroscope`] | Gyro | 3 × i16 BE XYZ, +0.0074768 °/s/LSB |
//! | [`parse_ppg_reading`] | PPG | wraps [`decode_ppg_samples`] with index + channel |
//!
//! ## Athena firmware (Muse S fw ≥ 4.x)
//!
//! All sensors are multiplexed on one characteristic using tag-based framing.
//! [`parse_athena_notification`] demultiplexes an entire notification and
//! returns a `Vec<MuseEvent>` covering all sensors present in that packet:
//!
//! | Tag | Sensor | Format |
//! |---|---|---|
//! | `0x11`, `0x12` | EEG (8 ch) | 14-bit LE packed, 2 samples/ch, 0.0885 µV/LSB |
//! | `0x47` | IMU | i16 LE, accel +0.0000610352 g/LSB, gyro −0.0074768 °/s/LSB |
//! | `0x34`–`0x36` | Optical/PPG | 20-bit LE, 4, 8 or 16 ch, 64 Hz |
//! | `0x53` | DRL/REF | u16 LE, 2 ch × 6 samples |
//! | `0x88`, `0x98` | Battery | u16 LE ÷ 256 |
//!
//! [`AthenaDecoder`] is the same decoder with gap detection, for a live
//! connection.

use std::collections::BTreeMap;

use crate::protocol::ATHENA_EEG_SCALE;
use crate::types::{
    DataGap, DrlRefData, EegReading, ImuData, MuseEvent, PpgReading, Sensor, TelemetryData,
    XyzSample,
};

// ── EEG ──────────────────────────────────────────────────────────────────────

/// Decode a packed 12-bit unsigned array (big-endian, 3 bytes → 2 samples).
///
/// Input layout (repeated for the full slice):
/// ```text
/// [AA BB CC] → sample0 = (AA << 4) | (BB >> 4)
///              sample1 = ((BB & 0x0F) << 8) | CC
/// ```
///
/// Returns one output value per complete group; trailing bytes that don't
/// form a full 3-byte group are silently ignored.
pub fn decode_unsigned_12bit(data: &[u8]) -> Vec<u16> {
    let mut out = Vec::with_capacity(data.len() * 2 / 3);
    let mut i = 0;
    while i < data.len() {
        if i + 1 < data.len() {
            out.push(((data[i] as u16) << 4) | ((data[i + 1] as u16) >> 4));
        }
        if i + 2 < data.len() {
            out.push((((data[i + 1] as u16) & 0x0F) << 8) | (data[i + 2] as u16));
        }
        i += 3;
    }
    out
}

/// Decode EEG payload bytes (everything after the 2-byte packet index) into µV.
///
/// Each 12-bit raw value is centred at 0x800 (mid-scale) and converted with
/// a scale factor of 0.48828125 µV/bit (= 1000 µV / 2048 steps):
/// `µV = (raw − 2048) × 0.48828125`
pub fn decode_eeg_samples(data: &[u8]) -> Vec<f64> {
    decode_unsigned_12bit(data)
        .into_iter()
        .map(|n| 0.48828125 * (n as f64 - 0x800 as f64))
        .collect()
}

// ── PPG ──────────────────────────────────────────────────────────────────────

/// Decode a 24-bit unsigned big-endian array (3 bytes per sample).
///
/// Returns one `u32` per complete 3-byte group; partial trailing bytes are ignored.
pub fn decode_unsigned_24bit(data: &[u8]) -> Vec<u32> {
    data.chunks_exact(3)
        .map(|c| ((c[0] as u32) << 16) | ((c[1] as u32) << 8) | (c[2] as u32))
        .collect()
}

/// Decode PPG payload bytes (after the 2-byte index) into raw 24-bit ADC values.
///
/// Thin wrapper around [`decode_unsigned_24bit`] provided for naming symmetry
/// with [`decode_eeg_samples`].
pub fn decode_ppg_samples(data: &[u8]) -> Vec<u32> {
    decode_unsigned_24bit(data)
}

// ── Telemetry ─────────────────────────────────────────────────────────────────

/// Parse a Classic firmware telemetry BLE notification into a [`TelemetryData`].
///
/// **Classic firmware only.** Athena battery data arrives via tags `0x88`/`0x98`
/// inside [`parse_athena_notification`], which uses a different layout
/// (little-endian u16, 20-byte payload; only `battery_level` is populated there).
///
/// All fields are big-endian `u16`. Expected layout:
///
/// | Bytes | Field | Conversion |
/// |---|---|---|
/// | 0–1 | sequence_id | raw |
/// | 2–3 | battery raw | ÷ 512 → % |
/// | 4–5 | fuel gauge  | × 2.2 → mV |
/// | 6–7 | (unused ADC) | — |
/// | 8–9 | temperature | raw ADC |
///
/// Returns `None` if `data` is shorter than 10 bytes.
pub fn parse_telemetry(data: &[u8]) -> Option<TelemetryData> {
    if data.len() < 10 {
        return None;
    }
    Some(TelemetryData {
        sequence_id: u16::from_be_bytes([data[0], data[1]]),
        battery_level: u16::from_be_bytes([data[2], data[3]]) as f32 / 512.0,
        fuel_gauge_voltage: u16::from_be_bytes([data[4], data[5]]) as f32 * 2.2,
        temperature: u16::from_be_bytes([data[8], data[9]]),
    })
}

// ── IMU ───────────────────────────────────────────────────────────────────────

/// Read a big-endian signed 16-bit integer from `data` at byte `offset`.
///
/// # Panics
/// Panics if `offset + 1 >= data.len()`.
fn read_i16_be(data: &[u8], offset: usize) -> i16 {
    i16::from_be_bytes([data[offset], data[offset + 1]])
}

/// Shared decoder for accelerometer and gyroscope notifications.
///
/// Both sensors use an identical wire format: a 2-byte big-endian sequence ID
/// followed by three XYZ samples, each sample being three consecutive big-endian
/// `i16` values (x, y, z), laid out at byte offsets 2, 8, and 14.
///
/// `scale` is multiplied into every raw `i16` before constructing the
/// [`XyzSample`]; callers supply sensor-specific factors:
/// * Accelerometer: 0.0000610352 g/LSB
/// * Gyroscope: 0.0074768 °/s/LSB
///
/// Returns `None` if `data` is shorter than 20 bytes.
fn parse_imu_reading(data: &[u8], scale: f32) -> Option<ImuData> {
    if data.len() < 20 {
        return None;
    }
    let seq = u16::from_be_bytes([data[0], data[1]]);

    // Three XYZ samples at byte offsets 2, 8, 14
    let sample = |off: usize| XyzSample {
        x: scale * read_i16_be(data, off) as f32,
        y: scale * read_i16_be(data, off + 2) as f32,
        z: scale * read_i16_be(data, off + 4) as f32,
    };

    Some(ImuData {
        sequence_id: seq,
        samples: [sample(2), sample(8), sample(14)],
    })
}

/// Parse a Classic firmware accelerometer notification into an [`ImuData`].
///
/// **Classic firmware only.** Athena IMU data (tag `0x47`) is handled inside
/// [`parse_athena_notification`] using little-endian `i16` with the same scale.
///
/// Scale: 0.0000610352 g/LSB (±2 G full-scale, 16-bit ADC:
/// 2 G / 32768 ≈ 6.1×10⁻⁵ g/LSB).
///
/// Returns `None` if the payload is shorter than 20 bytes.
pub fn parse_accelerometer(data: &[u8]) -> Option<ImuData> {
    parse_imu_reading(data, 0.0000610352)
}

/// Parse a Classic firmware gyroscope notification into an [`ImuData`].
///
/// **Classic firmware only.** Athena gyroscope data (tag `0x47`) is handled
/// inside [`parse_athena_notification`] using little-endian `i16` with a
/// **negated** scale (−0.0074768 °/s/LSB vs. +0.0074768 here).
///
/// Scale: +0.0074768 °/s/LSB (±245 dps full-scale, 16-bit ADC:
/// 245 / 32768 ≈ 7.48×10⁻³ °/s/LSB).
///
/// Returns `None` if the payload is shorter than 20 bytes.
pub fn parse_gyroscope(data: &[u8]) -> Option<ImuData> {
    parse_imu_reading(data, 0.0074768)
}

// ── PPG reading assembly ──────────────────────────────────────────────────────

/// Parse a full PPG BLE notification into a [`PpgReading`].
///
/// Wire layout: `[index_hi, index_lo, sample0_b0, sample0_b1, sample0_b2, …]`
///
/// Returns `None` if `data` is shorter than 2 bytes (no room for the index).
pub fn parse_ppg_reading(data: &[u8], ppg_channel: usize, timestamp: f64) -> Option<PpgReading> {
    if data.len() < 2 {
        return None;
    }
    let index = u16::from_be_bytes([data[0], data[1]]);
    let samples = decode_ppg_samples(&data[2..]);
    Some(PpgReading {
        index,
        ppg_channel,
        timestamp,
        samples,
    })
}

// ── Control response parsing ──────────────────────────────────────────────────

/// Incrementally assembles Muse control-channel BLE fragments into complete JSON.
///
/// The headset splits its JSON status replies across multiple BLE notifications.
/// Each decoded fragment is a partial string like `{"hn":`, `"Muse-AB12"`, `}`.
/// This accumulator buffers fragments in order and returns a full JSON string
/// the moment a closing `}` is observed.
///
/// # Usage
///
/// ```
/// # use muse_rs::parse::ControlAccumulator;
/// let mut acc = ControlAccumulator::new();
/// assert!(acc.push(r#"{"fw":"3.4."#).is_none());   // incomplete
/// let json = acc.push(r#"5"}"#).unwrap();           // closed → returns full JSON
/// assert_eq!(json, r#"{"fw":"3.4.5"}"#);
/// ```
///
/// Nested braces are tracked correctly, and characters before the first `{`
/// are discarded so stale data from a previous notification cannot corrupt
/// the next object.
pub struct ControlAccumulator {
    buffer: String,
    /// Brace nesting depth.  Becomes 1 on `{`, returns to 0 on the matching `}`.
    depth: usize,
}

impl ControlAccumulator {
    /// Create a new, empty accumulator.
    pub fn new() -> Self {
        Self {
            buffer: String::new(),
            depth: 0,
        }
    }

    /// Append a decoded fragment to the internal buffer.
    ///
    /// Returns the complete JSON string when the top-level closing `}` is
    /// encountered (depth returns to 0), otherwise returns `None`.
    /// The buffer is cleared after a complete object is returned.
    ///
    /// Characters received before the first `{` are silently discarded so
    /// that stale trailing data from a previous notification cannot corrupt
    /// the next object.
    pub fn push(&mut self, fragment: &str) -> Option<String> {
        for ch in fragment.chars() {
            if ch == '{' {
                if self.depth == 0 {
                    // Starting a new top-level object — discard any garbage
                    // that accumulated before this opening brace.
                    self.buffer.clear();
                }
                self.depth += 1;
                self.buffer.push(ch);
            } else if ch == '}' {
                if self.depth > 0 {
                    self.buffer.push(ch);
                    self.depth -= 1;
                    if self.depth == 0 {
                        let json = self.buffer.clone();
                        self.buffer.clear();
                        return Some(json);
                    }
                }
                // else: stray '}' before any '{' — ignore
            } else {
                // Only accumulate characters while inside a top-level object.
                if self.depth > 0 {
                    self.buffer.push(ch);
                }
            }
        }
        None
    }
}

impl Default for ControlAccumulator {
    fn default() -> Self {
        Self::new()
    }
}

// ── Athena packet parser ──────────────────────────────────────────────────────

/// Unpack N-bit little-endian unsigned integers from a packed byte slice.
///
/// Reads `floor(data.len() * 8 / bit_width)` values.  Bits are consumed
/// LSB-first, byte by byte, matching the layout used by Athena EEG packets.
///
/// Equivalent to the TypeScript `parseUintLEValues(buf, bitWidth)` in
/// `athena-parser.ts`.
fn parse_uint_le_bits(data: &[u8], bit_width: usize) -> Vec<u32> {
    let n = (data.len() * 8) / bit_width;
    (0..n)
        .map(|i| {
            let mut val = 0u32;
            for bit in 0..bit_width {
                let total = i * bit_width + bit;
                let byte_off = total / 8;
                let bit_in_byte = total % 8;
                if byte_off < data.len() && (data[byte_off] >> bit_in_byte) & 1 != 0 {
                    val |= 1 << bit;
                }
            }
            val
        })
        .collect()
}

/// Look up the payload size for a known Athena tag byte.
///
/// Based on the SENSORS table from the OpenMuse project
/// (`DominiqueMakowski/OpenMuse`, `decode.py`).  Payload sizes depend on
/// the **full tag byte**, not just the lower nibble, because some sensor
/// categories (optical, battery) have different sizes per variant.
///
/// Returns `None` for unknown tags, and for `0x88`, whose payload is
/// variable-length and runs to the end of its packet.
pub fn athena_payload_len(tag: u8) -> Option<usize> {
    match tag {
        // EEG — 4ch×4samples or 8ch×2samples, both 28 B (14-bit packed)
        0x11 | 0x12 => Some(28),
        // Optical 4ch — 3 samples × 4ch, 20-bit packed
        0x34 => Some(30),
        // Optical 8ch — 2 samples × 8ch, 20-bit packed
        0x35 => Some(40),
        // Optical 16ch — 1 sample × 16ch, 20-bit packed
        0x36 => Some(40),
        // IMU (accel + gyro) — 3 samples × 6ch, 16-bit LE signed
        0x47 => Some(36),
        // DRL/REF — 2ch × 6 samples, 16-bit LE unsigned
        0x53 => Some(24),
        // Battery (new firmware) — variable 188–230 B; only first 2 used
        0x88 => None,
        // Battery (old firmware) — 20 B
        0x98 => Some(20),
        _ => None,
    }
}

/// Bytes in an Athena packet header. The first subpacket's payload starts here.
const ATHENA_PACKET_HEADER: usize = 14;

/// Offset of the first subpacket's tag within its packet's header.
///
/// Bytes 9–13 of the header have the same shape as the five-byte header in
/// front of every later subpacket — tag, counter, three unknown bytes — so
/// the first subpacket is read exactly like the rest.
const ATHENA_FIRST_TAG: usize = 9;

/// Bytes in front of each subpacket's payload: tag, counter, three unknown.
const ATHENA_SUBPACKET_HEADER: usize = 5;

/// One sensor's block within an Athena packet.
struct Subpacket<'a> {
    tag: u8,
    /// The sensor's own sequence counter (0–255, wraps), one per tag.
    counter: u8,
    payload: &'a [u8],
}

/// Split one Athena notification into its subpackets.
///
/// # Layout
///
/// A notification holds one or more packets back to back, each opening with
/// its own length. It checks that length against what arrived
/// (`Ble4 char=%d size mismatch: actual=%zu declared=%u`), and OpenMuse walks
/// a notification packet by packet on it.
///
/// ```text
/// packet:
///   [0]        packet length, header included
///   [1]        packet counter
///   [2..6]     device clock, u32 LE, 256 kHz
///   [6..9]     unknown
///   [9]        first subpacket's tag
///   [10]       first subpacket's counter
///   [11..14]   unknown
///   [14..]     first subpacket's payload, then [TAG][COUNTER][3][PAYLOAD]…
/// ```
///
/// Each packet is parsed to its own end and no further. Reading on past it —
/// as this did before — treats the next packet's header as subpackets, and
/// any header byte that happens to equal a sensor tag produces a reading made
/// of header bytes.
///
/// An unknown tag ends its packet, because without a length nothing after it
/// can be located; scanning forward a byte at a time for something that looks
/// like a tag finds payload bytes as often as tags. A packet whose length is
/// impossible ends the notification.
fn athena_subpackets(data: &[u8]) -> Vec<Subpacket<'_>> {
    let mut out = Vec::new();
    let mut offset = 0;

    while offset < data.len() {
        let pkt_len = data[offset] as usize;
        if pkt_len < ATHENA_PACKET_HEADER || offset + pkt_len > data.len() {
            log::debug!(
                "Athena: packet at {offset} declares {pkt_len} bytes, {} remain; dropped",
                data.len() - offset
            );
            break;
        }
        let packet = &data[offset..offset + pkt_len];
        offset += pkt_len;

        let mut idx = ATHENA_FIRST_TAG;
        while idx + ATHENA_SUBPACKET_HEADER <= packet.len() {
            let tag = packet[idx];
            let start = idx + ATHENA_SUBPACKET_HEADER;
            let len = match athena_payload_len(tag) {
                Some(len) => len,
                // Variable-length, and the last thing in its packet.
                None if tag == 0x88 => packet.len() - start,
                None => {
                    log::debug!("Athena: unknown tag 0x{tag:02x}, rest of packet skipped");
                    break;
                }
            };
            if start + len > packet.len() {
                log::debug!("Athena: tag 0x{tag:02x} truncated, rest of packet skipped");
                break;
            }
            out.push(Subpacket {
                tag,
                counter: packet[idx + 1],
                payload: &packet[start..start + len],
            });
            idx = start + len;
        }
    }

    out
}

/// Which stream a tag belongs to, for gap accounting.
fn athena_sensor(tag: u8) -> Option<Sensor> {
    match tag {
        0x11 | 0x12 => Some(Sensor::Eeg),
        0x34..=0x36 => Some(Sensor::Optical),
        0x47 => Some(Sensor::Imu),
        0x53 => Some(Sensor::DrlRef),
        0x88 | 0x98 => Some(Sensor::Battery),
        _ => None,
    }
}

/// Samples per channel in one subpacket of `tag`.
fn athena_samples_per_packet(tag: u8) -> u32 {
    match tag {
        0x11 => 4,
        0x12 => 2,
        0x34 => 3,
        0x35 => 2,
        0x36 => 1,
        0x47 => 3,
        0x53 => 6,
        _ => 1,
    }
}

/// Emit one [`EegReading`] per channel from a 14-bit EEG payload.
///
/// Values are sample-major — every channel's first sample, then every
/// channel's second — per OpenMuse.
fn push_athena_eeg(
    payload: &[u8],
    n_ch: usize,
    n_samp: usize,
    index: u16,
    events: &mut Vec<MuseEvent>,
) {
    let raw = parse_uint_le_bits(payload, 14);
    for ch in 0..n_ch {
        let samples: Vec<f64> = (0..n_samp)
            .filter_map(|s| raw.get(s * n_ch + ch))
            .map(|&v| (v as f64 - 8192.0) * ATHENA_EEG_SCALE)
            .collect();
        events.push(MuseEvent::Eeg(EegReading {
            index,
            electrode: ch,
            timestamp: 0.0,
            samples,
        }));
    }
}

/// Emit one [`PpgReading`] per channel from a 20-bit optical payload,
/// sample-major like EEG.
fn push_athena_optical(
    payload: &[u8],
    n_ch: usize,
    n_samp: usize,
    index: u16,
    events: &mut Vec<MuseEvent>,
) {
    let raw = parse_uint_le_bits(payload, 20);
    for ch in 0..n_ch {
        let samples: Vec<u32> = (0..n_samp)
            .filter_map(|s| raw.get(s * n_ch + ch).copied())
            .collect();
        events.push(MuseEvent::Ppg(PpgReading {
            index,
            ppg_channel: ch,
            timestamp: 0.0,
            samples,
        }));
    }
}

/// Decode one subpacket into the events it carries.
fn decode_athena_subpacket(sp: &Subpacket<'_>, events: &mut Vec<MuseEvent>) {
    let index = sp.counter as u16;
    let payload = sp.payload;

    match sp.tag {
        // ── EEG ───────────────────────────────────────────────────────────────
        // 4 channels × 4 samples, or 8 × 2; 16 values of 14 bits in 28 bytes.
        0x11 => push_athena_eeg(payload, 4, 4, index, events),
        0x12 => push_athena_eeg(payload, 8, 2, index, events),

        // ── Optical ───────────────────────────────────────────────────────────
        0x34 => push_athena_optical(payload, 4, 3, index, events),
        0x35 => push_athena_optical(payload, 8, 2, index, events),
        // Confirmed against hardware: presets p1041 and p1042 select this
        // mode on fw 3.1.11. It used to be skipped outright, which is why
        // those two presets looked like they carried no optical data at all
        // — they carry the most of any preset.
        //
        // The layout is confirmed by the shape of the result rather than
        // assumed: de-interleaved this way, each of the sixteen channels
        // holds a steady DC level of its own (spreads of 99-4404 over five
        // seconds) and they fall into consecutive pairs. A wrong stride would
        // smear channels together and show every one of them swinging across
        // the whole range.
        0x36 => push_athena_optical(payload, 16, 1, index, events),

        // ── IMU ───────────────────────────────────────────────────────────────
        // 36 bytes is eighteen i16 LE: three samples of accelerometer xyz
        // followed by gyroscope xyz, interleaved per sample.
        0x47 => {
            let vals: Vec<i16> = payload
                .chunks_exact(2)
                .map(|c| i16::from_le_bytes([c[0], c[1]]))
                .collect();
            const AS: f32 = 0.0000610352;
            const GS: f32 = -0.0074768;
            let accel = |i: usize| XyzSample {
                x: vals[i * 6] as f32 * AS,
                y: vals[i * 6 + 1] as f32 * AS,
                z: vals[i * 6 + 2] as f32 * AS,
            };
            let gyro = |i: usize| XyzSample {
                x: vals[i * 6 + 3] as f32 * GS,
                y: vals[i * 6 + 4] as f32 * GS,
                z: vals[i * 6 + 5] as f32 * GS,
            };
            events.push(MuseEvent::Accelerometer(ImuData {
                sequence_id: index,
                samples: [accel(0), accel(1), accel(2)],
            }));
            events.push(MuseEvent::Gyroscope(ImuData {
                sequence_id: index,
                samples: [gyro(0), gyro(1), gyro(2)],
            }));
        }

        // ── DRL / REF ─────────────────────────────────────────────────────────
        // Twelve u16 LE: six samples of two channels, interleaved per sample.
        // See `DrlRefData` for how the layout was established.
        0x53 => {
            let vals: Vec<u16> = payload
                .chunks_exact(2)
                .map(|c| u16::from_le_bytes([c[0], c[1]]))
                .collect();
            events.push(MuseEvent::DrlRef(DrlRefData {
                index,
                samples: vals.chunks_exact(2).map(|p| [p[0], p[1]]).collect(),
            }));
        }

        // ── Battery ───────────────────────────────────────────────────────────
        // State of charge in the first two bytes: u16 LE / 256 = percent
        // (per OpenMuse, and matching the status JSON's `bp`). `0x88` carries
        // more after it, of varying length and unknown meaning.
        0x88 | 0x98 if payload.len() >= 2 => {
            let raw = u16::from_le_bytes([payload[0], payload[1]]);
            events.push(MuseEvent::Telemetry(TelemetryData {
                sequence_id: index,
                battery_level: (raw as f32 / 256.0).clamp(0.0, 100.0),
                fuel_gauge_voltage: 0.0,
                temperature: 0,
            }));
        }

        _ => {}
    }
}

/// Parse one Athena BLE notification into zero or more [`MuseEvent`]s.
///
/// Athena firmware (new Muse S) multiplexes all sensors onto a single GATT
/// characteristic ([`crate::protocol::ATHENA_SENSOR_CHARACTERISTIC`]) as
/// length-prefixed packets of tagged subpackets; see [`AthenaDecoder`] for
/// the layout.
///
/// # Known tags
///
/// | Tag  | Sensor       | Payload | Channels | Samples/ch | Rate    |
/// |------|--------------|---------|----------|------------|---------|
/// | 0x11 | EEG 4ch      | 28 B    | 4        | 4          | 256 Hz  |
/// | 0x12 | EEG 8ch      | 28 B    | 8        | 2          | 256 Hz  |
/// | 0x34 | Optical 4ch  | 30 B    | 4        | 3          | 64 Hz   |
/// | 0x35 | Optical 8ch  | 40 B    | 8        | 2          | 64 Hz   |
/// | 0x36 | Optical 16ch | 40 B    | 16       | 1          | 64 Hz   |
/// | 0x47 | IMU          | 36 B    | 6        | 3          | 52 Hz   |
/// | 0x53 | DRL/REF      | 24 B    | 2        | 6          | 32 Hz   |
/// | 0x88 | Battery (new)| varies  | 1        | 1          | ~0.2 Hz |
/// | 0x98 | Battery (old)| 20 B    | 1        | 1          | 1 Hz    |
///
/// Stateless, so it cannot see lost data; [`AthenaDecoder`] is the same
/// decoder with the counter bookkeeping that reports it.
pub fn parse_athena_notification(data: &[u8]) -> Vec<MuseEvent> {
    let mut events = Vec::new();
    for sp in athena_subpackets(data) {
        decode_athena_subpacket(&sp, &mut events);
    }
    events
}

/// [`parse_athena_notification`] plus gap detection, for one connection.
///
/// Each sensor numbers its subpackets with its own 8-bit counter. A counter
/// that moves on by more than one means subpackets were lost, and a
/// [`MuseEvent::Gap`] is emitted ahead of the data that follows the hole.
///
/// A counter that repeats or runs backwards is logged and otherwise passed
/// through; one that jumps back by more than half its range is taken as a
/// restart (a preset change, say) rather than as a gap of two hundred-odd.
#[derive(Default)]
pub struct AthenaDecoder {
    /// Last counter seen, by tag. By tag rather than by sensor because the
    /// counter belongs to the subpacket type, and a preset change can swap
    /// one tag for another of the same sensor.
    last: BTreeMap<u8, u8>,
}

impl AthenaDecoder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Decode one notification, with a [`MuseEvent::Gap`] in front of the
    /// data that follows any lost subpackets.
    pub fn decode(&mut self, data: &[u8]) -> Vec<MuseEvent> {
        let mut events = Vec::new();
        for sp in athena_subpackets(data) {
            if let Some(sensor) = athena_sensor(sp.tag) {
                if let Some(prev) = self.last.insert(sp.tag, sp.counter) {
                    match sp.counter.wrapping_sub(prev) {
                        1 => {}
                        0 => log::debug!(
                            "Athena: duplicate counter {} on tag 0x{:02x}",
                            sp.counter,
                            sp.tag
                        ),
                        step @ 2..=128 => {
                            let missing = step as u32 - 1;
                            events.push(MuseEvent::Gap(DataGap {
                                sensor,
                                missing_packets: missing,
                                missing_samples: missing * athena_samples_per_packet(sp.tag),
                            }));
                        }
                        _ => log::debug!(
                            "Athena: counter on tag 0x{:02x} went back from {prev} to {}",
                            sp.tag,
                            sp.counter
                        ),
                    }
                }
            }
            decode_athena_subpacket(&sp, &mut events);
        }
        events
    }
}
