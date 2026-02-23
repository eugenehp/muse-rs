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
//! | `0x34`, `0x35` | Optical/PPG | 30 B, 20-bit LE, 3×4ch, 64 Hz |
//! | `0x88`, `0x98` | Battery | u16 LE ÷ 512 |

use crate::protocol::ATHENA_EEG_SCALE;
use crate::types::{EegReading, ImuData, MuseEvent, PpgReading, TelemetryData, XyzSample};

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

/// Parse one Athena BLE notification packet into zero or more [`MuseEvent`]s.
///
/// Athena firmware (new Muse S) multiplexes all sensors onto a single GATT
/// characteristic ([`crate::protocol::ATHENA_SENSOR_CHARACTERISTIC`]) using
/// a tag-based binary framing.
///
/// # Packet layout
///
/// ```text
/// [0..8]  9-byte header  (byte[1] = global event index, rest unused here)
/// [9..]   tag-based entries, repeated:
///           byte 0     : tag
///           bytes 1-4  : 4-byte metadata (skipped)
///           bytes 5+   : payload (length determined by tag)
/// ```
///
/// # Known tags
///
/// | Tag         | Sensor  | Payload | Channels | Samples/ch | Rate   |
/// |-------------|---------|---------|----------|------------|--------|
/// | 0x11 / 0x12 | EEG     | 28 B    | 8        | 2          | 256 Hz |
/// | 0x47        | IMU     | 36 B    | –        | 3          | 52 Hz  |
/// | 0x34 / 0x35 | PPG     | 30 B    | 4        | 3          | 64 Hz  |
/// | 0x88 / 0x98 | Battery | 20 B    | –        | 1          | 1 Hz   |
///
/// Unknown tags advance the cursor by one byte to re-synchronise.
/// Athena tag data-type codes (lower 4 bits of the tag byte).
///
/// The upper 4 bits encode the sampling-rate index (see `ATHENA_FREQ_MAP`
/// in the TypeScript `athena-parser.ts`); only the lower nibble determines
/// the payload format.
mod athena_tag {
    pub const EEG_4CH: u8 = 0x1;
    pub const EEG_8CH: u8 = 0x2;
    pub const DRL_REF: u8 = 0x3;
    pub const OPTICAL_4CH: u8 = 0x4;
    pub const OPTICAL_8CH: u8 = 0x5;
    // 0x6 = OPTICAL_16CH (unused)
    pub const IMU: u8 = 0x7;
    pub const BATTERY: u8 = 0x8;
}

pub fn parse_athena_notification(data: &[u8]) -> Vec<MuseEvent> {
    const HEADER: usize = 9;
    const META: usize = 4; // opaque metadata bytes between tag and payload

    if data.len() < HEADER + 1 {
        return vec![];
    }

    let mut events = Vec::new();
    let mut idx = HEADER;

    while idx < data.len() {
        let tag = data[idx];
        let tag_type = tag & 0x0F; // lower nibble = data type
        let payload_start = idx + 1 + META;

        match tag_type {
            // ── EEG (4-ch or 8-ch) ───────────────────────────────────────────
            // 14-bit LE unsigned, channel-major layout.
            // Scale: (raw − 8192) × 0.0885 µV
            athena_tag::EEG_4CH | athena_tag::EEG_8CH => {
                const PLEN: usize = 28;
                let end = payload_start + PLEN;
                if end > data.len() {
                    idx += 1;
                    continue;
                }
                let raw = parse_uint_le_bits(&data[payload_start..end], 14);
                const SAMPLES_PER_CH: usize = 2;
                let num_ch = raw.len() / SAMPLES_PER_CH;
                for ch in 0..num_ch {
                    let samples: Vec<f64> = raw
                        [ch * SAMPLES_PER_CH..(ch + 1) * SAMPLES_PER_CH]
                        .iter()
                        .map(|&v| (v as f64 - 8192.0) * ATHENA_EEG_SCALE)
                        .collect();
                    events.push(MuseEvent::Eeg(EegReading {
                        index: 0,
                        electrode: ch,
                        timestamp: 0.0,
                        samples,
                    }));
                }
                idx = end;
            }

            // ── DRL / REF ─────────────────────────────────────────────────────
            // 7-byte payload, 14-bit LE unsigned.  Parsed but not emitted as a
            // MuseEvent yet — just advance the cursor to stay in sync.
            athena_tag::DRL_REF => {
                const PLEN: usize = 7;
                let end = payload_start + PLEN;
                idx = if end > data.len() { idx + 1 } else { end };
            }

            // ── IMU ───────────────────────────────────────────────────────────
            // 3 samples × (ACC[3] + GYRO[3]), 16-bit LE signed.
            athena_tag::IMU => {
                const PLEN: usize = 36;
                let end = payload_start + PLEN;
                if end > data.len() {
                    idx += 1;
                    continue;
                }
                let vals: Vec<i16> = data[payload_start..end]
                    .chunks_exact(2)
                    .map(|c| i16::from_le_bytes([c[0], c[1]]))
                    .collect();
                if vals.len() >= 6 {
                    const AS: f32 = 0.0000610352;
                    const GS: f32 = -0.0074768;
                    let acc = XyzSample {
                        x: vals[0] as f32 * AS,
                        y: vals[1] as f32 * AS,
                        z: vals[2] as f32 * AS,
                    };
                    let gyro = XyzSample {
                        x: vals[3] as f32 * GS,
                        y: vals[4] as f32 * GS,
                        z: vals[5] as f32 * GS,
                    };
                    events.push(MuseEvent::Accelerometer(ImuData {
                        sequence_id: 0,
                        samples: [acc, acc, acc],
                    }));
                    events.push(MuseEvent::Gyroscope(ImuData {
                        sequence_id: 0,
                        samples: [gyro, gyro, gyro],
                    }));
                }
                idx = end;
            }

            // ── Optical / PPG ──────────────────────────────────────────────────
            // 3 samples × 4 channels, 20-bit LE unsigned, 64 Hz.
            // Channels: 0 = ambient, 1 = infrared, 2 = red, 3 = (unused/extra)
            // Scale: raw / 32768 (matching TypeScript athena-parser.ts)
            //
            // We emit one PpgReading per channel (0..2) with 3 samples each,
            // mirroring the Classic PPG layout.
            athena_tag::OPTICAL_4CH | athena_tag::OPTICAL_8CH => {
                const PLEN: usize = 30;
                let end = payload_start + PLEN;
                if end > data.len() {
                    idx += 1;
                    continue;
                }
                let raw = parse_uint_le_bits(&data[payload_start..end], 20);
                // raw layout: 12 values = 3 samples × 4 channels (sample-major)
                // raw[s*4 + ch] = sample s, channel ch
                const NUM_SAMPLES: usize = 3;
                const NUM_CH: usize = 4;
                // Emit channels 0..2 (ambient, infrared, red); skip ch 3.
                for ch in 0..NUM_CH.min(3) {
                    let samples: Vec<u32> = (0..NUM_SAMPLES)
                        .filter_map(|s| raw.get(s * NUM_CH + ch).copied())
                        .collect();
                    events.push(MuseEvent::Ppg(PpgReading {
                        index: 0,
                        ppg_channel: ch,
                        timestamp: 0.0,
                        samples,
                    }));
                }
                idx = end;
            }

            // ── Battery ───────────────────────────────────────────────────────
            athena_tag::BATTERY => {
                const PLEN: usize = 20;
                let end = payload_start + PLEN;
                if end > data.len() {
                    idx += 1;
                    continue;
                }
                let block = &data[payload_start..end];
                if block.len() >= 2 {
                    let raw = u16::from_le_bytes([block[0], block[1]]);
                    let battery_level = (raw as f32 / 512.0).clamp(0.0, 100.0);
                    events.push(MuseEvent::Telemetry(TelemetryData {
                        sequence_id: 0,
                        battery_level,
                        fuel_gauge_voltage: 0.0,
                        temperature: 0,
                    }));
                }
                idx = end;
            }

            // ── Unknown tag: advance one byte to stay in sync ─────────────────
            _ => {
                log::debug!("Athena: unknown tag 0x{tag:02x} (type=0x{tag_type:x}) at offset {idx}");
                idx += 1;
            }
        }
    }

    events
}
