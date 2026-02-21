//! GATT UUIDs, sampling constants, and BLE wire-format helpers for Muse headsets.
//!
//! All UUIDs belong to the Muse vendor namespace
//! `273eXXXX-4c4d-454d-96be-f03bac821358`.

use uuid::Uuid;

// ── Service ──────────────────────────────────────────────────────────────────

/// Primary GATT service UUID advertised by all Muse devices.
///
/// Used as a scan filter to identify Muse headsets among nearby BLE peripherals.
#[allow(dead_code)]
pub const MUSE_SERVICE_UUID: Uuid = Uuid::from_u128(0x0000fe8d_0000_1000_8000_00805f9b34fb);

// ── Characteristics ───────────────────────────────────────────────────────────

/// Bidirectional control channel.
///
/// The host writes length-prefixed ASCII commands (e.g. `"d"`, `"h"`, `"p21"`)
/// and receives JSON status fragments in response notifications.
/// See [`encode_command`] and [`decode_response`] for the wire format.
pub const CONTROL_CHARACTERISTIC: Uuid =
    Uuid::from_u128(0x273e0001_4c4d_454d_96be_f03bac821358);

/// Telemetry characteristic (battery level, temperature, fuel-gauge voltage).
///
/// Notified approximately once per second on classic Muse devices.
pub const TELEMETRY_CHARACTERISTIC: Uuid =
    Uuid::from_u128(0x273e000b_4c4d_454d_96be_f03bac821358);

/// Gyroscope characteristic — 3 × XYZ samples per notification at ~52 Hz.
pub const GYROSCOPE_CHARACTERISTIC: Uuid =
    Uuid::from_u128(0x273e0009_4c4d_454d_96be_f03bac821358);

/// Accelerometer characteristic — 3 × XYZ samples per notification at ~52 Hz.
pub const ACCELEROMETER_CHARACTERISTIC: Uuid =
    Uuid::from_u128(0x273e000a_4c4d_454d_96be_f03bac821358);

/// EEG per-channel characteristics, indexed by electrode:
///
/// | Index | UUID suffix | Electrode |
/// |-------|-------------|-----------|
/// | 0     | 0003        | TP9       |
/// | 1     | 0004        | AF7       |
/// | 2     | 0005        | AF8       |
/// | 3     | 0006        | TP10      |
/// | 4     | 0007        | AUX       |
///
/// Subscribe to the first 4 for standard 4-channel EEG; add index 4 for the
/// auxiliary input (enabled via [`crate::muse_client::MuseClientConfig::enable_aux`]).
pub const EEG_CHARACTERISTICS: [Uuid; 5] = [
    Uuid::from_u128(0x273e0003_4c4d_454d_96be_f03bac821358), // TP9
    Uuid::from_u128(0x273e0004_4c4d_454d_96be_f03bac821358), // AF7
    Uuid::from_u128(0x273e0005_4c4d_454d_96be_f03bac821358), // AF8
    Uuid::from_u128(0x273e0006_4c4d_454d_96be_f03bac821358), // TP10
    Uuid::from_u128(0x273e0007_4c4d_454d_96be_f03bac821358), // AUX
];

/// Athena (new Muse S firmware) universal sensor characteristic.
///
/// All sensor data (EEG, IMU, optical, battery) is multiplexed on this single
/// characteristic using tag-based binary packets.  Classic Muse devices do not
/// expose this characteristic; its presence is used as the Athena detector in
/// [`crate::muse_client::MuseClient`].
pub const ATHENA_SENSOR_CHARACTERISTIC: Uuid =
    Uuid::from_u128(0x273e0013_4c4d_454d_96be_f03bac821358);

/// PPG (optical heart-rate) characteristics, indexed by channel:
///
/// | Index | UUID suffix | Channel    |
/// |-------|-------------|------------|
/// | 0     | 000f        | ambient    |
/// | 1     | 0010        | infrared   |
/// | 2     | 0011        | red        |
///
/// Available on Muse 2 and Muse S only.  Enable via
/// [`crate::muse_client::MuseClientConfig::enable_ppg`].
pub const PPG_CHARACTERISTICS: [Uuid; 3] = [
    Uuid::from_u128(0x273e000f_4c4d_454d_96be_f03bac821358), // ambient
    Uuid::from_u128(0x273e0010_4c4d_454d_96be_f03bac821358), // infrared
    Uuid::from_u128(0x273e0011_4c4d_454d_96be_f03bac821358), // red
];

// ── Sampling constants ────────────────────────────────────────────────────────

/// EEG sample rate in Hz (256 samples per second per channel).
pub const EEG_FREQUENCY: f64 = 256.0;

/// Number of EEG voltage samples packed into one classic Muse BLE notification.
///
/// At 256 Hz, 12 samples represent ≈ 46.9 ms of signal per packet.
pub const EEG_SAMPLES_PER_READING: usize = 12;

/// PPG sample rate in Hz (64 samples per second per optical channel).
pub const PPG_FREQUENCY: f64 = 64.0;

/// Number of raw 24-bit PPG values packed into one BLE notification.
///
/// At 64 Hz, 6 samples represent ≈ 93.75 ms of signal per packet.
pub const PPG_SAMPLES_PER_READING: usize = 6;

// ── Human-readable labels ─────────────────────────────────────────────────────

/// Electrode names in [`EEG_CHARACTERISTICS`] index order.
///
/// Index 4 (`"AUX"`) is only populated when
/// [`crate::muse_client::MuseClientConfig::enable_aux`] is `true`.
pub const EEG_CHANNEL_NAMES: [&str; 5] = ["TP9", "AF7", "AF8", "TP10", "AUX"];

/// Optical channel names in [`PPG_CHARACTERISTICS`] index order.
pub const PPG_CHANNEL_NAMES: [&str; 3] = ["ambient", "infrared", "red"];

// ── Athena-specific constants ─────────────────────────────────────────────────

/// Number of EEG channels carried in a single Athena EEG notification.
///
/// Channel order: TP9, AF7, AF8, TP10, FPz, AUX\_R, AUX\_L, AUX.
/// Only the first 4 map to the standard Muse electrode positions shared with
/// Classic firmware.  Channels 4–7 are Athena-only extended inputs.
pub const ATHENA_EEG_CHANNELS: usize = 8;

/// EEG samples per channel per Athena notification (14-bit LE, channel-major).
///
/// Each notification contains `ATHENA_EEG_CHANNELS × ATHENA_EEG_SAMPLES_PER_PKT`
/// = 16 values packed into 28 bytes (16 × 14 bits = 224 bits = 28 bytes exactly).
///
/// Compare to Classic firmware: 12 samples per notification per channel using
/// 12-bit big-endian packing (18 bytes of payload per characteristic).
pub const ATHENA_EEG_SAMPLES_PER_PKT: usize = 2;

/// EEG voltage scale factor for Athena firmware in µV per raw LSB.
///
/// `µV = (raw₁₄ − 8192) × ATHENA_EEG_SCALE`
///
/// Contrast with Classic firmware which uses a 12-bit ADC centred at 2048
/// with a scale of 0.48828125 µV/LSB.  The two scales differ by ~5.5×,
/// reflecting the different ADC resolution and gain settings.
pub const ATHENA_EEG_SCALE: f64 = 0.0885;

// ── Control commands ──────────────────────────────────────────────────────────

/// Encode a text command for the Muse control characteristic.
///
/// The Muse expects a length-prefixed frame:
/// ```text
/// byte 0     : payload length (= command.len() + 1 for the trailing '\n')
/// bytes 1..N : ASCII command string
/// byte N+1   : '\n' terminator
/// ```
///
/// This is equivalent to the TypeScript `encodeCommand` in `muse-utils.ts`.
///
/// # Example
///
/// ```
/// # use muse_rs::protocol::encode_command;
/// assert_eq!(encode_command("d"), &[0x02, b'd', b'\n']);
/// ```
pub fn encode_command(cmd: &str) -> Vec<u8> {
    // Build "X{cmd}\n", then replace the leading 'X' with (total_len - 1).
    let body = format!("X{cmd}\n");
    let mut bytes = body.into_bytes();
    bytes[0] = (bytes.len() - 1) as u8;
    bytes
}

/// Decode a raw BLE notification from the control characteristic into a string.
///
/// The headset uses the same length-prefix framing for responses:
/// ```text
/// byte 0     : payload length
/// bytes 1..N : UTF-8 (or lossy) response fragment
/// ```
///
/// Partial or multi-packet JSON responses should be assembled with
/// [`crate::parse::ControlAccumulator`].
///
/// Returns an empty string if `bytes` is empty.
pub fn decode_response(bytes: &[u8]) -> String {
    if bytes.is_empty() {
        return String::new();
    }
    let len = bytes[0] as usize;
    let end = (1 + len).min(bytes.len());
    String::from_utf8_lossy(&bytes[1..end]).into_owned()
}
