/// An EEG reading — one BLE notification from a single electrode.
///
/// # Classic firmware (Muse 1 / Muse 2 / Muse S ≤ fw 3.x)
/// One notification per channel at 256 Hz, carrying **12 samples** each
/// (≈ 46.9 ms of signal per packet).  Samples are decoded from 12-bit
/// big-endian packed values and scaled by 0.48828125 µV/LSB.
///
/// # Athena firmware (Muse S fw ≥ 4.x)
/// All channels arrive on a single universal characteristic.  Each notification
/// is split into per-channel `EegReading`s carrying **2 samples** decoded from
/// 14-bit little-endian packed values and scaled by 0.0885 µV/LSB.
/// Channels 4–7 (FPz, AUX\_R, AUX\_L, AUX) are only produced by Athena hardware.
#[derive(Debug, Clone)]
pub struct EegReading {
    /// Sequential packet index emitted by the headset (wraps at 0xFFFF).
    ///
    /// Used to reconstruct timestamps for Classic firmware.
    /// Always `0` for Athena (Athena notifications do not carry a per-channel index).
    pub index: u16,
    /// Electrode channel index.
    ///
    /// **Classic (channels 0–4):**
    /// * 0 = TP9 (left rear)
    /// * 1 = AF7 (left front)
    /// * 2 = AF8 (right front)
    /// * 3 = TP10 (right rear)
    /// * 4 = AUX (optional, `enable_aux: true` required)
    ///
    /// **Athena (channels 0–7):** same 0–3 as above, plus:
    /// * 4 = FPz (frontal midline)
    /// * 5 = AUX\_R
    /// * 6 = AUX\_L
    /// * 7 = AUX
    pub electrode: usize,
    /// Wall-clock timestamp in milliseconds since Unix epoch for the *first*
    /// sample in this packet.
    ///
    /// **Classic:** extrapolated from `index` and the known sample rate;
    /// accurate even under BLE jitter.
    ///
    /// **Athena:** always `0.0` — Athena packets do not carry a per-channel
    /// index, so timestamp reconstruction is not possible with current data.
    pub timestamp: f64,
    /// Voltage samples in µV.
    ///
    /// **Classic:** 12 samples per packet at 256 Hz.  
    /// **Athena:** 2 samples per packet at 256 Hz.
    pub samples: Vec<f64>,
}

/// A PPG (photoplethysmography) reading from the optical heart-rate sensor.
///
/// Available on Muse 2 and Muse S only. Each notification carries 6 raw
/// 24-bit samples at 64 Hz.
#[derive(Debug, Clone)]
pub struct PpgReading {
    /// Sequential packet index (wraps at 0xFFFF), same purpose as [`EegReading::index`].
    pub index: u16,
    /// Optical channel:
    /// * 0 = ambient (background light subtraction)
    /// * 1 = infrared
    /// * 2 = red
    pub ppg_channel: usize,
    /// Wall-clock timestamp in milliseconds since Unix epoch for the first sample.
    pub timestamp: f64,
    /// Raw 24-bit ADC values (not scaled to physical units).
    /// 6 samples per notification at 64 Hz.
    pub samples: Vec<u32>,
}

/// Battery and housekeeping telemetry packet.
///
/// Sent by the headset roughly once per second on both Classic and Athena
/// firmware, though the wire format differs:
///
/// | Field | Classic | Athena |
/// |---|---|---|
/// | `sequence_id` | from packet | always `0` |
/// | `battery_level` | u16 BE ÷ 512 | u16 LE ÷ 512 |
/// | `fuel_gauge_voltage` | u16 BE × 2.2 mV | always `0.0` |
/// | `temperature` | u16 BE raw ADC | always `0` |
#[derive(Debug, Clone)]
pub struct TelemetryData {
    /// Monotonically increasing packet counter (wraps at 0xFFFF).
    /// Always `0` for Athena notifications.
    pub sequence_id: u16,
    /// Battery state-of-charge in percent (0–100).
    /// Derived from the raw fuel-gauge reading divided by 512.
    pub battery_level: f32,
    /// Fuel-gauge terminal voltage in millivolts (Classic only).
    /// Raw reading multiplied by 2.2.  Always `0.0` for Athena.
    pub fuel_gauge_voltage: f32,
    /// Raw ADC temperature value (not converted to °C).
    /// Always `0` for Athena (field not present in Athena battery packets).
    pub temperature: u16,
}

/// A single 3-axis inertial measurement.
#[derive(Debug, Clone, Copy)]
pub struct XyzSample {
    /// X-axis value in sensor-specific units (g for accelerometer, °/s for gyroscope).
    pub x: f32,
    /// Y-axis value.
    pub y: f32,
    /// Z-axis value.
    pub z: f32,
}

/// A batch of inertial measurements from one BLE notification.
///
/// Both the accelerometer and gyroscope fire at ≈ 52 Hz and carry 3 XYZ
/// samples per notification on Classic firmware.  Athena packs 3 samples
/// as well but only the first is currently forwarded (indices 1 and 2 are
/// copies of index 0 in the Athena decoder).
///
/// # Wire format differences
///
/// | Property | Classic | Athena |
/// |---|---|---|
/// | Integer type | `i16` big-endian | `i16` little-endian |
/// | Accel scale | +0.0000610352 g/LSB | +0.0000610352 g/LSB |
/// | Gyro scale | +0.0074768 °/s/LSB | −0.0074768 °/s/LSB (negated) |
/// | `sequence_id` | from packet | always `0` |
#[derive(Debug, Clone)]
pub struct ImuData {
    /// Monotonically increasing packet counter (wraps at 0xFFFF).
    /// Always `0` for Athena notifications.
    pub sequence_id: u16,
    /// Three consecutive XYZ samples; index 0 is the oldest.
    /// For Athena, indices 1 and 2 are currently copies of index 0.
    pub samples: [XyzSample; 3],
}

/// A time-aligned multi-channel EEG snapshot.
///
/// Produced by code that zips per-electrode [`EegReading`]s together so every
/// channel has a value for the same timestamp.
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct EegSample {
    /// Packet index from the electrode that completed this sample set.
    pub index: u16,
    /// Wall-clock timestamp in milliseconds since Unix epoch.
    pub timestamp: f64,
    /// Voltage in µV for each channel in order `[TP9, AF7, AF8, TP10, AUX]`.
    /// Channels not yet received are `f64::NAN`.
    pub data: Vec<f64>,
}

/// A parsed Muse control/status response decoded from the control characteristic.
///
/// The headset replies to `v1` (device info), `s` (status), and similar
/// commands with a JSON object split across several BLE packets.
/// [`crate::parse::ControlAccumulator`] reassembles the fragments; this struct
/// carries the final result.
#[derive(Debug, Clone)]
pub struct ControlResponse {
    /// The raw, un-parsed JSON string.
    pub raw: String,
    /// Key-value pairs from the parsed JSON object.
    pub fields: serde_json::Map<String, serde_json::Value>,
}

/// All data events emitted by [`crate::muse_client::MuseClient`].
///
/// Consumers receive these values through the `mpsc::Receiver` returned by
/// [`crate::muse_client::MuseClient::connect`] or
/// [`crate::muse_client::MuseClient::connect_to`].
///
/// # Firmware availability
///
/// | Variant | Classic | Athena |
/// |---|---|---|
/// | `Eeg` (ch 0–3) | ✓ | ✓ |
/// | `Eeg` (ch 4–7) | ✗ | ✓ |
/// | `Ppg` | ✓ (opt-in) | ✗ |
/// | `Accelerometer` | ✓ | ✓ |
/// | `Gyroscope` | ✓ | ✓ |
/// | `Telemetry` | ✓ | ✓ |
/// | `Control` | ✓ | ✓ |
#[derive(Debug, Clone)]
pub enum MuseEvent {
    /// An EEG packet from one electrode channel.
    ///
    /// Produced for channels 0–3 on both Classic and Athena.
    /// Channels 4–7 (FPz, AUX\_R, AUX\_L, AUX) are Athena-only.
    /// See [`EegReading`] for per-firmware format differences.
    Eeg(EegReading),
    /// A PPG (photoplethysmography) optical packet.
    ///
    /// Classic requires `enable_ppg: true` in [`crate::muse_client::MuseClientConfig`].
    /// Athena optical data is always included with preset `p1045`.
    Ppg(PpgReading),
    /// Battery and housekeeping telemetry (~1 Hz).
    ///
    /// Produced by both Classic and Athena; some fields are `0` on Athena.
    /// See [`TelemetryData`] for per-firmware field availability.
    Telemetry(TelemetryData),
    /// Accelerometer batch (~52 Hz).
    ///
    /// Produced by both Classic and Athena.  The gyroscope sign convention and
    /// byte order differ between firmwares; see [`ImuData`] for details.
    Accelerometer(ImuData),
    /// Gyroscope batch (~52 Hz).
    ///
    /// Produced by both Classic and Athena.  Athena gyro values are negated
    /// relative to Classic (−0.0074768 vs +0.0074768 °/s/LSB).
    Gyroscope(ImuData),
    /// A complete JSON control/status response from the headset.
    ///
    /// Produced by both Classic and Athena in response to commands such as
    /// `v1` (device info) or `s` (status).
    Control(ControlResponse),
    /// The BLE link has been established and GATT services discovered.
    /// The inner `String` is the advertised device name (e.g. `"Muse-AB12"`).
    Connected(String),
    /// The BLE link was lost (headset turned off, out of range, etc.).
    ///
    /// After receiving this event the channel will be closed; no further
    /// events will arrive.  The TUI and CLI both restart scanning automatically
    /// when this event is seen.
    Disconnected,
}
