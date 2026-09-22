use std::collections::BTreeMap;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use anyhow::{anyhow, Context, Result};
use log::{debug, info, warn};
use tokio::sync::mpsc;
use uuid::Uuid;
use webbluetooth::chooser::{Candidate, FirstMatch};
use webbluetooth::stream::{select_all, BoxStream, StreamExt};
use webbluetooth::uuid::BluetoothUuid;
use webbluetooth::{
    Bluetooth, BluetoothDevice, DeviceFilter, Grant, LeScanOptions, RemoteGattCharacteristic,
    RemoteGattServer, RequestDeviceOptions,
};

use crate::parse::{
    decode_eeg_samples, parse_accelerometer, parse_athena_notification, parse_gyroscope,
    parse_ppg_reading, parse_telemetry, ControlAccumulator,
};
use crate::protocol::{
    decode_response, encode_command, ACCELEROMETER_CHARACTERISTIC, ATHENA_SENSOR_CHARACTERISTIC,
    CONTROL_CHARACTERISTIC, EEG_CHARACTERISTICS, EEG_FREQUENCY, EEG_SAMPLES_PER_READING,
    GYROSCOPE_CHARACTERISTIC, MUSE_SERVICE_UUID, PPG_CHARACTERISTICS, PPG_FREQUENCY,
    PPG_SAMPLES_PER_READING, TELEMETRY_CHARACTERISTIC,
};
use crate::types::{ControlResponse, EegReading, MuseEvent};

// ── GATT helpers ──────────────────────────────────────────────────────────────

// The adapter session is `Bluetooth::shared()`: one per process, cloned by
// every caller. It has to be shared, because a `MuseDevice` is only usable
// through the session that found it and the TUI scans with one `MuseClient`
// and connects with another.
//
// The `Uuid` constants in `crate::protocol` are passed to webbluetooth as they
// are, without conversion, through its `uuid` feature.

/// What a device request asks for: Muse headsets by name, plus access to the
/// vendor service every Muse characteristic lives under.
///
/// The service is named even though the filter matches on the name, because a
/// Web Bluetooth grant covers exactly the services a request lists and
/// `get_primary_service` refuses everything else.
fn request_options(name_prefix: &str) -> Result<RequestDeviceOptions> {
    Ok(RequestDeviceOptions::new()
        .filter(DeviceFilter::new().name_prefix(name_prefix))
        .optional_service(MUSE_SERVICE_UUID)?)
}

/// What a scan reports: advertisements from anything named like a Muse.
///
/// A scan grants nothing, so there is no service to name here — only what
/// should be reported.
fn scan_options(name_prefix: &str) -> LeScanOptions {
    LeScanOptions::new().filter(DeviceFilter::new().name_prefix(name_prefix))
}

/// What adopting a scanned device grants: the one vendor service, and no
/// manufacturer data.
fn muse_grant() -> Result<Grant> {
    Ok(Grant::new().service(MUSE_SERVICE_UUID)?)
}

/// Pick one characteristic out of a service's discovered set.
fn find_char(chars: &[RemoteGattCharacteristic], uuid: Uuid) -> Result<RemoteGattCharacteristic> {
    let want = BluetoothUuid::from(uuid);
    chars
        .iter()
        .find(|c| *c.uuid() == want)
        .cloned()
        .ok_or_else(|| anyhow!("Characteristic {uuid} not found"))
}

/// Subscribe to a characteristic and tag each notification with its UUID.
///
/// Web Bluetooth subscribes per characteristic and yields bare values, so the
/// UUID is attached here and [`stream::select_all`] merges the tagged streams
/// back into the single stream the dispatch loops read.
async fn subscribe_tagged(
    chars: &[RemoteGattCharacteristic],
    uuid: Uuid,
) -> Result<BoxStream<'static, (Uuid, Vec<u8>)>> {
    let characteristic = find_char(chars, uuid)?;
    let notifications = characteristic.start_notifications().await?;
    Ok(notifications.map(move |value| (uuid, value)).boxed())
}

// ── Timestamp helper ──────────────────────────────────────────────────────────

fn now_ms() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("system clock is before Unix epoch")
        .as_secs_f64()
        * 1000.0
}

// ── Per-channel timestamp tracker (mirrors JS getTimestamp) ──────────────────

/// Reconstructs a wall-clock timestamp for each BLE EEG/PPG notification
/// using the device's monotonic packet index.
///
/// The Muse headset does not embed absolute timestamps in its BLE packets.
/// Instead, each notification carries a 16-bit sequence counter that
/// increments by 1 per packet.  This tracker anchors the first packet to
/// `now()` and then extrapolates all subsequent timestamps from the index
/// delta and the known sample rate, mirroring the TypeScript `getTimestamp`
/// helper in `muse-utils.ts`.
///
/// Benefits over simply calling `now()` on every packet:
/// * Handles short bursts of late delivery (jitter) without skewing timestamps.
/// * Preserves the correct inter-packet spacing even when the event loop is
///   briefly busy.
/// * Correctly back-dates packets that arrive with an index already behind
///   the current anchor.
///
/// One `TimestampTracker` should be created per logical channel (electrode or
/// PPG channel) and reused for the lifetime of that connection.
struct TimestampTracker {
    /// The packet index seen in the most recently *forwarded* packet, or
    /// `None` before the first packet is received.
    last_index: Option<u16>,
    /// Wall-clock timestamp (ms since epoch) that corresponds to `last_index`,
    /// or `None` before the first packet is received.
    last_timestamp: Option<f64>,
}

impl TimestampTracker {
    /// Create a new tracker with no anchor.  The first call to [`get`] will
    /// anchor it to the current wall clock.
    fn new() -> Self {
        Self {
            last_index: None,
            last_timestamp: None,
        }
    }

    /// Return the wall-clock timestamp in milliseconds since Unix epoch for the
    /// notification identified by `event_index`.
    ///
    /// `samples_per_reading` and `frequency` are used to compute the duration
    /// of one reading: `reading_delta_ms = 1000 × samples_per_reading / frequency`.
    ///
    /// On the first call the anchor is set to `now() − reading_delta` so that
    /// the returned timestamp represents the *start* of the reading window
    /// rather than the moment the packet arrived.
    ///
    /// 16-bit counter wrap-around (0xFFFF → 0x0000) is handled by detecting
    /// when the raw delta would be larger than 0x1000 and adjusting accordingly.
    fn get(&mut self, event_index: u16, samples_per_reading: usize, frequency: f64) -> f64 {
        let reading_delta = 1000.0 * (1.0 / frequency) * samples_per_reading as f64;

        if self.last_index.is_none() || self.last_timestamp.is_none() {
            self.last_index = Some(event_index);
            self.last_timestamp = Some(now_ms() - reading_delta);
        }

        let mut idx = event_index as i32;
        let last = self.last_index.unwrap() as i32;

        // Handle 16-bit wrap-around: if the apparent backward delta is larger
        // than half the counter space, assume the counter has wrapped.
        while last - idx > 0x1000 {
            idx += 0x10000;
        }

        let ts = self.last_timestamp.unwrap();

        if idx == last {
            // Duplicate or re-delivered packet — return the existing anchor.
            ts
        } else if idx > last {
            // Normal forward progress: advance the anchor.
            let new_ts = ts + reading_delta * (idx - last) as f64;
            self.last_index = Some(event_index);
            self.last_timestamp = Some(new_ts);
            new_ts
        } else {
            // Packet arrived with an index behind the anchor (late delivery or
            // out-of-order).  Back-date without updating the anchor so future
            // packets continue to be stamped correctly.
            ts - reading_delta * (last - idx) as f64
        }
    }

    /// Reset the tracker.  The next call to [`get`] will re-anchor to the
    /// current wall clock, as if the tracker were freshly constructed.
    ///
    /// Call this when re-connecting or after a detected stream gap.
    fn reset(&mut self) {
        self.last_index = None;
        self.last_timestamp = None;
    }
}

// ── MuseDevice ────────────────────────────────────────────────────────────────

/// A Muse headset discovered during a BLE scan.
///
/// Returned by [`MuseClient::scan_all`]; pass to [`MuseClient::connect_to`]
/// to establish a streaming connection.
#[derive(Clone, Debug)]
pub struct MuseDevice {
    /// Advertised device name (e.g. `"Muse-AB12"`).
    pub name: String,
    /// Platform BLE identifier.
    /// • macOS / iOS — a per-host UUID the system assigns; the same headset
    ///   has a different one on a different machine, because Apple never
    ///   exposes the hardware address
    /// • elsewhere — the Bluetooth address (`AA:BB:CC:DD:EE:FF`)
    pub id: String,
    pub(crate) device: BluetoothDevice,
}

// ── MuseClientConfig ──────────────────────────────────────────────────────────

/// Configuration for [`MuseClient`].
///
/// The same config is used for both Classic and Athena firmware.
/// Fields that are firmware-specific are noted below.
#[derive(Debug, Clone)]
pub struct MuseClientConfig {
    /// Subscribe to the AUX (5th) EEG channel.
    ///
    /// **Classic firmware only** — changes the startup preset from `p21` to `p20`.
    /// No effect on Athena, whose eight channels already include the auxiliary
    /// inputs — see [`crate::protocol::ATHENA_EEG_CHANNEL_NAMES`].
    /// Default: `false`.
    pub enable_aux: bool,
    /// Subscribe to the optical (PPG) channels.
    ///
    /// **Classic** — changes the startup preset to `p50`, giving three
    /// channels.
    ///
    /// **Athena** — selects preset `p1044` (eight optical channels) over
    /// `p1041` (none). Athena optical data *is* decoded into
    /// [`crate::types::MuseEvent::Ppg`]; every channel the mode carries is
    /// reported, which is four, eight or sixteen depending on the preset.
    ///
    /// Default: `false`.
    pub enable_ppg: bool,
    /// BLE scan duration in seconds before giving up. Default: `15`.
    pub scan_timeout_secs: u64,
    /// Match devices whose advertised name starts with this string.
    ///
    /// The default `"Muse"` matches all known Muse models.  Use a more
    /// specific prefix (e.g. `"Muse-AB"`) to connect to a particular headset
    /// in a multi-device environment.  Default: `"Muse"`.
    pub name_prefix: String,
}

impl Default for MuseClientConfig {
    fn default() -> Self {
        Self {
            enable_aux: false,
            enable_ppg: false,
            scan_timeout_secs: 15,
            name_prefix: "Muse".into(),
        }
    }
}

// ── MuseClient ────────────────────────────────────────────────────────────────

/// BLE client for Muse EEG headsets.
///
/// Handles scanning, connecting, GATT subscription, and notification dispatch
/// for all supported Muse models.  Firmware is detected automatically at
/// connect time by checking for the presence of the Athena universal sensor
/// characteristic (`273e0013-…`):
///
/// * **Classic** (Muse 1, Muse 2, Muse S ≤ fw 3.x) — subscribes to one
///   characteristic per sensor and dispatches typed notifications.
/// * **Athena** (Muse S fw ≥ 4.x) — subscribes to the single universal
///   characteristic and demultiplexes tag-based packets.
///
/// No configuration change is needed to switch between firmware variants;
/// the same `MuseClientConfig` and event stream work for both.
pub struct MuseClient {
    config: MuseClientConfig,
}

impl MuseClient {
    pub fn new(config: MuseClientConfig) -> Self {
        Self { config }
    }

    // ── Public: scan ─────────────────────────────────────────────────────────

    /// Scan for **all** nearby Muse devices and return them.
    ///
    /// The scan runs for `config.scan_timeout_secs` seconds so that multiple
    /// devices in range can all be discovered before the function returns.
    ///
    /// Every device returned has already been granted access to the Muse
    /// service, so [`MuseClient::connect_to`] does not have to scan again.
    pub async fn scan_all(&self) -> Result<Vec<MuseDevice>> {
        let bluetooth = Bluetooth::shared();

        // Waits for the adapter's first state report — what the old macOS
        // "wait for poweredOn" polling loop was doing by hand.
        bluetooth
            .availability()
            .await
            .context("Bluetooth is not usable")?;

        info!(
            "scan_all: scanning for {} s …",
            self.config.scan_timeout_secs
        );
        let mut scan = bluetooth
            .request_le_scan(scan_options(&self.config.name_prefix))
            .await?;

        // A scan reports advertisements, and a headset advertises repeatedly;
        // keyed by id, each one is kept once.
        let mut seen: BTreeMap<String, Candidate> = BTreeMap::new();
        let window = Duration::from_secs(self.config.scan_timeout_secs);
        let _ = tokio::time::timeout(window, async {
            while let Some(candidate) = scan.next().await {
                if !seen.contains_key(&candidate.id) {
                    info!("scan_all: found {}  id={}", candidate.label(), candidate.id);
                }
                seen.insert(candidate.id.clone(), candidate);
            }
        })
        .await;
        scan.stop(); // stop the radio before connecting to anything

        // A scan grants nothing, so each sighting is adopted into a device
        // with the grant it will need once connected.
        let grant = muse_grant()?;
        let mut found = vec![];
        for (id, candidate) in seen {
            let name = candidate.label().to_owned();
            match bluetooth.adopt_candidate(&candidate, grant.clone()).await {
                Ok(device) => found.push(MuseDevice { name, id, device }),
                Err(e) => warn!("scan_all: {name} ({id}) could not be adopted: {e}"),
            }
        }
        info!("scan_all: {} device(s) found", found.len());
        Ok(found)
    }

    // ── Public: connect_to ────────────────────────────────────────────────────

    /// Connect to a specific device returned by [`MuseClient::scan_all`], subscribe to
    /// all enabled characteristics, and start the notification dispatch task.
    ///
    /// Returns an event receiver and a [`MuseHandle`] for sending commands.
    pub async fn connect_to(
        &self,
        device: MuseDevice,
    ) -> Result<(mpsc::Receiver<MuseEvent>, MuseHandle)> {
        self.setup_device(device.device, device.name).await
    }

    // ── Public: connect (convenience) ────────────────────────────────────────

    /// Scan for the first Muse device, connect, and start streaming.
    ///
    /// Equivalent to calling [`MuseClient::scan_all`] and then [`MuseClient::connect_to`]
    /// on the first result. Useful when only one headset is expected.
    pub async fn connect(&self) -> Result<(mpsc::Receiver<MuseEvent>, MuseHandle)> {
        let bluetooth = Bluetooth::shared();
        bluetooth
            .availability()
            .await
            .context("Bluetooth is not usable")?;

        info!(
            "Scanning for Muse devices (timeout: {} s) …",
            self.config.scan_timeout_secs
        );
        // `FirstMatch` returns as soon as something matching advertises, so
        // the timeout is a deadline rather than a fixed scan window.
        let chooser = FirstMatch::with_timeout(Duration::from_secs(self.config.scan_timeout_secs));
        let device = bluetooth
            .request_device_with(request_options(&self.config.name_prefix)?, &chooser)
            .await
            .map_err(|e| {
                anyhow!(
                    "No Muse device found within {} s: {e}",
                    self.config.scan_timeout_secs
                )
            })?;

        let device_name = device.name().unwrap_or_else(|| device.id().to_owned());
        info!("Found device: {device_name}");

        self.setup_device(device, device_name).await
    }

    // ── Private: setup_device ─────────────────────────────────────────────────

    /// Connect a device, subscribe to all enabled GATT characteristics, spawn
    /// the notification dispatch task, and return the event channel.
    async fn setup_device(
        &self,
        device: BluetoothDevice,
        device_name: String,
    ) -> Result<(mpsc::Receiver<MuseEvent>, MuseHandle)> {
        let gatt = device.gatt();

        // `connect()` has no deadline of its own — it waits for the peripheral
        // to come into range for as long as that takes — so one is imposed
        // here.  Ten seconds is generous for a BLE connection that usually
        // takes under two.
        tokio::time::timeout(Duration::from_secs(10), gatt.connect())
            .await
            .map_err(|_| anyhow!("BLE connect() timed out after 10 s"))??;

        // Every Muse characteristic lives under the one vendor service, so a
        // single discovery pass produces the whole set.
        let service = tokio::time::timeout(
            Duration::from_secs(15),
            gatt.get_primary_service(MUSE_SERVICE_UUID),
        )
        .await
        .map_err(|_| anyhow!("service discovery timed out after 15 s"))??;
        let chars = service.get_characteristics(None).await?;
        info!("Connected and services discovered: {device_name}");

        // ── Detect Athena firmware ─────────────────────────────────────────────
        // Athena (new Muse S) exposes a universal sensor char (0x273e0013…)
        // that carries all data in tag-based packets.  Classic Muse devices do
        // not have this characteristic.
        let athena_uuid = BluetoothUuid::from(ATHENA_SENSOR_CHARACTERISTIC);
        let is_athena = chars.iter().any(|c| *c.uuid() == athena_uuid);
        info!(
            "{device_name}: firmware detected as {}",
            if is_athena { "Athena" } else { "Classic" }
        );

        // ── Common control characteristic ──────────────────────────────────────
        let control_char = find_char(&chars, CONTROL_CHARACTERISTIC)?;
        let mut streams = vec![subscribe_tagged(&chars, CONTROL_CHARACTERISTIC).await?];

        // ── Event channel ─────────────────────────────────────────────────────
        let (tx, rx) = mpsc::channel::<MuseEvent>(256);
        let _ = tx.send(MuseEvent::Connected(device_name.clone())).await;

        // ── Disconnect watcher ──────────────────────────────────────────────
        // Fires when the BLE link drops (headset powered off, out of range,
        // and so on) — often sooner than the notification streams close.
        let disconnect_tx = tx.clone();
        let device_id = device.id().to_owned();
        let mut disconnects = device.watch_disconnect();
        tokio::spawn(async move {
            if disconnects.next().await.is_some() {
                info!("Disconnect watcher: device {device_id} disconnected.");
                let _ = disconnect_tx.send(MuseEvent::Disconnected).await;
            }
        });

        if is_athena {
            // ── Athena: subscribe to the single universal sensor characteristic ─
            streams.push(subscribe_tagged(&chars, ATHENA_SENSOR_CHARACTERISTIC).await?);
            let mut notifications = select_all(streams);

            tokio::spawn(async move {
                info!("Athena: notification stream subscribed, waiting for data…");
                let mut ctrl_acc = ControlAccumulator::new();
                let mut notif_count: u64 = 0;
                let mut sensor_count: u64 = 0;
                let mut eeg_event_count: u64 = 0;

                while let Some((uuid, data)) = notifications.next().await {
                    let data = &data[..];
                    notif_count += 1;

                    if uuid == CONTROL_CHARACTERISTIC {
                        let fragment = decode_response(data);
                        debug!("Athena control fragment: {:?}", fragment);
                        if let Some(json_str) = ctrl_acc.push(&fragment) {
                            match serde_json::from_str::<serde_json::Value>(&json_str) {
                                Ok(serde_json::Value::Object(map)) => {
                                    let _ = tx
                                        .send(MuseEvent::Control(ControlResponse {
                                            raw: json_str,
                                            fields: map,
                                        }))
                                        .await;
                                }
                                Ok(_) => {}
                                Err(e) => {
                                    warn!("Athena control JSON error: {e} | raw: {json_str}")
                                }
                            }
                        }
                        continue;
                    }

                    // All sensor data arrives on ATHENA_SENSOR_CHARACTERISTIC
                    sensor_count += 1;
                    let events = parse_athena_notification(data);
                    let n_eeg = events
                        .iter()
                        .filter(|e| matches!(e, MuseEvent::Eeg(_)))
                        .count();
                    eeg_event_count += n_eeg as u64;

                    if sensor_count <= 3 || sensor_count.is_multiple_of(500) {
                        info!(
                            "Athena sensor: notif #{notif_count} sensor #{sensor_count} \
                             uuid={} len={} events={} eeg={} (total eeg: {eeg_event_count})",
                            uuid,
                            data.len(),
                            events.len(),
                            n_eeg,
                        );
                        if sensor_count <= 3 && !data.is_empty() {
                            // Dump ALL tags found in the packet for debugging.
                            let pkt_len = data[0] as usize;
                            let mut tags = Vec::new();
                            let mut ti = 9usize; // skip 9-byte header
                            while ti < data.len() {
                                let t = data[ti];
                                tags.push(format!("0x{t:02x}@{ti}"));
                                let payload_start = ti + 1 + 4;
                                if let Some(plen) = crate::parse::athena_payload_len(t) {
                                    if payload_start + plen <= data.len() {
                                        ti = payload_start + plen;
                                    } else {
                                        break; // truncated
                                    }
                                } else if t == 0x88 {
                                    // Variable-length battery: skip to end.
                                    ti = pkt_len.min(data.len());
                                } else {
                                    ti += 1; // unknown
                                }
                            }
                            debug!("Athena sensor tags: [{}]", tags.join(", "));
                            debug!(
                                "Athena sensor raw (first 64 bytes): {:02x?}",
                                &data[..data.len().min(64)]
                            );
                        }
                    }

                    for event in events {
                        let _ = tx.send(event).await;
                    }
                }

                info!("Athena notification stream ended – device disconnected.");
                let _ = tx.send(MuseEvent::Disconnected).await;
            });
        } else {
            // ── Classic Muse: separate characteristic per sensor ───────────────
            streams.push(subscribe_tagged(&chars, TELEMETRY_CHARACTERISTIC).await?);
            streams.push(subscribe_tagged(&chars, ACCELEROMETER_CHARACTERISTIC).await?);
            streams.push(subscribe_tagged(&chars, GYROSCOPE_CHARACTERISTIC).await?);

            let num_eeg = if self.config.enable_aux { 5 } else { 4 };
            for &eeg_uuid in &EEG_CHARACTERISTICS[..num_eeg] {
                match subscribe_tagged(&chars, eeg_uuid).await {
                    Ok(s) => streams.push(s),
                    Err(e) => warn!("EEG char {eeg_uuid}: {e}"),
                }
            }

            if self.config.enable_ppg {
                for &ppg_uuid in &PPG_CHARACTERISTICS {
                    match subscribe_tagged(&chars, ppg_uuid).await {
                        Ok(s) => streams.push(s),
                        Err(e) => warn!("PPG char {ppg_uuid}: {e}"),
                    }
                }
            }

            let enable_ppg = self.config.enable_ppg;
            let enable_aux = self.config.enable_aux;
            let mut notifications = select_all(streams);

            tokio::spawn(async move {
                info!("Classic: notification stream subscribed, waiting for data…");
                let mut notif_count: u64 = 0;

                let mut eeg_ts: Vec<TimestampTracker> =
                    (0..5).map(|_| TimestampTracker::new()).collect();
                let mut ppg_ts: Vec<TimestampTracker> =
                    (0..3).map(|_| TimestampTracker::new()).collect();
                let mut ctrl_acc = ControlAccumulator::new();

                while let Some((uuid, data)) = notifications.next().await {
                    let data = &data[..];
                    notif_count += 1;
                    if notif_count <= 5 || notif_count.is_multiple_of(500) {
                        info!(
                            "Classic: notif #{notif_count} uuid={uuid} len={}",
                            data.len()
                        );
                    }

                    if uuid == CONTROL_CHARACTERISTIC {
                        let fragment = decode_response(data);
                        debug!("Control fragment: {:?}", fragment);
                        if let Some(json_str) = ctrl_acc.push(&fragment) {
                            match serde_json::from_str::<serde_json::Value>(&json_str) {
                                Ok(serde_json::Value::Object(map)) => {
                                    let _ = tx
                                        .send(MuseEvent::Control(ControlResponse {
                                            raw: json_str,
                                            fields: map,
                                        }))
                                        .await;
                                }
                                Ok(_) => {}
                                Err(e) => {
                                    warn!("Control JSON parse error: {e} | raw: {json_str}")
                                }
                            }
                        }
                        continue;
                    }

                    if uuid == TELEMETRY_CHARACTERISTIC {
                        if let Some(t) = parse_telemetry(data) {
                            let _ = tx.send(MuseEvent::Telemetry(t)).await;
                        }
                        continue;
                    }

                    if uuid == ACCELEROMETER_CHARACTERISTIC {
                        if let Some(a) = parse_accelerometer(data) {
                            let _ = tx.send(MuseEvent::Accelerometer(a)).await;
                        }
                        continue;
                    }

                    if uuid == GYROSCOPE_CHARACTERISTIC {
                        if let Some(g) = parse_gyroscope(data) {
                            let _ = tx.send(MuseEvent::Gyroscope(g)).await;
                        }
                        continue;
                    }

                    let num_eeg_chars = if enable_aux { 5 } else { 4 };
                    if let Some(electrode) = EEG_CHARACTERISTICS[..num_eeg_chars]
                        .iter()
                        .position(|&u| u == uuid)
                    {
                        if data.len() >= 2 {
                            let index = u16::from_be_bytes([data[0], data[1]]);
                            let timestamp = eeg_ts[electrode].get(
                                index,
                                EEG_SAMPLES_PER_READING,
                                EEG_FREQUENCY,
                            );
                            let samples = decode_eeg_samples(&data[2..]);
                            let _ = tx
                                .send(MuseEvent::Eeg(EegReading {
                                    index,
                                    electrode,
                                    timestamp,
                                    samples,
                                }))
                                .await;
                        }
                        continue;
                    }

                    if enable_ppg {
                        if let Some(ppg_channel) =
                            PPG_CHARACTERISTICS.iter().position(|&u| u == uuid)
                        {
                            if data.len() >= 2 {
                                let index = u16::from_be_bytes([data[0], data[1]]);
                                let timestamp = ppg_ts[ppg_channel].get(
                                    index,
                                    PPG_SAMPLES_PER_READING,
                                    PPG_FREQUENCY,
                                );
                                if let Some(reading) =
                                    parse_ppg_reading(data, ppg_channel, timestamp)
                                {
                                    let _ = tx.send(MuseEvent::Ppg(reading)).await;
                                }
                            }
                            continue;
                        }
                    }

                    debug!("Unknown notification from {uuid}");
                }

                info!("Classic notification stream ended – device disconnected.");
                let _ = tx.send(MuseEvent::Disconnected).await;
                for t in &mut eeg_ts {
                    t.reset();
                }
                for t in &mut ppg_ts {
                    t.reset();
                }
            });
        }

        let handle = MuseHandle {
            gatt,
            control_char,
            is_athena,
        };

        Ok((rx, handle))
    }
}

// ── MuseHandle ────────────────────────────────────────────────────────────────

/// A handle to an active Muse connection that lets you send control commands.
pub struct MuseHandle {
    gatt: RemoteGattServer,
    control_char: RemoteGattCharacteristic,
    /// `true` when the connected device runs the Athena firmware (new Muse S).
    pub is_athena: bool,
}

impl MuseHandle {
    /// Send a raw command string (e.g. `"h"`, `"d"`, `"p21"`).
    pub async fn send_command(&self, cmd: &str) -> Result<()> {
        let payload = encode_command(cmd);
        self.control_char
            .write_value_without_response(&payload)
            .await?;
        Ok(())
    }

    /// Pause data streaming.
    ///
    /// Sends `h` to both Classic and Athena firmware.
    pub async fn pause(&self) -> Result<()> {
        self.send_command("h").await
    }

    /// Resume data streaming after a pause.
    ///
    /// Sends both `dc001` (Athena) and `d` (Classic) when connected to an
    /// Athena device, because some Athena firmware versions (e.g. 3.x) only
    /// accept the Classic `d` command.  On Classic devices only `d` is sent.
    pub async fn resume(&self) -> Result<()> {
        if self.is_athena {
            // Try both: dc001 for newer Athena fw, d for older/transitional.
            self.send_command("dc001").await?;
            self.send_command("d").await
        } else {
            self.send_command("d").await
        }
    }

    /// Initialise the headset and begin streaming sensor data.
    ///
    /// # Classic startup sequence
    ///
    /// `h` → `s` → *preset* → `d`
    ///
    /// | `enable_ppg` | `enable_aux` | Preset |
    /// |---|---|---|
    /// | false | false | `p21` (EEG only) |
    /// | false | true  | `p20` (EEG + AUX) |
    /// | true  | —     | `p50` (EEG + PPG) |
    ///
    /// # Athena startup sequence
    ///
    /// `v4` → `s` → `h` → *preset* → `dc001` × 2 → `L1` → **2 s wait**
    ///
    /// | `enable_ppg` | Preset | Streams |
    /// |---|---|---|
    /// | true (default) | `p1041` | 8-channel EEG + 16-channel optical |
    /// | false | `p1045` | 8-channel EEG + 4-channel optical |
    ///
    /// `enable_aux` has no effect here: Athena's eight channels already
    /// include the auxiliary inputs, which is what AUX selects on Classic.
    ///
    /// **Optical cannot be switched off on Athena.** Every preset tried
    /// carries it, so `enable_ppg: false` selects the narrowest mode rather
    /// than silence. Only the Classic presets (`p21`) have none, and they drop
    /// EEG to four channels.
    ///
    /// The whole table is measurement, not documentation — every preset
    /// switched on a Muse S Athena (fw 3.1.11), counting what arrived:
    ///
    /// | preset | EEG | optical |
    /// |---|---|---|
    /// | `p1041`, `p1042` | 8ch | 16ch (`0x36`) |
    /// | `p1043`, `p1044` | 8ch | 8ch (`0x35`) |
    /// | `p1045`, `p1046` | 8ch | 4ch (`0x34`) |
    /// | `p1034` | 4ch | 8ch (`0x35`) |
    /// | `p1035` | 4ch | 4ch (`0x34`) |
    /// | `p21` | 4ch | none |
    ///
    /// Per-channel rate is the same across optical modes — 16 channels carry
    /// one sample per packet where 8 carry two — so the wider mode is more
    /// data rather than the same data rearranged. The previous default,
    /// `p1045`, asked for the *narrowest* optical mode of the three.
    ///
    /// The 2-second wait at the end is required by the Athena firmware before
    /// packets actually start flowing.
    pub async fn start(&self, enable_ppg: bool, enable_aux: bool) -> Result<()> {
        if self.is_athena {
            // Athena startup — mirrors MuseAthenaClient.start() in muse-jsx.
            //
            // Some Athena firmware versions (e.g. Muse S fw 3.x on
            // Athena_RevE hardware) reject `dc001` (rc:69) but accept the
            // Classic `d` command.  We send both so that streaming starts
            // regardless of firmware version.  The device silently ignores
            // whichever command it does not understand.
            let delay = |ms| tokio::time::sleep(Duration::from_millis(ms));
            self.send_command("v4").await?;
            delay(100).await;
            self.send_command("s").await?;
            delay(100).await;
            self.send_command("h").await?;
            delay(100).await;
            // The richest mode the device offers, rather than a fixed preset:
            // `p1041` carries sixteen optical channels where `p1045` carries
            // four, at the same per-channel rate.
            let preset = if enable_ppg { "p1041" } else { "p1045" };
            self.send_command(preset).await?;
            delay(100).await;
            // Athena data-start: send dc001 twice (per TypeScript reference),
            // then also send Classic `d` as fallback for fw 3.x which rejects
            // dc001 with rc:69.
            self.send_command("dc001").await?;
            delay(50).await;
            self.send_command("dc001").await?;
            delay(50).await;
            self.send_command("d").await?;
            delay(100).await;
            self.send_command("L1").await?;
            // Firmware needs ~2 s to settle before packets arrive.
            delay(2100).await;
            Ok(())
        } else {
            // Classic startup
            self.pause().await?;
            let preset = if enable_ppg {
                "p50"
            } else if enable_aux {
                "p20"
            } else {
                "p21"
            };
            self.send_command("s").await?;
            self.send_command(preset).await?;
            self.resume().await?;
            Ok(())
        }
    }

    /// Request firmware / hardware info (`v1` command).
    pub async fn request_device_info(&self) -> Result<()> {
        self.send_command("v1").await
    }

    /// Check if the peripheral is still connected at the BLE adapter level.
    /// Useful for implementing a connection watchdog — poll this periodically
    /// to detect disconnects faster than waiting for the notification stream
    /// to close.
    pub async fn is_connected(&self) -> bool {
        self.gatt.connected()
    }

    /// Gracefully disconnect.
    pub async fn disconnect(&self) -> Result<()> {
        self.gatt.disconnect();
        Ok(())
    }
}
