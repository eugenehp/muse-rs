//! # muse-rs
//!
//! Async Rust library and terminal UI for streaming EEG data from
//! [Interaxon Muse](https://choosemuse.com/) headsets over Bluetooth Low Energy.
//!
//! ## Supported hardware
//!
//! | Model | Firmware | EEG ch | PPG | Notes |
//! |---|---|---|---|---|
//! | Muse 1 (2014) | Classic | 4 | ✗ | baseline feature set |
//! | Muse 2 | Classic | 4 + AUX | ✓ | PPG requires `enable_ppg: true` |
//! | Muse S | Classic | 4 + AUX | ✓ | same protocol as Muse 2 |
//! | Muse S | **Athena** | **8** | ✗† | auto-detected; different wire format |
//!
//! † Athena optical data is received but not yet decoded into `MuseEvent::Ppg`.
//!
//! Firmware is detected automatically at connect time — no configuration is
//! required.  See the [README](https://github.com/eugenehp/muse-rs#firmware-variants-classic-vs-athena)
//! for a full protocol comparison.
//!
//! ## Quick start
//!
//! ```no_run
//! use muse_rs::prelude::*;
//!
//! #[tokio::main]
//! async fn main() -> anyhow::Result<()> {
//!     let client = MuseClient::new(MuseClientConfig::default());
//!     let (mut rx, handle) = client.connect().await?;
//!     handle.start(false, false).await?;
//!
//!     while let Some(event) = rx.recv().await {
//!         match event {
//!             MuseEvent::Eeg(r) => println!("EEG ch{}: {:?}", r.electrode, r.samples),
//!             MuseEvent::Disconnected => break,
//!             _ => {}
//!         }
//!     }
//!     Ok(())
//! }
//! ```
//!
//! ## Using as a library dependency
//!
//! Add to your `Cargo.toml`:
//!
//! ```toml
//! [dependencies]
//! # Full build (includes the ratatui TUI feature):
//! muse-rs = "0.0.1"
//!
//! # Library only — skips ratatui / crossterm compilation:
//! muse-rs = { version = "0.0.1", default-features = false }
//! ```
//!
//! ## Module overview
//!
//! | Module | Purpose |
//! |---|---|
//! | [`prelude`] | One-line glob import of the most commonly needed types |
//! | [`muse_client`] | BLE scanning, connecting, and the [`muse_client::MuseHandle`] command API |
//! | [`types`] | All event and data types produced by the client |
//! | [`protocol`] | GATT UUIDs, sampling constants, and BLE wire-format helpers |
//! | [`parse`] | Low-level byte-to-sample decoders for EEG, IMU, PPG, and Athena packets |

pub mod muse_client;
pub mod parse;
pub mod protocol;
pub mod types;

// ── Prelude ───────────────────────────────────────────────────────────────────

/// Convenience re-exports for downstream crates.
///
/// A single glob import covers the entire surface area needed to scan,
/// connect, and process events from a Muse headset:
///
/// ```no_run
/// use muse_rs::prelude::*;
///
/// # #[tokio::main]
/// # async fn main() -> anyhow::Result<()> {
/// let devices = MuseClient::new(MuseClientConfig::default()).scan_all().await?;
/// let (mut rx, handle) = MuseClient::new(MuseClientConfig::default())
///     .connect_to(devices.into_iter().next().unwrap()).await?;
/// handle.start(false, false).await?;
///
/// while let Some(ev) = rx.recv().await {
///     if let MuseEvent::Eeg(r) = ev {
///         println!("{:?}", r.samples);
///     }
/// }
/// # Ok(())
/// # }
/// ```
pub mod prelude {
    // ── Client ────────────────────────────────────────────────────────────────
    pub use crate::muse_client::{MuseClient, MuseClientConfig, MuseDevice, MuseHandle};

    // ── Events and data types ─────────────────────────────────────────────────
    pub use crate::types::{
        ControlResponse, EegReading, EegSample, ImuData, MuseEvent, PpgReading, TelemetryData,
        XyzSample,
    };

    // ── Protocol constants ────────────────────────────────────────────────────
    pub use crate::protocol::{
        EEG_CHANNEL_NAMES, EEG_FREQUENCY, EEG_SAMPLES_PER_READING, PPG_CHANNEL_NAMES,
        PPG_FREQUENCY, PPG_SAMPLES_PER_READING,
    };
}
