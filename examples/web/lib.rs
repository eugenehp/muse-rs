//! A web interface for a Muse headset: the same library the TUI drives, in a
//! browser, over the browser's own Web Bluetooth.
//!
//! Build it with `examples/web/build.sh`, which also lays out a directory you
//! can serve. `examples/web/README.md` has the details.
//!
//! # Shape of it
//!
//! There is no `wasm-bindgen` here, because `webbluetooth-wasm` does not use
//! one: the boundary is plain numbers across a hand-written shim, and this
//! example crosses it the same way.
//!
//! - Out of the page, into this module: [`muse_connect`], the one export, which
//!   the Connect button calls.
//! - Out of this module, into the page: the `muse` import namespace below —
//!   status text, EEG samples, battery. `app.js` supplies them.
//! - Everything Bluetooth: `webbluetooth`'s own `webbluetooth` namespace, which
//!   the shim fills in. This file never touches it.
//!
//! # Native builds are empty
//!
//! The target is `wasm32-unknown-unknown`. `cargo build --examples` on a native
//! target still builds this file, so the browser half is behind a `cfg` and
//! what remains natively is an empty library — the same arrangement as the
//! binaries, for the same reason.

#[cfg(target_arch = "wasm32")]
mod web {
    use muse_rs::muse_client::{MuseClient, MuseClientConfig};
    use muse_rs::protocol::{eeg_channel_name, EEG_FREQUENCY};
    use muse_rs::types::MuseEvent;

    // What the page provides. (A plain comment, not a doc comment: rustdoc
    // does not document extern blocks.)
    //
    // `#[link(wasm_import_module)]` names the namespace these arrive in, which
    // is what `app.js` passes to `instantiate` as its extra imports. Without
    // it they would land in `env`, where the shim puts nothing and the page
    // supplies nothing, and instantiation would fail on a missing import.
    #[link(wasm_import_module = "muse")]
    unsafe extern "C" {
        fn muse_status(ptr: *const u8, len: usize);
        fn muse_log(level: u32, ptr: *const u8, len: usize);
        fn muse_row(kind: u32, index: u32, hz: f64, ptr: *const u8, len: usize);
        fn muse_sample(electrode: u32, microvolts: f64);
        fn muse_imu(sensor: u32, x: f64, y: f64, z: f64);
        fn muse_battery(percent: f64);
        fn muse_active(active: u32);
    }

    /// Sends the library's `log` records to the page's console.
    ///
    /// Without this a browser build is silent. `env_logger` is native-only —
    /// it reads `RUST_LOG` and writes to stderr, and a page has neither — so
    /// every `info!` in `muse_client`, including the notification counters
    /// that say whether packets are arriving at all, goes nowhere. The
    /// console is the equivalent surface, and the volume is safe: the pumps
    /// log the first few notifications and then every five hundredth.
    struct PageLog;

    impl log::Log for PageLog {
        fn enabled(&self, _: &log::Metadata<'_>) -> bool {
            true
        }

        fn log(&self, record: &log::Record<'_>) {
            let line = format!("{}: {}", record.target(), record.args());
            // `Level` is 1..=5, error through trace; `app.js` maps it to the
            // matching console method.
            unsafe { muse_log(record.level() as u32, line.as_ptr(), line.len()) }
        }

        fn flush(&self) {}
    }

    static PAGE_LOG: PageLog = PageLog;

    /// Which IMU sensor a triple came from. Numbers rather than a string per
    /// sample: this crosses the boundary 156 times a second per sensor.
    const ACCELEROMETER: u32 = 0;
    const GYROSCOPE: u32 = 1;

    /// Which family a row belongs to, so the page can pair a sample with the
    /// row that was declared for it.
    const KIND_EEG: u32 = 0;
    const KIND_IMU: u32 = 1;

    /// Both IMU characteristics fire at about this rate — see the notes on
    /// `GYROSCOPE_CHARACTERISTIC` and `ImuData`. Unlike `EEG_FREQUENCY` the
    /// protocol module has no constant for it, so it is written down here.
    const IMU_FREQUENCY: f64 = 52.0;

    /// Tell the page a row exists, what to call it, and how fast it arrives.
    ///
    /// The rate is the point: EEG is 256 Hz and the IMU 52, so a view that
    /// gave every row the same number of samples would show five times as
    /// much history for the IMU as for the electrodes, on one x axis. The page
    /// sizes each row's buffer from this, and every row then covers the same
    /// span of time.
    fn declare_row(kind: u32, index: u32, hz: f64, name: &str) {
        unsafe { muse_row(kind, index, hz, name.as_ptr(), name.len()) }
    }

    /// Tell the page what is happening, in words it can put on screen.
    fn status(message: &str) {
        // The page copies the bytes out before it returns and never keeps the
        // pointer — see `withBuffer` in the shim for the same discipline on
        // the other side of the boundary.
        unsafe { muse_status(message.as_ptr(), message.len()) }
    }

    /// Connect to a headset and stream from it. The page's only entry point.
    ///
    /// Has to be reached from a click, not from page load: `requestDevice`
    /// opens the browser's own chooser, and browsers only allow that during a
    /// user gesture. Returning straight away matters for the same reason — the
    /// work goes onto the page's event loop rather than being awaited here, so
    /// the chooser opens inside the window the click grants.
    #[no_mangle]
    pub extern "C" fn muse_connect() {
        // Once per page. A second press is an `Err` rather than a panic, which
        // is why the result is discarded.
        let _ = log::set_logger(&PAGE_LOG);
        log::set_max_level(log::LevelFilter::Info);

        // The library's own spawn is internal, and a browser has no tokio
        // runtime to hand this to. `exec` is the executor the BLE calls are
        // already being polled by.
        webbluetooth_wasm::exec::spawn(async {
            // Bracketing the whole task, so the button comes back whether this
            // ended in a disconnection or an error.
            unsafe { muse_active(1) };
            if let Err(error) = stream().await {
                // `{:#}` rather than `{}`: the context chain is where the
                // useful half of a BLE failure is written down.
                status(&format!("{error:#}"));
            }
            unsafe { muse_active(0) };
        });
    }

    async fn stream() -> anyhow::Result<()> {
        status("Opening the browser's device chooser…");

        // Read the flags back off the config rather than repeating them, so
        // that changing one changes both the subscription and the startup
        // preset. Getting those out of step is how a channel ends up
        // subscribed but never sent.
        let config = MuseClientConfig::default();
        let (enable_ppg, enable_aux) = (config.enable_ppg, config.enable_aux);

        let (mut events, handle) = MuseClient::new(config).connect().await?;
        let is_athena = handle.is_athena;
        status("Connected. Starting the stream…");
        handle.start(enable_ppg, enable_aux).await?;
        // Declared up front rather than on first packet: the IMU rows should
        // hold their place in the view from the start, and a headset that is
        // sitting still still has axes worth showing.
        declare_row(KIND_IMU, ACCELEROMETER, IMU_FREQUENCY, "accel");
        declare_row(KIND_IMU, GYROSCOPE, IMU_FREQUENCY, "gyro");

        status("Streaming.");

        // Ends when the headset disconnects: the sending half is dropped with
        // the notification pump, which closes the channel.
        // Which electrodes have been declared to the page already. Eight
        // covers both firmwares; anything beyond it simply goes undeclared
        // rather than panicking on a channel a later firmware invents.
        let mut declared = [false; 8];

        while let Some(event) = events.recv().await {
            match event {
                MuseEvent::Eeg(reading) => {
                    let electrode = reading.electrode;

                    // Declare the row the first time the electrode appears,
                    // naming it from the firmware-aware table: index 4 is AUX
                    // on Classic and FPz on Athena, so a page left to guess
                    // would label one of the two wrongly.
                    if let Some(declared) = declared.get_mut(electrode) {
                        if !*declared {
                            *declared = true;
                            let label = eeg_channel_name(electrode, is_athena);
                            declare_row(KIND_EEG, electrode as u32, EEG_FREQUENCY, label);
                        }
                    }

                    for microvolts in reading.samples {
                        unsafe { muse_sample(electrode as u32, microvolts) }
                    }
                }
                // Both IMU packets carry three XYZ samples. The page draws a
                // sensor per row with the axes overlaid, so all it needs is
                // which sensor a triple came from.
                MuseEvent::Accelerometer(imu) => {
                    for sample in imu.samples {
                        unsafe {
                            muse_imu(
                                ACCELEROMETER,
                                f64::from(sample.x),
                                f64::from(sample.y),
                                f64::from(sample.z),
                            )
                        }
                    }
                }
                MuseEvent::Gyroscope(imu) => {
                    for sample in imu.samples {
                        unsafe {
                            muse_imu(
                                GYROSCOPE,
                                f64::from(sample.x),
                                f64::from(sample.y),
                                f64::from(sample.z),
                            )
                        }
                    }
                }
                MuseEvent::Telemetry(telemetry) => unsafe {
                    muse_battery(f64::from(telemetry.battery_level))
                },
                // Optical arrives decoded too; this page does not draw it.
                // Adding it is a `match` arm and a row, not any more protocol
                // work.
                _ => {}
            }
        }

        status("Disconnected.");
        Ok(())
    }
}
