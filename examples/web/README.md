# Web interface

The same library the TUI drives, running in a browser over the browser's own
Web Bluetooth. Connect a headset from a button, watch the electrodes scroll.

Nothing is re-implemented here. The decoding, the firmware detection, the
Athena-vs-Classic electrode mapping — all of it is the `muse-rs` library,
compiled to `wasm32-unknown-unknown`. This example is a canvas and about a
hundred lines of glue.

## Running it

```sh
examples/web/build.sh
python3 -m http.server --directory examples/web/dist 8000
```

Then open <http://localhost:8000/> and press **Connect a headset**.

Two requirements that are not this example's choice to make:

- **Chrome or Edge.** Safari and Firefox do not implement Web Bluetooth. The
  page says so rather than failing quietly.
- **`localhost` or HTTPS.** Web Bluetooth needs a secure context, so a
  `file://` URL will not do — hence the server above.

You will also need the `wasm32-unknown-unknown` target:

```sh
rustup target add wasm32-unknown-unknown
```

## What `build.sh` produces

`dist/` is generated and git-ignored:

| File | Where it comes from |
|---|---|
| `web.wasm` | `cargo build --example web --target wasm32-unknown-unknown` |
| `index.html`, `app.js`, `render.js` | copied from this directory |
| `webbluetooth.js` | copied out of the resolved `webbluetooth-wasm` crate |

The shim is deliberately **not** vendored into this repository. It is the other
half of the module's ABI, so it is taken from whichever `webbluetooth-wasm` the
lock file resolved to and cannot drift out of step with the `.wasm` beside it.

## Two threads, and why the split falls where it does

Streaming a headset is about 1,400 events a second, and drawing eight traces of
a thousand points at 60 Hz is on the order of half a million path operations a
second. Done on one thread, the drawing is what makes the page feel stuck.

So the drawing moves, and the Bluetooth cannot:

| | Main thread (`app.js`) | Worker (`render.js`) |
|---|---|---|
| Web Bluetooth + the wasm module | ✓ | |
| Ring buffers, autoscaling, every canvas call | | ✓ |
| Per-second DOM writes (status, battery) | ✓ | |

`navigator.bluetooth` is exposed on `Window` and not on `WorkerNavigator`, so a
worker cannot reach the radio at all; `requestDevice` additionally has to be
called from a user gesture, which only the main thread has. The shim marshals
straight into the module's memory (`state.exports.wbt_alloc`, `wbt_settle`), so
the module has to share a realm with it. That fixes the BLE half in place. What
is left on the main thread after the move is a few DOM writes a second.

The canvas is handed over with `transferControlToOffscreen()`, after which this
page cannot draw to it and the worker can. Samples cross once per frame rather
than once each — at 256 Hz per electrode a frame carries a few dozen — and they
are copied rather than transferred, because at that size the copy is free and
one reused pair of buffers keeps the allocation rate flat. A `SharedArrayBuffer`
would avoid even the copy, but it needs COOP/COEP headers from whatever serves
the page, and `python3 -m http.server` does not send them.

## How it is put together

There is no `wasm-bindgen`. `webbluetooth-wasm` does not use one — the boundary
is plain numbers over a small hand-written shim — and this example crosses it
the same way. That makes the whole interface between the page and the module
something you can list:

**The module exports** (`examples/web/lib.rs`)

- `muse_connect()` — the Connect button. Everything else follows from it.
- `wbt_alloc`, `wbt_free`, `wbt_settle`, `wbt_event`, `memory` — not ours; they
  come from `webbluetooth-wasm` and are how the page answers a request.

**The module imports**

- `muse.*` — this example's own: `muse_row` (a row exists, with its name and
  its rate), `muse_sample`, `muse_imu`, `muse_status`, `muse_battery`,
  `muse_active`, and `muse_log`. `app.js` supplies them and forwards the
  drawing-related ones to the worker.
- `webbluetooth.wbt_call` — every Bluetooth operation, through one import. The
  shim supplies it.

Two details worth knowing, because both are easy to get wrong:

**Connecting has to come from a click.** `requestDevice` opens the browser's own
device chooser, and browsers only permit that during a user gesture.
`muse_connect` therefore spawns the work and returns immediately rather than
awaiting it, so the chooser opens while the gesture is still live.

**The page does not name the electrodes.** It is told them. The Classic and
Athena mappings diverge at index 4 — AUX on one, FPz on the other — so only the
side that knows the firmware can label a trace, and that is the library:
`protocol::eeg_channel_name(electrode, handle.is_athena)`.

## What the view does

**Rows.** One per electrode — four on Classic, up to eight on Athena — then
accelerometer and gyroscope, each with its three axes overlaid so they can be
compared against each other. Every row is mean-removed, because raw EEG sits on
a large per-electrode DC offset and an accelerometer axis sits on gravity;
plotted against zero, the interesting part of either is a flat line jammed
against the edge of its row. Each row is then scaled to its own peak, which is
the figure printed after its name.

**Every row covers the same four seconds.** This is worth stating because it
does not come for free: EEG arrives at 256 Hz and the IMU at about 52, so a
buffer of a fixed number of samples would put five times as much history in the
IMU rows as in the electrode rows, on one shared x axis. Instead the module
declares each row's rate (`muse_row`) and the view sizes that row's buffer to
`rate × 4 s` — 1024 samples for an electrode, 208 for an IMU sensor — so the
same instant sits at the same x in every row.

**Smoothing** (the checkbox, on by default) draws a moving average over a
dimmed copy of the raw trace, so it never hides what the headset actually sent.
The window is the TUI's — 9 samples at 256 Hz — but held here as the duration
that works out to, ≈35 ms, and converted back per row. A 52 Hz IMU row is
therefore averaged over the same *time* as a 256 Hz electrode row rather than
five times as much of it.

## Borrowing this for your own page

Two things to carry over:

1. **A browser has no tokio runtime.** The library handles its own internals
   (see `src/platform.rs`), but to spawn *your* top-level task you want
   `webbluetooth-wasm`'s `exec::spawn`, as `muse_connect` does. Add it under
   `[target.'cfg(target_arch = "wasm32")'.dependencies]`, at the same version
   `webbluetooth` resolves to.
2. **Emit the shim from your build**, rather than copying it once by hand.
   `webbluetooth_wasm::SHIM_JS` is the file's contents as a `&str`, compiled in
   from the same source tree as the ABI, which is why `build.sh` copies it
   fresh on every build.
