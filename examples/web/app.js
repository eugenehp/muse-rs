// The page half of the web example: two threads, split where the browser
// forces the split.
//
// Here, on the main thread: Web Bluetooth and the wasm module that drives it.
// That is not a preference. `navigator.bluetooth` is exposed on `Window` and
// not on `WorkerNavigator`, so a worker cannot reach the radio at all; and
// `requestDevice` has to be called from a user gesture, which only this thread
// has. The shim marshals directly into the module's memory besides
// (`state.exports.wbt_alloc`, `wbt_settle`), so the module has to share a realm
// with it.
//
// In `render.js`: the ring buffers and every canvas operation, through an
// `OffscreenCanvas`. That is the half that was costing frames. Redrawing eight
// traces of a thousand points at 60 Hz is on the order of half a million path
// operations a second, where the data crossing the wasm boundary is about two
// thousand samples a second — so the drawing is what moves off, and the main
// thread is left with the BLE it cannot delegate plus a few DOM writes a second.
//
// `webbluetooth.js` is the shim from `webbluetooth-wasm`, copied in by
// `build.sh` rather than vendored, so it cannot drift from the module's ABI.
import { instantiate } from './webbluetooth.js';

const connect = document.querySelector('#connect');
const statusLine = document.querySelector('#status');
const batteryLine = document.querySelector('#battery');
const rateLine = document.querySelector('#rate');
const smoothing = document.querySelector('#smooth');
const canvas = document.querySelector('#traces');

// Room for one frame of samples, with margin: 256 Hz across eight electrodes is
// roughly 34 a frame at 60 Hz, so this overflows only if the page is stalled for
// a third of a second — and `push` flushes early rather than dropping any.
const BATCH = 1024;

const electrodes = new Uint8Array(BATCH);
const microvolts = new Float64Array(BATCH);
let pending = 0;

// The IMU is far slower — 52 Hz in triples, per sensor — so it gets a smaller
// buffer of its own rather than being interleaved into the EEG one, which
// would mean carrying an axis tag on every EEG sample to tell them apart.
const IMU_BATCH = 256;
const imuSensors = new Uint8Array(IMU_BATCH);
const imuX = new Float64Array(IMU_BATCH);
const imuY = new Float64Array(IMU_BATCH);
const imuZ = new Float64Array(IMU_BATCH);
let imuPending = 0;

let worker = null;

// Counted here rather than in the worker: this is the point the data actually
// crosses out of the module, so a zero here says the samples never arrived,
// where a zero further in would only say they were not drawn.
let received = 0;
let receivedAtLastTick = 0;

// ── The module's imports ────────────────────────────────────────────────────

let instance = null;
const decoder = new TextDecoder();

// A string from the module's memory. Built per call: the buffer is detached
// whenever linear memory grows, so a cached view would go stale.
function text(pointer, length) {
  return decoder.decode(new Uint8Array(instance.exports.memory.buffer, pointer, length));
}

// Batched rather than posted one at a time: a message per sample would be two
// thousand a second to carry a few dozen numbers each frame. Copied rather than
// transferred, because at this size the copy is nothing and reusing one pair of
// buffers here keeps the allocation rate flat. A SharedArrayBuffer would avoid
// even that, but needs COOP/COEP headers on whatever serves the page, and
// `python3 -m http.server` does not send them.
function flush() {
  if (pending === 0 && imuPending === 0) return;

  // One message per frame carrying whichever streams produced anything, rather
  // than one per stream.
  const message = {};
  if (pending > 0) {
    message.samples = {
      electrodes: electrodes.slice(0, pending),
      microvolts: microvolts.slice(0, pending),
    };
    pending = 0;
  }
  if (imuPending > 0) {
    message.imu = {
      sensors: imuSensors.slice(0, imuPending),
      x: imuX.slice(0, imuPending),
      y: imuY.slice(0, imuPending),
      z: imuZ.slice(0, imuPending),
    };
    imuPending = 0;
  }
  worker.postMessage(message);
}

// What `examples/web/lib.rs` declares as its `muse` import namespace. The shim
// spreads this alongside its own `webbluetooth` namespace.
const imports = {
  muse: {
    muse_status(pointer, length) {
      statusLine.textContent = text(pointer, length);
    },

    muse_row(kind, index, hz, pointer, length) {
      worker.postMessage({ row: { kind, index, hz, name: text(pointer, length) } });
    },

    muse_sample(electrode, value) {
      received += 1;
      if (pending === BATCH) flush();
      electrodes[pending] = electrode;
      microvolts[pending] = value;
      pending += 1;
    },

    muse_imu(sensor, x, y, z) {
      if (imuPending === IMU_BATCH) flush();
      imuSensors[imuPending] = sensor;
      imuX[imuPending] = x;
      imuY[imuPending] = y;
      imuZ[imuPending] = z;
      imuPending += 1;
    },

    muse_battery(percent) {
      batteryLine.textContent = `${percent.toFixed(0)}%`;
    },

    // The library's own logs. Silent otherwise in a browser: `env_logger`
    // wants RUST_LOG and stderr, and a page has neither.
    muse_log(level, pointer, length) {
      const line = text(pointer, length);
      // log::Level is 1..=5. `trace` is mapped to `debug` rather than
      // `console.trace`, which would print a JS stack that means nothing here.
      const method = [, 'error', 'warn', 'info', 'debug', 'debug'][level] || 'log';
      console[method](`[muse] ${line}`);
    },

    // Bracketed around the whole streaming task on the Rust side, so this is
    // also how the button comes back after a failure.
    muse_active(active) {
      connect.disabled = active === 1;
      // Clear the traces when a stream starts rather than when one ends: a
      // finished run stays on screen to be looked at, and a new headset starts
      // from an empty canvas instead of splicing onto the last one's data.
      if (active === 1) worker.postMessage({ reset: true });
    },
  },
};

// ── Wiring ──────────────────────────────────────────────────────────────────

// Canvas pixels are device pixels. The worker owns the `OffscreenCanvas`, so it
// is told the size in both units and does the scaling; this side only watches
// for changes.
function postSize() {
  const bounds = canvas.getBoundingClientRect();
  worker.postMessage({
    size: {
      width: bounds.width,
      height: bounds.height,
      ratio: window.devicePixelRatio || 1,
    },
  });
}

function fail(message) {
  statusLine.textContent = message;
  connect.disabled = true;
}

if (!navigator.bluetooth) {
  // Worth saying plainly: the usual cause is Safari or Firefox, where Web
  // Bluetooth is not implemented, and no amount of retrying will help.
  fail('This browser has no Web Bluetooth. Chrome or Edge, over localhost or HTTPS.');
} else if (!canvas.transferControlToOffscreen) {
  // Not expected to happen: every browser with Web Bluetooth has had
  // OffscreenCanvas for years. Reported rather than left to throw, so the
  // failure names itself if it ever does.
  fail('This browser cannot hand a canvas to a worker (OffscreenCanvas missing).');
} else {
  worker = new Worker('./render.js');

  // Transferred, not copied: after this the worker draws and this thread
  // cannot, which is the point of it.
  const offscreen = canvas.transferControlToOffscreen();
  worker.postMessage({ canvas: offscreen }, [offscreen]);

  postSize();
  new ResizeObserver(postSize).observe(canvas);

  // Smoothing is the worker's business — it owns the samples — so the checkbox
  // only forwards its state. Sent once up front so the two agree before any
  // data arrives.
  const postSmoothing = () => worker.postMessage({ smooth: smoothing.checked });
  smoothing.addEventListener('change', postSmoothing);
  postSmoothing();

  instance = await instantiate(fetch('./web.wasm'), imports);

  // Hand samples over a frame at a time. This runs on the main thread because
  // that is where they arrive, but all it does is post.
  const pump = () => {
    flush();
    requestAnimationFrame(pump);
  };
  requestAnimationFrame(pump);

  // Once a second, because it is a diagnostic and not an animation: the
  // useful question is whether samples are arriving at all, and at roughly
  // the rate the headset claims.
  setInterval(() => {
    const perSecond = received - receivedAtLastTick;
    receivedAtLastTick = received;
    rateLine.textContent = received === 0 ? '—' : `${received} (${perSecond}/s)`;
  }, 1000);

  connect.disabled = false;
  statusLine.textContent = 'Ready.';

  connect.addEventListener('click', () => {
    // Straight through to the module, still inside the click: the chooser has
    // to open while the gesture is live.
    instance.exports.muse_connect();
  });
}
