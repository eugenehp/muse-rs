// The worker half: ring buffers and every canvas operation.
//
// Nothing here knows about Bluetooth. It is handed an `OffscreenCanvas`, then a
// stream of `{ row }`, `{ samples }`, `{ imu }`, `{ size }`, `{ smooth }`
// and `{ reset }` messages, and it draws. Keeping it that ignorant is what
// makes it movable: the main thread has to own the radio (see the note at the
// top of `app.js`), but nothing about turning numbers into pixels needs to be
// there.
//
// A classic worker rather than a module one: it imports nothing, so there is
// nothing for `type: 'module'` to buy.

// How much history is on screen. Held in seconds rather than samples because
// the streams do not share a rate: EEG is 256 Hz and the IMU 52, so a fixed
// sample count would put five times as much history in the IMU rows as in the
// electrode rows while drawing them against one x axis. Each row's buffer is
// sized from its own rate, and every row then covers the same four seconds.
const WINDOW_SECONDS = 4;

// The TUI smooths over 9 samples at 256 Hz. Kept here as the duration that
// works out to — ≈35 ms — so the same amount of *time* is averaged on a row
// arriving at any rate, which is what makes two rows comparable by eye. At
// 256 Hz this is the TUI's 9 samples exactly.
const SMOOTH_SECONDS = 9 / 256;

// Distinct rather than pretty, and eight of them because Athena has eight
// electrodes where Classic has four or five.
const COLOURS = [
  '#4cc9f0', '#f4a261', '#90be6d', '#f28cb1',
  '#b8a1ff', '#ffd166', '#8ecae6', '#e07a5f',
];

// X, Y, Z. Reused by both IMU rows so an axis means the same colour in each.
const AXES = ['#4cc9f0', '#f4a261', '#90be6d'];

const LABEL = '#8a8a94';

// Presentation per family. The rate and the name come from the module, which
// owns the protocol constants; how a row should look is the view's business.
const KIND_EEG = 0;
const KIND_IMU = 1;

const STYLE = {
  [KIND_EEG]: {
    unit: 'µV',
    digits: 0,
    // A flat electrode should read as flat rather than have its noise
    // amplified to fill the row.
    floor: 20,
    series: 1,
    axes: null,
    colour: (index) => [COLOURS[index % COLOURS.length]],
  },
  [KIND_IMU]: {
    // Accelerometer in g, gyroscope in °/s — index 0 and 1, as declared.
    unit: (index) => (index === 0 ? 'g' : '°/s'),
    digits: (index) => (index === 0 ? 2 : 0),
    floor: (index) => (index === 0 ? 0.05 : 5),
    series: 3,
    axes: ['x', 'y', 'z'],
    colour: () => AXES,
  },
};

const rows = new Map();

function ring(capacity) {
  return { samples: new Float64Array(capacity), count: 0 };
}

const pick = (value, index) => (typeof value === 'function' ? value(index) : value);

/// Create a row, or return the one already there.
///
/// `hz` decides the buffer size and therefore the time span. A row that
/// receives a sample before its declaration is created at the family's usual
/// rate rather than dropped, and the declaration then corrects it.
function row(kind, index, hz, name) {
  const key = `${kind}:${index}`;
  const existing = rows.get(key);
  const capacity = Math.max(2, Math.round(hz * WINDOW_SECONDS));

  if (existing && existing.series[0].samples.length === capacity) {
    if (name) existing.name = name;
    return existing;
  }

  const style = STYLE[kind];
  const created = {
    order: kind * 1000 + index,
    name: name || (existing && existing.name) || `${kind}:${index}`,
    hz,
    unit: pick(style.unit, index),
    digits: pick(style.digits, index),
    floor: pick(style.floor, index),
    axes: style.axes,
    colours: style.colour(index),
    // A rate correction starts the history again rather than reinterpreting
    // samples that were taken at a different spacing.
    series: Array.from({ length: style.series }, () => ring(capacity)),
  };
  rows.set(key, created);
  return created;
}

// Fallbacks for a sample that somehow precedes its declaration.
const DEFAULT_HZ = { [KIND_EEG]: 256, [KIND_IMU]: 52 };

function rowFor(kind, index) {
  return rows.get(`${kind}:${index}`) || row(kind, index, DEFAULT_HZ[kind]);
}

function push(target, value) {
  target.samples[target.count % target.samples.length] = value;
  target.count += 1;
}

let canvas = null;
let context = null;
// CSS pixels. The canvas is sized in device pixels and scaled once, which is
// what keeps the traces sharp on a retina display.
let width = 0;
let height = 0;
let ratio = 1;
let smooth = true;

// `requestAnimationFrame` is defined in a dedicated worker, and is the right
// thing to use: it stops when the page is hidden, where a timer would carry on
// drawing frames nobody sees. The fallback is for the case where it is not.
const schedule =
  typeof requestAnimationFrame === 'function'
    ? (callback) => requestAnimationFrame(callback)
    : (callback) => setTimeout(callback, 16);

// Kept apart from the message that carried it, so that a size arriving before
// the canvas — which message order rules out, but nothing else would — is
// remembered rather than thrown away.
function applySize() {
  if (!context) return;
  // Guarded: a layout can report a zero-sized box mid-pass, and a canvas
  // cannot have a zero dimension.
  canvas.width = Math.max(1, Math.round(width * ratio));
  canvas.height = Math.max(1, Math.round(height * ratio));
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
}

self.onmessage = (event) => {
  const message = event.data;

  if (message.canvas) {
    canvas = message.canvas;
    context = canvas.getContext('2d');
    applySize();
    schedule(draw);
  }

  if (message.size) {
    ({ width, height, ratio } = message.size);
    applySize();
  }

  if (message.row) {
    const { kind, index, hz, name } = message.row;
    row(kind, index, hz, name);
  }

  if (message.samples) {
    const { electrodes, microvolts } = message.samples;
    for (let i = 0; i < electrodes.length; i += 1) {
      push(rowFor(KIND_EEG, electrodes[i]).series[0], microvolts[i]);
    }
  }

  if (message.imu) {
    const { sensors, x, y, z } = message.imu;
    for (let i = 0; i < sensors.length; i += 1) {
      const target = rowFor(KIND_IMU, sensors[i]);
      push(target.series[0], x[i]);
      push(target.series[1], y[i]);
      push(target.series[2], z[i]);
    }
  }

  if (message.smooth !== undefined) {
    smooth = message.smooth;
  }

  if (message.reset) {
    rows.clear();
  }
};

// ── Drawing ─────────────────────────────────────────────────────────────────

/// The window's samples, oldest first and mean removed.
///
/// The mean goes because raw Muse EEG sits on a large per-electrode DC offset,
/// and an accelerometer axis sits on whatever gravity is doing to it: plotted
/// against zero, the interesting part of either is a flat line jammed against
/// the edge of its row.
function windowOf(target) {
  const span = target.samples.length;
  const count = Math.min(target.count, span);
  const values = new Float64Array(count);
  let total = 0;
  for (let i = 0; i < count; i += 1) {
    // Oldest first: the ring buffer's newest sample is at `count - 1`.
    const value = target.samples[(target.count - count + i) % span];
    values[i] = value;
    total += value;
  }
  const mean = count > 0 ? total / count : 0;
  for (let i = 0; i < count; i += 1) values[i] -= mean;
  return values;
}

/// Centred moving average, edges clamped so the length is preserved — the same
/// shape as `smooth_signal` in the TUI, computed off a prefix sum so the window
/// size costs nothing.
function smoothed(values, window) {
  if (values.length < 3 || window < 2) return values;
  const half = window >> 1;
  const prefix = new Float64Array(values.length + 1);
  for (let i = 0; i < values.length; i += 1) prefix[i + 1] = prefix[i] + values[i];

  const out = new Float64Array(values.length);
  for (let i = 0; i < values.length; i += 1) {
    const start = Math.max(0, i - half);
    const end = Math.min(values.length, i + half + 1);
    out[i] = (prefix[end] - prefix[start]) / (end - start);
  }
  return out;
}

/// A dimmed version of a trace colour, for the raw signal drawn behind the
/// smoothed one. Derived rather than kept as a second palette, so adding a
/// colour above cannot leave a missing dim twin.
function dim(colour) {
  const value = parseInt(colour.slice(1), 16);
  const scale = (channel) => Math.round(channel * 0.42);
  return `rgb(${scale((value >> 16) & 255)} ${scale((value >> 8) & 255)} ${scale(value & 255)})`;
}

function trace(values, span, peak, middle, rowHeight, colour) {
  context.beginPath();
  for (let i = 0; i < values.length; i += 1) {
    // Against the row's own capacity, which is `WINDOW_SECONDS` of it: two
    // rows at different rates then cover the same time across the same width.
    const x = (i / (span - 1)) * width;
    const y = middle - (values[i] / peak) * (rowHeight * 0.42);
    if (i === 0) context.moveTo(x, y);
    else context.lineTo(x, y);
  }
  context.lineWidth = 1;
  context.strokeStyle = colour;
  context.stroke();
}

function draw() {
  context.clearRect(0, 0, width, height);

  const visible = [...rows.values()].sort((a, b) => a.order - b.order);
  if (visible.length === 0) {
    schedule(draw);
    return;
  }

  const rowHeight = height / visible.length;

  visible.forEach((entry, index) => {
    const middle = index * rowHeight + rowHeight / 2;
    const span = entry.series[0].samples.length;
    const windows = entry.series.map(windowOf);

    // One scale for the whole row: for an IMU row that keeps the three axes
    // comparable with each other, which is the point of overlaying them.
    let peak = entry.floor;
    for (const values of windows) {
      for (let i = 0; i < values.length; i += 1) {
        peak = Math.max(peak, Math.abs(values[i]));
      }
    }

    // The same averaged duration on every row, whatever its rate.
    const window = Math.max(2, Math.round(SMOOTH_SECONDS * entry.hz));

    windows.forEach((values, series) => {
      const colour = entry.colours[series % entry.colours.length];
      if (smooth) {
        // Raw behind, smoothed in front — the TUI's arrangement, so that
        // smoothing never hides what the headset actually sent.
        trace(values, span, peak, middle, rowHeight, dim(colour));
        trace(smoothed(values, window), span, peak, middle, rowHeight, colour);
      } else {
        trace(values, span, peak, middle, rowHeight, colour);
      }
    });

    // ── Label ────────────────────────────────────────────────────────────
    context.font = '11px ui-monospace, SFMono-Regular, monospace';
    context.fillStyle = LABEL;
    const caption = `${entry.name}  ±${peak.toFixed(entry.digits)} ${entry.unit}`;
    context.fillText(caption, 8, index * rowHeight + 14);

    // For a multi-series row, say which colour is which axis.
    if (entry.axes) {
      let x = 8 + context.measureText(`${caption}  `).width;
      entry.axes.forEach((axis, series) => {
        context.fillStyle = entry.colours[series];
        context.fillText(axis, x, index * rowHeight + 14);
        x += context.measureText(`${axis} `).width;
      });
    }
  });

  schedule(draw);
}
