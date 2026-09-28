#!/bin/sh
# Build the web example and lay out a directory that can be served.
#
# Five files end up in dist/: the module, the page, its glue, the worker that
# does the drawing, and the shim. The shim is not vendored in this repository —
# it is copied out of whichever `webbluetooth-wasm` the lock file resolved to, so
# it cannot drift from the ABI the module was just compiled against.
set -eu

here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
root=$(CDPATH= cd -- "$here/../.." && pwd)
dist="$here/dist"

# No feature flags: the TUI dependencies are gated by target, so the default
# feature set is already correct for wasm. See `Cargo.toml`.
cargo build --manifest-path "$root/Cargo.toml" --example web \
  --target wasm32-unknown-unknown --release

# Ask cargo where the shim is rather than guessing at a registry path, which
# encodes both the version and the checksum directory.
shim=$(
  cargo metadata --manifest-path "$root/Cargo.toml" --format-version 1 \
    --filter-platform wasm32-unknown-unknown |
    python3 -c 'import json, os, sys
packages = json.load(sys.stdin)["packages"]
backend = next(p for p in packages if p["name"] == "webbluetooth-wasm")
print(os.path.join(os.path.dirname(backend["manifest_path"]), "js", "webbluetooth.js"))'
)

mkdir -p "$dist"
cp "$root/target/wasm32-unknown-unknown/release/examples/web.wasm" "$dist/web.wasm"
cp "$shim" "$dist/webbluetooth.js"
cp "$here/index.html" "$here/app.js" "$here/render.js" "$dist/"

# ── Temporary: patch the shim ────────────────────────────────────────────────
#
# webbluetooth-wasm 0.0.2 reports a notification's characteristic by UUID,
# while its own Rust backend keys both the subscriber map and the value cache
# by the handle path ("<device>/svc/N/chr/N"). The lookup therefore always
# misses: every notification is decoded and then dropped, which shows up as a
# page that connects, discovers and writes happily and never receives a packet.
#
# The fix is one token upstream, in crates/webbluetooth-wasm/js/webbluetooth.js
# — `.str(event.target.uuid)` should be `.str(key)`, and `key` is already in
# scope on the line above. Until a release carries it, the copy laid out here
# is patched rather than shipping an example that cannot receive data. Delete
# this block once the dependency is updated; the guard makes it a no-op if the
# shim has already been fixed.
if grep -q '\.str(event\.target\.uuid)' "$dist/webbluetooth.js"; then
  sed 's/\.str(event\.target\.uuid)/.str(key)/' "$dist/webbluetooth.js" > "$dist/.shim.patched"
  mv "$dist/.shim.patched" "$dist/webbluetooth.js"
  echo "note: applied the temporary webbluetooth-wasm 0.0.2 notification-key patch (see build.sh)"
fi

cat <<EOF

Built $dist:
$(cd "$dist" && ls -1 | sed 's/^/  /')

Web Bluetooth needs a secure context, so open it over localhost rather than as
a file:// URL:

  python3 -m http.server --directory $dist 8000

then visit http://localhost:8000/ in Chrome or Edge.
EOF
