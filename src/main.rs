//! Entry point for the `muse-rs` command-line client.
//!
//! The body of it lives in `cli.rs`. That is a module here, rather than being
//! this file, so that a single `cfg` takes all seven hundred lines of it out at
//! once on `wasm32-unknown-unknown`: this binary reads commands from stdin and
//! prints to a terminal, and it drives tokio's multi-threaded runtime, none of
//! which a browser page has. Rather than fail the build for a target the binary
//! was never meant for, it compiles to an empty `main` there.
//!
//! The library is the part a browser wants, and that does build for wasm — see
//! `src/platform.rs` for how spawning and timers get there.

#[cfg(not(target_arch = "wasm32"))]
#[path = "cli.rs"]
mod cli;

#[cfg(not(target_arch = "wasm32"))]
fn main() -> anyhow::Result<()> {
    cli::run()
}

/// A page has no stdin to read and no terminal to print to.
#[cfg(target_arch = "wasm32")]
fn main() {}
