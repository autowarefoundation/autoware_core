# Feature flags and build configurations

The crate has one code base and several build configurations, selected by Cargo features. The
`std` feature is default-on so a plain `cargo build`/`test`/`clippy` has `std` (test harness +
panic handler); the `no_std` build opts out with `--no-default-features`.

## Features

| Feature | Default | Effect |
|---|---|---|
| `std` | ✅ | Host/ROS build. Pulls `arc-swap` (the lock-free engine-state double-buffer) and puts the align scratch in a thread-local. The engine is `Sync`. |
| `parallel` | ✅ | rayon-backed `compute_derivatives` reduction. Implies `std`. Bit-identical to serial (per-point contributions reduced in index order), so it is a pure throughput option — the serial backend stays the predictable WCET baseline. |
| `mt` | ❌ | Multi-core `no_std` (kernel). Replaces the single-core `RefCell` cells with `awkernel_sync` mutexes and removes the engine-owned align scratch (callers pass a `&mut MatchScratch`). Ignored when `std` is also on. |
| `ros` | ❌ | Builds the rosidl `bindgen` bindings for the `geometry_msgs` C structs + the `Pose`-pointer FFI shims. Independent of `std`. |

## Parallelism and worker threads

The `parallel` backend runs the derivative reduction on rayon's **process-global thread pool**.
There are two independent knobs:

- **Enable parallel** — set the `num_threads` param `> 1` (`NdtParams.num_threads`, the ROS node's
  `num_threads` parameter, or `ScanMatcher::set_params`). This is a *switch*: `> 1` selects the rayon
  backend, `≤ 1` stays serial. It does **not** by itself decide how many workers rayon uses.
- **Set the worker count** — size the process-global pool, in one of three ways (all equivalent,
  process-wide):

  1. **The `num_threads` param, via the node handle.** When `autoware_ndt_scan_matcher_rs_new`
     builds the handle with `num_threads > 1`, it sizes the global pool to that value once, before
     any align. So a ROS node needs only its existing `num_threads` parameter.
  2. **Explicit API.** Call `init_thread_pool(n)` (C ABI:
     `autoware_ndt_scan_matcher_rs_init_thread_pool(n)`) once, early. Best-effort and idempotent.
  3. **Environment.** `RAYON_NUM_THREADS=n` (rayon's built-in), read on first use.

  If none is set, the pool defaults to the number of logical CPUs.

Because the pool is process-global, `n` is the total worker count for the whole process (not
per-engine). The reduction is bit-identical regardless of `n`, so this only trades throughput for
WCET predictability — the serial backend (`num_threads ≤ 1`) stays the predictable RT baseline.

## The three engine configurations

The interior mutability of the engine (its target map + params) is chosen at compile time. This
is the single most important thing to understand about the build matrix; it is detailed in
[Concurrency and interior mutability](../arch/concurrency.md).

| Configuration | How to build | Cells | Align scratch | `Sync`? |
|---|---|---|---|---|
| **std** (default) | `cargo build` | `ArcSwap<EngineState>` (lock-free) | thread-local | yes |
| **`no_std` single-core** | `--no-default-features` | `RefCell<Arc<…>>` | engine-owned | **no** (rejected at compile time) |
| **`no_std` multi-core** | `--no-default-features --features mt` | `awkernel_sync::Mutex<Arc<…>>` | **caller-owned** `MatchScratch` | yes |

Notes:

- `parallel` implies `std`, so `mt` + `parallel` resolves to the `std` backend (`mt` is ignored).
- Under `mt` the implicit-scratch align API (`align`, `result`, `score_arrays`, …) is **compiled
  out**; only the `_with` methods that take a `&mut MatchScratch` exist. A cross-call scratch
  dependency therefore cannot even compile.
- The `no_std` builds are *libraries* linked into a final binary that supplies the
  `#[panic_handler]`; building the `no_std` staticlib standalone will report a missing panic
  handler. The kernel gate used in CI is a targeted `cargo rustc --no-default-features --lib
  --target {x86_64,aarch64}-unknown-none --crate-type rlib`.

## Source of truth

The feature definitions and their rationale live in the crate's `Cargo.toml`; the engine's
interior-mutability matrix is documented at the top of `src/engine.rs`; the crate-level overview
is the `//!` doc in `src/lib.rs`.
