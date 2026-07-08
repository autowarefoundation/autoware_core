# Portability and the Host ports

How the same core runs under ROS, a kernel, or an async runtime.

Planned contents:

- `ScanMatcher` generic over `H: Host`; `match_scan` / `match_scan_with_covariance` (sync) and
  `update_map` (async).
- The port seam recap (`MapSource`/`OutputSink`/`Clock`), and the adapters that implement it.
- The `no_std` gating discipline: portable modules vs `#[cfg(feature = "std")]` node/FFI glue.
- Reference adapters and examples: `examples/tokio_ndt.rs`, `examples/threads_ndt.rs`.
- The kernel (Track-B) target and its constraints.

> Status: outline (draft to be written).
> Source: `src/scan_matcher.rs`, `src/host.rs`, `examples/{tokio_ndt,threads_ndt}.rs`, `src/lib.rs`
> (no_std gating).
