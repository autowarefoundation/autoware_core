# Portability and the Host ports

The same algorithm core runs under ROS, a bare-metal kernel, or an async runtime because it is
written against the [Host ports](host-vtable.md), not against ROS.

## `ScanMatcher` over `H: Host`

`scan_matcher::ScanMatcher` wraps the [`NdtEngine`](engine.md) and drives it generically:

- `match_scan(guess, source, &mut scratch) -> MatchResult` and
  `match_scan_with_covariance(...) -> (MatchResult, CovarianceResult)` — **synchronous**, the WCET
  hot path.
- `update_map(&source, center, radius).await` — **async**, over the `MapSource` port.

It holds no per-align mutable state, so a shared `&ScanMatcher` is sound across concurrent tasks in
the std and `mt` configs (map updates publish atomically).

## The `no_std` gating discipline

`lib.rs` is `#![cfg_attr(not(any(test, feature = "std")), no_std)]`. The portable modules
(`scan_matcher`, `host`, `engine`, `convergence`, `covariance`, `cov_estimate`, `ndt`, `voxel_grid`,
`tpe`, …) are `no_std`; the ROS/FFI glue (`node*`, `ffi`, `ffi_host`, `sensor_points`) is
`#[cfg(feature = "std")]`. New algorithmic code goes in the portable core so the kernel build keeps
it; only ROS/FFI glue may be `std`-gated.

## Reference adapters

- `examples/tokio_ndt.rs` — the async reference host over synthetic data (proves the `Host` seam).
- `examples/threads_ndt.rs` — multi-threaded shared-engine usage.

The kernel (Track-B) target is the reason for this split: it supplies its own async runtime and map
source (flash/DMA) and builds the core with `--no-default-features` (single-core) or `--features mt`
(multi-core); see [The `mt` multi-core engine](../rt/mt.md).

> Source: `src/scan_matcher.rs`, `src/host.rs`, `examples/{tokio_ndt,threads_ndt}.rs`, `src/lib.rs`.
