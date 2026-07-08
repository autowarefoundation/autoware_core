# The WCET contract

`ndt::align` is the one RT-critical function whose worst-case execution time is contracted. The
contract is stated in its doc comment and is the crate's real-time centerpiece.

## The bound

- The outer loop runs **at most `params.max_iterations`** times (a static cap).
- Each iteration does exactly: one f32 cloud transform (`O(P)`, `P` = source points), one
  `compute_derivatives` pass (`O(P · K)`, `K` = neighbors per point), and one fixed-size 6×6 SVD
  solve.
- `K ≤ MAX_NEIGHBORS` (the `radius_search` cap); `P` must be bounded by the caller (downsample).

So the align frame is bounded by `max_iterations · (O(P·K) + O(1))`.

## The accepted residual

The kd-tree traversal is worst-case `O(N_leaves)` for adversarial point distributions — an
**accepted residual**, benign for physical, roughly-uniform voxel maps (see
[Voxel grid and kd-tree](../arch/voxel-grid.md)).

## What the path must not do

No panic (the crate's deny-`unwrap`/`expect`/`panic`/`indexing` lints — see
[Panic-free, bounded execution](panic-free.md)), no blocking, no logging/formatting, no user
callbacks, and — after warmup — no allocation ([Zero-allocation guarantees](zero-alloc.md)). Only
fixed-width float math. The **serial** backend is the predictable baseline; the parallel backend is a
throughput option, not the WCET reference.

## Measured

The per-frame WCET micro-benchmark `examples/wcet_frame.rs` measures the tail directly; current
numbers are in [Benchmarking](../quality/benchmarks.md#l2--per-frame-wcet-exampleswcet_framers). A
`rust-realtime-review` accompanies each engine/align patch.

> Source: `src/ndt.rs` (the `align` WCET contract, ≈ lines 317 / 617), `examples/wcet_frame.rs`.
