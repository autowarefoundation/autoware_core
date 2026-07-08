# The WCET contract

The worst-case execution-time contract of the align hot path, the crate's real-time centerpiece.

Planned contents:

- The contract stated in `ndt::align`'s doc: outer loop ≤ `max_iterations` (static cap); per
  iteration one `compute_derivatives` pass (`O(P·K)`), one 6×6 SVD, one f32 transform (`O(P)`).
- `K ≤ MAX_NEIGHBORS` (the `radius_search` cap); `P` bounded by caller downsampling.
- The accepted residual: adversarial kd-tree traversal is worst-case `O(N_leaves)` (benign for
  physical, roughly-uniform maps).
- No panic, no blocking, no logging/formatting, no user callbacks on the path.
- Serial backend as the predictable baseline; the WCET re-audit process after engine changes.

> Status: outline (draft to be written).
> Source: `src/ndt.rs` WCET contract (≈ lines 317, 617), `examples/wcet_frame.rs`.
