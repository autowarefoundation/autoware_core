# Serial and parallel derivatives

How the NDT score, gradient, and 6×6 Hessian are accumulated for a candidate pose, and why the
parallel backend is bit-identical to the serial one.

## Angle derivatives and per-point kernels

Once per pose, `compute_angle_derivatives` precomputes the angular Jacobian (`j_ang`, 8×4) and
Hessian (`h_ang`, 16×4) tables — the derivatives of the rotation w.r.t. `[roll, pitch, yaw]`. For
each transformed source point, `point_contribution` finds its voxel neighbors (radius search) and
accumulates that point's score, gradient, and point-Hessian contribution; `finalize` folds in the
optional [regularization](../concepts/ndt-primer.md) term and assembles the `Derivatives`.

## Serial vs parallel

`compute_derivatives` is the serial backend (one reused neighbor buffer). `compute_derivatives_parallel`
(under the `parallel` feature) runs the per-point work on rayon's process-global pool via
`par_iter().zip(...).map_init(...).collect_into_vec(&mut ws.contribs)`, giving each worker its own
reusable neighbor buffer. `align` selects it when `params.num_threads > 1`.

## Bit-for-bit identical

`collect_into_vec` preserves point-index order, so the parallel contributions are folded in the
**same order** as serial → the two agree **bit-for-bit** (pinned by tests). Parallelism is therefore
a pure throughput option, never a numeric change; the serial backend stays the predictable WCET
baseline. The worker count is sized separately (see
[Parallelism and worker threads](../start/features.md#parallelism-and-worker-threads)).

## Exact Hessian

Since PR #1217 fixed the `h_ang` "d1" pitch² sign (`+sy` → `-sy`), the assembled Hessian equals the
true second derivative and finite-difference-validates over all six rows (see
[Divergences from upstream](../port/divergences.md)).

> Source: `src/derivatives.rs`, `src/ndt.rs` (`compute_derivatives[_parallel]`, `finalize`); the
> `*_bit_identical` tests.
