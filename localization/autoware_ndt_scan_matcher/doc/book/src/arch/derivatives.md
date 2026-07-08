# Serial and parallel derivatives

How the score/gradient/Hessian are accumulated, and why the parallel backend is bit-identical.

Planned contents:

- The angular derivative constants and per-point derivative kernels
  (`compute_*_derivatives`, `update_derivatives`).
- `compute_derivatives` (serial) vs `compute_derivatives_parallel` (rayon).
- Order-preserving reduction (`collect_into_vec`, per-point-local contributions reduced in
  point-index order) → bit-for-bit identical result.
- Why parallel is a pure throughput option and serial is the WCET baseline.
- The `h_ang` angular Hessian table and the PR #1217 d1 sign fix that made it exact (link to
  [Divergences](../port/divergences.md)).

> Status: outline (draft to be written).
> Source: `src/derivatives.rs`, `src/ndt.rs` (`compute_derivatives[_parallel]`), `tests/` bit-identity
> tests.
