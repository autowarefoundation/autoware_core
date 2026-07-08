# Numeric parity

Keeping the Rust math bit-comparable to the C++/pcl reference.

Planned contents:

- The f32 `Matrix4f` cloud-transform pipeline (mirroring C++ exactly, including the deliberate
  f32/usize casts documented in the lint allowlist).
- `libm` for `sqrt`/trig (IEEE correctly-rounded `sqrt` keeps the differential oracle exact).
- Gauss constants (`outlier_ratio`, `resolution`) parity.
- Serial/parallel bit-identity (order-preserving reduction).
- The NDT angle Hessian (`h_ang`): since the PR #1217 d1 sign fix it is the exact analytic
  Hessian and finite-difference-validates (see [Divergences](divergences.md)).
- Tolerances used in the differential tests.

> Status: outline (draft to be written).
> Source: `src/transform.rs`, `src/ndt.rs`, `src/derivatives.rs`.
