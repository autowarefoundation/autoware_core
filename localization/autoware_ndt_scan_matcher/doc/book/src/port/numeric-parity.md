# Numeric parity

Keeping the Rust math bit-comparable to the C++/pcl reference, so the [differential
tests](differential.md) can hold tight tolerances.

## The f32 `Matrix4f` pipeline

The per-iteration cloud transform runs in **f32**, mirroring the C++ `Matrix4f` pipeline exactly —
including the deliberate `f32`/`usize`↔int casts that the crate's lint allowlist documents. Matching
the quantization (not "improving" it to f64) is what keeps the poses bit-comparable.

## Correctly-rounded scalar math

`libm` provides `sqrt`/trig in both std and `no_std`. IEEE `sqrt` is correctly rounded, so it matches
`std::sqrt` and keeps the differential oracle exact; the Gauss constants (`gauss_constants` from
`outlier_ratio` / `resolution`) are computed identically.

## Serial/parallel bit-identity

The parallel derivative reduction is order-preserving, so it is bit-for-bit identical to serial (see
[Serial and parallel derivatives](../arch/derivatives.md)) — parallelism never perturbs parity.

## The Hessian

The NDT angle Hessian (`h_ang`) is the exact analytic second derivative since the PR #1217 "d1"
pitch² sign fix; the full 6×6 Hessian now finite-difference-validates. Historically this was the one
place the port intentionally reproduced a C++ quirk — see [Divergences from upstream](divergences.md).

## Tolerances

The differential checks use: pose translation ≤ 1e-3 m, rotation ≤ 1e-3 rad, TP / NVTL ≤ 1e-4,
iteration count **exact**.

> Source: `src/transform.rs`, `src/ndt.rs`, `src/derivatives.rs`.
