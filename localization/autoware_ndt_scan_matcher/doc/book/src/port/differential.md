# Differential testing

The ON-vs-OFF oracle in practice.

Planned contents:

- Building both paths (`NDT_USE_RUST` ON/OFF) and diffing observables.
- Unit gtests (helpers, covariance), property tests (`voxel_grid`/`kdtree` vs brute force).
- The `standard_sequence_*` integration tests and how to run/filter them.
- `extern "C"` shims get Rust direct-call tests (llvm-cov sees only Rust).
- Observables compared: pose, covariance, TP, NVTL, iteration count, convergence status,
  diagnostics, service response, map-update behavior.

> Status: outline (draft to be written).
> Source: the crate `tests/` + the package's C++ gtests and `standard_sequence_*` launch tests.
