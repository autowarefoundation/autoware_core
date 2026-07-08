# Test taxonomy

The kinds of tests and what each one guarantees.

Planned contents:

- **Unit** (`cargo test --lib`) — kernels, pose buffers, convergence, covariance, FFI shims via
  Rust direct calls.
- **Doctests** (`cargo test --doc`) — the public-API examples in this book.
- **Property** — `voxel_grid`/`kdtree` vs brute force; covariance symmetry/PSD.
- **Differential (ON vs OFF)** — the authoritative oracle (see
  [Differential testing](../port/differential.md)).
- **Zero-alloc** (`tests/zero_alloc.rs`) and **WCET** (`examples/wcet_frame.rs`).
- **Concurrency** (`tests/concurrency.rs`).
- **Integration/launch** (`standard_sequence_*`) — slow, need PCD maps.
- **Miri** for the unsafe FFI; coverage as a diagnostic map.
- How to run each and how to filter.

> Status: outline (draft to be written).
> Source: crate `tests/`, `examples/`; the package's C++ gtests and launch tests.
