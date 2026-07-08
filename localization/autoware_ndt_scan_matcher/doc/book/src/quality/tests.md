# Test taxonomy

The kinds of tests and what each one guarantees. [Build and test](../start/build-and-test.md) is
the single source for the exact commands; this table maps each kind to its runner.

| Kind | What it guards | How to run |
|---|---|---|
| **Unit** | kernels, pose buffers, convergence, covariance, FFI shims (Rust direct calls) | `cargo test --lib` |
| **Doctests** | the public-API examples in this book | `cargo test --doc` |
| **Integration (crate)** | zero-allocation-per-frame (`tests/zero_alloc.rs`), concurrency (`tests/concurrency.rs`) | `cargo test` |
| **Property** | `voxel_grid`/`kdtree` vs brute force; covariance symmetry/PSD | `cargo test --lib` + gtests |
| **Differential (ON vs OFF)** | the authoritative oracle — Rust vs C++ (see [Differential testing](../port/differential.md)) | `colcon test` after `-DNDT_USE_RUST=ON` (registers `cargo test --features ros` + the ~17 Rust gtests) |
| **Integration/launch** | `standard_sequence_*` etc. — slow, need PCD maps | `colcon test --ctest-args -R standard_sequence` |
| **WCET / bench** | worst-case frame time (`examples/wcet_frame.rs`); C++ vs Rust replay | see [Benchmarking](benchmarks.md) |
| **`no_std` gate** | the portable core compiles without `std` | `cargo rustc --no-default-features --lib --target x86_64-unknown-none --crate-type rlib` |
| **Miri** | unsafe FFI (`ffi_ptr`) soundness | `cargo +nightly miri test` |
| **Coverage** | diagnostic map (not a target) | `cargo llvm-cov` |

> Status: outline (draft to be written).
> Source: crate `tests/`, `examples/`; the package `CMakeLists.txt` gtest registrations and launch
> test.
