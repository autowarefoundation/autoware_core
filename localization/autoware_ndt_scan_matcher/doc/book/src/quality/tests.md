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

## Current results (snapshot — as of 2026-07-08)

Numbers drift as tests are added; regenerate with the commands below.

**Crate (`cargo`, run from `autoware_ndt_scan_matcher_rs/`):**

| Suite | Tests | Command |
|---|---:|---|
| Unit | 192 | `cargo test --lib` |
| Doctests | 9 | `cargo test --doc` |
| `tests/concurrency.rs` | 1 | `cargo test --test concurrency` |
| `tests/zero_alloc.rs` | 1 | `cargo test --test zero_alloc` |
| **Total** | **203** | `cargo test` |

All passing (0 failed). Regenerate: `cargo test`.

**Package (colcon, `-DNDT_USE_RUST=ON`):** the differential/FFI suite is registered only under the
Rust backend — `autoware_ndt_scan_matcher_rs_cargo_test` (the crate's own `cargo test --features
ros` via CTest) plus the Rust gtests `test_voxel_grid`, `test_align`, `test_ndt_engine`,
`test_estimate_covariance_multi`, `test_tpe_ffi`, `test_convergence_verdict`,
`test_node_pose_callbacks`, `test_map_update_verdict`, `test_node_run_align`,
`test_estimate_pose_covariance`, `test_ndt_align_service_decision`,
`test_ndt_scan_matcher_rs_handle`, `test_regularization_buffer`, `test_initial_pose_buffer`,
`test_map_update_state`, `test_sensor_points_prepare`, `test_sensor_points_match`. The always-on
C++/integration suites (`standard_sequence_*`, `once_initialize_*`, `particles_num_*`,
`missing_sensor_*`, `align_service_tpe_baseline`, `test_estimate_covariance`,
`test_ndt_scan_matcher_helper`, launch test) run in both backends. Case counts are not snapshotted
here (the integration tests need PCD maps). Regenerate: `colcon test --packages-select
autoware_ndt_scan_matcher` after a `-DNDT_USE_RUST=ON` build.

## Coverage (snapshot — as of 2026-07-08)

`cargo llvm-cov --lib --summary-only` (tool: `cargo-llvm-cov`, installed in the dev image):

**Total: 88.2% region / 87.7% line / 89.6% function** (14,064 regions, 9,142 lines).

| Module | Line cover |
|---|---:|
| `convergence` / `transform` / `ffi_ptr` | 100% |
| `voxel_grid` / `kdtree` / `helper` / `covariance` / `cov_estimate` / `ndt` | 97–99% |
| `engine` / `derivatives` | 95–98% |
| `tpe` / `pose_buffer` / `ffi` / `node` / `node_align_service` | 92–96% |
| `node_handle` | 86% |
| `sensor_points` | 24% |
| `ffi_host` / `node_map_update` / `scan_matcher` | 0% |

> **The `cargo llvm-cov` number observes only Rust `cargo test`.** The low/0% modules
> (`ffi_host`, `node_map_update`, `scan_matcher`, and most of `sensor_points`) are the FFI-host
> vtable, the ROS map-update glue, and the async orchestration — exercised through the C++ gtests
> (the `-DNDT_USE_RUST=ON` differential suite) or the `examples/tokio_ndt.rs` reference host, which
> `cargo llvm-cov` does not instrument. They are **not** real coverage holes. Coverage here is a
> diagnostic map, not a target (see [Behavior equivalence and verification](../port/verification.md)).

Regenerate: `cargo llvm-cov --lib --summary-only` (HTML report: `cargo llvm-cov --lib --html` →
`target/llvm-cov/html/index.html`).

> Source: crate `tests/`, `examples/`; the package `CMakeLists.txt` gtest registrations and launch
> test; `cargo llvm-cov` output.
