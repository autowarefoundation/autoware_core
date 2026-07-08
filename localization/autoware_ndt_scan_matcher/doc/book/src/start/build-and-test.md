# Build and test

`autoware_ndt_scan_matcher` is a standard ROS 2 (Humble) `ament_cmake` package. The Rust crate
`autoware_ndt_scan_matcher_rs` is compiled as part of the package build (via corrosion/cargo) and
linked into the C++ library over the C ABI.

## Building with colcon

From your Autoware Core colcon workspace root, after `rosdep install` has satisfied dependencies:

```sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-up-to autoware_ndt_scan_matcher \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Selecting the backend (C++ vs Rust)

The package builds one of two backends, chosen by the CMake option **`NDT_USE_RUST`** (defined in
`CMakeLists.txt`):

| `NDT_USE_RUST` | Backend | Notes |
|---|---|---|
| `OFF` (default) | legacy C++ (`multigrid_ndt_omp`) | No dependency on Rust/corrosion at all. |
| `ON` | Rust port over FFI | Builds the nested `autoware_ndt_scan_matcher_rs` crate via corrosion, generates the cbindgen C header, links it, and compiles the C++ shell with `-DNDT_USE_RUST` so the node/map-update translation units dispatch to the Rust engine. |

Build the **Rust** backend by passing the flag:

```sh
colcon build --symlink-install \
  --packages-up-to autoware_ndt_scan_matcher \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DNDT_USE_RUST=ON
```

The two backends are selected at the *translation-unit* level (not with in-function `#ifdef`s).

## Running the tests

```sh
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select autoware_ndt_scan_matcher --return-code-on-test-failure
colcon test-result --verbose
```

> **The Rust test suite only exists under `-DNDT_USE_RUST=ON`.** With the default `OFF` build,
> `colcon test` runs only the C++/legacy tests (the launch test, the `standard_sequence_*` /
> `once_initialize_*` / `particles_num_*` / `missing_sensor_*` / `align_service_tpe_baseline`
> integration gtests, and the fast `test_estimate_covariance` / `test_ndt_scan_matcher_helper`).
> Building with `-DNDT_USE_RUST=ON` additionally registers the crate's own `cargo test` (run through
> CTest as `autoware_ndt_scan_matcher_rs_cargo_test`) **plus the ~17 Rust differential/FFI gtests**
> (`test_voxel_grid`, `test_align`, `test_ndt_engine`, `test_estimate_covariance_multi`,
> `test_tpe_ffi`, `test_convergence_verdict`, `test_map_update_verdict`, `test_sensor_points_match`,
> …). So to exercise the port through colcon, build with `-DNDT_USE_RUST=ON` first.

Target a single ctest by regex:

```sh
colcon test --packages-select autoware_ndt_scan_matcher \
  --ctest-args -R test_estimate_covariance
```

**Fast vs slow.** The fast math tests are `test_estimate_covariance` and
`test_ndt_scan_matcher_helper`. The `standard_sequence_*` / launch tests are slow (300 s timeouts,
need PCD maps), so filter with `--ctest-args -R` while iterating.

## Working on the Rust crate directly

The crate lives at `autoware_ndt_scan_matcher/autoware_ndt_scan_matcher_rs/`. Iterating with cargo
is much faster than a full colcon build, and covers everything except the ROS-message FFI shims
(see the `ros` note below):

```sh
cd .../autoware_ndt_scan_matcher/autoware_ndt_scan_matcher_rs

# Tests
cargo test               # unit + doctests + integration tests (tests/zero_alloc.rs, tests/concurrency.rs)
cargo test --lib         # unit tests only (fastest)
cargo test --doc         # the doctests shown throughout this book

# Lints (the hardening gate — see Quality Gates)
cargo clippy --all-targets -- -D warnings                                   # std (default)
cargo clippy --no-default-features --features mt,awkernel_sync/std -- -D warnings   # no_std multi-core

# no_std build gate (the portable core must compile without std)
rustup target add x86_64-unknown-none            # once, if not already installed
cargo rustc --no-default-features --lib --target x86_64-unknown-none --crate-type rlib
#   (repeat with aarch64-unknown-none for the arm64 kernel target)

# API reference
cargo doc --no-deps --open
```

Notes:

- The **`ros` feature** builds the `geometry_msgs` bindgen bindings, which need the rosidl C headers
  on the include path. That is wired up by colcon (the CTest `autoware_ndt_scan_matcher_rs_cargo_test`
  runs `cargo test --features ros` with `ROS_INCLUDE_DIRS` set), so run the `ros`-gated shims through
  colcon rather than bare `cargo`. Plain `cargo test` covers everything else.
- The single-core `no_std` config is verified with the `cargo rustc … --target …-unknown-none` gate
  above, **not** `cargo clippy --no-default-features` (that fails standalone — a `no_std` staticlib
  needs a `#[panic_handler]` from the final binary).
- **Miri** (unsafe FFI) and **coverage** are in [Behavior equivalence and verification](../port/verification.md);
  **benchmarks** are in [Benchmarking](../quality/benchmarks.md).

See [Feature flags and build configurations](features.md) for the `std` / `parallel` / `mt` / `ros`
matrix, and [Using the Rust crate](using-the-crate.md) for the API tour.
