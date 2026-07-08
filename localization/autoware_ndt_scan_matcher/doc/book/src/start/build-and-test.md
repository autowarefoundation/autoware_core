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

## Running the tests

```sh
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select autoware_ndt_scan_matcher --return-code-on-test-failure
colcon test-result --verbose
```

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
is much faster than a full colcon build:

```sh
cd .../autoware_ndt_scan_matcher/autoware_ndt_scan_matcher_rs
cargo test --lib          # unit tests
cargo test --doc          # the doctests shown throughout this book
cargo clippy --all-targets -- -D warnings   # the lint gate (see Quality Gates)
cargo doc --no-deps --open                  # the API reference
```

See [Feature flags and build configurations](features.md) for the `no_std` / `mt` builds, and
[Using the Rust crate](using-the-crate.md) for the API tour.
