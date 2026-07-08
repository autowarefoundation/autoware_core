# Scope and non-goals

Exactly which responsibilities are Rust's and which stay in C++.

## Rust owns (the algorithm)

- The NDT engine and voxel-grid map, the align kernel, and the TP/NVTL scores.
- Covariance estimation and the align-service pose search (TPE).
- Node-level algorithmic state: pose buffers, activation flag, map-update policy/bookkeeping.
- The convergence decision, publish/skip decisions, and diagnostics *content*.

These live in the `autoware_ndt_scan_matcher_rs` crate — a portable `no_std` core plus a `std` FFI
shell (see [System overview](../arch/overview.md)).

## C++ / rclcpp owns (the runtime boundary)

- `rclcpp::Node` construction; publishers, subscribers, services, timers.
- ROS 2 parameter declaration; callback entry points.
- TF lookup, the map-loader service call, and the actual message publication.
- Diagnostics publication through the existing ROS APIs; component registration.

C++ requests these side effects on Rust's behalf only through the [Host vtable](../arch/host-vtable.md).

## Non-goals

- No full ROS 2 wrapper in Rust; no `rclrs` as the node runtime.
- No STL / Eigen / PCL / `rclcpp` / `tf2` types across the FFI boundary.
- Rust never subscribes to topics or calls `rclcpp` directly.
- No borrowed C++ message memory retained in Rust (copy into Rust-owned data first).

## Backend selection

Which implementation compiles is chosen at build time by the CMake option `NDT_USE_RUST` (default
`OFF` = legacy C++; `ON` = the Rust port over FFI), applied at the translation-unit level rather than
with in-function `#ifdef`s. See [Build and test](../start/build-and-test.md#selecting-the-backend-c-vs-rust).

> Source: the package `README.md`; `src/lib.rs` (module layout + `no_std` gating); `CMakeLists.txt`
> (`NDT_USE_RUST` translation-unit selection).
