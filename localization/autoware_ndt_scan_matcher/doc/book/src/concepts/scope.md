# Scope and non-goals

Exactly which responsibilities moved to Rust and which stay in C++.

Planned contents:

- **Rust owns:** NDT engine + voxel map, align kernel, scores, covariance, align-service search,
  pose buffers, convergence, map-update policy, diagnostics content, publish-decision logic.
- **C++ / rclcpp keeps:** node construction, pub/sub/services/timers, parameter declaration,
  callback entry points, TF lookup, map-loader service call, actual message publication,
  diagnostics publication, component registration.
- Non-goals: no full ROS 2 wrapper in Rust, no `rclrs`, no STL/Eigen/PCL/rclcpp/tf2 types across
  FFI, no borrowed C++ memory stored in Rust.
- The "Definition of Done" checklist.

> Status: outline (draft to be written).
> Source: the package `README.md`; `src/lib.rs` (module layout + no_std gating).
