# Module index

One line per Rust source module (`autoware_ndt_scan_matcher_rs/src/`).

## Core algorithm (portable, `no_std`)

- `lib.rs` — crate root, feature gating, `nalgebra` re-export, C-ABI smoke/thread-pool shims.
- `engine.rs` — persistent `NdtEngine` handle, `MatchScratch`, config/map/align API + object-level FFI.
- `ndt.rs` — `align`, derivative assembly, `NdtParams` / `AlignResult` / `AlignWorkspace`.
- `derivatives.rs` — angular + per-point score/gradient/Hessian kernels.
- `voxel_grid.rs` / `kdtree.rs` — target voxel map + spatial index.
- `convergence.rs` — the pure convergence verdict.
- `covariance.rs` / `cov_estimate.rs` — pose-covariance math + the four estimation modes.
- `transform.rs` — SE3 transforms, Gauss constants, euler↔matrix.
- `tpe.rs` — Tree-Structured Parzen Estimator.
- `pose_buffer.rs` — time-ordered pose interpolation buffer (`SmartPoseBuffer` port).
- `helper.rs` — pure C++ helper ports (`rotate_covariance`, `count_oscillation`).

## Ports & orchestration (portable)

- `host.rs` — the `MapSource` / `OutputSink` / `Clock` / `Host` traits + result types.
- `scan_matcher.rs` — `ScanMatcher` over the Host ports (+ `apply_map_update`).

## FFI boundary

- `ffi_ptr.rs` — audited raw-pointer helpers + guard macros (core + alloc).
- `ffi.rs` — panic-containment boundary + `AwStatus`/`Error` (std).
- `ffi_host.rs` — the `AwHost` ROS side-effects vtable + `AwStr`/`AwPose` (std).

## `std` ROS node shell

- `node.rs` — thin pose/trigger callbacks + convergence FFI.
- `node_handle.rs` — the opaque `NdtScanMatcherRs` handle, `Params` / `AwNdtParams`, node state.
- `node_map_update.rs` — the map-source vtable + `apply_map_update` glue.
- `node_align_service.rs` — the align-service decisions + Rust-owned TPE search + trace ABI.
- `sensor_points.rs` — the sensor-callback prologue (decode / TF / transform / validation).

## Generated

- `ros_msgs` — bindgen `geometry_msgs` C structs (`ros` feature; allow-listed).

> Source: the crate `src/` module docs.
