# Module index

One line per Rust source module (`src/*.rs`). To be filled:

- `lib.rs` — crate root, feature gating, C ABI smoke shims.
- `engine.rs` — persistent `NdtEngine` handle, `MatchScratch`, config/map/align API.
- `ndt.rs` — `align`, derivative assembly, `NdtParams`/`AlignResult`/`AlignWorkspace`.
- `voxel_grid.rs` / `kdtree.rs` — target map + spatial index.
- `derivatives.rs` — score/gradient/Hessian kernels.
- `convergence.rs` / `covariance.rs` / `cov_estimate.rs` — pure decision/estimation kernels.
- `transform.rs` — SE3 transforms + Gauss constants.
- `tpe.rs` — Tree-Structured Parzen Estimator.
- `pose_buffer.rs` — time-ordered pose interpolation.
- `host.rs` / `scan_matcher.rs` — portable ports + orchestration.
- `ffi.rs` / `ffi_ptr.rs` / `ffi_host.rs` — the FFI boundary.
- `node.rs` / `node_handle.rs` / `node_map_update.rs` / `node_align_service.rs` /
  `sensor_points.rs` — the `std` ROS node shell.
- `helper.rs` — pure C++ helper ports (`rotate_covariance`, `count_oscillation`).

> Status: outline (draft to be written).
> Source: the crate `src/` module docs.
