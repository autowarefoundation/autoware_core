# C++ to Rust map

A cross-reference from C++ symbols to their Rust counterparts, for reviewers tracing a change across
the boundary.

## Engine / algorithm

| C++                                                          | Rust                                                                            |
| ------------------------------------------------------------ | ------------------------------------------------------------------------------- |
| `pclomp::MultiGridNormalDistributionsTransform` (`ndt_omp/`) | `engine::NdtEngine` + `ndt::align`                                              |
| `MultiVoxelGridCovariance`                                   | `voxel_grid::VoxelGridMap` (+ `voxel_grid::Leaf`, `kdtree::KdTree`)             |
| `computeAngleDerivatives` / `computeDerivatives`             | `derivatives::compute_angle_derivatives`, `ndt::compute_derivatives[_parallel]` |
| `estimate_covariance` (multi-NDT / score / Laplace)          | `cov_estimate::*`, `covariance::*`                                              |
| `TreeStructuredParzenEstimator` (`localization_util`)        | `tpe::TreeStructuredParzenEstimator`                                            |
| `SmartPoseBuffer` (`localization_util`)                      | `pose_buffer::PoseBuffer`                                                       |
| `count_oscillation`, covariance rotate helpers               | `helper::*`                                                                     |

## Node shell

| C++                                                                               | Rust                                                                                 |
| --------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------ |
| `NDTScanMatcher` (`ndt_scan_matcher_core.cpp`)                                    | thin `rclcpp` shell → `node_handle::NdtScanMatcherRs` (opaque handle)                |
| `callback_sensor_points_main`                                                     | `sensor_points::*` (prologue) + `node`/`engine` `run_align` + `on_sensor_points`     |
| `callback_initial_pose` / `callback_regularization_pose` / `service_trigger_node` | `node::on_*` + the handle's pose buffers / activation                                |
| `MapUpdateModule` (`map_update_module*.cpp`)                                      | `node_handle::MapUpdateState` + `scan_matcher::apply_map_update` + `node_map_update` |
| `service_ndt_align` / `align_pose`                                                | `node_align_service::*` (decision + `run_align_service_search_impl`)                 |
| convergence gate in the sensor callback                                           | `convergence::evaluate_convergence`                                                  |
| ROS side effects (publish, TF, log, map-load)                                     | the `AwHost` vtable → `host::Host` trait                                             |

## Boundary

| C++                                                    | Rust                                                                           |
| ------------------------------------------------------ | ------------------------------------------------------------------------------ |
| `NDTScanMatcherRS` RAII wrapper + `make_aw_ndt_params` | `autoware_ndt_scan_matcher_rs_new/_free`, `node_handle::{Params, AwNdtParams}` |
| `AwHost` / `AwPointCloud2View` / `AwPose` trampolines  | `ffi_host`, `sensor_points`; derefs via `ffi_ptr`; panic containment in `ffi`  |

> Source: cross-reference of the C++ package `src/` (and `ndt_omp/`) against the Rust crate `src/`.
