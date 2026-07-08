# C++ to Rust map

A cross-reference from C++ symbols/files to their Rust counterparts, for reviewers tracing a
change across the boundary.

Planned contents (table to be filled):

- C++ `NDTScanMatcher` callbacks/services ↔ Rust `on_*` FFI entry points.
- C++ `MultiVoxelGridCovariance` ↔ `voxel_grid::VoxelGridMap`.
- C++ NDT `align`/`computeTransformation` ↔ `ndt::align`.
- C++ `SmartPoseBuffer` ↔ `pose_buffer::PoseBuffer`.
- C++ convergence check ↔ `convergence::evaluate_convergence`.
- C++ covariance modes ↔ `cov_estimate` / `covariance`.
- C++ `TreeStructuredParzenEstimator` ↔ `tpe::TreeStructuredParzenEstimator`.
- Node-state fields ↔ `NdtScanMatcherRs` handle members.

> Status: outline (draft to be written).
> Source: cross-reference of the C++ package `src/` and the Rust crate `src/`.
