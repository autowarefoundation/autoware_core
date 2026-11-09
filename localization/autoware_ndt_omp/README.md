# autoware_ndt_omp

## Purpose

This package holds ndt_omp, an implementation of NDT (Normal Distributions Transform) scan matching on the CPU with OpenMP. [autoware_ndt_scan_matcher](../autoware_ndt_scan_matcher/README.md) uses it.

The code is derived from <https://github.com/tier4/ndt_omp>. See [the notes on its origin](include/autoware/ndt_omp/README.md) and [its license](include/autoware/ndt_omp/LICENSE).

## Contents

- `pclomp::MultiGridNormalDistributionsTransform` in `autoware/ndt_omp/multigrid_ndt_omp.h`: NDT with a map that consists of several point clouds, which can be added and removed one at a time.
- `pclomp::MultiVoxelGridCovariance` in `autoware/ndt_omp/multi_voxel_grid_covariance_omp.h`: the voxel grid of this map.

The parameter and result types (`pclomp::NdtParams` and `pclomp::NdtResult`) are in [autoware_ndt_scan_matcher_common](../autoware_ndt_scan_matcher_common/README.md).

## Compile options

A package that uses the classes of this package must compile with the same SIMD options as this package. Otherwise, the `Eigen` types in the classes can have a different memory alignment, and the program crashes. See `CMakeLists.txt`.
