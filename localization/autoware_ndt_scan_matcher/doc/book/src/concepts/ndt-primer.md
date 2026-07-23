# NDT scan matching primer

Enough Normal Distributions Transform (NDT) background to read the rest of the book. NDT estimates a
vehicle's pose by aligning a live LiDAR scan to a prebuilt point-cloud map.

## The registration problem

Given a **target** map (points in the `map` frame) and a **source** scan (points in `base_link`),
find the rigid transform `T(p)` — a 6-DoF pose `p = [x, y, z, roll, pitch, yaw]` — that best places
the source onto the map. NDT differs from ICP in how it represents the target: instead of matching
point-to-point, it **voxelizes the target into per-cell normal distributions** and scores the source
against those smooth Gaussians. That makes the objective differentiable and cheap to evaluate, so a
few Newton steps converge.

## The voxel grid

The target is bucketed into a regular grid of `resolution`-sized voxels. Each voxel with at least
`min_points` points is summarized by a **mean** and an **inverse covariance** (its Gaussian). The
covariance is conditioned by inflating its smallest eigenvalues (`eig_mult`) so thin/planar cells
stay invertible. This is the C++ `MultiVoxelGridCovariance`; in Rust it is
[`voxel_grid::VoxelGridMap`](../arch/voxel-grid.md), queried by a kd-tree over voxel centroids.

## The score, gradient, and Hessian

For a candidate pose `p`, each source point is transformed by `T(p)`, its nearest voxel(s) are found
by radius search, and the point contributes a Gaussian response of its offset from the voxel mean.
Summed over the cloud this is the **NDT score**; its first and second derivatives w.r.t. `p` give the
**gradient** and **Hessian** that drive a Newton step. The mixture uses two Gauss constants derived
from `outlier_ratio` and `resolution` (a robustifying outlier model). The derivative kernels live in
[Serial and parallel derivatives](../arch/derivatives.md); the optimization loop (transform →
derivatives → 6×6 SVD solve → step) is [The align hot path](../arch/align.md).

The two scores NDT reports from a converged pose — transform probability and nearest-voxel
transformation likelihood — are covered in [Scores: TP and NVTL](scores.md).

> Source: `src/voxel_grid.rs`, `src/derivatives.rs`, `src/ndt.rs`, `src/transform.rs`
> (`gauss_constants`); the package `README.md`. See the NDT references in the
> [appendix](../appendix/references.md).
