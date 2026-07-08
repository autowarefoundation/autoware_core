# NDT scan matching primer

A short primer on Normal Distributions Transform (NDT) registration, enough to read the
architecture chapters without prior NDT background.

Planned contents:

- The localization problem: aligning a live sensor cloud to a prebuilt point-cloud map.
- NDT vs ICP — why NDT voxelizes the target into per-voxel normal distributions.
- Building the voxel grid: leaf size, mean + covariance per voxel, the eigenvalue-inflation
  conditioning (`min_points`, `eig_mult`).
- The score/gradient/Hessian a candidate pose induces, and the Gauss constants
  (`outlier_ratio`, `resolution`).
- The optimization loop at a glance (Newton step via SVD) — forward-references
  [The align hot path](../arch/align.md).

> Status: outline (draft to be written).
> Source: `src/voxel_grid.rs`, `src/derivatives.rs`, `src/transform.rs` (`gauss_constants`),
> package `README.md`; standard NDT references (see [References](../appendix/references.md)).
