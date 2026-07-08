# Voxel grid and kd-tree

The target-map data structure the align path queries.

Planned contents:

- `Leaf` (centroid + inverse covariance), `VoxelGrid` (per-tile), `VoxelGridMap` (multi-tile,
  mirrors `MultiVoxelGridCovariance`).
- Building a grid: leaf size, `min_points`, `eig_mult` eigenvalue conditioning.
- Flattening tiles in id order and building the kd-tree over centroids (`create_kdtree`).
- `radius_search(point, radius, max_nn, out)` and the `MAX_NEIGHBORS` cap that bounds the align.
- Tile add/remove and kd-tree invalidation.
- The kd-tree traversal cost and its worst-case (an accepted WCET residual for adversarial
  distributions).

> Status: outline (draft to be written).
> Source: `src/voxel_grid.rs`, `src/kdtree.rs`.
