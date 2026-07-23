# Voxel grid and kd-tree

The target-map data structure the align path queries. It ports the C++ `MultiVoxelGridCovariance`.

## Structures

- **`Leaf`** — one occupied voxel: its centroid and inverse covariance (the Gaussian the score
  evaluates against).
- **`VoxelGrid`** — one map tile's voxels, built from a point cloud.
- **`VoxelGridMap`** — the multi-tile map (a `BTreeMap<u64, VoxelGrid>`), plus a flat leaf list and a
  kd-tree over centroids for neighbor queries.

## Building a grid

`VoxelGridMap::new([leaf_size; 3], min_points, eig_mult)` sets the voxel size and conditioning knobs.
`add_target(points, id)` buckets `points` into voxels; a voxel with at least `min_points` points
becomes a `Leaf` with a mean + covariance, and the covariance's smallest eigenvalues are inflated by
`eig_mult` so thin/planar cells stay invertible (the pcl default 6 / 0.01). Adding or removing a tile
invalidates the kd-tree.

## The kd-tree

`create_kdtree()` flattens all tiles' leaves in id order (matching the C++ `std::map` order) and
builds a 3-D kd-tree over the centroids. `radius_search(point, radius, max_nn, out)` returns the flat
leaf indices whose centroid is within `radius`, up to `max_nn`; `leaf(idx)` fetches a leaf by that
flat index. The tree splits on `axis = depth % 3`.

## Cost and the WCET residual

Each align iteration does one `radius_search` per source point, capped at `MAX_NEIGHBORS`. The
kd-tree traversal is worst-case `O(N_leaves)` for adversarial point distributions — an **accepted
residual** for the [WCET contract](../rt/wcet.md), benign for physical, roughly-uniform voxel maps.

> Source: `src/voxel_grid.rs`, `src/kdtree.rs`.
