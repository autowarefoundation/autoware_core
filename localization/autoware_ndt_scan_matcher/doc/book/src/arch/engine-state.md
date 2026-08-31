# Engine state and the config API

The engine's data lives in `EngineState`, published through interior-mutability cells (see
[Concurrency](concurrency.md)). This chapter covers its fields and the `&self` API that mutates them.

## `EngineState`

- `map` — the target [`VoxelGridMap`](voxel-grid.md).
- `params` — the `NdtParams` (resolution, epsilon, step size, iteration cap, outlier ratio,
  `num_threads`).
- `conv` — the `ConvergenceParams` (`converged_param_type` + the two thresholds).
- `cov_config` — the `CovarianceConfig` (estimation mode, scale, temperature, configured 6×6, XY
  search offsets).
- `id_map` / `next_id` — the cell-id (raw bytes) → tile-`u64` mapping the engine owns for
  string-keyed tiles.

Regularization is **not** part of `EngineState`; it lives in its own tiny cell so setting it per
frame never clones the map.

## Config setters (`&self`)

`set_params`, `set_convergence_params`, `set_covariance_config`, and `set_regularization` publish new
state. Each is a read-copy-update: clone the current state, mutate the field, publish (see
[Concurrency](concurrency.md)). `set_params` also rebuilds the (still-empty) map at the new
resolution, mirroring the C++ order (params before `addTarget`).

## Map lifecycle

- `add_target(points, id)` / `add_target_bytes(points, cell_id)` — register a tile (byte-id variants
  assign a fresh `u64` on first use); `remove_target*` drop one.
- `create_kdtree()` — rebuild the kd-tree over voxel centroids after tile changes.
- `has_target()` — whether any tile is loaded.
- `commit_from(&src)` — atomically publish another engine's fully-built state (the map-update commit;
  see [Map update](map-update.md)).
- `clone_empty()` — copy the config but start with an empty map (the rebuild path).

> Source: `src/engine.rs`.
