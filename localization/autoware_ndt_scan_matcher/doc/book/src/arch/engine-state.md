# Engine state and the config API

`EngineState` and the `&self` setters that publish it.

Planned contents:

- `EngineState` fields: `VoxelGridMap`, `NdtParams`, `ConvergenceParams`, `CovarianceConfig`,
  cell-id → tile-id `id_map`.
- Config setters: `set_params`, `set_convergence_params`, `set_covariance_config`,
  `set_regularization` (separate tiny cell so it never clones the map).
- Map lifecycle: `add_target` / `add_target_bytes`, `remove_target*`, `create_kdtree`,
  `has_target`, `commit_from`, `clone_empty`.
- The `EngineState` clone/publish (rcu) discipline and how it ties to
  [Concurrency](concurrency.md).

> Status: outline (draft to be written).
> Source: `src/engine.rs`.
