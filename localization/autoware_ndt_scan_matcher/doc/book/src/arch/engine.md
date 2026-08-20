# The NDT engine

`NdtEngine` (`src/engine.rs`) is the persistent handle that wraps the target map + params over a
stable interface. C++ drives incremental map updates, alignment, and scoring through it. The
defining design choices are **`&self`-only methods** and **interior mutability**, chosen so a
shared `&NdtEngine` is sound across concurrent ROS callbacks _without an external mutex_.

## State

The engine holds its mutable state behind interior-mutability cells:

- **`EngineState`** — the target `VoxelGridMap`, the `NdtParams`, the convergence params, the
  covariance config, and the cell-id → tile-id mapping.
- **regularization** — a tiny separate cell, so setting it per frame never clones the map.
- **align scratch** — the reused workspace + last result (see [MatchScratch](scratch.md)).

The config API is a set of `&self` setters that publish new state: `set_params`,
`set_convergence_params`, `set_covariance_config`, `set_regularization`. Map lifecycle:
`add_target` / `add_target_bytes`, `remove_target*`, `create_kdtree`, and `commit_from` (the
atomic map-update commit). See [Engine state and the config API](engine-state.md).

## Concurrency: three configurations

The interior-mutability cell is chosen at compile time. This is the crate's central concurrency
decision; the full treatment is in [Concurrency and interior mutability](concurrency.md).

| Build                    | Cell                           | Read/align path                            | Scratch          | `Sync` |
| ------------------------ | ------------------------------ | ------------------------------------------ | ---------------- | ------ |
| **std** (default)        | `ArcSwap<EngineState>`         | `load` an immutable snapshot **lock-free** | thread-local     | yes    |
| **`no_std` single-core** | `RefCell<Arc<…>>`              | borrow                                     | engine-owned     | **no** |
| **`no_std` `mt`**        | `awkernel_sync::Mutex<Arc<…>>` | short critical section (refcount bump)     | **caller-owned** | yes    |

Key properties in the concurrent configs (std, `mt`):

- **No lock is held across an alignment.** The align path loads an `Arc` snapshot and works on
  it; a concurrent map update publishes a _new_ state, and the old snapshot lives until its last
  reader drops it.
- **Map updates never mutate in place.** `apply_map_update` builds the new map on a **private
  staging engine**, then `commit_from` swaps it in with a single atomic store — a lock-free
  double-buffer. A concurrent align therefore never observes a partial or kd-tree-less map.
- A compile-time `assert_send_sync::<NdtEngine>()` proves `Sync` in the std/`mt` configs; the
  single-core `no_std` engine is intentionally `!Sync`, so the compiler rejects sharing it.

This replaces the C++ giant `ndt_ptr_` mutex on the hot path.

## What the engine is _not_

The engine is the read/update data structure, not the control loop. The WCET-bounded computation
is the free function [`ndt::align`](align.md); node-level state (pose buffers, activation) lives
in the [`NdtScanMatcherRs` shell](overview.md), not here.

## Sub-chapters

- [Engine state and the config API](engine-state.md)
- [Concurrency and interior mutability](concurrency.md)
- [MatchScratch and the align entry points](scratch.md)

> Source: the module doc at the top of `src/engine.rs`; `tests/concurrency.rs`.
