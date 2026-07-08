# Concurrency and interior mutability

The engine is `&self`-only so a shared `&NdtEngine` is sound across concurrent ROS callbacks
*without* an external mutex. Its mutable state sits behind an interior-mutability cell whose backend
is chosen at compile time — the crate's central concurrency decision.

## The `Swap<T>` cell

A `Swap<T>` alias abstracts the backend, with uniform `swap_new` / `swap_load` / `swap_store` /
`swap_rcu` free functions:

| Build | `Swap<T>` backend | read/align path |
|---|---|---|
| **std** (default) | `arc_swap::ArcSwap<T>` | `load_full()` — an owned `Arc<T>` snapshot, **lock-free** |
| **`no_std` single-core** | `RefCell<Arc<T>>` | `borrow().clone()` |
| **`no_std` `mt`** | `awkernel_sync::Mutex<Arc<T>>` | short critical section: one `Arc` refcount bump |

`swap_load` returns an owned `Arc<T>` so a reader holds a stable snapshot while a concurrent
`store`/`rcu` publishes a new version; the old `Arc` lives until its last reader drops it.

## Read-copy-update and the `mt` retry

Config/map changes go through `swap_rcu`: build the next value from the current one, publish
atomically. Under `mt` the closure runs **outside** the lock (it may deep-clone a whole map — which
must never happen with interrupts disabled) and publishes with an optimistic `Arc::ptr_eq` retry;
old snapshots drop outside the lock too. Writers (params setup, map commit) are rare, so it converges
immediately in practice.

## Load-free hot path + lock-free map update

- **No lock is held across an alignment.** The align path `load`s a snapshot and works on it.
- **Map updates never mutate in place.** [`apply_map_update`](map-update.md) builds the new map on a
  private staging engine, then `commit_from` swaps it in with one atomic store — a concurrent align
  never sees a partial or kd-tree-less map.

## Send/Sync

A compile-time `const _: () = assert_send_sync::<NdtEngine>()` proves the engine is `Sync` in the
std and `mt` configs. The single-core `no_std` engine is intentionally `!Sync` (`RefCell`), so the
compiler rejects sharing it. Per-align scratch placement follows the config —
[MatchScratch](scratch.md).

> Source: the module doc + `Swap` / `swap_*` helpers in `src/engine.rs`; `tests/concurrency.rs`.
