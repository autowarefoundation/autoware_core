# Concurrency and interior mutability

The three interior-mutability configurations of the engine, in depth. This is the crate's central
concurrency design.

Planned contents:

- The `Swap<T>` cell abstraction and its three backends: `arc_swap::ArcSwap` (std),
  `RefCell<Arc<…>>` (no_std single-core), `awkernel_sync::Mutex<Arc<…>>` (`mt`).
- `swap_load` / `swap_store` / `swap_rcu` uniform API; the `mt` optimistic `Arc::ptr_eq` retry
  that runs the closure outside the lock (never a deep clone with interrupts disabled).
- Why no lock is held across an alignment; snapshot-then-align.
- The lock-free map-update double-buffer: private staging engine → `commit_from`.
- `&self`-only + the compile-time `assert_send_sync` proof; single-core `!Sync` by design.
- Scratch placement: thread-local (std) vs engine-owned (single-core) vs caller-owned (`mt`).

> Status: outline (draft to be written).
> Source: the module doc + `Swap`/`swap_*` helpers in `src/engine.rs`; `tests/concurrency.rs`.
