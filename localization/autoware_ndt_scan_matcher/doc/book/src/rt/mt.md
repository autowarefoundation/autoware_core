# The `mt` multi-core engine

The multi-core `no_std` (kernel) configuration.

Planned contents:

- `awkernel_sync::Mutex` cells whose guards disable interrupts → every critical section is a few
  instructions (an `Arc` refcount bump or a pointer swap), never an align.
- The rcu publish with the optimistic `Arc::ptr_eq` retry (closure runs outside the lock; old
  snapshots drop outside the lock).
- Caller-owned `MatchScratch` (the implicit-scratch API is compiled out); the ownership rule.
- `Sync` engine like std; the pinned `awkernel_sync` git rev and the final-binary backend feature
  requirement (`x86`/`aarch64`/`rv64`/`rv32` / `awkernel_sync/std`).
- Verify commands and the known lang-item errors for a standalone build.

> Status: outline (draft to be written).
> Source: `src/engine.rs` (`mt` cfg paths), `Cargo.toml` (`mt` feature), `examples/threads_ndt.rs`.
