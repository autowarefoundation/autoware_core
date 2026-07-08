# Map update

How the target map is updated at runtime without disturbing concurrent alignment.

Planned contents:

- The policy/state Rust owns (`MapUpdateState`: `last_update_position`, `loaded_map_ids`,
  `need_rebuild`); first-update + keep-up decisions; out-of-range check.
- `apply_map_update`: async `MapSource::load` → build on a **private staging engine** →
  `commit_from` atomic swap. Empty-delta no-op.
- Incremental (`clone`) vs rebuild (`clone_empty`, the `need_rebuild` path).
- The C++ / Rust split: C++ keeps the pcd-loader service I/O + tile apply + debug-map publish;
  Rust owns the decision/state.

> Status: outline (draft to be written).
> Source: `src/scan_matcher.rs` (`apply_map_update`), `src/node_map_update.rs`, `src/engine.rs`
> (`commit_from`, `clone_empty`).
