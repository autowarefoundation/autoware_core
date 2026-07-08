# Porting strategy and phases

How the migration was staged: consolidation, not greenfield. The crate already had the hard parts;
the phases mostly *moved ownership* and *thinned the boundary*.

Planned contents:

- The architectural target (thin rclcpp shell + Rust-owned core) and the migration rules.
- The phase sequence (Phase 0 → 8): FFI stabilization, node-state ownership, per-callback
  forwarders (initial pose, regularization, trigger), the sensor-callback body (Phase 5), map-update
  state (Phase 6), the align service (Phase 7, TPE deferred), and adapter/`#ifdef` removal (Phase 8).
- Build selection at the translation-unit / target level instead of function-body `#ifdef`.
- Current status snapshot and what remains.

> Status: outline (draft to be written).
> Source: the C++ package `src/` (build-selected translation units) and the Rust crate `src/`
> (`node*.rs`).
