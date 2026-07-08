# Panic-free, bounded execution

The crate's lint hardening viewed through a real-time lens.

Planned contents:

- No `unwrap`/`expect`/`panic`/`unreachable`/indexing/slicing in non-test code → no panic source
  on the RT path.
- Explicit checked/saturating/wrapping integer arithmetic (`overflow-checks = true` makes a silent
  integer op a panic source); float math uses `#[allow(arithmetic_side_effects)]` only in kernels.
- No lossy `as` casts (TryFrom); no `let _` discarding a `Result`.
- Bounded loops and buffers on the align path; no blocking, no allocation, no logging.
- How a real-time review of each patch works; the relationship to the [WCET contract](wcet.md).

> Status: outline (draft to be written).
> Source: `Cargo.toml` `[lints]`, `src/ndt.rs`.
