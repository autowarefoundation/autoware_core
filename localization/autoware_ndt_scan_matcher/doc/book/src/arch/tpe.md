# The TPE pose search

The Tree-Structured Parzen Estimator that drives the align-service (initial-pose) search.

Planned contents:

- The propose/evaluate loop: `get_next_input` → evaluate → `add_trial`.
- Prior sampling (5 dims + uniform yaw), startup trials, then above/below Gaussian KDE
  expected-improvement selection.
- Determinism: a Rust-owned `SplitMix64` + Box-Muller, seeded per request — **not** libstdc++'s
  `std::normal_distribution` (see [Divergences](../port/divergences.md) and
  [Verification](../port/verification.md) for why exact candidate parity is out of scope).
- The C ABI surface (`AwTpe`, `AW_TPE_*` status/direction codes).

> Status: outline (draft to be written).
> Source: `src/tpe.rs`, `src/node_align_service.rs`.
