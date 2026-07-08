# Benchmarking

C++ vs Rust performance comparison methodology.

Planned contents:

- Principle: **capture-once, replay-everywhere** (record real inputs, replay deterministically).
- The three tiers: **L1** node / end-to-end (primary, ships first), **L2** kernel micro-benchmark
  (locate where time goes), **L3** offline differential replay (realistic distributions, zero
  jitter).
- Fidelity controls & pitfalls to keep the comparison fair.
- Metrics & reporting; phasing/milestones; CMake wiring precedent.

> Status: outline (draft to be written).
> Source: the package benchmark harnesses (added alongside this chapter).
