# MatchScratch and the align entry points

`MatchScratch` is the per-align scratch — a reused `AlignWorkspace` plus the last `AlignResult`. It is
never shared mutable state across threads; reusing it across frames keeps the align allocation-free
after warmup ([Zero-allocation guarantees](../rt/zero-alloc.md)). It exposes `result()` (an owned
copy of the last result) and `score_arrays()` (the per-iteration TP/NVTL traces).

## Two families of align methods

- **Implicit-scratch API** — `align`, `align_outcome`, `result`, `score_arrays`, the score helpers.
  These use an implicit scratch and exist only in the std and single-core `no_std` builds. Under std
  the scratch is a **thread-local** (`SCRATCH`, at most one align per thread → exclusive without a
  lock); under single-core `no_std` it is owned by the engine.
- **Explicit `_with` API** — `align_with`, `align_outcome_with`, `estimate_covariance`, the
  `*_with` scorers. The caller passes a `&mut MatchScratch`. This is the **universal** path and the
  **only** one under the `mt` feature.

## Why the implicit API is compiled out under `mt`

A thread-local (std) or engine-owned (single-core) scratch can't be shared safely across the many
tasks of a multi-core kernel. Under `mt` each task/thread owns its own `MatchScratch` and passes it
in explicitly; the implicit variants are `#[cfg]`-compiled out, so a cross-call scratch dependency
cannot even compile. Reuse one scratch per task/thread across frames; never share it.

> Source: `src/engine.rs` (`MatchScratch`, `align*`, the `SCRATCH` thread-local).
