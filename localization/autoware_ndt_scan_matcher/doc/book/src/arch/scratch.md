# MatchScratch and the align entry points

The per-align scratch and the two families of align methods.

Planned contents:

- `MatchScratch` = reused `AlignWorkspace` + last `AlignResult`; `result()` / `score_arrays()`.
- Implicit-scratch API (`align`, `align_outcome`, `result`, …) — present only under
  std / single-core; backed by a thread-local (std) or the engine (single-core).
- Explicit `_with` API (`align_with`, `align_outcome_with`, `estimate_covariance`, …) — the
  universal path, and the **only** one under `mt`.
- Ownership rule under `mt`: one scratch per task/thread, reused across frames, never shared.
- Why the implicit API is compiled out under `mt` (a cross-call scratch dependency can't compile).

> Status: outline (draft to be written).
> Source: `src/engine.rs` (`MatchScratch`, `align*`, `SCRATCH` thread-local).
