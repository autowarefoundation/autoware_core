# Zero-allocation guarantees

Why the align frame allocates nothing after warmup, and how that is enforced.

Planned contents:

- Which buffers are pre-reserved and reused: result `Vec`s, `trans_cloud`, `neighbor_idx`
  (bounded by `MAX_NEIGHBORS`); the stack-only fixed-size 6×6 SVD.
- The warmup model (buffers grow on the first align, steady-state after).
- The `tests/zero_alloc.rs` measurement (how it asserts zero allocations per frame).
- Interaction with `MatchScratch` reuse across frames.

> Status: outline (draft to be written).
> Source: `tests/zero_alloc.rs`, `src/ndt.rs` (`AlignWorkspace`), `src/engine.rs` (`MatchScratch`).
