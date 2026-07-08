# Zero-allocation guarantees

After warmup the align frame allocates **nothing** — a load-bearing property for the real-time story
and a hard test.

## Reused buffers

All per-frame buffers are pre-reserved and reused, held in the [`MatchScratch`](../arch/scratch.md)
(its `AlignWorkspace` + last `AlignResult`):

- the result `Vec`s (`transformation_array`, the TP/NVTL trace arrays) — cleared, not reallocated;
- `trans_cloud` (the transformed source) — reused via a `mem::take`/put-back, no realloc;
- `neighbor_idx` — bounded by `MAX_NEIGHBORS`;
- the 6×6 SVD is fixed-size and **stack-only**.

## Warmup model

The buffers grow to their steady-state size on the first align(s); from then on each frame reuses
them, so no heap traffic occurs on the hot path. Reusing one `MatchScratch` per task/thread across
frames is what preserves this — a fresh scratch re-warms.

## The test

`tests/zero_alloc.rs` asserts zero allocations per frame after warmup (an allocator hook counts
allocations across a steady-state align). It runs on the serial backend — the parallel backend
allocates its per-worker contribution buffers, so it is explicitly **not** the WCET baseline (see
[The WCET contract](wcet.md)).

> Source: `tests/zero_alloc.rs`; `src/ndt.rs` (`AlignWorkspace`), `src/engine.rs` (`MatchScratch`).
