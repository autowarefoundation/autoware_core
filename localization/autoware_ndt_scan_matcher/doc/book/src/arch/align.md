# The align hot path

`ndt::align` (`src/ndt.rs`) is the RT-critical alignment kernel — the Rust equivalent of the
C++ `align` / `computeTransformation` on the default path (`use_line_search = false`,
`computeStepLengthMT`). It is the one function whose worst-case execution time is contracted; see
[The WCET contract](../rt/wcet.md).

## Signature

```rust,ignore
pub fn align(
    map: &VoxelGridMap,      // target map with kd-tree built
    source: &[[f32; 3]],     // sensor cloud, base_link frame
    guess: &Matrix4<f32>,    // initial pose (C++ Matrix4f guess)
    params: &NdtParams,
    ws: &mut AlignWorkspace, // reused buffers
    out: &mut AlignResult,   // result slot; Vec fields reused
)
```

## The iteration

Each of at most `params.max_iterations` iterations does:

1. **Transform** the source cloud by the current pose estimate `p` (an f32 `Matrix4f` transform,
   `O(P)` for `P` source points — mirroring the C++ pipeline exactly).
2. **Compute derivatives** — score, 6-vector gradient, and 6×6 Hessian — by, for each transformed
   point, a voxel-grid `radius_search` (`≤ MAX_NEIGHBORS` neighbours) and accumulating each
   neighbour's Gaussian contribution. This is `O(P · K)`, the dominant cost.
3. **Solve** `H · Δ = −gradient` via a fixed-size 6×6 SVD (mirroring the C++
   `JacobiSVD(ComputeFullU|ComputeFullV)`), and step the pose.

The loop stops on the translation-epsilon convergence test or the iteration cap. The result
carries the final pose, TP/NVTL scores, iteration count, Hessian, and the per-iteration score
traces ([TP and NVTL](../concepts/scores.md)).

## Serial and parallel are bit-identical

`params.num_threads > 1` selects the rayon backend (built under the `parallel` feature) for the
derivative reduction. Contributions are per-point-local and reduced in **point-index order**
(`collect_into_vec`), so the parallel result is **bit-for-bit identical** to the serial one. The
parallel backend is therefore a pure throughput option; the serial backend stays the predictable
WCET baseline. See [Serial and parallel derivatives](derivatives.md).

## Allocation and panics

After warmup the align frame does **zero allocation**: the result `Vec`s, `trans_cloud`, and the
`neighbor_idx` buffer (bounded by `MAX_NEIGHBORS`) are pre-reserved and reused, and the 6×6 SVD is
stack-only. It also cannot panic — the crate denies `unwrap`/`expect`/`panic`/`indexing_slicing`
in non-test code. Both properties are load-bearing for the real-time story and are tested
(`tests/zero_alloc.rs`) and audited by a real-time review of each engine/align patch.

## The map is read-only here

`align` borrows the map immutably. Map building/update is the control plane
([map update](map-update.md)), which happens on a staging engine and is committed atomically, so
the align path never contends with it.

## Sub-chapters

- [Voxel grid and kd-tree](voxel-grid.md)
- [Serial and parallel derivatives](derivatives.md)

> Source: the `align` doc + WCET contract in `src/ndt.rs` (≈ lines 317, 617); `src/derivatives.rs`;
> `src/voxel_grid.rs`; `tests/zero_alloc.rs`; `examples/wcet_frame.rs`.
