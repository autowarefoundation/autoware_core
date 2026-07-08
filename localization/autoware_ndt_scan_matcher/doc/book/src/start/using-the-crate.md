# Using the Rust crate

`autoware_ndt_scan_matcher_rs` is usable as a plain Rust library, independently of ROS. This
chapter tours the public API. The examples are the crate's own rustdoc doctests (verified with
`cargo test --doc`); browse the full reference with `cargo doc --no-deps --open`.

## The `nalgebra` re-export

Pose guesses and results are `nalgebra` matrices (`Matrix4<f32>` for poses, `Matrix6<f64>` for
Hessians/covariances). The crate **re-exports the exact `nalgebra` version it is built against**,
so construct those matrices through `autoware_ndt_scan_matcher_rs::nalgebra` rather than pinning
your own (a different version is a distinct, incompatible type).

## Primary API: `NdtEngine`

The persistent engine ([`engine::NdtEngine`](../arch/engine.md)) is the main entry point. Load a
target map, build the kd-tree, then align sensor clouds:

```rust
use autoware_ndt_scan_matcher_rs::engine::NdtEngine;
use autoware_ndt_scan_matcher_rs::nalgebra::Matrix4;

// Empty engine: 2.0 m voxels; MultiVoxelGridCovariance defaults (min 6 points / eig 0.01).
let engine = NdtEngine::new(2.0, 6, 0.01);

// Register a target map tile (id 0) and build the kd-tree over the voxel centroids.
let target: Vec<[f32; 3]> = (0u8..64).map(|i| [f32::from(i) * 0.05, 0.0, 0.0]).collect();
engine.add_target(&target, 0);
engine.create_kdtree();
assert!(engine.has_target());

// Align a source cloud from an identity initial guess, then read the result back.
let source = target.clone();
engine.align(&Matrix4::identity(), &source);
let result = engine.result();
assert!(result.iteration_num >= 0);
```

### Scratch and the `_with` methods

The align scratch (workspace + last result) is a [`engine::MatchScratch`](../arch/scratch.md).
The `_with` methods take one you own — reuse it across frames to stay allocation-free after
warmup. These are the **only** align entry points under the `mt` feature (the implicit-scratch
variants are compiled out):

```rust
use autoware_ndt_scan_matcher_rs::engine::{MatchScratch, NdtEngine};
use autoware_ndt_scan_matcher_rs::nalgebra::Matrix4;

let engine = NdtEngine::new(2.0, 6, 0.01);
let target: Vec<[f32; 3]> = (0u8..64).map(|i| [f32::from(i) * 0.05, 0.0, 0.0]).collect();
engine.add_target(&target, 0);
engine.create_kdtree();

let mut scratch = MatchScratch::new();
engine.align_with(&Matrix4::identity(), &target, &mut scratch);
assert!(scratch.result().iteration_num >= 0);
```

## Portable orchestration: `ScanMatcher`

[`scan_matcher::ScanMatcher`](../arch/portability.md) wraps the engine and drives it over the
[host ports](../arch/host-vtable.md) (`MapSource` / `OutputSink` / `Clock`). Map loading is
`async`; the match is the synchronous WCET hot path. It runs unchanged under ROS, a kernel, or an
async runtime — `examples/tokio_ndt.rs` is the reference host implementation.

## The lower-level kernel: `ndt::align`

[`ndt::align`](../arch/align.md) is the RT-critical alignment function the engine drives. Use it
directly when you hold a `VoxelGridMap` yourself:

```rust
use autoware_ndt_scan_matcher_rs::ndt::{align, AlignResult, AlignWorkspace, NdtParams};
use autoware_ndt_scan_matcher_rs::voxel_grid::VoxelGridMap;
use autoware_ndt_scan_matcher_rs::nalgebra::Matrix4;

let mut map = VoxelGridMap::new([2.0; 3], 6, 0.01);
let target: Vec<[f32; 3]> = (0u8..64).map(|i| [f32::from(i) * 0.05, 0.0, 0.0]).collect();
map.add_target(&target, 0);
map.create_kdtree();

let params = NdtParams { resolution: 2.0, ..NdtParams::default() };
let mut ws = AlignWorkspace::new();
let mut out = AlignResult::default();
align(&map, &target, &Matrix4::identity(), &params, &mut ws, &mut out);
assert!(out.iteration_num >= 0);
```

## Other public modules

- [`tpe::TreeStructuredParzenEstimator`](../arch/tpe.md) — the align-service pose-search sampler.
- [`convergence`](../concepts/scores.md), `covariance`, `cov_estimate` — pure decision/estimation
  kernels reused across the ROS and kernel builds.
- [`pose_buffer`](../port/strategy.md) — a time-ordered pose interpolation buffer.
