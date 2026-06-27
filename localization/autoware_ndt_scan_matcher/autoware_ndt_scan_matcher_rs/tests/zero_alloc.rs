// Copyright 2024 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Zero-allocation gate for the per-frame derivative pass (`plan/ndt_in_rust.md` → "Runtime
//! allocation policy"). A counting global allocator asserts that `compute_derivatives` performs
//! **no heap allocation after warmup**: the first call may grow the workspace's neighbor buffer,
//! but a second call with the same inputs must reuse it (`clear()` keeps capacity).
//!
//! This lives in its own integration-test binary so the global allocator does not perturb the unit
//! tests.

#![allow(
    unsafe_code,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::as_conversions,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::indexing_slicing,
    clippy::float_cmp,
    clippy::allow_attributes,
    reason = "test code"
)]

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use autoware_ndt_scan_matcher_rs::ndt::{
    AlignResult, AlignWorkspace, NdtParams, align, compute_derivatives,
};
use autoware_ndt_scan_matcher_rs::transform::{gauss_constants, transform_point};
use autoware_ndt_scan_matcher_rs::voxel_grid::VoxelGridMap;
use nalgebra::{Matrix4, Vector3, Vector6};

/// A pass-through allocator that counts allocations while `ENABLED` is set. The default
/// `GlobalAlloc::realloc` routes through `alloc`, so `Vec` growth is counted too.
struct Counting;

static ENABLED: AtomicBool = AtomicBool::new(false);
static ALLOCS: AtomicU64 = AtomicU64::new(0);

// SAFETY: delegates every call to the System allocator unchanged; only adds an atomic counter.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if ENABLED.load(Ordering::SeqCst) {
            ALLOCS.fetch_add(1, Ordering::SeqCst);
        }
        // SAFETY: same layout contract as the System allocator.
        unsafe { System.alloc(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        // SAFETY: ptr/layout came from System.alloc above.
        unsafe { System.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static ALLOCATOR: Counting = Counting;

/// Run `f` with allocation counting enabled and return the number of allocations it performed.
/// The `ENABLED` flag is process-global, so the two measurements live in ONE `#[test]` (sequential)
/// — separate parallel tests would have one test's setup counted in the other's window.
fn count_allocs<R>(f: impl FnOnce() -> R) -> u64 {
    let before = ALLOCS.load(Ordering::SeqCst);
    ENABLED.store(true, Ordering::SeqCst);
    let _r = f();
    ENABLED.store(false, Ordering::SeqCst);
    ALLOCS.load(Ordering::SeqCst).saturating_sub(before)
}

fn dense_cluster(cx: f32, cy: f32, cz: f32) -> Vec<[f32; 3]> {
    (0..8)
        .map(|i| {
            let f = i as f32 * 0.02;
            [cx + f, cy - f, cz + 0.5 * f]
        })
        .collect()
}

fn transform_cloud(source: &[[f32; 3]], p: &Vector6<f64>) -> Vec<[f32; 3]> {
    source
        .iter()
        .map(|s| {
            let v = Vector3::new(f64::from(s[0]), f64::from(s[1]), f64::from(s[2]));
            let t = transform_point(p, &v);
            [t[0] as f32, t[1] as f32, t[2] as f32]
        })
        .collect()
}

// Both measurements run sequentially in ONE test (the counting flag is process-global; see
// count_allocs). compute_derivatives must be allocation-free after warmup; align's per-frame
// allocation must be a small O(1) constant (SVD-internal), recorded in
// porting_notes/ndt_wcet_audit.md.
#[test]
fn engine_allocations_after_warmup() {
    // --- compute_derivatives: 0 allocations after warmup ---
    {
        let mut map = VoxelGridMap::new([1.0, 1.0, 1.0], 6, 0.01);
        map.add_target(&dense_cluster(0.5, 0.5, 0.5), 0);
        map.add_target(&dense_cluster(2.5, 0.5, 0.5), 1);
        map.create_kdtree();

        let source: Vec<[f32; 3]> = vec![[0.55, 0.5, 0.5], [0.5, 0.45, 0.52], [2.55, 0.5, 0.5]];
        let p = Vector6::new(0.05, -0.03, 0.02, 0.04, -0.02, 0.03);
        let trans = transform_cloud(&source, &p);
        let gauss = gauss_constants(0.55, 1.0);
        let mut ws = AlignWorkspace::new();

        let warm = compute_derivatives(&map, &source, &trans, &p, 1.0, &gauss, None, &mut ws);
        assert!(
            warm.score > 0.0,
            "fixture should produce a non-trivial score"
        );

        let allocs = count_allocs(|| {
            compute_derivatives(&map, &source, &trans, &p, 1.0, &gauss, None, &mut ws)
        });
        assert_eq!(
            allocs, 0,
            "compute_derivatives allocated {allocs} time(s) after warmup"
        );
    }

    // --- align: small O(1) per-frame allocation after warmup ---
    let mut map = VoxelGridMap::new([2.0, 2.0, 2.0], 6, 0.01);
    for (id, &(cx, cy, cz)) in [
        (0.0_f32, 0.0_f32, 0.0_f32),
        (8.0, 0.0, 0.0),
        (0.0, 8.0, 0.0),
        (0.0, 0.0, 8.0),
        (8.0, 8.0, 8.0),
    ]
    .iter()
    .enumerate()
    {
        map.add_target(&dense_cluster(cx, cy, cz), id as u64);
    }
    map.create_kdtree();

    // ~40-point source (8 per cluster) translated by a known offset — large enough that an O(P)
    // per-frame allocation would show up clearly.
    let centers = [
        (0.0_f32, 0.0_f32, 0.0_f32),
        (8.0, 0.0, 0.0),
        (0.0, 8.0, 0.0),
        (0.0, 0.0, 8.0),
        (8.0, 8.0, 8.0),
    ];
    let mut source: Vec<[f32; 3]> = Vec::new();
    for &(cx, cy, cz) in &centers {
        for q in dense_cluster(cx, cy, cz) {
            source.push([q[0] + 0.2, q[1] - 0.1, q[2]]);
        }
    }
    let params = NdtParams {
        trans_epsilon: 0.01,
        step_size: 0.1,
        resolution: 2.0,
        max_iterations: 30,
        outlier_ratio: 0.55,
        regularization: None,
    };
    let guess = Matrix4::identity();

    let mut ws = AlignWorkspace::new();
    let mut out = AlignResult::default();

    // Warm up (grows neighbor_idx + pre-reserves the result/cloud buffers).
    align(&map, &source, &guess, &params, &mut ws, &mut out);
    // Second align with the same shapes: buffers reused, no growth.
    let allocs = count_allocs(|| align(&map, &source, &guess, &params, &mut ws, &mut out));

    eprintln!(
        "align allocations after warmup: {allocs} (points: {}, iterations: {})",
        source.len(),
        out.iteration_num
    );
    // The hot path is zero-alloc after warmup: all buffers (result Vecs, trans_cloud via
    // transform_cloud_f32, neighbor_idx bounded by MAX_NEIGHBORS) are pre-reserved + reused, and the
    // fixed-size 6x6 SVD is stack-only. (The earlier 1 alloc/frame was a trans_cloud over-reserve,
    // now fixed — see porting_notes/ndt_wcet_audit.md.)
    assert_eq!(
        allocs,
        0,
        "align allocated {allocs} times after warmup with {} points / {} iterations — the hot path \
         must be zero-alloc",
        source.len(),
        out.iteration_num
    );
}
