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
    clippy::float_cmp
)]

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use autoware_ndt_scan_matcher_rs::ndt::{AlignWorkspace, compute_derivatives};
use autoware_ndt_scan_matcher_rs::transform::{gauss_constants, transform_point};
use autoware_ndt_scan_matcher_rs::voxel_grid::VoxelGridMap;
use nalgebra::{Vector3, Vector6};

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

#[test]
fn compute_derivatives_does_not_allocate_after_warmup() {
    let mut map = VoxelGridMap::new([1.0, 1.0, 1.0], 6, 0.01);
    map.add_target(&dense_cluster(0.5, 0.5, 0.5), 0);
    map.add_target(&dense_cluster(2.5, 0.5, 0.5), 1);
    map.create_kdtree();

    let source: Vec<[f32; 3]> = vec![[0.55, 0.5, 0.5], [0.5, 0.45, 0.52], [2.55, 0.5, 0.5]];
    let p = Vector6::new(0.05, -0.03, 0.02, 0.04, -0.02, 0.03);
    let trans = transform_cloud(&source, &p);
    let gauss = gauss_constants(0.55, 1.0);

    let mut ws = AlignWorkspace::new();

    // Warm up: first call may grow ws.neighbor_idx.
    let warm = compute_derivatives(&map, &source, &trans, &p, 1.0, &gauss, None, &mut ws);
    assert!(
        warm.score > 0.0,
        "fixture should produce a non-trivial score"
    );

    // Measured call: must allocate nothing.
    ENABLED.store(true, Ordering::SeqCst);
    let measured = compute_derivatives(&map, &source, &trans, &p, 1.0, &gauss, None, &mut ws);
    ENABLED.store(false, Ordering::SeqCst);

    // Use the result so the call can't be optimized away.
    assert_eq!(warm.score, measured.score);
    let allocs = ALLOCS.load(Ordering::SeqCst);
    assert_eq!(
        allocs, 0,
        "compute_derivatives allocated {allocs} time(s) after warmup"
    );
}
