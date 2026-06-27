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

//! Frame-time (WCET) benchmark for `align` — a manual diagnostic, NOT a CI gate (timing is
//! environment-dependent). Reports min / mean / p50 / p99 / p99.9 / max over many frames on a fixed
//! synthetic fixture; for WCET, the **max + tail** matter (not the mean). Run with:
//!   `cargo run --release --example wcet_frame`
//! Record the baseline in `porting_notes/ndt_wcet_audit.md`. (Synthetic, single core, warm cache —
//! a relative regression watch, not a hardware WCET proof.)

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::as_conversions,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::indexing_slicing,
    clippy::print_stdout,
    clippy::arithmetic_side_effects
)]

use std::time::Instant;

use autoware_ndt_scan_matcher_rs::ndt::{AlignResult, AlignWorkspace, NdtParams, align};
use autoware_ndt_scan_matcher_rs::voxel_grid::VoxelGridMap;
use nalgebra::Matrix4;

const FRAMES: usize = 20_000;

fn dense_cluster(cx: f32, cy: f32, cz: f32) -> Vec<[f32; 3]> {
    (0..8)
        .map(|i| {
            let f = i as f32 * 0.02;
            [cx + f, cy - f, cz + 0.5 * f]
        })
        .collect()
}

fn main() {
    // A 6x6 grid of clusters (≈ 288 map points) and a source = a copy translated by a small offset.
    let mut map_points: Vec<[f32; 3]> = Vec::new();
    for i in 0..6 {
        for j in 0..6 {
            map_points.extend(dense_cluster((i as f32) * 3.0, (j as f32) * 3.0, 0.0));
        }
    }
    let source: Vec<[f32; 3]> = map_points
        .iter()
        .map(|p| [p[0] + 0.3, p[1] - 0.2, p[2] + 0.1])
        .collect();

    let mut map = VoxelGridMap::new([2.0, 2.0, 2.0], 6, 0.01);
    map.add_target(&map_points, 0);
    map.create_kdtree();

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

    // Warmup.
    for _ in 0..100 {
        align(&map, &source, &guess, &params, &mut ws, &mut out);
    }

    let mut nanos: Vec<u128> = Vec::with_capacity(FRAMES);
    for _ in 0..FRAMES {
        let t = Instant::now();
        align(&map, &source, &guess, &params, &mut ws, &mut out);
        nanos.push(t.elapsed().as_nanos());
    }
    nanos.sort_unstable();

    let pct = |p: f64| nanos[((p * (FRAMES as f64 - 1.0)) as usize).min(FRAMES - 1)];
    let mean = nanos.iter().sum::<u128>() / (FRAMES as u128);
    let us = |n: u128| (n as f64) / 1000.0;

    println!(
        "align frame time over {FRAMES} frames ({} src pts, {} iters):",
        source.len(),
        out.iteration_num
    );
    println!("  min  {:.2} us", us(nanos[0]));
    println!("  mean {:.2} us", us(mean));
    println!("  p50  {:.2} us", us(pct(0.50)));
    println!("  p99  {:.2} us", us(pct(0.99)));
    println!("  p99.9 {:.2} us", us(pct(0.999)));
    println!("  max  {:.2} us", us(nanos[FRAMES - 1]));
}
