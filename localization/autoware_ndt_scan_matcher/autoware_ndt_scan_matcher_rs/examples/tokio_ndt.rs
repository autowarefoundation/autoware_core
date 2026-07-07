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

//! Reference implementation of the NDT scan matcher running **standalone in async Rust (Tokio), with
//! no ROS** — the portability + async-boundary proof for `plan/ndt_in_rust.md` ("full Rust port").
//!
//! It implements the [`Host`] ports (`MapSource`/`OutputSink`/`Clock`) with **synthetic, deterministic**
//! data (PCD/recorded-scan input comes later), runs `ScanMatcher::update_map(...).await` then a
//! synchronous `match_scan`, and recovers a known synthetic transform. The same `ScanMatcher` +
//! `Host` trait are what the ROS node (via an `FfiHost` adapter) and the no_std async kernel will use.
//!
//! Run: `cargo run --example tokio_ndt`

#![allow(
    clippy::as_conversions,
    clippy::cast_precision_loss,
    clippy::indexing_slicing,
    clippy::arithmetic_side_effects,
    clippy::float_cmp,
    clippy::doc_markdown,
    clippy::allow_attributes,
    reason = "example code: synthetic data construction + nalgebra fixed-index reads + prints"
)]

use autoware_ndt_scan_matcher_rs::host::{
    Clock, MapDelta, MapSource, MapTile, MatchResult, OutputSink,
};
use autoware_ndt_scan_matcher_rs::scan_matcher::{MatchScratch, ScanMatcher};
use nalgebra::Matrix4;

/// A deterministic dense cluster around `(cx, cy, cz)` (>min_points, non-degenerate covariance, all
/// within one ~2 m voxel), keyed by `id`.
fn dense_tile(cx: f32, cy: f32, cz: f32, id: &[u8]) -> MapTile {
    let mut points = Vec::new();
    for k in 0..60 {
        let f = k as f32;
        points.push([
            cx + (f * 0.7).sin() * 0.3,
            cy + (f * 1.1).cos() * 0.3,
            cz + (f * 0.3).sin() * 0.15,
        ]);
    }
    MapTile {
        id: id.to_vec(),
        points,
    }
}

fn synthetic_tiles() -> [MapTile; 2] {
    [
        dense_tile(0.0, 0.0, 0.0, b"0"),
        dense_tile(8.0, 4.0, 0.0, b"1"),
    ]
}

/// Synthetic, deterministic host: the map is two fixed clusters; output is printed; clock is a stub.
struct TokioHost;

impl MapSource for TokioHost {
    async fn load(&self, _center: [f64; 2], _radius: f64) -> MapDelta {
        // Genuinely cross an await point on the Tokio runtime, then return the synthetic delta.
        tokio::task::yield_now().await;
        MapDelta {
            add: synthetic_tiles().into_iter().collect(),
            remove: Vec::new(),
        }
    }
}

impl OutputSink for TokioHost {
    fn publish_result(&self, r: &MatchResult) {
        println!(
            "ndt result: translation=({:.4}, {:.4}, {:.4})  tp={:.4}  nvl={:.4}  iters={}  \
             converged={}  oscillation={}",
            r.pose[(0, 3)],
            r.pose[(1, 3)],
            r.pose[(2, 3)],
            r.transform_probability,
            r.nearest_voxel_likelihood,
            r.iteration_num,
            r.converged,
            r.oscillation_num,
        );
    }
}

impl Clock for TokioHost {
    fn now_ns(&self) -> i64 {
        0
    }
}

#[tokio::main(flavor = "current_thread")]
async fn main() {
    let host = TokioHost;

    let matcher = ScanMatcher::new(2.0, 6, 0.01);
    matcher.set_params(0.01, 0.1, 2.0, 30, 0.55, 1);
    // Gate convergence on the transform-probability score (type 0). A 0.0 threshold is the defensive
    // lower bound for this synthetic fit — a good recovery yields a clearly positive score.
    matcher.set_convergence_params(0, 0.0, 0.0);
    // Estimate covariance by MULTI_NDT (type 2): re-align a small XY candidate grid against the live map
    // and take the spread. `output_pose_covariance` is the configured 6x6 diagonal (its [0]/[7] act as
    // the x/y floors in adjust_diagonal_covariance).
    let mut output_cov = [0.0_f64; 36];
    output_cov[0] = 0.0225; // x floor (0.15^2)
    output_cov[7] = 0.0225; // y floor
    output_cov[14] = 0.01;
    output_cov[21] = 0.01;
    output_cov[28] = 0.01;
    output_cov[35] = 0.01;
    let offset_x = [0.5, -0.5, 0.0, 0.0];
    let offset_y = [0.0, 0.0, 0.5, -0.5];
    matcher.set_covariance_config(2, 1.0, 1.0, output_cov, &offset_x, &offset_y);

    // Load the map from the host (async), then confirm it landed.
    matcher.update_map(&host, [0.0, 0.0], 50.0).await;
    assert!(
        matcher.has_target(),
        "map should be loaded after update_map"
    );

    // Synthetic scan: the same clusters translated by a known offset; NDT should pull them back.
    let offset = [0.2_f32, -0.15, 0.1];
    let mut scan = Vec::new();
    for tile in synthetic_tiles() {
        for p in tile.points {
            scan.push([p[0] + offset[0], p[1] + offset[1], p[2] + offset[2]]);
        }
    }

    // Caller-owned scratch: one per task, reused across frames (the `mt` usage model).
    let mut scratch = MatchScratch::new();
    let (result, cov) =
        matcher.match_scan_with_covariance(&Matrix4::<f32>::identity(), &scan, &mut scratch);
    host.publish_result(&result);
    println!(
        "covariance xy block: [{:.5}, {:.5}; {:.5}, {:.5}]",
        cov.covariance[0], cov.covariance[1], cov.covariance[6], cov.covariance[7],
    );

    // Smoke checks: a finite, bounded pose (correctness vs the C++ engine is covered by the
    // differential gtests; here we only prove the pipeline runs end-to-end without ROS).
    assert!(
        result.pose[(0, 3)].is_finite()
            && result.pose[(1, 3)].is_finite()
            && result.pose[(2, 3)].is_finite(),
        "align produced a non-finite pose"
    );
    assert!(
        result.pose[(0, 3)].abs() < 1.0 && result.pose[(1, 3)].abs() < 1.0,
        "recovered translation is implausibly large"
    );
    assert!(
        result.converged,
        "the synthetic recovery should report converged (iters < cap, score > 0)"
    );
    assert!(
        cov.covariance[0].is_finite() && cov.covariance[7].is_finite(),
        "covariance must be finite"
    );
    assert!(
        cov.covariance[0] > 0.0 && cov.covariance[7] > 0.0,
        "the xy covariance diagonal must be positive"
    );

    println!("OK: NDT scan matcher ran standalone in async Rust (Tokio), no ROS.");
}
