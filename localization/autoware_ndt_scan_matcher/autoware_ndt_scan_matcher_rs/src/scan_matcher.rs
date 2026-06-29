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

//! Portable, `no_std` NDT scan-matcher node orchestration over the [`crate::host`] ports.
//!
//! Wraps the existing [`crate::engine::NdtEngine`] (the `no_std` engine) and drives it against a host
//! interface, so the same logic runs under ROS, a bare-metal kernel, or the Tokio example. This is the
//! minimal foundation (map update + single-scan match); it grows as the ROS callbacks are ported.
//! Map loading is `async` (via the [`MapSource`] port); the align hot path is synchronous.

use nalgebra::Matrix4;

use crate::engine::NdtEngine;
use crate::host::{MapSource, MatchResult};

/// A scan matcher: the persistent NDT engine + the portable orchestration over it.
pub struct ScanMatcher {
    engine: NdtEngine,
}

impl ScanMatcher {
    /// New matcher with an empty map (`resolution` voxel/neighbor size; `min_points`/`eig_mult` the
    /// `MultiVoxelGridCovariance` defaults 6 / 0.01).
    #[must_use]
    pub fn new(resolution: f64, min_points: i32, eig_mult: f64) -> Self {
        Self {
            engine: NdtEngine::new(resolution, min_points, eig_mult),
        }
    }

    /// Set the alignment params (the C++ `setParams`).
    pub fn set_params(
        &self,
        trans_epsilon: f64,
        step_size: f64,
        resolution: f64,
        max_iterations: i32,
        outlier_ratio: f64,
        num_threads: usize,
    ) {
        self.engine.set_params(
            trans_epsilon,
            step_size,
            resolution,
            max_iterations,
            outlier_ratio,
            num_threads,
        );
    }

    /// Whether any map tile is loaded.
    #[must_use]
    pub fn has_target(&self) -> bool {
        self.engine.has_target()
    }

    /// Load the map delta around `center` (within `radius`) from the host and publish it atomically:
    /// build it on a private staging engine, then `commit_from` (the lock-free double-buffer). Mirrors
    /// the ROS map-update path; portable because the tiles come from the [`MapSource`] port.
    pub async fn update_map<S: MapSource>(&self, source: &S, center: [f64; 2], radius: f64) {
        let delta = source.load(center, radius).await;
        let staging = self.engine.clone();
        for tile in &delta.add {
            staging.add_target_bytes(&tile.points, &tile.id);
        }
        for id in &delta.remove {
            staging.remove_target_bytes(id);
        }
        staging.create_kdtree();
        self.engine.commit_from(&staging);
    }

    /// Align `source` (base_link-frame points) from `guess` and return the result. Synchronous — this
    /// is the WCET hot path; no allocation/await.
    #[must_use]
    pub fn match_scan(&self, guess: &Matrix4<f32>, source: &[[f32; 3]]) -> MatchResult {
        self.engine.align(guess, source);
        let r = self.engine.result();
        MatchResult {
            pose: r.pose,
            transform_probability: r.transform_probability,
            nearest_voxel_likelihood: r.nearest_voxel_likelihood,
            iteration_num: r.iteration_num,
        }
    }
}
