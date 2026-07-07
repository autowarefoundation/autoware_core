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
//!
//! The match methods take a caller-owned `&mut MatchScratch` (one per task/thread, reused across
//! frames): the matcher itself holds no per-align mutable state, so a shared `&ScanMatcher` is
//! sound across concurrent tasks in the std and `mt` configs (map updates included — `update_map`
//! publishes atomically), with no scratch lock held across an align anywhere.

use nalgebra::Matrix4;

pub use crate::engine::MatchScratch;
use crate::engine::NdtEngine;
use crate::host::{CovarianceResult, MapSource, MatchResult};

/// A scan matcher: the persistent NDT engine + the portable orchestration over it.
///
/// # Examples
///
/// Load a map through a [`MapSource`], then match a scan. The map load is `async`; the match is the
/// synchronous WCET hot path. (`no_run`: driving the future needs an async runtime — ROS, a kernel,
/// or the Tokio example.)
///
/// ```no_run
/// use autoware_ndt_scan_matcher_rs::host::{MapDelta, MapSource, MapTile};
/// use autoware_ndt_scan_matcher_rs::scan_matcher::{MatchScratch, ScanMatcher};
/// use autoware_ndt_scan_matcher_rs::nalgebra::Matrix4;
///
/// // A trivial map source (ROS backs this with the `pcd_loader` service).
/// struct Synthetic;
/// impl MapSource for Synthetic {
///     async fn load(&self, _center: [f64; 2], _radius: f64) -> MapDelta {
///         let points = (0u8..64).map(|i| [f32::from(i) * 0.05, 0.0, 0.0]).collect();
///         MapDelta { add: vec![MapTile { id: b"tile-0".to_vec(), points }], remove: vec![] }
///     }
/// }
///
/// async fn run() {
///     let matcher = ScanMatcher::new(2.0, 6, 0.01);
///     matcher.set_params(0.01, 0.1, 2.0, 30, 0.55, 1);
///     matcher.update_map(&Synthetic, [0.0, 0.0], 100.0).await;
///
///     let source: Vec<[f32; 3]> = (0u8..64).map(|i| [f32::from(i) * 0.05, 0.0, 0.0]).collect();
///     let mut scratch = MatchScratch::new();
///     let result = matcher.match_scan(&Matrix4::identity(), &source, &mut scratch);
///     println!("converged: {}", result.converged);
/// }
/// ```
pub struct ScanMatcher {
    engine: NdtEngine,
}

/// Compile-time proof the matcher is shareable across tasks/threads in the two concurrent configs
/// (std and `mt`); the plain `no_std` single-core build stays intentionally `!Sync`.
#[cfg(any(feature = "std", feature = "mt"))]
const fn assert_send_sync<T: Send + Sync>() {}
#[cfg(any(feature = "std", feature = "mt"))]
const _: () = assert_send_sync::<ScanMatcher>();

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
    ///
    /// # Arguments
    /// * `trans_epsilon` — translation convergence tolerance.
    /// * `step_size` — More-Thuente line-search step size.
    /// * `resolution` — voxel/leaf size in metres.
    /// * `max_iterations` — optimizer iteration cap.
    /// * `outlier_ratio` — Gaussian mixture outlier ratio (default 0.55).
    /// * `num_threads` — derivative-reduction worker count (`> 1` uses rayon under the `parallel`
    ///   feature; bit-identical to serial).
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

    /// Set the `score_estimation` scalars that gate the convergence verdict (the C++
    /// `converged_param_type` + the two `converged_param_*` thresholds). `match_scan` reads them.
    pub fn set_convergence_params(
        &self,
        converged_param_type: i32,
        tp_threshold: f64,
        nvtl_threshold: f64,
    ) {
        self.engine
            .set_convergence_params(converged_param_type, tp_threshold, nvtl_threshold);
    }

    /// Set the `covariance` hyper-params read by [`Self::match_scan_with_covariance`]: the estimation
    /// mode (`0` `FIXED_VALUE`, `1` `LAPLACE`, `2` `MULTI_NDT`, `3` `MULTI_NDT_SCORE`), the scale +
    /// softmax temperature, the configured 6x6, and the candidate XY search offsets.
    pub fn set_covariance_config(
        &self,
        estimation_type: i32,
        scale_factor: f64,
        temperature: f64,
        output_pose_covariance: [f64; 36],
        offset_x: &[f64],
        offset_y: &[f64],
    ) {
        self.engine.set_covariance_config(
            estimation_type,
            scale_factor,
            temperature,
            output_pose_covariance,
            offset_x,
            offset_y,
        );
    }

    /// Whether any map tile is loaded.
    #[must_use]
    pub fn has_target(&self) -> bool {
        self.engine.has_target()
    }

    /// Load the map delta around `center` (within `radius`) from the host and publish it atomically.
    /// Incremental (keeps the current map); see [`apply_map_update`] for the shared logic.
    pub async fn update_map<S: MapSource>(&self, source: &S, center: [f64; 2], radius: f64) {
        apply_map_update(&self.engine, source, center, radius, false).await;
    }

    /// Align `source` (base_link-frame points) from `guess` and return the result + convergence
    /// verdict. Synchronous — this is the WCET hot path (one bounded `Vec` of ≤ `max_iterations + 1`
    /// iteration positions for the oscillation count; no await). `scratch` is caller-owned — one
    /// per task/thread, reused across frames (never shared), so concurrent matches on a shared
    /// `&ScanMatcher` are sound in every build config.
    ///
    /// # Arguments
    /// * `guess` — initial pose estimate as a 4×4 homogeneous transform.
    /// * `source` — the sensor cloud to align, in the `base_link` frame (`[x, y, z]`, metres).
    /// * `scratch` — caller-owned per-align workspace; reuse it across frames, never share it.
    #[must_use]
    pub fn match_scan(
        &self,
        guess: &Matrix4<f32>,
        source: &[[f32; 3]],
        scratch: &mut MatchScratch,
    ) -> MatchResult {
        let o = self.engine.align_outcome_with(guess, source, scratch);
        MatchResult {
            pose: o.pose,
            transform_probability: o.transform_probability,
            nearest_voxel_likelihood: o.nearest_voxel_likelihood,
            iteration_num: o.iteration_num,
            converged: o.verdict.is_converged,
            oscillation_num: o.oscillation_num,
        }
    }

    /// Align like [`Self::match_scan`], then estimate the pose covariance from that align result using
    /// the configured covariance mode (see [`Self::set_covariance_config`]). Returns both. Mirrors the
    /// ROS sensor callback's align→covariance order. The align result (incl. the hessian) is read from
    /// the caller's `scratch` session before the covariance candidates reuse its workspace, so there is
    /// no cross-call ordering dependency — safe under concurrent matches in every build config.
    /// Allocates (candidate poses + the result-trace clone) — use the plain [`Self::match_scan`] on the
    /// allocation-free hot path when covariance is not needed.
    ///
    /// # Arguments
    /// * `guess` — initial pose estimate as a 4×4 homogeneous transform.
    /// * `source` — the sensor cloud to align, in the `base_link` frame (`[x, y, z]`, metres).
    /// * `scratch` — caller-owned workspace; the align result read from it seeds the covariance.
    #[must_use]
    pub fn match_scan_with_covariance(
        &self,
        guess: &Matrix4<f32>,
        source: &[[f32; 3]],
        scratch: &mut MatchScratch,
    ) -> (MatchResult, CovarianceResult) {
        let o = self.engine.align_outcome_with(guess, source, scratch);
        // The full align result (carries the hessian the covariance estimate needs) — read from
        // THIS scratch session, so it is the hessian of the align above.
        let r = scratch.result();
        let cov = self.engine.estimate_covariance(
            &r.pose,
            &r.hessian,
            guess,
            source,
            r.nearest_voxel_likelihood,
            scratch,
        );
        let match_result = MatchResult {
            pose: o.pose,
            transform_probability: o.transform_probability,
            nearest_voxel_likelihood: o.nearest_voxel_likelihood,
            iteration_num: o.iteration_num,
            converged: o.verdict.is_converged,
            oscillation_num: o.oscillation_num,
        };
        (
            match_result,
            CovarianceResult {
                covariance: cov.ndt_covariance,
            },
        )
    }
}

/// Load a map delta from `source` around `center` (within `radius`) and publish it into `engine`
/// atomically: build the new map on a **private staging engine**, then `commit_from` (the lock-free
/// double-buffer), so a concurrent align never observes a partial map. `rebuild` starts the staging
/// engine empty (dropping the current tiles — the C++ `need_rebuild`); otherwise it clones the current
/// map and applies the incremental delta. Shared by [`ScanMatcher::update_map`] and the ROS map-update
/// FFI. `async` only at the `source.load` boundary; the staging build is synchronous.
pub async fn apply_map_update<S: MapSource>(
    engine: &NdtEngine,
    source: &S,
    center: [f64; 2],
    radius: f64,
    rebuild: bool,
) {
    let delta = source.load(center, radius).await;
    // Empty delta → no-op (don't republish): mirrors the C++ "no maps to add/remove → no commit", and
    // avoids a wasteful kdtree rebuild on idle map-update cycles.
    if delta.add.is_empty() && delta.remove.is_empty() {
        return;
    }
    let staging = if rebuild {
        engine.clone_empty()
    } else {
        engine.clone()
    };
    for tile in &delta.add {
        staging.add_target_bytes(&tile.points, &tile.id);
    }
    for id in &delta.remove {
        staging.remove_target_bytes(id);
    }
    staging.create_kdtree();
    engine.commit_from(&staging);
}
