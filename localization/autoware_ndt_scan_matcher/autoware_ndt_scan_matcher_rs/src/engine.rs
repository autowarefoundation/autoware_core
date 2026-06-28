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

//! Persistent NDT engine handle (E6a). Wraps the target map + params + scratch workspace + last
//! align result so the C++ node adapter can drive incremental map updates, alignment, and scoring
//! over a stable C ABI (an opaque `AwNdtEngine*`), instead of the one-shot `ndt_align` FFI. The
//! handle is clone-able (the node's map-update double-buffers by copying the whole NDT object).
//! Control-plane: the WCET-bounded path is the inner [`crate::ndt::align`], not this wrapper.

use alloc::boxed::Box;

use nalgebra::Matrix4;

use crate::ndt::{
    AlignResult, AlignWorkspace, AwNdtAlignOutput, NdtParams, Regularization, align,
    nearest_voxel_score_each_point, nearest_voxel_transformation_likelihood,
    transformation_probability,
};
use crate::transform::gauss_constants;
use crate::voxel_grid::VoxelGridMap;

/// Persistent NDT engine: the target map, the active params (incl. optional regularization), a
/// reused align workspace, and the most recent align result. Mirrors the stateful C++
/// `MultiGridNormalDistributionsTransform` object the node holds across frames.
pub struct NdtEngine {
    map: VoxelGridMap,
    params: NdtParams,
    min_points: i32,
    eig_mult: f64,
    workspace: AlignWorkspace,
    last: AlignResult,
}

impl Clone for NdtEngine {
    fn clone(&self) -> Self {
        // The scratch workspace is not state — a clone starts fresh (it re-warms on first align).
        Self {
            map: self.map.clone(),
            params: self.params,
            min_points: self.min_points,
            eig_mult: self.eig_mult,
            workspace: AlignWorkspace::new(),
            last: self.last.clone(),
        }
    }
}

impl NdtEngine {
    /// New engine with an empty map. `resolution` is the voxel/leaf size and the neighbor radius;
    /// `min_points`/`eig_mult` match the C++ `MultiVoxelGridCovariance` defaults (6, 0.01).
    #[must_use]
    pub fn new(resolution: f64, min_points: i32, eig_mult: f64) -> Self {
        Self {
            map: VoxelGridMap::new([resolution; 3], min_points, eig_mult),
            params: NdtParams {
                resolution,
                ..NdtParams::default()
            },
            min_points,
            eig_mult,
            workspace: AlignWorkspace::new(),
            last: AlignResult::default(),
        }
    }

    /// Update the alignment params (the C++ `setParams`). The regularization is preserved — it is
    /// set separately via [`Self::set_regularization`], mirroring `setRegularizationPose`. Mirroring
    /// the C++ (which applies `resolution` as the leaf size at `addTarget`), the empty map is rebuilt
    /// at the new resolution — the node always calls `set_params` before `add_target`.
    pub fn set_params(
        &mut self,
        trans_epsilon: f64,
        step_size: f64,
        resolution: f64,
        max_iterations: i32,
        outlier_ratio: f64,
        num_threads: usize,
    ) {
        self.params.trans_epsilon = trans_epsilon;
        self.params.step_size = step_size;
        self.params.resolution = resolution;
        self.params.max_iterations = max_iterations;
        self.params.outlier_ratio = outlier_ratio;
        self.params.num_threads = num_threads;
        if self.map.is_empty() {
            self.map = VoxelGridMap::new([resolution; 3], self.min_points, self.eig_mult);
        }
    }

    /// Set (or, when `scale == 0`, clear) the longitudinal regularization toward `(x, y)`.
    #[allow(
        clippy::float_cmp,
        clippy::allow_attributes,
        reason = "exact == 0 is the C++ disable sentinel (setRegularizationPose vs unset)"
    )]
    pub fn set_regularization(&mut self, x: f32, y: f32, scale: f32) {
        self.params.regularization = if scale == 0.0 {
            None
        } else {
            Some(Regularization {
                pose_xy: [x, y],
                scale_factor: scale,
            })
        };
    }

    /// Add a target map tile keyed by `id` (the C++ `addTarget(cloud, cell_id)`; the adapter maps the
    /// node's string `cell_id` to a `u64`). Needs a following [`Self::create_kdtree`].
    pub fn add_target(&mut self, points: &[[f32; 3]], id: u64) {
        self.map.add_target(points, id);
    }

    /// Remove the map tile registered under `id` (the C++ `removeTarget`).
    pub fn remove_target(&mut self, id: u64) {
        self.map.remove_target(id);
    }

    /// Rebuild the kd-tree over the current tiles' centroids (the C++ `createVoxelKdtree`).
    pub fn create_kdtree(&mut self) {
        self.map.create_kdtree();
    }

    /// Whether any target tile is loaded (the C++ `hasTarget`).
    #[must_use]
    pub fn has_target(&self) -> bool {
        !self.map.is_empty()
    }

    /// Align `source` from `guess`, storing the result (the C++ `align(out, guess, source)`).
    pub fn align(&mut self, guess: &Matrix4<f32>, source: &[[f32; 3]]) {
        align(
            &self.map,
            source,
            guess,
            &self.params,
            &mut self.workspace,
            &mut self.last,
        );
    }

    /// The most recent align result (the C++ `getResult`).
    #[must_use]
    pub fn result(&self) -> &AlignResult {
        &self.last
    }

    /// Score `cloud` (already in the target frame) without aligning — the C++
    /// `calculateTransformationProbability`.
    pub fn calc_transformation_probability(&mut self, cloud: &[[f32; 3]]) -> f64 {
        let gauss = gauss_constants(self.params.outlier_ratio, self.params.resolution);
        transformation_probability(
            &self.map,
            cloud,
            self.params.resolution,
            &gauss,
            &mut self.workspace,
        )
    }

    /// Nearest-voxel likelihood of `cloud` without aligning — the C++
    /// `calculateNearestVoxelTransformationLikelihood`.
    pub fn calc_nearest_voxel_likelihood(&mut self, cloud: &[[f32; 3]]) -> f64 {
        let gauss = gauss_constants(self.params.outlier_ratio, self.params.resolution);
        nearest_voxel_transformation_likelihood(
            &self.map,
            cloud,
            self.params.resolution,
            &gauss,
            &mut self.workspace,
        )
    }

    /// Per-point nearest-voxel score (the C++ `calculateNearestVoxelScoreEachPoint`); `out[i] > 0`
    /// iff point `i` found a neighbor. `out` is filled to `cloud.len()`.
    pub fn nearest_voxel_score_each_point(
        &mut self,
        cloud: &[[f32; 3]],
        out: &mut alloc::vec::Vec<f32>,
    ) {
        let gauss = gauss_constants(self.params.outlier_ratio, self.params.resolution);
        nearest_voxel_score_each_point(
            &self.map,
            cloud,
            self.params.resolution,
            &gauss,
            &mut self.workspace,
            out,
        );
    }

    /// The configured iteration cap (the C++ `getMaximumIterations`).
    #[must_use]
    pub fn max_iterations(&self) -> i32 {
        self.params.max_iterations
    }

    /// The per-iteration score traces from the last align (`transform_probability_array` /
    /// `nearest_voxel_likelihood_array` of the C++ `NdtResult`).
    #[must_use]
    pub fn score_arrays(&self) -> (&[f32], &[f32]) {
        (
            &self.last.transform_probability_array,
            &self.last.nearest_voxel_likelihood_array,
        )
    }
}

// ---- C ABI shims (opaque `*mut NdtEngine` handle; pointers validated per rust-c-ffi-safety) ----

#[allow(
    clippy::arithmetic_side_effects,
    clippy::indexing_slicing,
    clippy::allow_attributes,
    reason = "r/c in 0..4 index a fixed 16-element slice and a fixed-size Matrix4"
)]
fn matrix4_from_row_major(buf: &[f32]) -> Matrix4<f32> {
    let mut m = Matrix4::<f32>::zeros();
    for r in 0..4 {
        for c in 0..4 {
            m[(r, c)] = buf[(r * 4) + c];
        }
    }
    m
}

/// Create a new engine handle (empty map). Free with `..._ndt_engine_free`.
#[expect(unsafe_code, reason = "C ABI boundary; returns an owned handle")]
#[unsafe(no_mangle)]
pub extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_new(
    resolution: f64,
    min_points: i32,
    eig_mult: f64,
) -> *mut NdtEngine {
    Box::into_raw(Box::new(NdtEngine::new(resolution, min_points, eig_mult)))
}

/// # Safety
/// `engine` is a handle from `..._ndt_engine_new`/`_clone` (or null → no-op); not used afterwards.
#[expect(unsafe_code, reason = "C ABI boundary; reclaims an owned handle")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_free(engine: *mut NdtEngine) {
    if engine.is_null() {
        return;
    }
    // SAFETY: `engine` came from `Box::into_raw` and is dropped exactly once.
    drop(unsafe { Box::from_raw(engine) });
}

/// Deep-copy an engine (the node's map-update double-buffer). Null → null.
/// # Safety
/// `engine` is a valid handle or null.
#[expect(unsafe_code, reason = "C ABI boundary; clones an owned handle")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_clone(
    engine: *const NdtEngine,
) -> *mut NdtEngine {
    if engine.is_null() {
        return core::ptr::null_mut();
    }
    // SAFETY: non-null per the check; caller guarantees a valid handle.
    let e = unsafe { &*engine };
    Box::into_raw(Box::new(e.clone()))
}

/// # Safety
/// `engine` is a valid handle (or null → no-op).
#[expect(unsafe_code, reason = "C ABI boundary; dereferences a handle")]
#[allow(
    clippy::cast_sign_loss,
    clippy::as_conversions,
    clippy::allow_attributes,
    reason = "num_threads is a non-negative count crossing the C ABI as i32"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_set_params(
    engine: *mut NdtEngine,
    trans_epsilon: f64,
    step_size: f64,
    resolution: f64,
    max_iterations: i32,
    outlier_ratio: f64,
    num_threads: i32,
) {
    if engine.is_null() {
        return;
    }
    // SAFETY: non-null per the check; caller guarantees a valid handle.
    let e = unsafe { &mut *engine };
    e.set_params(
        trans_epsilon,
        step_size,
        resolution,
        max_iterations,
        outlier_ratio,
        num_threads.max(0) as usize,
    );
}

/// # Safety
/// `engine` is a valid handle (or null → no-op).
#[expect(unsafe_code, reason = "C ABI boundary; dereferences a handle")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_set_regularization(
    engine: *mut NdtEngine,
    pose_x: f32,
    pose_y: f32,
    scale: f32,
) {
    if engine.is_null() {
        return;
    }
    // SAFETY: non-null per the check; caller guarantees a valid handle.
    let e = unsafe { &mut *engine };
    e.set_regularization(pose_x, pose_y, scale);
}

/// Add a map tile of `n` xyz `f32` triples (`3 * n` floats) under `id`.
/// # Safety
/// `engine` is a valid handle (or null → no-op); `points` addresses `3 * n` readable `f32`.
#[expect(unsafe_code, reason = "C ABI boundary; reads a caller-owned cloud")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_add_target(
    engine: *mut NdtEngine,
    points: *const f32,
    n: usize,
    id: u64,
) {
    if engine.is_null() || points.is_null() {
        return;
    }
    // SAFETY: non-null per the check; caller guarantees `3 * n` f32 at `points`.
    let (e, pts) = unsafe {
        (
            &mut *engine,
            core::slice::from_raw_parts(points.cast::<[f32; 3]>(), n),
        )
    };
    e.add_target(pts, id);
}

/// # Safety
/// `engine` is a valid handle (or null → no-op).
#[expect(unsafe_code, reason = "C ABI boundary; dereferences a handle")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_remove_target(
    engine: *mut NdtEngine,
    id: u64,
) {
    if engine.is_null() {
        return;
    }
    // SAFETY: non-null per the check; caller guarantees a valid handle.
    unsafe { &mut *engine }.remove_target(id);
}

/// # Safety
/// `engine` is a valid handle (or null → no-op).
#[expect(unsafe_code, reason = "C ABI boundary; dereferences a handle")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_create_kdtree(
    engine: *mut NdtEngine,
) {
    if engine.is_null() {
        return;
    }
    // SAFETY: non-null per the check; caller guarantees a valid handle.
    unsafe { &mut *engine }.create_kdtree();
}

/// # Safety
/// `engine` is a valid handle (or null → returns false).
#[expect(unsafe_code, reason = "C ABI boundary; dereferences a handle")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_has_target(
    engine: *const NdtEngine,
) -> bool {
    if engine.is_null() {
        return false;
    }
    // SAFETY: non-null per the check; caller guarantees a valid handle.
    unsafe { &*engine }.has_target()
}

/// # Safety
/// `engine` is a valid handle (or null → returns 0).
#[expect(unsafe_code, reason = "C ABI boundary; dereferences a handle")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_max_iterations(
    engine: *const NdtEngine,
) -> i32 {
    if engine.is_null() {
        return 0;
    }
    // SAFETY: non-null per the check; caller guarantees a valid handle.
    unsafe { &*engine }.max_iterations()
}

/// Align `source` (`3 * n` `f32`) from `guess` (16 row-major `f32`), storing the result internally
/// (retrieve with `..._get_result`).
/// # Safety
/// `engine` is a valid handle (or null → no-op); `guess` addresses 16 `f32`, `source` `3 * n` `f32`.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; reads caller-owned guess + cloud"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_align(
    engine: *mut NdtEngine,
    guess: *const f32,
    source: *const f32,
    n: usize,
) {
    if engine.is_null() || guess.is_null() || source.is_null() {
        return;
    }
    // SAFETY: non-null per the check; caller guarantees the documented lengths.
    let (e, guess_buf, src) = unsafe {
        (
            &mut *engine,
            core::slice::from_raw_parts(guess, 16),
            core::slice::from_raw_parts(source.cast::<[f32; 3]>(), n),
        )
    };
    e.align(&matrix4_from_row_major(guess_buf), src);
}

/// Score `cloud` (`3 * n` `f32`) without aligning.
/// # Safety
/// `engine` is a valid handle (or null → returns 0); `cloud` addresses `3 * n` readable `f32`.
#[expect(unsafe_code, reason = "C ABI boundary; reads a caller-owned cloud")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_calc_transformation_probability(
    engine: *mut NdtEngine,
    cloud: *const f32,
    n: usize,
) -> f64 {
    if engine.is_null() || cloud.is_null() {
        return 0.0;
    }
    // SAFETY: non-null per the check; caller guarantees `3 * n` f32 at `cloud`.
    let (e, c) = unsafe {
        (
            &mut *engine,
            core::slice::from_raw_parts(cloud.cast::<[f32; 3]>(), n),
        )
    };
    e.calc_transformation_probability(c)
}

/// Nearest-voxel likelihood of `cloud` (`3 * n` `f32`) without aligning.
/// # Safety
/// `engine` is a valid handle (or null → returns 0); `cloud` addresses `3 * n` readable `f32`.
#[expect(unsafe_code, reason = "C ABI boundary; reads a caller-owned cloud")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_calc_nearest_voxel_likelihood(
    engine: *mut NdtEngine,
    cloud: *const f32,
    n: usize,
) -> f64 {
    if engine.is_null() || cloud.is_null() {
        return 0.0;
    }
    // SAFETY: non-null per the check; caller guarantees `3 * n` f32 at `cloud`.
    let (e, c) = unsafe {
        (
            &mut *engine,
            core::slice::from_raw_parts(cloud.cast::<[f32; 3]>(), n),
        )
    };
    e.calc_nearest_voxel_likelihood(c)
}

/// Write the last align result into the C output buffers (same shape/contract as the `ndt_align`
/// FFI; each field null-skippable).
/// # Safety
/// `engine` is a valid handle (or null → no-op); `output` is a valid `AwNdtAlignOutput` whose
/// non-null pointers address their documented lengths.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; writes caller-owned output buffers"
)]
#[allow(
    clippy::arithmetic_side_effects,
    clippy::indexing_slicing,
    clippy::as_conversions,
    clippy::cast_possible_truncation,
    clippy::allow_attributes,
    reason = "fixed-size matrix marshaling (r/c in 0..4 / 0..6); bounded transform-array copy"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_get_result(
    engine: *const NdtEngine,
    output: *const AwNdtAlignOutput,
) {
    if engine.is_null() || output.is_null() {
        return;
    }
    // SAFETY: non-null per the check; caller guarantees a valid handle + output struct.
    let (e, out) = unsafe { (&*engine, &*output) };
    let result = e.result();
    // SAFETY: each non-null output pointer addresses its documented length.
    unsafe {
        if !out.pose.is_null() {
            let pose = core::slice::from_raw_parts_mut(out.pose, 16);
            for r in 0..4 {
                for c in 0..4 {
                    pose[(r * 4) + c] = result.pose[(r, c)];
                }
            }
        }
        if !out.iteration_num.is_null() {
            *out.iteration_num = result.iteration_num;
        }
        if !out.transform_probability.is_null() {
            *out.transform_probability = result.transform_probability;
        }
        if !out.nearest_voxel_likelihood.is_null() {
            *out.nearest_voxel_likelihood = result.nearest_voxel_likelihood;
        }
        if !out.hessian.is_null() {
            let h = core::slice::from_raw_parts_mut(out.hessian, 36);
            for r in 0..6 {
                for c in 0..6 {
                    h[(r * 6) + c] = result.hessian[(r, c)];
                }
            }
        }
        if !out.transformation_array.is_null() && !out.transforms_count.is_null() {
            let cap = out.transforms_cap as usize;
            let buf = core::slice::from_raw_parts_mut(out.transformation_array, cap * 16);
            for (k, m) in result.transformation_array.iter().take(cap).enumerate() {
                for r in 0..4 {
                    for c in 0..4 {
                        buf[(k * 16) + (r * 4) + c] = m[(r, c)];
                    }
                }
            }
            *out.transforms_count = result.transformation_array.len() as u32;
        }
    }
}

/// Per-point nearest-voxel score for `cloud` (`3 * n` `f32`): writes `n` scores to `out_scores`
/// (`out_scores[i] > 0` iff point `i` found a neighbor).
/// # Safety
/// `engine` is a valid handle (or null → no-op); `cloud` addresses `3 * n` `f32`, `out_scores` `n`.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; reads a cloud, writes per-point scores"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_calc_nearest_voxel_score_each_point(
    engine: *mut NdtEngine,
    cloud: *const f32,
    n: usize,
    out_scores: *mut f32,
) {
    if engine.is_null() || cloud.is_null() || out_scores.is_null() {
        return;
    }
    // SAFETY: non-null per the check; caller guarantees `3 * n` f32 at `cloud`, `n` at `out_scores`.
    let (e, c, out) = unsafe {
        (
            &mut *engine,
            core::slice::from_raw_parts(cloud.cast::<[f32; 3]>(), n),
            core::slice::from_raw_parts_mut(out_scores, n),
        )
    };
    let mut scores = alloc::vec::Vec::new();
    e.nearest_voxel_score_each_point(c, &mut scores);
    out.copy_from_slice(&scores);
}

/// Write the last align's per-iteration score traces. `out_tp`/`out_nvl` receive up to `cap` `f32`
/// each; `*out_count` gets the true length (== `iteration_num + 1`).
/// # Safety
/// `engine` is a valid handle (or null → no-op); `out_tp`/`out_nvl` address `cap` `f32`, `out_count`
/// one `u32`.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; writes caller-owned score-trace buffers"
)]
#[allow(
    clippy::as_conversions,
    clippy::cast_possible_truncation,
    clippy::indexing_slicing,
    clippy::allow_attributes,
    reason = "trace length crosses the C ABI as u32; k = min(len, cap) bounds the slicing"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_ndt_engine_get_score_arrays(
    engine: *const NdtEngine,
    out_tp: *mut f32,
    out_nvl: *mut f32,
    cap: u32,
    out_count: *mut u32,
) {
    if engine.is_null() || out_tp.is_null() || out_nvl.is_null() || out_count.is_null() {
        return;
    }
    // SAFETY: non-null per the check; caller guarantees `cap` f32 at each array + one u32.
    let (e, tp_out, nvl_out, count) = unsafe {
        (
            &*engine,
            core::slice::from_raw_parts_mut(out_tp, cap as usize),
            core::slice::from_raw_parts_mut(out_nvl, cap as usize),
            &mut *out_count,
        )
    };
    let (tp, nvl) = e.score_arrays();
    let k = tp.len().min(cap as usize);
    tp_out[..k].copy_from_slice(&tp[..k]);
    nvl_out[..k].copy_from_slice(&nvl[..k]);
    *count = tp.len() as u32;
}

// --- Phase N4a: sensor-callback align orchestrator (std-gated node glue) ---
// Folds align + oscillation count + the convergence verdict into one Rust call against the live
// engine, so the C++ sensor callback no longer drives align via the adapter + the C++
// `count_oscillation` + the separate `evaluate_convergence` FFI. Reuses the existing engine align,
// `helper::count_oscillation`, and `node::evaluate_convergence`. Uses `crate::node` (std-only), so
// the whole orchestrator is `std`-gated and excluded from the no_std rlib.

/// The `score_estimation` scalars that gate convergence (mirrors `AwConvergenceInput`'s param fields).
#[cfg(feature = "std")]
#[derive(Clone, Copy, Debug)]
pub struct ConvergenceParams {
    pub converged_param_type: i32,
    pub converged_param_transform_probability: f64,
    pub converged_param_nearest_voxel_transformation_likelihood: f64,
}

/// Result of [`run_align`]: the scalars + convergence verdict the C++ sensor callback needs (the
/// variable-length arrays/marker are still read via `get_result` in N4a).
#[cfg(feature = "std")]
#[derive(Clone, Copy, Debug)]
pub struct AlignOutcome {
    pub pose: Matrix4<f32>,
    pub transform_probability: f32,
    pub nearest_voxel_likelihood: f32,
    pub iteration_num: i32,
    pub max_iterations: i32,
    pub oscillation_num: i32,
    pub verdict: crate::node::ConvergenceVerdict,
}

/// Align the live engine from `guess`, then derive the oscillation count (from the iteration
/// trajectory translations, matching the C++ `count_oscillation(transformation_msg_array)`) and the
/// convergence verdict. Pure orchestration over existing ports — no new math.
#[cfg(feature = "std")]
#[must_use]
pub fn run_align(
    engine: &mut NdtEngine,
    guess: &Matrix4<f32>,
    source: &[[f32; 3]],
    conv: &ConvergenceParams,
) -> AlignOutcome {
    engine.align(guess, source);
    let max_iterations = engine.max_iterations();
    let result = engine.result();
    let positions: alloc::vec::Vec<[f64; 3]> = result
        .transformation_array
        .iter()
        .map(|m| {
            [
                f64::from(m[(0, 3)]),
                f64::from(m[(1, 3)]),
                f64::from(m[(2, 3)]),
            ]
        })
        .collect();
    let oscillation_num = crate::helper::count_oscillation(&positions);
    let verdict = crate::node::evaluate_convergence(&crate::node::ConvergenceInput {
        iteration_num: result.iteration_num,
        max_iterations,
        oscillation_num,
        transform_probability: f64::from(result.transform_probability),
        nearest_voxel_transformation_likelihood: f64::from(result.nearest_voxel_likelihood),
        converged_param_type: conv.converged_param_type,
        converged_param_transform_probability: conv.converged_param_transform_probability,
        converged_param_nearest_voxel_transformation_likelihood: conv
            .converged_param_nearest_voxel_transformation_likelihood,
    });
    AlignOutcome {
        pose: result.pose,
        transform_probability: result.transform_probability,
        nearest_voxel_likelihood: result.nearest_voxel_likelihood,
        iteration_num: result.iteration_num,
        max_iterations,
        oscillation_num,
        verdict,
    }
}

/// C ABI mirror of [`ConvergenceParams`] (same field order/types). Plain scalars.
#[cfg(feature = "std")]
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct AwAlignParams {
    pub converged_param_type: i32,
    pub converged_param_transform_probability: f64,
    pub converged_param_nearest_voxel_transformation_likelihood: f64,
}

/// C ABI result of [`autoware_ndt_scan_matcher_rs_node_run_align`]. `pose` is row-major 4x4; the
/// embedded verdict is the existing [`crate::node::AwConvergenceVerdict`].
#[cfg(feature = "std")]
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AwAlignOutcome {
    pub pose: [f32; 16],
    pub transform_probability: f32,
    pub nearest_voxel_likelihood: f32,
    pub iteration_num: i32,
    pub max_iterations: i32,
    pub oscillation_num: i32,
    pub verdict: crate::node::AwConvergenceVerdict,
}

/// FFI entry: align the live engine + return the scalars/verdict ([`run_align`]). No-op if any
/// pointer is null.
///
/// # Safety
/// `engine` is a valid live handle (or null → no-op). It is reborrowed as `&mut NdtEngine` for the
/// duration of this call, so the caller MUST guarantee **exclusive, non-concurrent** access to that
/// engine for the call: no other thread may touch the same engine (ROS 2 callbacks can run
/// concurrently across callback groups). The node satisfies this by only ever reaching the handle
/// inside its `Guarded<…>` engine mutex (`ndt_ptr_.with`). Rust does not retain the pointer past the
/// call. `guess` addresses 16 `f32`, `source` `3 * n` `f32`, `params` a valid [`AwAlignParams`],
/// `out` a writable [`AwAlignOutcome`].
#[cfg(feature = "std")]
#[expect(
    unsafe_code,
    reason = "C ABI boundary; reads caller-owned guess/cloud/params, validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_node_run_align(
    engine: *mut NdtEngine,
    guess: *const f32,
    source: *const f32,
    n: usize,
    params: *const AwAlignParams,
    out: *mut AwAlignOutcome,
) {
    if engine.is_null() || guess.is_null() || source.is_null() || params.is_null() || out.is_null()
    {
        return;
    }
    // SAFETY: non-null per the check; caller guarantees the documented lengths + valid structs.
    let (eng, guess_buf, src, prm) = unsafe {
        (
            &mut *engine,
            core::slice::from_raw_parts(guess, 16),
            core::slice::from_raw_parts(source.cast::<[f32; 3]>(), n),
            &*params,
        )
    };
    let outcome = run_align(
        eng,
        &matrix4_from_row_major(guess_buf),
        src,
        &ConvergenceParams {
            converged_param_type: prm.converged_param_type,
            converged_param_transform_probability: prm.converged_param_transform_probability,
            converged_param_nearest_voxel_transformation_likelihood: prm
                .converged_param_nearest_voxel_transformation_likelihood,
        },
    );
    let mat = &outcome.pose;
    // Row-major 4x4 as an explicit literal (nalgebra `(r,c)` indexing; no array computed-index).
    let pose = [
        mat[(0, 0)],
        mat[(0, 1)],
        mat[(0, 2)],
        mat[(0, 3)],
        mat[(1, 0)],
        mat[(1, 1)],
        mat[(1, 2)],
        mat[(1, 3)],
        mat[(2, 0)],
        mat[(2, 1)],
        mat[(2, 2)],
        mat[(2, 3)],
        mat[(3, 0)],
        mat[(3, 1)],
        mat[(3, 2)],
        mat[(3, 3)],
    ];
    let vd = &outcome.verdict;
    // SAFETY: `out` is non-null per the check and a valid, writable AwAlignOutcome per the contract.
    unsafe {
        *out = AwAlignOutcome {
            pose,
            transform_probability: outcome.transform_probability,
            nearest_voxel_likelihood: outcome.nearest_voxel_likelihood,
            iteration_num: outcome.iteration_num,
            max_iterations: outcome.max_iterations,
            oscillation_num: outcome.oscillation_num,
            verdict: crate::node::AwConvergenceVerdict {
                valid_param_type: vd.valid_param_type,
                is_ok_iteration_num: vd.is_ok_iteration_num,
                is_local_optimal_solution_oscillation: vd.is_local_optimal_solution_oscillation,
                is_ok_score: vd.is_ok_score,
                is_converged: vd.is_converged,
                score: vd.score,
                score_threshold: vd.score_threshold,
            },
        };
    }
}

#[cfg(test)]
#[allow(
    unsafe_code,
    clippy::float_cmp,
    clippy::indexing_slicing,
    clippy::arithmetic_side_effects,
    clippy::as_conversions,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::unreadable_literal,
    clippy::borrow_as_ptr,
    clippy::too_many_lines,
    clippy::allow_attributes,
    reason = "test code"
)]
mod tests {
    use super::*;
    use crate::ndt::{NdtParams, align};
    use alloc::vec::Vec;

    fn dense_cluster(cx: f32, cy: f32, cz: f32) -> Vec<[f32; 3]> {
        (0..8)
            .map(|i| {
                let f = i as f32 * 0.02;
                [cx + f, cy - f, cz + 0.5 * f]
            })
            .collect()
    }

    fn configured(engine: &mut NdtEngine) {
        engine.set_params(0.01, 0.1, 1.0, 30, 0.55, 1);
    }

    // Building the map incrementally through the engine + aligning equals a one-shot ndt::align on a
    // map with the same two tiles (the engine is a thin stateful wrapper — bit-identical).
    #[test]
    fn engine_align_matches_one_shot() {
        let tile_a = dense_cluster(0.5, 0.5, 0.5);
        let tile_b = dense_cluster(4.5, 0.5, 0.5);
        let source: Vec<[f32; 3]> = tile_a
            .iter()
            .chain(tile_b.iter())
            .map(|p| [p[0] + 0.1, p[1] - 0.05, p[2]])
            .collect();
        let guess = Matrix4::<f32>::identity();

        let mut engine = NdtEngine::new(1.0, 6, 0.01);
        configured(&mut engine);
        engine.add_target(&tile_a, 0);
        engine.add_target(&tile_b, 1);
        engine.create_kdtree();
        assert!(engine.has_target());
        engine.align(&guess, &source);
        let got = engine.result().clone();

        let mut map = VoxelGridMap::new([1.0; 3], 6, 0.01);
        map.add_target(&tile_a, 0);
        map.add_target(&tile_b, 1);
        map.create_kdtree();
        let params = NdtParams {
            trans_epsilon: 0.01,
            step_size: 0.1,
            resolution: 1.0,
            max_iterations: 30,
            outlier_ratio: 0.55,
            regularization: None,
            num_threads: 1,
        };
        let mut ws = AlignWorkspace::new();
        let mut want = AlignResult::default();
        align(&map, &source, &guess, &params, &mut ws, &mut want);

        assert_eq!(got.iteration_num, want.iteration_num);
        assert_eq!(got.pose, want.pose);
        assert_eq!(got.hessian, want.hessian);
        assert_eq!(got.transform_probability, want.transform_probability);
        assert_eq!(got.nearest_voxel_likelihood, want.nearest_voxel_likelihood);
    }

    fn two_tile_engine() -> (NdtEngine, Vec<[f32; 3]>) {
        let tile_a = dense_cluster(0.5, 0.5, 0.5);
        let tile_b = dense_cluster(4.5, 0.5, 0.5);
        let source: Vec<[f32; 3]> = tile_a
            .iter()
            .chain(tile_b.iter())
            .map(|p| [p[0] + 0.1, p[1] - 0.05, p[2]])
            .collect();
        let mut engine = NdtEngine::new(1.0, 6, 0.01);
        configured(&mut engine);
        engine.add_target(&tile_a, 0);
        engine.add_target(&tile_b, 1);
        engine.create_kdtree();
        (engine, source)
    }

    const TP_PARAMS: ConvergenceParams = ConvergenceParams {
        converged_param_type: 0, // TRANSFORM_PROBABILITY
        converged_param_transform_probability: 0.0,
        converged_param_nearest_voxel_transformation_likelihood: 0.0,
    };

    // run_align == composing engine.align + result + helper::count_oscillation + evaluate_convergence
    // by hand (the orchestrator adds no new math, just folds the existing ports).
    #[test]
    fn run_align_matches_manual_composition() {
        let (mut engine, source) = two_tile_engine();
        let guess = Matrix4::<f32>::identity();
        let outcome = run_align(&mut engine, &guess, &source, &TP_PARAMS);

        // Manual reference on a fresh, identically-built engine.
        let (mut ref_engine, ref_source) = two_tile_engine();
        ref_engine.align(&guess, &ref_source);
        let max_it = ref_engine.max_iterations();
        let r = ref_engine.result();
        let positions: Vec<[f64; 3]> = r
            .transformation_array
            .iter()
            .map(|m| {
                [
                    f64::from(m[(0, 3)]),
                    f64::from(m[(1, 3)]),
                    f64::from(m[(2, 3)]),
                ]
            })
            .collect();
        let osc = crate::helper::count_oscillation(&positions);
        let verdict = crate::node::evaluate_convergence(&crate::node::ConvergenceInput {
            iteration_num: r.iteration_num,
            max_iterations: max_it,
            oscillation_num: osc,
            transform_probability: f64::from(r.transform_probability),
            nearest_voxel_transformation_likelihood: f64::from(r.nearest_voxel_likelihood),
            converged_param_type: 0,
            converged_param_transform_probability: 0.0,
            converged_param_nearest_voxel_transformation_likelihood: 0.0,
        });

        assert_eq!(outcome.pose, r.pose);
        assert_eq!(outcome.transform_probability, r.transform_probability);
        assert_eq!(outcome.nearest_voxel_likelihood, r.nearest_voxel_likelihood);
        assert_eq!(outcome.iteration_num, r.iteration_num);
        assert_eq!(outcome.max_iterations, max_it);
        assert_eq!(outcome.oscillation_num, osc);
        assert_eq!(outcome.verdict, verdict);
    }

    // The FFI shim writes exactly what the pure run_align computes.
    #[test]
    fn ffi_run_align_matches_pure() {
        let (mut engine, source) = two_tile_engine();
        let guess = Matrix4::<f32>::identity();
        let pure = run_align(&mut engine, &guess, &source, &TP_PARAMS);

        let (mut ffi_engine, ffi_source) = two_tile_engine();
        let guess16 = [
            1.0f32, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ];
        let flat: Vec<f32> = ffi_source.iter().flat_map(|p| *p).collect();
        let params = AwAlignParams {
            converged_param_type: 0,
            converged_param_transform_probability: 0.0,
            converged_param_nearest_voxel_transformation_likelihood: 0.0,
        };
        let mut out = AwAlignOutcome::default();
        // SAFETY: engine valid; guess 16 f32; flat 3*n f32; params/out valid.
        unsafe {
            autoware_ndt_scan_matcher_rs_node_run_align(
                &mut ffi_engine,
                guess16.as_ptr(),
                flat.as_ptr(),
                ffi_source.len(),
                &params,
                &mut out,
            );
        }
        assert_eq!(out.transform_probability, pure.transform_probability);
        assert_eq!(out.nearest_voxel_likelihood, pure.nearest_voxel_likelihood);
        assert_eq!(out.iteration_num, pure.iteration_num);
        assert_eq!(out.max_iterations, pure.max_iterations);
        assert_eq!(out.oscillation_num, pure.oscillation_num);
        assert_eq!(out.verdict.is_converged, pure.verdict.is_converged);
        assert_eq!(out.pose[0], pure.pose[(0, 0)]);
        assert_eq!(out.pose[3], pure.pose[(0, 3)]);
        assert_eq!(out.pose[15], pure.pose[(3, 3)]);
    }

    // Null pointers are a no-op (no write, no panic).
    #[test]
    fn ffi_run_align_null_is_noop() {
        let mut out = AwAlignOutcome {
            iteration_num: 123,
            ..AwAlignOutcome::default()
        };
        let params = AwAlignParams {
            converged_param_type: 0,
            converged_param_transform_probability: 0.0,
            converged_param_nearest_voxel_transformation_likelihood: 0.0,
        };
        let guess16 = [0.0f32; 16];
        // SAFETY: a null engine must leave `out` untouched.
        unsafe {
            autoware_ndt_scan_matcher_rs_node_run_align(
                core::ptr::null_mut(),
                guess16.as_ptr(),
                guess16.as_ptr(),
                0,
                &params,
                &mut out,
            );
        }
        assert_eq!(out.iteration_num, 123);
    }

    // clone() is an independent deep copy: mutating the original's map after cloning must not affect
    // the clone's alignment.
    #[test]
    fn engine_clone_is_independent() {
        let tile_a = dense_cluster(0.5, 0.5, 0.5);
        let tile_b = dense_cluster(4.5, 0.5, 0.5);
        let source: Vec<[f32; 3]> = tile_a.iter().map(|p| [p[0] + 0.1, p[1], p[2]]).collect();
        let guess = Matrix4::<f32>::identity();

        let mut original = NdtEngine::new(1.0, 6, 0.01);
        configured(&mut original);
        original.add_target(&tile_a, 0);
        original.add_target(&tile_b, 1);
        original.create_kdtree();

        let mut clone = original.clone();
        clone.align(&guess, &source);
        let clone_before = clone.result().pose;

        // Mutate the original after the clone; the clone must be unaffected.
        original.remove_target(1);
        original.create_kdtree();
        original.align(&guess, &source);

        clone.align(&guess, &source);
        assert_eq!(
            clone.result().pose,
            clone_before,
            "clone shares state with original"
        );
    }

    #[test]
    fn engine_remove_target_and_has_target() {
        let mut engine = NdtEngine::new(1.0, 6, 0.01);
        assert!(!engine.has_target());
        engine.add_target(&dense_cluster(0.5, 0.5, 0.5), 0);
        engine.add_target(&dense_cluster(4.5, 0.5, 0.5), 1);
        assert!(engine.has_target());
        engine.remove_target(0);
        engine.remove_target(1);
        assert!(!engine.has_target());
    }

    // FFI handle round-trip == the pure engine API (marshaling check).
    #[test]
    fn ffi_engine_matches_pure() {
        let tile = dense_cluster(0.5, 0.5, 0.5);
        let flat: Vec<f32> = tile.iter().flatten().copied().collect();
        let source: Vec<[f32; 3]> = tile.iter().map(|p| [p[0] + 0.1, p[1], p[2]]).collect();
        let src_flat: Vec<f32> = source.iter().flatten().copied().collect();
        let guess16 = [
            1.0_f32, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ];

        // pure
        let mut pure = NdtEngine::new(1.0, 6, 0.01);
        configured(&mut pure);
        pure.add_target(&tile, 0);
        pure.create_kdtree();
        pure.align(&matrix4_from_row_major(&guess16), &source);
        let want = pure.result().clone();
        let want_tp_score = pure.calc_transformation_probability(&source);
        let want_nvl_score = pure.calc_nearest_voxel_likelihood(&source);
        let want_maxit = pure.max_iterations();
        let mut want_scores = Vec::new();
        pure.nearest_voxel_score_each_point(&source, &mut want_scores);
        let (want_tp_arr, want_nvl_arr) = {
            let (a, b) = pure.score_arrays();
            (a.to_vec(), b.to_vec())
        };

        // FFI
        // SAFETY: every pointer below addresses its documented length.
        unsafe {
            let e = autoware_ndt_scan_matcher_rs_ndt_engine_new(1.0, 6, 0.01);
            autoware_ndt_scan_matcher_rs_ndt_engine_set_params(e, 0.01, 0.1, 1.0, 30, 0.55, 1);
            autoware_ndt_scan_matcher_rs_ndt_engine_add_target(e, flat.as_ptr(), tile.len(), 0);
            autoware_ndt_scan_matcher_rs_ndt_engine_create_kdtree(e);
            assert!(autoware_ndt_scan_matcher_rs_ndt_engine_has_target(e));
            autoware_ndt_scan_matcher_rs_ndt_engine_align(
                e,
                guess16.as_ptr(),
                src_flat.as_ptr(),
                source.len(),
            );

            let mut pose = [0.0_f32; 16];
            let mut iter = 0_i32;
            let mut tp = 0.0_f32;
            let mut nvl = 0.0_f32;
            let mut hessian = [0.0_f64; 36];
            let mut count = 0_u32;
            let out = AwNdtAlignOutput {
                pose: pose.as_mut_ptr(),
                iteration_num: &mut iter,
                transform_probability: &mut tp,
                nearest_voxel_likelihood: &mut nvl,
                hessian: hessian.as_mut_ptr(),
                transformation_array: core::ptr::null_mut(),
                transforms_cap: 0,
                transforms_count: &mut count,
            };
            autoware_ndt_scan_matcher_rs_ndt_engine_get_result(e, &out);

            assert_eq!(iter, want.iteration_num);
            assert_eq!(tp, want.transform_probability);
            assert_eq!(nvl, want.nearest_voxel_likelihood);
            for r in 0..4 {
                for c in 0..4 {
                    assert_eq!(pose[(r * 4) + c], want.pose[(r, c)]);
                }
            }

            // Remaining shims marshal to the same values as the pure API.
            assert_eq!(
                autoware_ndt_scan_matcher_rs_ndt_engine_max_iterations(e),
                want_maxit
            );
            assert_eq!(
                autoware_ndt_scan_matcher_rs_ndt_engine_calc_transformation_probability(
                    e,
                    src_flat.as_ptr(),
                    source.len()
                ),
                want_tp_score
            );
            assert_eq!(
                autoware_ndt_scan_matcher_rs_ndt_engine_calc_nearest_voxel_likelihood(
                    e,
                    src_flat.as_ptr(),
                    source.len()
                ),
                want_nvl_score
            );
            // set_regularization shim (no-op with scale 0); clone is an independent live handle.
            autoware_ndt_scan_matcher_rs_ndt_engine_set_regularization(e, 0.0, 0.0, 0.0);
            let e2 = autoware_ndt_scan_matcher_rs_ndt_engine_clone(e);
            assert!(autoware_ndt_scan_matcher_rs_ndt_engine_has_target(e2));
            autoware_ndt_scan_matcher_rs_ndt_engine_free(e2);
            // remove the only tile -> no target left.
            // per-point score + per-iteration score arrays marshal to the pure values.
            let mut ffi_scores = vec![0.0_f32; source.len()];
            autoware_ndt_scan_matcher_rs_ndt_engine_calc_nearest_voxel_score_each_point(
                e,
                src_flat.as_ptr(),
                source.len(),
                ffi_scores.as_mut_ptr(),
            );
            assert_eq!(ffi_scores, want_scores);

            let cap = (want_tp_arr.len() as u32) + 4;
            let mut tp_arr = vec![0.0_f32; cap as usize];
            let mut nvl_arr = vec![0.0_f32; cap as usize];
            let mut scount = 0_u32;
            autoware_ndt_scan_matcher_rs_ndt_engine_get_score_arrays(
                e,
                tp_arr.as_mut_ptr(),
                nvl_arr.as_mut_ptr(),
                cap,
                &mut scount,
            );
            assert_eq!(scount as usize, want_tp_arr.len());
            assert_eq!(&tp_arr[..scount as usize], want_tp_arr.as_slice());
            assert_eq!(&nvl_arr[..scount as usize], want_nvl_arr.as_slice());

            autoware_ndt_scan_matcher_rs_ndt_engine_remove_target(e, 0);
            assert!(!autoware_ndt_scan_matcher_rs_ndt_engine_has_target(e));
            autoware_ndt_scan_matcher_rs_ndt_engine_free(e);
        }
    }
}
