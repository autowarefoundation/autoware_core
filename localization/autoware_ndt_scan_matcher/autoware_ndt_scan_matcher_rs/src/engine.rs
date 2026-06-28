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
    nearest_voxel_transformation_likelihood, transformation_probability,
};
use crate::transform::gauss_constants;
use crate::voxel_grid::VoxelGridMap;

/// Persistent NDT engine: the target map, the active params (incl. optional regularization), a
/// reused align workspace, and the most recent align result. Mirrors the stateful C++
/// `MultiGridNormalDistributionsTransform` object the node holds across frames.
pub struct NdtEngine {
    map: VoxelGridMap,
    params: NdtParams,
    workspace: AlignWorkspace,
    last: AlignResult,
}

impl Clone for NdtEngine {
    fn clone(&self) -> Self {
        // The scratch workspace is not state — a clone starts fresh (it re-warms on first align).
        Self {
            map: self.map.clone(),
            params: self.params,
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
            workspace: AlignWorkspace::new(),
            last: AlignResult::default(),
        }
    }

    /// Update the alignment params (the C++ `setParams`). The regularization is preserved — it is
    /// set separately via [`Self::set_regularization`], mirroring `setRegularizationPose`.
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

    /// The configured iteration cap (the C++ `getMaximumIterations`).
    #[must_use]
    pub fn max_iterations(&self) -> i32 {
        self.params.max_iterations
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
            autoware_ndt_scan_matcher_rs_ndt_engine_remove_target(e, 0);
            assert!(!autoware_ndt_scan_matcher_rs_ndt_engine_has_target(e));
            autoware_ndt_scan_matcher_rs_ndt_engine_free(e);
        }
    }
}
