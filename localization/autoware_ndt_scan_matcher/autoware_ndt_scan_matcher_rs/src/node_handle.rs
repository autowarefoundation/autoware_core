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

//! The opaque object-level node handle (`NdtScanMatcherRs`) + its C ABI lifecycle, and the
//! `AwNdtParams` struct that crosses the boundary once at construction.
//!
//! Foundation slice (roadmap `plan/ndt_in_rust_next.md` → Phase 0/1): this introduces the opaque
//! handle the later phases hang off, plus a single validated [`Params`] conversion replacing the
//! piecemeal scalar-by-scalar engine configuration. The handle owns [`Params`] + the node-state
//! scaffolding (`NodeState`/`MapUpdateState`) that later slices migrate out of C++. It does **not**
//! own the engine yet — that stays in the C++ `NdtRustAdapter` during the transition (touching it now
//! would collide with the map-update double-buffer copy semantics; see Phase 8). The handle is
//! therefore inert this slice: constructed/destroyed and exercised by tests, not yet driving a
//! callback. std-only: it is the ROS-node shell, excluded from the `no_std` kernel build.

use std::sync::Mutex;

use crate::ffi::{Error, ffi_boundary_ptr};
use crate::pose_buffer::{InterpolateResult, PoseBuffer, TimedPoseWithCov};

/// The validated, Rust-owned parameters the node needs (the union of the engine's `set_params` /
/// `set_convergence_params` / `set_covariance_config` inputs). Converted once from [`AwNdtParams`]
/// at construction; the offset-model vectors are **copied** so nothing borrows C++ memory past the
/// call (roadmap rules 10/11).
#[derive(Clone, Debug)]
pub struct Params {
    // Engine construction (the `MultiVoxelGridCovariance` knobs).
    pub resolution: f64,
    pub min_points: i32,
    pub eig_mult: f64,
    // Alignment (`set_params`).
    pub trans_epsilon: f64,
    pub step_size: f64,
    pub max_iterations: i32,
    pub outlier_ratio: f64,
    pub num_threads: usize,
    // Convergence (`set_convergence_params`).
    pub converged_param_type: i32,
    pub converged_param_transform_probability: f64,
    pub converged_param_nearest_voxel_transformation_likelihood: f64,
    // Covariance (`set_covariance_config`).
    pub covariance_estimation_type: i32,
    pub covariance_scale_factor: f64,
    pub covariance_temperature: f64,
    pub output_pose_covariance: [f64; 36],
    pub initial_pose_offset_model_x: Vec<f64>,
    pub initial_pose_offset_model_y: Vec<f64>,
    // Regularization pose buffer (`ndt.regularization.*` + the SmartPoseBuffer tolerances).
    pub regularization_enable: bool,
    pub regularization_pose_timeout_sec: f64,
    pub regularization_pose_distance_tolerance_m: f64,
}

/// Node-level state the algorithm core does not own — the migration target of Phases 1–4. Defaulted
/// (inactive, no pose, no skips) at construction; later slices move the C++ `is_activated_` /
/// `latest_ekf_position_` / `skipping_publish_num_` into here.
#[derive(Clone, Debug, Default)]
pub struct NodeState {
    pub is_activated: bool,
    /// `latest_ekf_position_` (x, y, z), for dynamic map loading. Full timed pose lands in Phase 2.
    pub latest_ekf_position: Option<[f64; 3]>,
    pub skipping_publish_num: i64,
}

/// Map-update bookkeeping — the migration target of Phase 6. Defaulted at construction.
#[derive(Clone, Debug, Default)]
pub struct MapUpdateState {
    /// Position (x, y) at the last successful map update.
    pub last_update_position: Option<[f64; 2]>,
    /// Cell ids currently loaded (raw PCD `cell_id` bytes — not assumed UTF-8).
    pub loaded_map_ids: Vec<Vec<u8>>,
    pub need_rebuild: bool,
}

/// The opaque node object C++ holds (as `AwNdtScanMatcher *`). Owns the validated params, node-state
/// scaffolding, and (Phase 1 slice A) the Rust-owned regularization pose buffer; the engine is still
/// owned by the C++ `NdtRustAdapter` this slice. Accessed through a shared `*const` from concurrent
/// ROS callbacks, so mutable node state uses interior locking (`Mutex`) — separate from, and finer
/// than, the lock-free engine.
pub struct NdtScanMatcherRs {
    pub params: Params,
    pub state: NodeState,
    pub map_update_state: MapUpdateState,
    /// `Some` iff `params.regularization_enable` (mirrors the C++ conditional buffer creation).
    regularization_buffer: Option<Mutex<PoseBuffer>>,
}

impl NdtScanMatcherRs {
    fn new(params: Params) -> Self {
        let regularization_buffer = if params.regularization_enable {
            Some(Mutex::new(PoseBuffer::new(
                params.regularization_pose_timeout_sec,
                params.regularization_pose_distance_tolerance_m,
            )))
        } else {
            None
        };
        Self {
            params,
            state: NodeState::default(),
            map_update_state: MapUpdateState::default(),
            regularization_buffer,
        }
    }

    /// Push a regularization pose into the Rust-owned buffer (no-op if regularization is disabled or
    /// the lock is poisoned). Called by `on_regularization_pose`.
    pub(crate) fn push_regularization(&self, pose: &TimedPoseWithCov) {
        if let Some(buf) = &self.regularization_buffer
            && let Ok(mut b) = buf.lock()
        {
            b.push_back(*pose);
        }
    }

    /// Interpolate the regularization pose at `target_ns`, then drop entries older than it (mirrors
    /// the C++ `add_regularization_pose`: `interpolate` then `pop_old`). `None` if disabled, locked-
    /// poisoned, or the buffer cannot interpolate.
    pub(crate) fn interpolate_regularization(&self, target_ns: i64) -> Option<InterpolateResult> {
        let mut b = self.regularization_buffer.as_ref()?.lock().ok()?;
        let result = b.interpolate(target_ns)?;
        b.pop_old(target_ns);
        Some(result)
    }
}

/// C ABI mirror of the node's parameters. Scalars cross by value; the variable-length covariance
/// offset models cross as `(ptr, len)` borrowed for the duration of the `_new` call only (Rust copies
/// them). `#[repr(C)]` fixes the layout.
#[repr(C)]
pub struct AwNdtParams {
    pub resolution: f64,
    pub min_points: i32,
    pub eig_mult: f64,
    pub trans_epsilon: f64,
    pub step_size: f64,
    pub max_iterations: i32,
    pub outlier_ratio: f64,
    pub num_threads: i32,
    pub converged_param_type: i32,
    pub converged_param_transform_probability: f64,
    pub converged_param_nearest_voxel_transformation_likelihood: f64,
    pub covariance_estimation_type: i32,
    pub covariance_scale_factor: f64,
    pub covariance_temperature: f64,
    pub output_pose_covariance: [f64; 36],
    pub initial_pose_offset_model_x: *const f64,
    pub initial_pose_offset_model_x_len: usize,
    pub initial_pose_offset_model_y: *const f64,
    pub initial_pose_offset_model_y_len: usize,
    pub regularization_enable: bool,
    pub regularization_pose_timeout_sec: f64,
    pub regularization_pose_distance_tolerance_m: f64,
}

/// C-ABI view of a `geometry_msgs::PoseWithCovarianceStamped`, borrowed for the call only (Rust copies
/// what it keeps). `orientation` is `[x, y, z, w]`; `covariance` is the row-major 6x6. No `frame_id`
/// yet — the regularization path does no frame check; it is added when the initial-pose slice needs it.
#[repr(C)]
pub struct AwPoseWithCovarianceStampedView {
    pub stamp_ns: i64,
    pub position: [f64; 3],
    pub orientation: [f64; 4],
    pub covariance: [f64; 36],
}

impl AwPoseWithCovarianceStampedView {
    pub(crate) fn to_timed(&self) -> TimedPoseWithCov {
        TimedPoseWithCov {
            stamp_ns: self.stamp_ns,
            position: self.position,
            orientation: self.orientation,
            covariance: self.covariance,
        }
    }
}

/// C-ABI out-struct for an interpolated pose (`orientation` = `[x, y, z, w]`).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AwInterpolatedPose {
    pub position: [f64; 3],
    pub orientation: [f64; 4],
}

/// View `[ptr, ptr+len)` as an `f64` slice, treating a null/zero-length pointer as empty.
///
/// # Safety
/// When `len > 0`, `ptr` must point to `len` readable, initialized, `f64`-aligned values that outlive
/// the borrow.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointer validated per rust-c-ffi-safety"
)]
unsafe fn f64_slice<'a>(ptr: *const f64, len: usize) -> &'a [f64] {
    if ptr.is_null() || len == 0 {
        return &[];
    }
    // SAFETY: non-null with len > 0 per the check; caller guarantees `len` readable f64 values.
    unsafe { core::slice::from_raw_parts(ptr, len) }
}

impl Params {
    /// Validate + copy an [`AwNdtParams`] into an owned [`Params`]. Rejects mismatched offset-model
    /// lengths (mirrors the C++ `HyperParameters` check) and a negative `num_threads`.
    ///
    /// # Safety
    /// The offset-model pointers in `p` must each be valid for their stated length (or null with
    /// length 0). They are read once and copied; no pointer is retained.
    #[expect(
        unsafe_code,
        reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
    )]
    unsafe fn try_from_ffi(p: &AwNdtParams) -> Result<Self, Error> {
        if p.initial_pose_offset_model_x_len != p.initial_pose_offset_model_y_len {
            return Err(Error::InvalidParam);
        }
        let num_threads = usize::try_from(p.num_threads).map_err(|_| Error::InvalidParam)?;
        // SAFETY: caller guarantees each offset pointer is valid for its stated length (or null/0).
        let offset_x =
            unsafe { f64_slice(p.initial_pose_offset_model_x, p.initial_pose_offset_model_x_len) };
        // SAFETY: same contract as above for the y model.
        let offset_y =
            unsafe { f64_slice(p.initial_pose_offset_model_y, p.initial_pose_offset_model_y_len) };
        Ok(Self {
            resolution: p.resolution,
            min_points: p.min_points,
            eig_mult: p.eig_mult,
            trans_epsilon: p.trans_epsilon,
            step_size: p.step_size,
            max_iterations: p.max_iterations,
            outlier_ratio: p.outlier_ratio,
            num_threads,
            converged_param_type: p.converged_param_type,
            converged_param_transform_probability: p.converged_param_transform_probability,
            converged_param_nearest_voxel_transformation_likelihood: p
                .converged_param_nearest_voxel_transformation_likelihood,
            covariance_estimation_type: p.covariance_estimation_type,
            covariance_scale_factor: p.covariance_scale_factor,
            covariance_temperature: p.covariance_temperature,
            output_pose_covariance: p.output_pose_covariance,
            initial_pose_offset_model_x: offset_x.to_vec(),
            initial_pose_offset_model_y: offset_y.to_vec(),
            regularization_enable: p.regularization_enable,
            regularization_pose_timeout_sec: p.regularization_pose_timeout_sec,
            regularization_pose_distance_tolerance_m: p.regularization_pose_distance_tolerance_m,
        })
    }
}

/// Construct the opaque node handle from `params`. Returns a heap-owned pointer (free with
/// [`autoware_ndt_scan_matcher_rs_free`]) or null on null/invalid params or a contained panic.
///
/// # Safety
/// `params` must point to a valid, aligned [`AwNdtParams`] (or be null → returns null) whose
/// offset-model pointers are each valid for their stated length. Nothing is retained past the call.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_new(
    params: *const AwNdtParams,
) -> *mut NdtScanMatcherRs {
    ffi_boundary_ptr(|| {
        // SAFETY: caller guarantees a valid, aligned AwNdtParams (or null → None → NullPtr error).
        let p = unsafe { params.as_ref() }.ok_or(Error::NullPtr)?;
        // SAFETY: the offset-model pointers in `p` are valid for their stated lengths per the contract.
        let params = unsafe { Params::try_from_ffi(p) }?;
        Ok(Box::into_raw(Box::new(NdtScanMatcherRs::new(params))))
    })
}

/// Free a handle from [`autoware_ndt_scan_matcher_rs_new`]. Null-safe; double-free is undefined.
///
/// # Safety
/// `ptr` must be null, or a pointer returned by `_new` that has not already been freed.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointer validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_free(ptr: *mut NdtScanMatcherRs) {
    if !ptr.is_null() {
        // SAFETY: non-null and produced by `_new` (Box::into_raw) per the contract; freed once.
        drop(unsafe { Box::from_raw(ptr) });
    }
}

/// Interpolate the regularization pose at `stamp_ns` from the handle's Rust-owned buffer, writing the
/// result into `*out` and returning `true`; returns `false` (leaving `*out` untouched) if
/// regularization is disabled or the buffer cannot interpolate (the C++ `if (!opt) return;`). Also
/// drops buffer entries older than `stamp_ns` (the C++ `pop_old`). Transitional: the still-C++ sensor
/// callback calls this; it is removed when the sensor callback itself moves to Rust (Phase 5).
///
/// # Safety
/// `handle` must be a valid, live `NdtScanMatcherRs` from `_new` (or null → `false`), and `out` a
/// valid, aligned, writable [`AwInterpolatedPose`] (or null → `false`).
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_regularization_interpolate(
    handle: *const NdtScanMatcherRs,
    stamp_ns: i64,
    out: *mut AwInterpolatedPose,
) -> bool {
    if handle.is_null() || out.is_null() {
        return false;
    }
    // SAFETY: non-null per the check; caller guarantees a valid, live handle.
    let h = unsafe { &*handle };
    let Some(result) = h.interpolate_regularization(stamp_ns) else {
        return false;
    };
    // SAFETY: `out` is non-null per the check and a valid, aligned, writable struct per the contract.
    unsafe {
        *out = AwInterpolatedPose {
            position: result.position,
            orientation: result.orientation,
        };
    }
    true
}

#[cfg(test)]
#[allow(
    unsafe_code,
    clippy::unwrap_used,
    clippy::indexing_slicing,
    clippy::float_cmp,
    clippy::allow_attributes,
    reason = "test code: C ABI new/free calls; exact-equal float asserts on deterministic passthrough"
)]
mod tests {
    use super::*;

    fn sample_params(ox: &[f64], oy: &[f64]) -> AwNdtParams {
        let mut cov = [0.0_f64; 36];
        cov[0] = 1.0;
        cov[35] = 2.0;
        AwNdtParams {
            resolution: 2.0,
            min_points: 6,
            eig_mult: 0.01,
            trans_epsilon: 0.01,
            step_size: 0.1,
            max_iterations: 30,
            outlier_ratio: 0.55,
            num_threads: 4,
            converged_param_type: 1,
            converged_param_transform_probability: 3.0,
            converged_param_nearest_voxel_transformation_likelihood: 2.3,
            covariance_estimation_type: 2,
            covariance_scale_factor: 1.5,
            covariance_temperature: 0.05,
            output_pose_covariance: cov,
            initial_pose_offset_model_x: ox.as_ptr(),
            initial_pose_offset_model_x_len: ox.len(),
            initial_pose_offset_model_y: oy.as_ptr(),
            initial_pose_offset_model_y_len: oy.len(),
            regularization_enable: true,
            regularization_pose_timeout_sec: 1000.0,
            regularization_pose_distance_tolerance_m: 1000.0,
        }
    }

    fn pose_view(stamp_ns: i64, x: f64, y: f64) -> AwPoseWithCovarianceStampedView {
        AwPoseWithCovarianceStampedView {
            stamp_ns,
            position: [x, y, 0.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
            covariance: [0.0; 36],
        }
    }

    #[test]
    fn new_then_free_roundtrips_params() {
        let ox = [0.0, 0.5, -0.5];
        let oy = [0.0, 0.5, 0.5];
        let p = sample_params(&ox, &oy);
        // SAFETY: `p` is a valid AwNdtParams with offset pointers valid for their lengths.
        let handle = unsafe { autoware_ndt_scan_matcher_rs_new(&raw const p) };
        assert!(!handle.is_null());
        // SAFETY: non-null handle from `_new`; we own it for the duration of the test.
        let m = unsafe { &*handle };
        assert_eq!(m.params.resolution, 2.0);
        assert_eq!(m.params.num_threads, 4);
        assert_eq!(m.params.output_pose_covariance[35], 2.0);
        assert_eq!(m.params.initial_pose_offset_model_x, vec![0.0, 0.5, -0.5]);
        assert_eq!(m.params.initial_pose_offset_model_y, vec![0.0, 0.5, 0.5]);
        // Node-state scaffolding starts empty.
        assert!(!m.state.is_activated);
        assert!(m.state.latest_ekf_position.is_none());
        assert!(m.map_update_state.last_update_position.is_none());
        // SAFETY: `handle` is a live pointer from `_new`, freed exactly once here.
        unsafe { autoware_ndt_scan_matcher_rs_free(handle) };
    }

    #[test]
    fn null_params_yields_null_handle() {
        // SAFETY: null is an explicitly handled input (→ null handle, no UB).
        let handle = unsafe { autoware_ndt_scan_matcher_rs_new(core::ptr::null()) };
        assert!(handle.is_null());
    }

    #[test]
    fn mismatched_offset_lengths_yields_null_handle() {
        let ox = [0.0, 0.5];
        let oy = [0.0];
        let p = sample_params(&ox, &oy);
        // SAFETY: valid struct; the length mismatch is rejected, returning a null handle.
        let handle = unsafe { autoware_ndt_scan_matcher_rs_new(&raw const p) };
        assert!(handle.is_null());
    }

    #[test]
    fn negative_num_threads_yields_null_handle() {
        let ox: [f64; 0] = [];
        let oy: [f64; 0] = [];
        let mut p = sample_params(&ox, &oy);
        p.num_threads = -1;
        // SAFETY: valid struct; the negative thread count is rejected, returning a null handle.
        let handle = unsafe { autoware_ndt_scan_matcher_rs_new(&raw const p) };
        assert!(handle.is_null());
    }

    #[test]
    fn free_null_is_noop() {
        // SAFETY: null is explicitly handled as a no-op.
        unsafe { autoware_ndt_scan_matcher_rs_free(core::ptr::null_mut()) };
    }

    #[test]
    fn regularization_push_then_interpolate_via_ffi() {
        let ox: [f64; 0] = [];
        let oy: [f64; 0] = [];
        let p = sample_params(&ox, &oy); // regularization_enable = true
        // SAFETY: valid params.
        let handle = unsafe { autoware_ndt_scan_matcher_rs_new(&raw const p) };
        assert!(!handle.is_null());
        // SAFETY: live handle from `_new`.
        let h = unsafe { &*handle };
        h.push_regularization(&pose_view(1, 0.0, 0.0).to_timed());
        h.push_regularization(&pose_view(2_000_000_001, 2.0, 4.0).to_timed());

        let mut out = AwInterpolatedPose::default();
        // SAFETY: live handle + writable out.
        let ok = unsafe {
            autoware_ndt_scan_matcher_rs_regularization_interpolate(handle, 1_000_000_001, &raw mut out)
        };
        assert!(ok);
        assert!((out.position[0] - 1.0).abs() < 1e-9);
        assert!((out.position[1] - 2.0).abs() < 1e-9);
        // SAFETY: freed once.
        unsafe { autoware_ndt_scan_matcher_rs_free(handle) };
    }

    #[test]
    fn regularization_interpolate_false_when_disabled() {
        let ox: [f64; 0] = [];
        let oy: [f64; 0] = [];
        let mut p = sample_params(&ox, &oy);
        p.regularization_enable = false;
        // SAFETY: valid params.
        let handle = unsafe { autoware_ndt_scan_matcher_rs_new(&raw const p) };
        assert!(!handle.is_null());
        let mut out = AwInterpolatedPose::default();
        // SAFETY: live handle + writable out; disabled buffer → false.
        let ok = unsafe {
            autoware_ndt_scan_matcher_rs_regularization_interpolate(handle, 1_000_000_001, &raw mut out)
        };
        assert!(!ok);
        // SAFETY: freed once.
        unsafe { autoware_ndt_scan_matcher_rs_free(handle) };
    }

    #[test]
    fn regularization_interpolate_null_handle_is_false() {
        let mut out = AwInterpolatedPose::default();
        // SAFETY: null handle is explicitly handled → false.
        let ok = unsafe {
            autoware_ndt_scan_matcher_rs_regularization_interpolate(
                core::ptr::null(),
                1,
                &raw mut out,
            )
        };
        assert!(!ok);
    }
}
