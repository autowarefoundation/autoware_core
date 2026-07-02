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
use std::sync::atomic::{AtomicBool, Ordering};

use crate::ffi::{Error, ffi_boundary_ptr};
use crate::node::{MapUpdateInput, evaluate_map_update};
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
    // Initial pose: the expected frame (`frame.map_frame`, owned bytes) + the SmartPoseBuffer
    // tolerances (`validation.initial_pose_*`).
    pub map_frame: Vec<u8>,
    pub initial_pose_timeout_sec: f64,
    pub initial_pose_distance_tolerance_m: f64,
    // Sensor-points prologue (`frame.base_frame` + `sensor_points.*`).
    pub base_frame: Vec<u8>,
    pub sensor_points_timeout_sec: f64,
    pub sensor_points_required_distance: f64,
}

/// Map-update decision state (Phase 6): the position of the last attempted map update, and whether
/// the next update must rebuild from scratch (set on the first update + whenever the loaded map can
/// no longer keep up with the lidar range; cleared on a successful update). The C++
/// `MapUpdateModule::last_update_position_` + `BuilderState::need_rebuild`. The loaded cell ids live
/// in the engine, not here.
#[derive(Clone, Debug, Default)]
pub struct MapUpdateState {
    /// Position (x, y) at the last attempted map update.
    pub last_update_position: Option<[f64; 2]>,
    pub need_rebuild: bool,
}

/// The opaque node object C++ holds (as `AwNdtScanMatcher *`). Owns the validated params, node-state
/// scaffolding, and (Phase 1 slice A) the Rust-owned regularization pose buffer; the engine is still
/// owned by the C++ `NdtRustAdapter` this slice. Accessed through a shared `*const` from concurrent
/// ROS callbacks, so mutable node state uses interior locking (`Mutex`) — separate from, and finer
/// than, the lock-free engine.
pub struct NdtScanMatcherRs {
    pub params: Params,
    /// `is_activated_` (the C++ `std::atomic<bool>`): read by every callback gate, set by the trigger.
    is_activated: AtomicBool,
    /// `latest_ekf_position_` (x, y, z), set by `on_initial_pose`, read by the map-update timer.
    latest_ekf_position: Mutex<Option<[f64; 3]>>,
    /// The initial-pose interpolation buffer (always present).
    initial_pose_buffer: Mutex<PoseBuffer>,
    /// `Some` iff `params.regularization_enable` (mirrors the C++ conditional buffer creation).
    regularization_buffer: Option<Mutex<PoseBuffer>>,
    /// Map-update decision state (Phase 6): last-update position + need-rebuild policy.
    map_update_state: Mutex<MapUpdateState>,
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
        let initial_pose_buffer = Mutex::new(PoseBuffer::new(
            params.initial_pose_timeout_sec,
            params.initial_pose_distance_tolerance_m,
        ));
        Self {
            params,
            is_activated: AtomicBool::new(false),
            latest_ekf_position: Mutex::new(None),
            initial_pose_buffer,
            regularization_buffer,
            map_update_state: Mutex::new(MapUpdateState::default()),
        }
    }

    /// Read the activation flag (the C++ `is_activated_`). Relaxed: each callback reads/writes it
    /// independently; no ordering relative to other state is relied upon.
    #[must_use]
    pub(crate) fn is_activated(&self) -> bool {
        self.is_activated.load(Ordering::Relaxed)
    }

    /// Set the activation flag (`service_trigger_node`). On activation, clear the initial-pose buffer
    /// so stale poses don't survive a re-activation (mirrors the C++ trigger).
    pub(crate) fn set_activated(&self, activate: bool) {
        self.is_activated.store(activate, Ordering::Relaxed);
        if activate && let Ok(mut b) = self.initial_pose_buffer.lock() {
            b.clear();
        }
    }

    /// Push an initial pose into the buffer + record its position as the latest EKF position
    /// (`on_initial_pose`'s accepted path). No-op on a poisoned lock.
    pub(crate) fn push_initial_pose(&self, pose: &TimedPoseWithCov) {
        if let Ok(mut b) = self.initial_pose_buffer.lock() {
            b.push_back(*pose);
        }
        if let Ok(mut p) = self.latest_ekf_position.lock() {
            *p = Some(pose.position);
        }
    }

    /// The latest EKF position, for the map-update timer (the C++ `latest_ekf_position_`).
    #[must_use]
    pub(crate) fn latest_ekf_position(&self) -> Option<[f64; 3]> {
        self.latest_ekf_position.lock().ok().and_then(|p| *p)
    }

    /// The stateful map-update decision (the C++ `should_update_map`). With no last-update position
    /// yet → force a rebuild and update. Otherwise run the pure distance verdict and, if the loaded
    /// map can no longer keep up (`out_of_keep_up`), latch `need_rebuild`. The C++ shell emits the
    /// diagnostics from the returned struct.
    #[must_use]
    pub(crate) fn map_update_evaluate(
        &self,
        cur: [f64; 2],
        lidar_radius: f64,
        map_radius: f64,
        update_distance: f64,
    ) -> AwMapUpdateDecision {
        let Ok(mut st) = self.map_update_state.lock() else {
            return AwMapUpdateDecision::default();
        };
        let Some(last) = st.last_update_position else {
            st.need_rebuild = true;
            return AwMapUpdateDecision {
                distance: 0.0,
                should_update: true,
                out_of_keep_up: false,
                need_rebuild: true,
                is_first_update: true,
            };
        };
        let verdict = evaluate_map_update(&MapUpdateInput {
            current_x: cur[0],
            current_y: cur[1],
            last_update_x: last[0],
            last_update_y: last[1],
            lidar_radius,
            map_radius,
            update_distance,
        });
        if verdict.out_of_keep_up {
            st.need_rebuild = true;
        }
        AwMapUpdateDecision {
            distance: verdict.distance,
            should_update: verdict.should_update,
            out_of_keep_up: verdict.out_of_keep_up,
            need_rebuild: st.need_rebuild,
            is_first_update: false,
        }
    }

    /// Whether the next map update must rebuild from scratch (the C++ `builder_state.need_rebuild`).
    #[must_use]
    pub(crate) fn map_update_need_rebuild(&self) -> bool {
        self.map_update_state.lock().is_ok_and(|st| st.need_rebuild)
    }

    /// Record an attempted map update at `pos` (the C++ `update_map_internal` tail / failure path):
    /// the last-update position always advances; `need_rebuild` clears only on success.
    pub(crate) fn map_update_record(&self, pos: [f64; 2], success: bool) {
        if let Ok(mut st) = self.map_update_state.lock() {
            st.last_update_position = Some(pos);
            if success {
                st.need_rebuild = false;
            }
        }
    }

    /// Whether `cur` is outside the loaded map's keep-up range (the C++ `out_of_map_range`); `true`
    /// when no update has happened yet.
    #[must_use]
    pub(crate) fn map_update_out_of_range(
        &self,
        cur: [f64; 2],
        lidar_radius: f64,
        map_radius: f64,
    ) -> bool {
        let Ok(st) = self.map_update_state.lock() else {
            return true;
        };
        let Some(last) = st.last_update_position else {
            return true;
        };
        evaluate_map_update(&MapUpdateInput {
            current_x: cur[0],
            current_y: cur[1],
            last_update_x: last[0],
            last_update_y: last[1],
            lidar_radius,
            map_radius,
            update_distance: 0.0,
        })
        .out_of_keep_up
    }

    /// Interpolate the initial pose at `target_ns`, then `pop_old` (mirrors the sensor callback).
    /// `None` on a poisoned lock or when the buffer cannot interpolate.
    pub(crate) fn interpolate_initial_pose(&self, target_ns: i64) -> Option<InterpolateResult> {
        let mut b = self.initial_pose_buffer.lock().ok()?;
        let result = b.interpolate(target_ns)?;
        b.pop_old(target_ns);
        Some(result)
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
    pub map_frame: *const u8,
    pub map_frame_len: usize,
    pub initial_pose_timeout_sec: f64,
    pub initial_pose_distance_tolerance_m: f64,
    pub base_frame: *const u8,
    pub base_frame_len: usize,
    pub sensor_points_timeout_sec: f64,
    pub sensor_points_required_distance: f64,
}

/// C-ABI view of a `geometry_msgs::PoseWithCovarianceStamped`, borrowed for the call only (Rust copies
/// what it keeps). `orientation` is `[x, y, z, w]`; `covariance` is the row-major 6x6. `frame_id`
/// crosses as `(ptr, len)` UTF-8 bytes (the initial-pose frame check; the regularization path ignores
/// it).
#[repr(C)]
pub struct AwPoseWithCovarianceStampedView {
    pub stamp_ns: i64,
    pub position: [f64; 3],
    pub orientation: [f64; 4],
    pub covariance: [f64; 36],
    pub frame_id: *const u8,
    pub frame_id_len: usize,
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

/// C-ABI out-struct for the initial-pose interpolation the sensor callback needs: the interpolated
/// pose (position + orientation + the row-major 6x6 covariance carried from the older bracket entry,
/// republished on `initial_pose_with_covariance`) plus the **positions** of the two bracket entries
/// (`publish_initial_to_result` reads only their positions).
/// C-ABI out-struct for the map-update decision (`..._map_update_evaluate`). `distance` is the move
/// since the last update; `should_update`/`out_of_keep_up`/`need_rebuild` drive the C++ diagnostics +
/// the rebuild flag; `is_first_update` marks the no-prior-update case.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AwMapUpdateDecision {
    pub distance: f64,
    pub should_update: bool,
    pub out_of_keep_up: bool,
    pub need_rebuild: bool,
    pub is_first_update: bool,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct AwInitialPoseInterpolation {
    pub interpolated_position: [f64; 3],
    pub interpolated_orientation: [f64; 4],
    pub interpolated_covariance: [f64; 36],
    pub old_position: [f64; 3],
    pub new_position: [f64; 3],
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

/// View `[ptr, ptr+len)` as a byte slice, treating a null/zero-length pointer as empty.
///
/// # Safety
/// When `len > 0`, `ptr` must point to `len` readable, initialized bytes that outlive the borrow.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointer validated per rust-c-ffi-safety"
)]
unsafe fn byte_slice<'a>(ptr: *const u8, len: usize) -> &'a [u8] {
    if ptr.is_null() || len == 0 {
        return &[];
    }
    // SAFETY: non-null with len > 0 per the check; caller guarantees `len` readable bytes.
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
        // SAFETY: caller guarantees `map_frame` is valid for `map_frame_len` (or null/0).
        let map_frame = unsafe { byte_slice(p.map_frame, p.map_frame_len) };
        // SAFETY: caller guarantees `base_frame` is valid for `base_frame_len` (or null/0).
        let base_frame = unsafe { byte_slice(p.base_frame, p.base_frame_len) };
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
            map_frame: map_frame.to_vec(),
            initial_pose_timeout_sec: p.initial_pose_timeout_sec,
            initial_pose_distance_tolerance_m: p.initial_pose_distance_tolerance_m,
            base_frame: base_frame.to_vec(),
            sensor_points_timeout_sec: p.sensor_points_timeout_sec,
            sensor_points_required_distance: p.sensor_points_required_distance,
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

/// Read the node activation flag from the handle (the C++ `is_activated_`). `false` if `handle` is
/// null. Transitional read for the still-C++ sensor/timer gates (removed in Phase 5).
///
/// # Safety
/// `handle` must be a valid, live `NdtScanMatcherRs` from `_new`, or null → `false`.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointer validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_is_activated(
    handle: *const NdtScanMatcherRs,
) -> bool {
    if handle.is_null() {
        return false;
    }
    // SAFETY: non-null per the check; caller guarantees a valid, live handle.
    unsafe { &*handle }.is_activated()
}

/// Write the latest EKF position `[x, y, z]` into `*out_xyz` and return `true`; `false` (leaving
/// `*out_xyz` untouched) if none is recorded yet or `handle`/`out_xyz` is null. Transitional read for
/// the still-C++ map-update timer (removed when map update moves to Rust, Phase 6).
///
/// # Safety
/// `handle` must be a valid, live `NdtScanMatcherRs` from `_new` (or null → `false`), and `out_xyz`
/// must point to 3 writable, aligned `f64` (or null → `false`).
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_latest_ekf_position(
    handle: *const NdtScanMatcherRs,
    out_xyz: *mut f64,
) -> bool {
    if handle.is_null() || out_xyz.is_null() {
        return false;
    }
    // SAFETY: non-null per the check; caller guarantees a valid, live handle.
    let Some(p) = unsafe { &*handle }.latest_ekf_position() else {
        return false;
    };
    // SAFETY: `out_xyz` points to 3 writable, aligned f64 per the contract.
    unsafe {
        core::slice::from_raw_parts_mut(out_xyz, 3).copy_from_slice(&p);
    }
    true
}

/// Interpolate the initial pose at `stamp_ns` from the handle's buffer (then `pop_old`), writing the
/// interpolated pose + the two bracket positions into `*out` and returning `true`; `false` (leaving
/// `*out` untouched) if the buffer cannot interpolate or a pointer is null (the C++ sensor callback's
/// `if (!opt) return false;`). Transitional: removed when the sensor callback moves to Rust (Phase 5).
///
/// # Safety
/// `handle` must be a valid, live `NdtScanMatcherRs` from `_new` (or null → `false`), and `out` a
/// valid, aligned, writable [`AwInitialPoseInterpolation`] (or null → `false`).
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_initial_pose_interpolate(
    handle: *const NdtScanMatcherRs,
    stamp_ns: i64,
    out: *mut AwInitialPoseInterpolation,
) -> bool {
    if handle.is_null() || out.is_null() {
        return false;
    }
    // SAFETY: non-null per the check; caller guarantees a valid, live handle.
    let h = unsafe { &*handle };
    let Some(result) = h.interpolate_initial_pose(stamp_ns) else {
        return false;
    };
    // SAFETY: `out` is non-null per the check and a valid, aligned, writable struct per the contract.
    unsafe {
        *out = AwInitialPoseInterpolation {
            interpolated_position: result.position,
            interpolated_orientation: result.orientation,
            interpolated_covariance: result.covariance,
            old_position: result.old.position,
            new_position: result.new_entry.position,
        };
    }
    true
}

/// The stateful map-update decision (the C++ `should_update_map`): writes the verdict into `*out` and
/// returns `true`; `false` (leaving `*out` untouched) only on a null pointer. Mutates the handle's
/// `need_rebuild` (first update / out-of-keep-up). Phase 6.
///
/// # Safety
/// `handle` must be a valid, live `NdtScanMatcherRs` from `_new` (or null → `false`); `out` a valid,
/// aligned, writable [`AwMapUpdateDecision`] (or null → `false`).
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_map_update_evaluate(
    handle: *const NdtScanMatcherRs,
    cur_x: f64,
    cur_y: f64,
    lidar_radius: f64,
    map_radius: f64,
    update_distance: f64,
    out: *mut AwMapUpdateDecision,
) -> bool {
    if handle.is_null() || out.is_null() {
        return false;
    }
    // SAFETY: non-null per the check; caller guarantees a valid, live handle.
    let decision = unsafe { &*handle }.map_update_evaluate(
        [cur_x, cur_y],
        lidar_radius,
        map_radius,
        update_distance,
    );
    // SAFETY: `out` is non-null per the check and a valid, aligned, writable struct per the contract.
    unsafe {
        *out = decision;
    }
    true
}

/// Whether the next map update must rebuild from scratch (the C++ `builder_state.need_rebuild`).
/// `false` if `handle` is null. Phase 6.
///
/// # Safety
/// `handle` must be a valid, live `NdtScanMatcherRs` from `_new`, or null → `false`.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointer validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_map_update_need_rebuild(
    handle: *const NdtScanMatcherRs,
) -> bool {
    if handle.is_null() {
        return false;
    }
    // SAFETY: non-null per the check; caller guarantees a valid, live handle.
    unsafe { &*handle }.map_update_need_rebuild()
}

/// Record an attempted map update at `(x, y)` (the C++ `update_map_internal`): the last-update
/// position always advances; `need_rebuild` clears only when `success`. No-op if `handle` is null.
/// Phase 6.
///
/// # Safety
/// `handle` must be a valid, live `NdtScanMatcherRs` from `_new`, or null → no-op.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointer validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_map_update_record(
    handle: *const NdtScanMatcherRs,
    x: f64,
    y: f64,
    success: bool,
) {
    if handle.is_null() {
        return;
    }
    // SAFETY: non-null per the check; caller guarantees a valid, live handle.
    unsafe { &*handle }.map_update_record([x, y], success);
}

/// Whether `(cur_x, cur_y)` is outside the loaded map's keep-up range (the C++ `out_of_map_range`);
/// `true` when no update has happened yet or `handle` is null. Phase 6.
///
/// # Safety
/// `handle` must be a valid, live `NdtScanMatcherRs` from `_new`, or null → `true`.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointer validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_map_update_out_of_range(
    handle: *const NdtScanMatcherRs,
    cur_x: f64,
    cur_y: f64,
    lidar_radius: f64,
    map_radius: f64,
) -> bool {
    if handle.is_null() {
        return true;
    }
    // SAFETY: non-null per the check; caller guarantees a valid, live handle.
    unsafe { &*handle }.map_update_out_of_range([cur_x, cur_y], lidar_radius, map_radius)
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
            map_frame: core::ptr::null(),
            map_frame_len: 0,
            initial_pose_timeout_sec: 1000.0,
            initial_pose_distance_tolerance_m: 1000.0,
            base_frame: core::ptr::null(),
            base_frame_len: 0,
            sensor_points_timeout_sec: 1.0,
            sensor_points_required_distance: 0.0,
        }
    }

    fn pose_view(stamp_ns: i64, x: f64, y: f64) -> AwPoseWithCovarianceStampedView {
        AwPoseWithCovarianceStampedView {
            stamp_ns,
            position: [x, y, 0.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
            covariance: [0.0; 36],
            frame_id: core::ptr::null(),
            frame_id_len: 0,
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
        // Node state starts empty (inactive, no EKF position yet).
        assert!(!m.is_activated());
        assert!(m.latest_ekf_position().is_none());
        assert!(m.map_update_state.lock().unwrap().last_update_position.is_none());
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
    fn map_update_first_then_record_then_in_range() {
        let ox: [f64; 0] = [];
        let oy: [f64; 0] = [];
        let p = sample_params(&ox, &oy);
        // SAFETY: valid params.
        let handle = unsafe { autoware_ndt_scan_matcher_rs_new(&raw const p) };
        // SAFETY: live handle.
        let h = unsafe { &*handle };
        // lidar_radius=50, map_radius=150, update_distance=20.
        // First evaluate: no last position → rebuild + should_update.
        let d0 = h.map_update_evaluate([0.0, 0.0], 50.0, 150.0, 20.0);
        assert!(d0.is_first_update);
        assert!(d0.should_update);
        assert!(d0.need_rebuild);
        assert!(h.map_update_need_rebuild());
        // Record a successful update at the origin → clears need_rebuild, sets last position.
        h.map_update_record([0.0, 0.0], true);
        assert!(!h.map_update_need_rebuild());
        // A small move (5 m < 20 m) → no update, still keeping up.
        let d1 = h.map_update_evaluate([3.0, 4.0], 50.0, 150.0, 20.0);
        assert!(!d1.is_first_update);
        assert!(!d1.should_update);
        assert!(!d1.out_of_keep_up);
        assert!((d1.distance - 5.0).abs() < 1e-9);
        assert!(!h.map_update_out_of_range([3.0, 4.0], 50.0, 150.0));
        // SAFETY: freed once.
        unsafe { autoware_ndt_scan_matcher_rs_free(handle) };
    }

    #[test]
    fn map_update_out_of_keep_up_latches_rebuild() {
        let ox: [f64; 0] = [];
        let oy: [f64; 0] = [];
        let p = sample_params(&ox, &oy);
        // SAFETY: valid params.
        let handle = unsafe { autoware_ndt_scan_matcher_rs_new(&raw const p) };
        // SAFETY: live handle.
        let h = unsafe { &*handle };
        h.map_update_record([0.0, 0.0], true); // last=origin, need_rebuild=false
        // distance 120; 120 + lidar(50) = 170 > map(150) → out_of_keep_up → latch rebuild; and
        // 120 > update_distance(20) → should_update.
        let d = h.map_update_evaluate([120.0, 0.0], 50.0, 150.0, 20.0);
        assert!(d.out_of_keep_up);
        assert!(d.should_update);
        assert!(d.need_rebuild);
        assert!(h.map_update_need_rebuild()); // latched
        assert!(h.map_update_out_of_range([120.0, 0.0], 50.0, 150.0));
        // A failed update advances the position but does NOT clear the rebuild latch.
        h.map_update_record([120.0, 0.0], false);
        assert!(h.map_update_need_rebuild());
        // SAFETY: freed once.
        unsafe { autoware_ndt_scan_matcher_rs_free(handle) };
    }

    #[test]
    fn map_update_out_of_range_true_before_first_update() {
        let ox: [f64; 0] = [];
        let oy: [f64; 0] = [];
        let p = sample_params(&ox, &oy);
        // SAFETY: valid params.
        let handle = unsafe { autoware_ndt_scan_matcher_rs_new(&raw const p) };
        // SAFETY: live handle.
        let h = unsafe { &*handle };
        assert!(h.map_update_out_of_range([0.0, 0.0], 50.0, 150.0)); // no last position yet
        // SAFETY: freed once.
        unsafe { autoware_ndt_scan_matcher_rs_free(handle) };
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
