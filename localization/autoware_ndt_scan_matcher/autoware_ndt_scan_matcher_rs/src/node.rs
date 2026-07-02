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

//! Phase N0 — node callback logic in Rust (proof of pattern). The C++ rclcpp shell owns the node
//! state + ROS I/O and exposes them to Rust via a **host interface**: a `#[repr(C)]` vtable of C
//! function pointers + an opaque context (the `NDTScanMatcher*`). A migrated callback's body lives
//! here and drives the node purely through that vtable. ROS-node glue only (not the `no_std` engine
//! path) — this module is `std`-gated so the `no_std` rlib excludes it.

use core::ffi::c_void;

// The pure convergence decision now lives in the no_std `convergence` module (so the portable
// `scan_matcher` can reuse it); this module keeps the `Aw*` C-ABI mirrors + the `extern "C"` shim.
use crate::convergence::{ConvergenceInput, evaluate_convergence};
use crate::node_handle::{AwPoseWithCovarianceStampedView, NdtScanMatcherRs};

/// A node callback's `DiagnosticsInterface` (the `/diagnostics` status it builds + publishes), as a
/// C-ABI vtable over an opaque handle (the `DiagnosticsInterface*`). Built C++-side via
/// `make_diagnostics`; lets a Rust-owned callback emit the exact same diagnostics the C++ body did
/// (key order + values preserved). Keys/messages cross as `(ptr, len)` (UTF-8, not NUL-terminated).
/// Field order must match the C `AwDiagnostics` struct. The full vtable mirrors the C++
/// `DiagnosticsInterface` (all 7 ops); the safe wrapper methods below are the diagnostics surface for
/// every migrated callback — `on_trigger` (slice 1) uses 4; the rest (`add_f64`/`add_str`/
/// `update_level`) are the public API later callback slices build on.
#[repr(C)]
pub struct Diagnostics {
    diag: *mut c_void,
    /// `clear()` — reset the status to OK with no key-values.
    clear: extern "C" fn(*mut c_void),
    /// `add_key_value(key, bool)`.
    add_key_value_bool: extern "C" fn(*mut c_void, *const u8, usize, bool),
    /// `add_key_value(key, int64)`.
    add_key_value_i64: extern "C" fn(*mut c_void, *const u8, usize, i64),
    /// `add_key_value(key, double)`.
    add_key_value_f64: extern "C" fn(*mut c_void, *const u8, usize, f64),
    /// `add_key_value(key, std::string)`.
    add_key_value_str: extern "C" fn(*mut c_void, *const u8, usize, *const u8, usize),
    /// `update_level_and_message(level, msg)` (`level` = `DiagnosticStatus` byte: 1 WARN, 2 ERROR).
    update_level_and_message: extern "C" fn(*mut c_void, i8, *const u8, usize),
    /// `publish(rclcpp::Time(stamp_nanoseconds))`.
    publish: extern "C" fn(*mut c_void, i64),
}

/// `DiagnosticStatus` severity bytes (mirrors `diagnostic_msgs::msg::DiagnosticStatus`).
pub const DIAGNOSTIC_WARN: i8 = 1;
pub const DIAGNOSTIC_ERROR: i8 = 2;

impl Diagnostics {
    /// `clear()`. (Methods just forward to the vtable fn-pointers, which are safe to call; the raw
    /// `diag` handle is dereferenced only inside the C++ trampolines.)
    pub fn reset(&self) {
        (self.clear)(self.diag);
    }
    pub fn add_bool(&self, key: &str, v: bool) {
        (self.add_key_value_bool)(self.diag, key.as_ptr(), key.len(), v);
    }
    pub fn add_i64(&self, key: &str, v: i64) {
        (self.add_key_value_i64)(self.diag, key.as_ptr(), key.len(), v);
    }
    pub fn add_f64(&self, key: &str, v: f64) {
        (self.add_key_value_f64)(self.diag, key.as_ptr(), key.len(), v);
    }
    pub fn add_str(&self, key: &str, v: &str) {
        (self.add_key_value_str)(self.diag, key.as_ptr(), key.len(), v.as_ptr(), v.len());
    }
    pub fn update_level(&self, level: i8, msg: &str) {
        (self.update_level_and_message)(self.diag, level, msg.as_ptr(), msg.len());
    }
    pub fn publish_at(&self, stamp_ns: i64) {
        (self.publish)(self.diag, stamp_ns);
    }
}

/// The whole body of `service_trigger_node` (callback-level): build the diagnostics (clear +
/// `service_call_time_stamp`), set the activation flag on the handle (and on enable clear the
/// initial-pose buffer so stale poses don't survive a re-activation — done inside
/// [`NdtScanMatcherRs::set_activated`]), emit the `is_activated` / `is_succeed_service` key-values, and
/// publish. Returns `res->success` (the C++ wrapper just assigns it). `now_ns` is
/// `this->now().nanoseconds()` (the call timestamp + publish stamp). Mirrors the C++ key order/values.
///
/// # Safety
/// `handle` is a valid, live `NdtScanMatcherRs` from `_new` (or null → returns `false`); `diag` is a
/// valid diagnostics vtable (or null → `false`) whose fn pointers + handle outlive the call.
#[expect(
    unsafe_code,
    reason = "C ABI host-interface boundary; validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_node_on_trigger(
    handle: *const NdtScanMatcherRs,
    diag: *const Diagnostics,
    activate: bool,
    now_ns: i64,
) -> bool {
    if handle.is_null() || diag.is_null() {
        return false;
    }
    // SAFETY: non-null per the check; caller guarantees valid, live handle + diagnostics handle.
    let (h, d) = unsafe { (&*handle, &*diag) };
    d.reset();
    d.add_i64("service_call_time_stamp", now_ns);
    h.set_activated(activate);
    d.add_bool("is_activated", activate);
    d.add_bool("is_succeed_service", true);
    d.publish_at(now_ns);
    true
}

/// Outcome of [`autoware_ndt_scan_matcher_rs_node_on_initial_pose`] (a summary code for tests; the
/// callback also emits the corresponding diagnostics itself). `i32` so it crosses the C ABI plainly.
pub const INITIAL_POSE_ACCEPTED: i32 = 0;
/// The node is not activated — the pose is dropped (emits a WARN).
pub const INITIAL_POSE_NOT_ACTIVATED: i32 = 1;
/// The message `frame_id` did not match the configured map frame (emits an ERROR).
pub const INITIAL_POSE_WRONG_FRAME: i32 = 2;

/// The whole body of `callback_initial_pose` (callback-level): build the diagnostics (clear +
/// `topic_time_stamp`), gate on activation then on the message frame matching the map frame (emitting
/// `is_activated` / `is_expected_frame_id` + a WARN/ERROR on failure), on acceptance push the pose into
/// the handle's Rust-owned initial-pose buffer and record its position as the latest EKF position, and
/// always publish. Mirrors the C++ body's key order/values + the exact WARN/ERROR messages. Returns the
/// `INITIAL_POSE_*` summary code (the C++ wrapper ignores it; it keeps the existing tests precise).
/// `view.stamp_ns` is the message header stamp (the `topic_time_stamp` + publish stamp).
///
/// # Safety
/// `handle` is a valid, live `NdtScanMatcherRs` from `_new` (or null → returns `NOT_ACTIVATED`); `diag`
/// is a valid diagnostics vtable (or null → `NOT_ACTIVATED`); `view` is a valid, aligned
/// [`AwPoseWithCovarianceStampedView`] (or null → `NOT_ACTIVATED`) whose `frame_id` addresses
/// `frame_id_len` readable bytes (or null/0). All read once.
#[expect(
    unsafe_code,
    reason = "C ABI host-interface boundary; validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_node_on_initial_pose(
    handle: *const NdtScanMatcherRs,
    diag: *const Diagnostics,
    view: *const AwPoseWithCovarianceStampedView,
) -> i32 {
    if handle.is_null() || diag.is_null() || view.is_null() {
        return INITIAL_POSE_NOT_ACTIVATED;
    }
    // SAFETY: non-null per the check; caller guarantees valid, live handle + diagnostics + view.
    let (h, d, v) = unsafe { (&*handle, &*diag, &*view) };
    d.reset();
    d.add_i64("topic_time_stamp", v.stamp_ns);

    // check is_activated
    let is_activated = h.is_activated();
    d.add_bool("is_activated", is_activated);
    if !is_activated {
        d.update_level(DIAGNOSTIC_WARN, "Node is not activated.");
        d.publish_at(v.stamp_ns);
        return INITIAL_POSE_NOT_ACTIVATED;
    }

    // check is_expected_frame_id
    // SAFETY: caller guarantees `view.frame_id` is readable for `view.frame_id_len` (or null/0).
    let frame = unsafe { str_bytes(v.frame_id, v.frame_id_len) };
    let map = h.params.map_frame.as_slice();
    let is_expected_frame_id = frame == map;
    d.add_bool("is_expected_frame_id", is_expected_frame_id);
    if !is_expected_frame_id {
        let frame_str = alloc::string::String::from_utf8_lossy(frame);
        let map_str = alloc::string::String::from_utf8_lossy(map);
        let message = alloc::format!(
            "Received initial pose message with frame_id {frame_str}, but expected {map_str}. \
             Please check the frame_id in the input topic and ensure it is correct."
        );
        d.update_level(DIAGNOSTIC_ERROR, &message);
        d.publish_at(v.stamp_ns);
        return INITIAL_POSE_WRONG_FRAME;
    }

    // push into the buffer + record the latest EKF position (both on the handle).
    h.push_initial_pose(&v.to_timed());
    d.publish_at(v.stamp_ns);
    INITIAL_POSE_ACCEPTED
}

/// The whole body of `callback_regularization_pose` (callback-level): build the diagnostics (clear +
/// `topic_time_stamp`), push the pose into the **Rust-owned** regularization buffer on the handle, and
/// publish. No-op if any pointer is null (or regularization is disabled — the handle push is a no-op).
/// The stamp + pose come from `view` (the buffer push replaces the old host `push_regularization_pose`
/// vtable call; Phase 1 slice A).
///
/// # Safety
/// `handle` is a valid, live `NdtScanMatcherRs` from `_new` (or null → no-op); `diag` is a valid
/// diagnostics vtable (or null → no-op) whose fn pointers + handle outlive the call; `view` is a valid,
/// aligned [`AwPoseWithCovarianceStampedView`] (or null → no-op), read once.
#[expect(
    unsafe_code,
    reason = "C ABI host-interface boundary; validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_node_on_regularization_pose(
    handle: *const NdtScanMatcherRs,
    diag: *const Diagnostics,
    view: *const AwPoseWithCovarianceStampedView,
) {
    if handle.is_null() || diag.is_null() || view.is_null() {
        return;
    }
    // SAFETY: non-null per the check; caller guarantees valid, live handle + diagnostics + view.
    let (h, d, v) = unsafe { (&*handle, &*diag, &*view) };
    d.reset();
    d.add_i64("topic_time_stamp", v.stamp_ns);
    h.push_regularization(&v.to_timed());
    d.publish_at(v.stamp_ns);
}

/// View `[ptr, ptr+len)` as a byte slice, treating a null/zero-length pointer as empty (so the
/// frame-id comparison never builds a slice from a null base).
///
/// # Safety
/// When `len > 0`, `ptr` must point to `len` readable, initialized bytes that outlive the borrow.
#[expect(
    unsafe_code,
    reason = "C ABI host-interface boundary; validated per rust-c-ffi-safety"
)]
unsafe fn str_bytes<'a>(ptr: *const u8, len: usize) -> &'a [u8] {
    if ptr.is_null() || len == 0 {
        return &[];
    }
    // SAFETY: non-null with len > 0 per the check; caller guarantees `len` readable bytes.
    unsafe { core::slice::from_raw_parts(ptr, len) }
}

/// C ABI mirror of [`ConvergenceInput`] (same field order/types). Plain scalars — no pointers.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct AwConvergenceInput {
    pub iteration_num: i32,
    pub max_iterations: i32,
    pub oscillation_num: i32,
    pub transform_probability: f64,
    pub nearest_voxel_transformation_likelihood: f64,
    pub converged_param_type: i32,
    pub converged_param_transform_probability: f64,
    pub converged_param_nearest_voxel_transformation_likelihood: f64,
}

/// C ABI mirror of [`crate::convergence::ConvergenceVerdict`] (same field order). `bool` is a 1-byte,
/// C-ABI-stable type.
/// (`#[repr(C)]` exempts this from `struct_excessive_bools` — the FFI layout is fixed by the ABI.)
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AwConvergenceVerdict {
    pub valid_param_type: bool,
    pub is_ok_iteration_num: bool,
    pub is_local_optimal_solution_oscillation: bool,
    pub is_ok_score: bool,
    pub is_converged: bool,
    pub score: f64,
    pub score_threshold: f64,
}

/// FFI entry for the NDT convergence decision: reads `*input`, runs [`evaluate_convergence`], writes
/// `*out`. No-op if either pointer is null.
///
/// # Safety
/// `input` must point to a valid, aligned [`AwConvergenceInput`] and `out` to a valid, aligned,
/// writable [`AwConvergenceVerdict`] (or either may be null → no-op). Both are read/written once.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_node_evaluate_convergence(
    input: *const AwConvergenceInput,
    out: *mut AwConvergenceVerdict,
) {
    if input.is_null() || out.is_null() {
        return;
    }
    // SAFETY: non-null per the check; caller guarantees a valid, aligned struct (read once).
    let i = unsafe { &*input };
    let verdict = evaluate_convergence(&ConvergenceInput {
        iteration_num: i.iteration_num,
        max_iterations: i.max_iterations,
        oscillation_num: i.oscillation_num,
        transform_probability: i.transform_probability,
        nearest_voxel_transformation_likelihood: i.nearest_voxel_transformation_likelihood,
        converged_param_type: i.converged_param_type,
        converged_param_transform_probability: i.converged_param_transform_probability,
        converged_param_nearest_voxel_transformation_likelihood: i
            .converged_param_nearest_voxel_transformation_likelihood,
    });
    // SAFETY: `out` is non-null per the check and a valid, aligned, writable verdict per the contract.
    unsafe {
        *out = AwConvergenceVerdict {
            valid_param_type: verdict.valid_param_type,
            is_ok_iteration_num: verdict.is_ok_iteration_num,
            is_local_optimal_solution_oscillation: verdict.is_local_optimal_solution_oscillation,
            is_ok_score: verdict.is_ok_score,
            is_converged: verdict.is_converged,
            score: verdict.score,
            score_threshold: verdict.score_threshold,
        };
    }
}

/// Inputs to the dynamic-map-update distance decision: the current position, the position at the
/// last map update, and the `dynamic_map_loading` radii/thresholds. The "no last update yet" case is
/// handled C++-side (it is trivial control flow), so this fn always has a real last position.
#[derive(Clone, Copy, Debug)]
pub struct MapUpdateInput {
    pub current_x: f64,
    pub current_y: f64,
    pub last_update_x: f64,
    pub last_update_y: f64,
    /// `dynamic_map_loading.lidar_radius`.
    pub lidar_radius: f64,
    /// `dynamic_map_loading.map_radius`.
    pub map_radius: f64,
    /// `dynamic_map_loading.update_distance`.
    pub update_distance: f64,
}

/// The map-update decision: how far we have moved since the last update, whether the loaded map can
/// still keep up, and whether a (incremental) update should be triggered.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MapUpdateVerdict {
    /// `hypot(dx, dy)` from the last update position.
    pub distance: f64,
    /// `distance + lidar_radius > map_radius`: the loaded map no longer covers the lidar range
    /// (drives both `out_of_map_range` and `should_update_map`'s critical-rebuild path).
    pub out_of_keep_up: bool,
    /// `distance > update_distance`: far enough to trigger a map update.
    pub should_update: bool,
}

/// Pure dynamic-map-update distance decision, ported verbatim from `MapUpdateModule`'s
/// `should_update_map` / `out_of_map_range`. `f64::hypot` resolves to libm `hypot`, matching the C++
/// `std::hypot`. No allocation; O(1) — RT-clean.
#[must_use]
pub fn evaluate_map_update(input: &MapUpdateInput) -> MapUpdateVerdict {
    // Primitive-f64 arithmetic (not nalgebra operators), so `arithmetic_side_effects` does not fire.
    let dx = input.current_x - input.last_update_x;
    let dy = input.current_y - input.last_update_y;
    let distance = dx.hypot(dy);
    MapUpdateVerdict {
        distance,
        out_of_keep_up: distance + input.lidar_radius > input.map_radius,
        should_update: distance > input.update_distance,
    }
}

/// C ABI mirror of [`MapUpdateInput`] (same field order/types). Plain scalars — no pointers.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct AwMapUpdateInput {
    pub current_x: f64,
    pub current_y: f64,
    pub last_update_x: f64,
    pub last_update_y: f64,
    pub lidar_radius: f64,
    pub map_radius: f64,
    pub update_distance: f64,
}

/// C ABI mirror of [`MapUpdateVerdict`] (same field order). `bool` is a 1-byte, C-ABI-stable type.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AwMapUpdateVerdict {
    pub distance: f64,
    pub out_of_keep_up: bool,
    pub should_update: bool,
}

/// FFI entry for the dynamic-map-update decision: reads `*input`, runs [`evaluate_map_update`],
/// writes `*out`. No-op if either pointer is null.
///
/// # Safety
/// `input` must point to a valid, aligned [`AwMapUpdateInput`] and `out` to a valid, aligned,
/// writable [`AwMapUpdateVerdict`] (or either may be null → no-op). Read/written once.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_node_evaluate_map_update(
    input: *const AwMapUpdateInput,
    out: *mut AwMapUpdateVerdict,
) {
    if input.is_null() || out.is_null() {
        return;
    }
    // SAFETY: non-null per the check; caller guarantees a valid, aligned struct (read once).
    let i = unsafe { &*input };
    let verdict = evaluate_map_update(&MapUpdateInput {
        current_x: i.current_x,
        current_y: i.current_y,
        last_update_x: i.last_update_x,
        last_update_y: i.last_update_y,
        lidar_radius: i.lidar_radius,
        map_radius: i.map_radius,
        update_distance: i.update_distance,
    });
    // SAFETY: `out` is non-null per the check and a valid, aligned, writable verdict per the contract.
    unsafe {
        *out = AwMapUpdateVerdict {
            distance: verdict.distance,
            out_of_keep_up: verdict.out_of_keep_up,
            should_update: verdict.should_update,
        };
    }
}

#[cfg(test)]
#[allow(
    unsafe_code,
    dead_code,
    clippy::borrow_as_ptr,
    clippy::float_cmp,
    clippy::arithmetic_side_effects,
    clippy::allow_attributes,
    reason = "test code: mock-host C ABI calls; the diagnostics mock provides all 7 vtable ops (some unused per test); exact-equal float asserts on deterministic passthrough; counter increments"
)]
mod tests {
    use super::*;

    // ConvergedParamType discriminants (mirrors hyper_parameters.hpp).
    const TP: i32 = 0;
    const NVTL: i32 = 1;

    // A baseline that converges via the TP score (5 < 30 iterations, no oscillation, 3.0 > 2.0).
    // Tests tweak individual fields with struct-update syntax.
    fn base() -> ConvergenceInput {
        ConvergenceInput {
            iteration_num: 5,
            max_iterations: 30,
            oscillation_num: 0,
            transform_probability: 3.0,
            nearest_voxel_transformation_likelihood: 5.0,
            converged_param_type: TP,
            converged_param_transform_probability: 2.0,
            converged_param_nearest_voxel_transformation_likelihood: 4.0,
        }
    }

    #[test]
    fn ffi_evaluate_convergence_matches_pure() {
        let cases = [
            base(),
            ConvergenceInput {
                iteration_num: 30,
                oscillation_num: 11,
                converged_param_type: NVTL,
                ..base()
            },
            ConvergenceInput {
                converged_param_type: 2,
                ..base()
            }, // invalid type
        ];
        for c in cases {
            let pure = evaluate_convergence(&c);
            let inp = AwConvergenceInput {
                iteration_num: c.iteration_num,
                max_iterations: c.max_iterations,
                oscillation_num: c.oscillation_num,
                transform_probability: c.transform_probability,
                nearest_voxel_transformation_likelihood: c.nearest_voxel_transformation_likelihood,
                converged_param_type: c.converged_param_type,
                converged_param_transform_probability: c.converged_param_transform_probability,
                converged_param_nearest_voxel_transformation_likelihood: c
                    .converged_param_nearest_voxel_transformation_likelihood,
            };
            let mut out = AwConvergenceVerdict::default();
            // SAFETY: `inp` is valid; `out` is a valid, writable verdict.
            unsafe { autoware_ndt_scan_matcher_rs_node_evaluate_convergence(&inp, &mut out) };
            assert_eq!(out.valid_param_type, pure.valid_param_type);
            assert_eq!(out.is_ok_iteration_num, pure.is_ok_iteration_num);
            assert_eq!(
                out.is_local_optimal_solution_oscillation,
                pure.is_local_optimal_solution_oscillation
            );
            assert_eq!(out.is_ok_score, pure.is_ok_score);
            assert_eq!(out.is_converged, pure.is_converged);
            assert_eq!(out.score, pure.score);
            assert_eq!(out.score_threshold, pure.score_threshold);
        }
    }

    #[test]
    fn ffi_evaluate_convergence_null_is_noop() {
        let mut out = AwConvergenceVerdict {
            is_converged: true,
            ..AwConvergenceVerdict::default()
        };
        // SAFETY: a null input must leave `out` untouched.
        unsafe {
            autoware_ndt_scan_matcher_rs_node_evaluate_convergence(core::ptr::null(), &mut out);
        }
        assert!(out.is_converged);
        // A null output is a no-op (no write, no panic).
        let inp = base();
        let aw = AwConvergenceInput {
            iteration_num: inp.iteration_num,
            max_iterations: inp.max_iterations,
            oscillation_num: inp.oscillation_num,
            transform_probability: inp.transform_probability,
            nearest_voxel_transformation_likelihood: inp.nearest_voxel_transformation_likelihood,
            converged_param_type: inp.converged_param_type,
            converged_param_transform_probability: inp.converged_param_transform_probability,
            converged_param_nearest_voxel_transformation_likelihood: inp
                .converged_param_nearest_voxel_transformation_likelihood,
        };
        // SAFETY: null output pointer is explicitly handled.
        unsafe {
            autoware_ndt_scan_matcher_rs_node_evaluate_convergence(&aw, core::ptr::null_mut());
        }
    }

    // A live node handle for the callback tests: `map_frame` is the expected initial-pose frame; the
    // initial-pose + regularization buffers use the C++ node's 1000/1000 tolerances. The `map_frame`
    // slice must outlive the `_new` call (Rust copies it).
    fn make_handle(map_frame: &[u8]) -> *mut NdtScanMatcherRs {
        let p = crate::node_handle::AwNdtParams {
            resolution: 1.0,
            min_points: 6,
            eig_mult: 0.01,
            trans_epsilon: 0.0,
            step_size: 0.0,
            max_iterations: 1,
            outlier_ratio: 0.55,
            num_threads: 1,
            converged_param_type: 0,
            converged_param_transform_probability: 0.0,
            converged_param_nearest_voxel_transformation_likelihood: 0.0,
            covariance_estimation_type: 0,
            covariance_scale_factor: 1.0,
            covariance_temperature: 1.0,
            output_pose_covariance: [0.0; 36],
            initial_pose_offset_model_x: core::ptr::null(),
            initial_pose_offset_model_x_len: 0,
            initial_pose_offset_model_y: core::ptr::null(),
            initial_pose_offset_model_y_len: 0,
            regularization_enable: true,
            regularization_pose_timeout_sec: 1000.0,
            regularization_pose_distance_tolerance_m: 1000.0,
            map_frame: map_frame.as_ptr(),
            map_frame_len: map_frame.len(),
            initial_pose_timeout_sec: 1000.0,
            initial_pose_distance_tolerance_m: 1000.0,
            base_frame: core::ptr::null(),
            base_frame_len: 0,
            sensor_points_timeout_sec: 1.0,
            sensor_points_required_distance: 0.0,
        };
        // SAFETY: valid params; `map_frame` is valid for its length for the duration of the call.
        unsafe { crate::node_handle::autoware_ndt_scan_matcher_rs_new(&raw const p) }
    }

    // Build an initial/regularization pose view with the given frame + position.
    fn make_view(stamp_ns: i64, frame_id: &[u8], pos: [f64; 3]) -> AwPoseWithCovarianceStampedView {
        AwPoseWithCovarianceStampedView {
            stamp_ns,
            position: pos,
            orientation: [0.0, 0.0, 0.0, 1.0],
            covariance: [0.0; 36],
            frame_id: frame_id.as_ptr(),
            frame_id_len: frame_id.len(),
        }
    }

    // Mock diagnostics: each vtable op appends a human-readable event so a callback's full diagnostics
    // sequence (order + keys + values) can be asserted. `diag` points at a per-test `DiagRecorder`.
    #[derive(Default)]
    struct DiagRecorder {
        events: alloc::vec::Vec<alloc::string::String>,
    }
    fn drec<'a>(diag: *mut c_void) -> &'a mut DiagRecorder {
        // SAFETY: in these tests `diag` always points to a live `DiagRecorder` (set via `diagnostics`).
        unsafe { &mut *diag.cast::<DiagRecorder>() }
    }
    fn key_of(ptr: *const u8, len: usize) -> alloc::string::String {
        // SAFETY: the mock is always called with a valid (ptr, len) UTF-8 key/message from Rust.
        let bytes = unsafe { core::slice::from_raw_parts(ptr, len) };
        alloc::string::String::from_utf8_lossy(bytes).into_owned()
    }
    extern "C" fn t_diag_clear(diag: *mut c_void) {
        drec(diag).events.push("clear".into());
    }
    extern "C" fn t_diag_add_bool(diag: *mut c_void, k: *const u8, klen: usize, v: bool) {
        drec(diag)
            .events
            .push(format!("bool {}={}", key_of(k, klen), v));
    }
    extern "C" fn t_diag_add_i64(diag: *mut c_void, k: *const u8, klen: usize, v: i64) {
        drec(diag)
            .events
            .push(format!("i64 {}={}", key_of(k, klen), v));
    }
    extern "C" fn t_diag_add_f64(diag: *mut c_void, k: *const u8, klen: usize, v: f64) {
        drec(diag)
            .events
            .push(format!("f64 {}={}", key_of(k, klen), v));
    }
    extern "C" fn t_diag_add_str(
        diag: *mut c_void,
        k: *const u8,
        klen: usize,
        v: *const u8,
        vlen: usize,
    ) {
        drec(diag)
            .events
            .push(format!("str {}={}", key_of(k, klen), key_of(v, vlen)));
    }
    extern "C" fn t_diag_update_level(diag: *mut c_void, level: i8, msg: *const u8, mlen: usize) {
        drec(diag)
            .events
            .push(format!("level {} {}", level, key_of(msg, mlen)));
    }
    extern "C" fn t_diag_publish(diag: *mut c_void, stamp_ns: i64) {
        drec(diag).events.push(format!("publish {stamp_ns}"));
    }
    fn diagnostics(d: &mut DiagRecorder) -> Diagnostics {
        Diagnostics {
            diag: core::ptr::from_mut(d).cast::<c_void>(),
            clear: t_diag_clear,
            add_key_value_bool: t_diag_add_bool,
            add_key_value_i64: t_diag_add_i64,
            add_key_value_f64: t_diag_add_f64,
            add_key_value_str: t_diag_add_str,
            update_level_and_message: t_diag_update_level,
            publish: t_diag_publish,
        }
    }

    #[test]
    fn on_trigger_runs_full_body_and_emits_diagnostics() {
        let mut dr = DiagRecorder::default();
        let d = diagnostics(&mut dr);
        let handle = make_handle(b"map");
        assert!(!handle.is_null());
        // SAFETY: live handle from `_new`.
        let h = unsafe { &*handle };

        // activate: flag set true, full diagnostics sequence in order.
        // SAFETY: `handle`/`d` are valid for the call.
        let ok = unsafe { autoware_ndt_scan_matcher_rs_node_on_trigger(handle, &d, true, 123) };
        assert!(ok);
        assert!(h.is_activated());
        assert_eq!(
            dr.events,
            alloc::vec![
                "clear".to_string(),
                "i64 service_call_time_stamp=123".to_string(),
                "bool is_activated=true".to_string(),
                "bool is_succeed_service=true".to_string(),
                "publish 123".to_string(),
            ]
        );

        // deactivate: flag false; same diagnostics shape with is_activated=false.
        dr.events.clear();
        // SAFETY: as above.
        let ok2 = unsafe { autoware_ndt_scan_matcher_rs_node_on_trigger(handle, &d, false, 456) };
        assert!(ok2);
        assert!(!h.is_activated());
        assert_eq!(
            dr.events,
            alloc::vec![
                "clear".to_string(),
                "i64 service_call_time_stamp=456".to_string(),
                "bool is_activated=false".to_string(),
                "bool is_succeed_service=true".to_string(),
                "publish 456".to_string(),
            ]
        );

        // null handle or null diagnostics → returns false, no effect.
        // SAFETY: passing null is explicitly handled.
        assert!(!unsafe {
            autoware_ndt_scan_matcher_rs_node_on_trigger(core::ptr::null(), &d, true, 0)
        });
        // SAFETY: as above.
        assert!(!unsafe {
            autoware_ndt_scan_matcher_rs_node_on_trigger(handle, core::ptr::null(), true, 0)
        });
        // SAFETY: freed once.
        unsafe { crate::node_handle::autoware_ndt_scan_matcher_rs_free(handle) };
    }

    #[test]
    fn trigger_activation_clears_initial_pose_buffer() {
        let handle = make_handle(b"map");
        // SAFETY: live handle.
        let h = unsafe { &*handle };
        h.set_activated(true);
        // Two poses → interpolatable.
        h.push_initial_pose(&crate::pose_buffer::TimedPoseWithCov {
            stamp_ns: 1,
            position: [0.0; 3],
            orientation: [0.0, 0.0, 0.0, 1.0],
            covariance: [0.0; 36],
        });
        h.push_initial_pose(&crate::pose_buffer::TimedPoseWithCov {
            stamp_ns: 2_000_000_001,
            position: [1.0, 0.0, 0.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
            covariance: [0.0; 36],
        });
        assert!(h.interpolate_initial_pose(1_000_000_001).is_some());
        // Re-activation clears the buffer → no longer interpolatable.
        h.set_activated(true);
        assert!(h.interpolate_initial_pose(1_000_000_001).is_none());
        // SAFETY: freed once.
        unsafe { crate::node_handle::autoware_ndt_scan_matcher_rs_free(handle) };
    }

    // Drive on_initial_pose with the given frame/position at a fixed stamp (100).
    fn call_initial(handle: *const NdtScanMatcherRs, d: &Diagnostics, frame_id: &[u8], pos: [f64; 3]) -> i32 {
        let v = make_view(100, frame_id, pos);
        // SAFETY: `handle`/`d` valid; `v` is a valid view valid for the call.
        unsafe { autoware_ndt_scan_matcher_rs_node_on_initial_pose(handle, d, &raw const v) }
    }

    #[test]
    fn initial_pose_rejected_when_not_activated() {
        let mut dr = DiagRecorder::default();
        let d = diagnostics(&mut dr);
        let handle = make_handle(b"map"); // not activated
        // SAFETY: live handle.
        let h = unsafe { &*handle };
        let status = call_initial(handle, &d, b"map", [1.0, 2.0, 3.0]);
        assert_eq!(status, INITIAL_POSE_NOT_ACTIVATED);
        assert!(h.latest_ekf_position().is_none());
        assert_eq!(
            dr.events,
            alloc::vec![
                "clear".to_string(),
                "i64 topic_time_stamp=100".to_string(),
                "bool is_activated=false".to_string(),
                "level 1 Node is not activated.".to_string(),
                "publish 100".to_string(),
            ]
        );
        // SAFETY: freed once.
        unsafe { crate::node_handle::autoware_ndt_scan_matcher_rs_free(handle) };
    }

    #[test]
    fn initial_pose_rejected_when_frame_mismatch() {
        let mut dr = DiagRecorder::default();
        let d = diagnostics(&mut dr);
        let handle = make_handle(b"map");
        // SAFETY: live handle.
        let h = unsafe { &*handle };
        h.set_activated(true);
        let status = call_initial(handle, &d, b"lidar", [1.0, 2.0, 3.0]);
        assert_eq!(status, INITIAL_POSE_WRONG_FRAME);
        assert!(h.latest_ekf_position().is_none());
        assert_eq!(
            dr.events,
            alloc::vec![
                "clear".to_string(),
                "i64 topic_time_stamp=100".to_string(),
                "bool is_activated=true".to_string(),
                "bool is_expected_frame_id=false".to_string(),
                "level 2 Received initial pose message with frame_id lidar, but expected map. \
                 Please check the frame_id in the input topic and ensure it is correct."
                    .to_string(),
                "publish 100".to_string(),
            ]
        );
        // SAFETY: freed once.
        unsafe { crate::node_handle::autoware_ndt_scan_matcher_rs_free(handle) };
    }

    #[test]
    fn initial_pose_accepted_pushes_and_sets_position() {
        let mut dr = DiagRecorder::default();
        let d = diagnostics(&mut dr);
        let handle = make_handle(b"map");
        // SAFETY: live handle.
        let h = unsafe { &*handle };
        h.set_activated(true);
        let status = call_initial(handle, &d, b"map", [1.5, -2.5, 0.25]);
        assert_eq!(status, INITIAL_POSE_ACCEPTED);
        assert_eq!(h.latest_ekf_position(), Some([1.5, -2.5, 0.25]));
        assert_eq!(
            dr.events,
            alloc::vec![
                "clear".to_string(),
                "i64 topic_time_stamp=100".to_string(),
                "bool is_activated=true".to_string(),
                "bool is_expected_frame_id=true".to_string(),
                "publish 100".to_string(),
            ]
        );
        // SAFETY: freed once.
        unsafe { crate::node_handle::autoware_ndt_scan_matcher_rs_free(handle) };
    }

    #[test]
    fn initial_pose_accepts_empty_matching_frames() {
        // Degenerate frame ids: both empty -> equal -> accepted (exercises the zero-length path).
        let mut dr = DiagRecorder::default();
        let d = diagnostics(&mut dr);
        let handle = make_handle(b""); // empty map_frame
        // SAFETY: live handle.
        let h = unsafe { &*handle };
        h.set_activated(true);
        let status = call_initial(handle, &d, b"", [0.0, 0.0, 0.0]);
        assert_eq!(status, INITIAL_POSE_ACCEPTED);
        assert_eq!(h.latest_ekf_position(), Some([0.0, 0.0, 0.0]));
        // SAFETY: freed once.
        unsafe { crate::node_handle::autoware_ndt_scan_matcher_rs_free(handle) };
    }

    #[test]
    fn initial_pose_null_handle_is_not_activated() {
        let mut dr = DiagRecorder::default();
        let d = diagnostics(&mut dr);
        let v = make_view(0, b"map", [0.0; 3]);
        // SAFETY: null handle is explicitly handled; the other pointers are valid.
        let status = unsafe {
            autoware_ndt_scan_matcher_rs_node_on_initial_pose(core::ptr::null(), &d, &raw const v)
        };
        assert_eq!(status, INITIAL_POSE_NOT_ACTIVATED);
        assert!(dr.events.is_empty()); // null handle → no diagnostics emitted
    }

    #[test]
    fn regularization_pose_emits_diagnostics_and_null_is_noop() {
        let mut dr = DiagRecorder::default();
        let d = diagnostics(&mut dr);
        let handle = make_handle(b"map");
        assert!(!handle.is_null());
        let v = make_view(77, b"map", [1.0, 2.0, 0.0]);
        // SAFETY: live handle, valid diagnostics, valid view.
        unsafe { autoware_ndt_scan_matcher_rs_node_on_regularization_pose(handle, &d, &raw const v) };
        assert_eq!(
            dr.events,
            alloc::vec![
                "clear".to_string(),
                "i64 topic_time_stamp=77".to_string(),
                "publish 77".to_string(),
            ]
        );

        // null view → no-op (no diagnostics).
        dr.events.clear();
        // SAFETY: null view is explicitly handled.
        unsafe {
            autoware_ndt_scan_matcher_rs_node_on_regularization_pose(handle, &d, core::ptr::null());
        }
        assert!(dr.events.is_empty());

        // SAFETY: live handle from `_new`, freed exactly once.
        unsafe { crate::node_handle::autoware_ndt_scan_matcher_rs_free(handle) };
    }

    // A map-update input centered at the origin's last-update position; tests tweak fields.
    fn map_base() -> MapUpdateInput {
        MapUpdateInput {
            current_x: 0.0,
            current_y: 0.0,
            last_update_x: 0.0,
            last_update_y: 0.0,
            lidar_radius: 50.0,
            map_radius: 150.0,
            update_distance: 20.0,
        }
    }

    #[test]
    fn map_update_distance_is_euclidean() {
        // 3-4-5 triangle from the last update position.
        let v = evaluate_map_update(&MapUpdateInput {
            current_x: 3.0,
            current_y: 4.0,
            ..map_base()
        });
        assert_eq!(v.distance, 5.0);
    }

    #[test]
    fn map_update_should_update_boundary_is_strict() {
        // distance == update_distance (20) is NOT > 20 → no update...
        let at = evaluate_map_update(&MapUpdateInput {
            current_x: 20.0,
            ..map_base()
        });
        assert!(!at.should_update);
        // ...just past it does.
        let over = evaluate_map_update(&MapUpdateInput {
            current_x: 20.0001,
            ..map_base()
        });
        assert!(over.should_update);
    }

    #[test]
    fn map_update_out_of_keep_up_boundary_is_strict() {
        // distance + lidar_radius (50) == map_radius (150) at distance 100 → NOT out of keep-up...
        let at = evaluate_map_update(&MapUpdateInput {
            current_x: 100.0,
            ..map_base()
        });
        assert!(!at.out_of_keep_up);
        // ...just past 100 the map can no longer keep up.
        let over = evaluate_map_update(&MapUpdateInput {
            current_x: 100.0001,
            ..map_base()
        });
        assert!(over.out_of_keep_up);
    }

    #[test]
    fn ffi_map_update_matches_pure() {
        let cases = [
            map_base(),
            MapUpdateInput {
                current_x: 3.0,
                current_y: 4.0,
                ..map_base()
            },
            MapUpdateInput {
                current_x: 200.0,
                ..map_base()
            },
        ];
        for c in cases {
            let pure = evaluate_map_update(&c);
            let inp = AwMapUpdateInput {
                current_x: c.current_x,
                current_y: c.current_y,
                last_update_x: c.last_update_x,
                last_update_y: c.last_update_y,
                lidar_radius: c.lidar_radius,
                map_radius: c.map_radius,
                update_distance: c.update_distance,
            };
            let mut out = AwMapUpdateVerdict::default();
            // SAFETY: `inp` is valid; `out` is a valid, writable verdict.
            unsafe { autoware_ndt_scan_matcher_rs_node_evaluate_map_update(&inp, &mut out) };
            assert_eq!(out.distance, pure.distance);
            assert_eq!(out.out_of_keep_up, pure.out_of_keep_up);
            assert_eq!(out.should_update, pure.should_update);
        }
    }

    #[test]
    fn ffi_map_update_null_is_noop() {
        let mut out = AwMapUpdateVerdict {
            should_update: true,
            ..AwMapUpdateVerdict::default()
        };
        // SAFETY: a null input must leave `out` untouched.
        unsafe {
            autoware_ndt_scan_matcher_rs_node_evaluate_map_update(core::ptr::null(), &mut out);
        }
        assert!(out.should_update);
    }
}
