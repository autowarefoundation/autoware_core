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

//! Deterministic align-service gate/response decisions for the ROS node shell.
//!
//! This module intentionally does not port the TPE/search loop. It owns only the branch/status
//! decisions around the service so the nondeterministic align search can be isolated behind a later
//! strategy boundary.

/// Initial pose TF lookup failed.
pub const NDT_ALIGN_SERVICE_STATUS_TRANSFORM_UNAVAILABLE: i32 = 0;
/// Target map has not been loaded into the NDT engine.
pub const NDT_ALIGN_SERVICE_STATUS_MAP_UNAVAILABLE: i32 = 1;
/// Sensor source cloud is not available yet.
pub const NDT_ALIGN_SERVICE_STATUS_SENSOR_UNAVAILABLE: i32 = 2;
/// Deterministic gates passed; C++ should run the existing align/TPE implementation.
pub const NDT_ALIGN_SERVICE_STATUS_READY_TO_ALIGN: i32 = 3;
/// Align completed and response success/reliability have been decided from the score.
pub const NDT_ALIGN_SERVICE_STATUS_ALIGNED: i32 = 4;
/// FFI input contained an invalid flag bit pattern.
pub const NDT_ALIGN_SERVICE_STATUS_INVALID_INPUT: i32 = 5;

/// A deterministic align-service decision event.
pub const NDT_ALIGN_TRACE_EVENT_DECISION: i32 = 0;
/// A semantic summary of the C++ TPE/search loop used before the search is ported to Rust.
pub const NDT_ALIGN_TRACE_EVENT_SEARCH_SUMMARY: i32 = 1;
/// A deterministic summary of the align-service response payload returned to ROS.
pub const NDT_ALIGN_TRACE_EVENT_RESPONSE: i32 = 2;

/// No align-service gate diagnostic/log message is required.
pub const NDT_ALIGN_SERVICE_MESSAGE_NONE: i32 = 0;
/// TF lookup failed; C++ formats the target/source frame names.
pub const NDT_ALIGN_SERVICE_MESSAGE_TRANSFORM_UNAVAILABLE: i32 = 1;
/// Target map is missing.
pub const NDT_ALIGN_SERVICE_MESSAGE_MAP_UNAVAILABLE: i32 = 2;
/// Sensor source cloud is missing.
pub const NDT_ALIGN_SERVICE_MESSAGE_SENSOR_UNAVAILABLE: i32 = 3;

/// Diagnostic status level constants mirrored as plain ABI integers.
pub const NDT_ALIGN_SERVICE_DIAGNOSTIC_OK: i32 = 0;
pub const NDT_ALIGN_SERVICE_DIAGNOSTIC_WARN: i32 = 1;
pub const NDT_ALIGN_SERVICE_DIAGNOSTIC_ERROR: i32 = 2;

/// One semantic trace event emitted by the align-service decision FFI.
///
/// This intentionally records semantic decision fields only, not ROS message bytes, wall-clock time,
/// memory addresses, or raw logs.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AwNdtAlignServiceTraceEvent {
    pub kind: i32,
    pub status: i32,
    pub success: u8,
    pub reliable: u8,
    pub should_align: u8,
    pub valid: u8,
    pub score_available: u8,
    pub score: f64,
    pub reliable_score_threshold: f64,
    pub particles_requested: i64,
    pub particles_evaluated: i64,
    pub marker_publish_count: i64,
    pub cloud_publish_count: i64,
    pub best_iteration: i32,
    pub best_score: f64,
    pub diagnostic_level: i32,
    pub message_kind: i32,
    pub response_stamp_ns: i64,
    pub response_position: [f64; 3],
    pub response_orientation: [f64; 4],
    pub response_covariance: [f64; 36],
}

impl Default for AwNdtAlignServiceTraceEvent {
    fn default() -> Self {
        Self {
            kind: 0,
            status: 0,
            success: 0,
            reliable: 0,
            should_align: 0,
            valid: 0,
            score_available: 0,
            score: 0.0,
            reliable_score_threshold: 0.0,
            particles_requested: 0,
            particles_evaluated: 0,
            marker_publish_count: 0,
            cloud_publish_count: 0,
            best_iteration: 0,
            best_score: 0.0,
            diagnostic_level: NDT_ALIGN_SERVICE_DIAGNOSTIC_OK,
            message_kind: NDT_ALIGN_SERVICE_MESSAGE_NONE,
            response_stamp_ns: 0,
            response_position: [0.0; 3],
            response_orientation: [0.0; 4],
            response_covariance: [0.0; 36],
        }
    }
}

/// Caller-owned trace buffer for align-service semantic events.
///
/// `events` points to `capacity` writable [`AwNdtAlignServiceTraceEvent`] slots. Rust appends by
/// writing at `events[len]` and incrementing `len`. If the event cannot be written, `overflowed` is
/// set to `1` and `len` is left unchanged.
#[repr(C)]
pub struct AwNdtAlignServiceTrace {
    pub events: *mut AwNdtAlignServiceTraceEvent,
    pub capacity: usize,
    pub len: usize,
    pub overflowed: u8,
}

/// Deterministic inputs to the align-service decision.
///
/// Boolean-like fields are `u8` instead of Rust `bool` so the FFI boundary can reject invalid C bit
/// patterns before interpreting them as booleans.
#[repr(C)]
pub struct AwNdtAlignServiceInput {
    pub transform_initial_pose_ok: u8,
    pub map_points_ok: u8,
    pub sensor_points_ok: u8,
    pub align_score_available: u8,
    pub align_score: f64,
    pub reliable_score_threshold: f64,
}

/// Deterministic align-service branch/response decision.
///
/// Boolean-like outputs are `u8` to keep the C ABI plain and explicit. Values are always `0` or `1`.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AwNdtAlignServiceDecision {
    pub status: i32,
    pub success: u8,
    pub reliable: u8,
    pub should_align: u8,
    pub valid: u8,
}

/// Inputs needed to assemble the successful align-service response after C++ TPE/search completes.
///
/// `position` and `orientation` describe the aligned pose selected by the search. `request_covariance`
/// is the original service request covariance that the historical C++ response copied unchanged.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AwNdtAlignServiceAlignedInput {
    pub stamp_ns: i64,
    pub position: [f64; 3],
    pub orientation: [f64; 4],
    pub request_covariance: [f64; 36],
    pub align_score: f64,
    pub reliable_score_threshold: f64,
}

/// Deterministic response payload for the align service.
///
/// C++ remains responsible for ROS header frame assignment and message publication/logging.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AwNdtAlignServiceResponse {
    pub status: i32,
    pub success: u8,
    pub reliable: u8,
    pub valid: u8,
    pub stamp_ns: i64,
    pub position: [f64; 3],
    pub orientation: [f64; 4],
    pub covariance: [f64; 36],
}

/// Semantic summary of one align-service TPE/search run.
///
/// Counts are signed at the C ABI boundary so Rust can reject invalid negative values without
/// relying on C++ unsigned conversions. This is trace data only; it does not drive production
/// service behavior.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AwNdtAlignServiceSearchSummaryInput {
    pub particles_requested: i64,
    pub particles_evaluated: i64,
    pub marker_publish_count: i64,
    pub cloud_publish_count: i64,
    pub best_iteration: i32,
    pub best_score: f64,
    pub reliable_score_threshold: f64,
}

impl Default for AwNdtAlignServiceResponse {
    fn default() -> Self {
        Self {
            status: NDT_ALIGN_SERVICE_STATUS_INVALID_INPUT,
            success: 0,
            reliable: 0,
            valid: 0,
            stamp_ns: 0,
            position: [0.0; 3],
            orientation: [0.0; 4],
            covariance: [0.0; 36],
        }
    }
}

impl Default for AwNdtAlignServiceSearchSummaryInput {
    fn default() -> Self {
        Self {
            particles_requested: 0,
            particles_evaluated: 0,
            marker_publish_count: 0,
            cloud_publish_count: 0,
            best_iteration: 0,
            best_score: 0.0,
            reliable_score_threshold: 0.0,
        }
    }
}

/// Deterministic action C++ should take for an align-service gate decision.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AwNdtAlignServiceGateAction {
    pub status: i32,
    pub success: u8,
    pub reliable: u8,
    pub should_align: u8,
    pub valid: u8,
    pub diagnostic_level: i32,
    pub message_kind: i32,
}

fn flag(v: u8) -> Option<bool> {
    match v {
        0 => Some(false),
        1 => Some(true),
        _ => None,
    }
}

fn byte(v: bool) -> u8 {
    u8::from(v)
}

fn decision(
    status: i32,
    success: bool,
    reliable: bool,
    should_align: bool,
) -> AwNdtAlignServiceDecision {
    AwNdtAlignServiceDecision {
        status,
        success: byte(success),
        reliable: byte(reliable),
        should_align: byte(should_align),
        valid: 1,
    }
}

fn invalid_decision() -> AwNdtAlignServiceDecision {
    AwNdtAlignServiceDecision {
        status: NDT_ALIGN_SERVICE_STATUS_INVALID_INPUT,
        success: 0,
        reliable: 0,
        should_align: 0,
        valid: 0,
    }
}

fn score_available_byte(input: &AwNdtAlignServiceInput) -> u8 {
    byte(flag(input.align_score_available).unwrap_or(false))
}

fn make_trace_event(
    input: &AwNdtAlignServiceInput,
    decision: AwNdtAlignServiceDecision,
) -> AwNdtAlignServiceTraceEvent {
    let action = gate_action_from_decision(decision);
    AwNdtAlignServiceTraceEvent {
        kind: NDT_ALIGN_TRACE_EVENT_DECISION,
        status: decision.status,
        success: decision.success,
        reliable: decision.reliable,
        should_align: decision.should_align,
        valid: decision.valid,
        score_available: score_available_byte(input),
        score: input.align_score,
        reliable_score_threshold: input.reliable_score_threshold,
        particles_requested: 0,
        particles_evaluated: 0,
        marker_publish_count: 0,
        cloud_publish_count: 0,
        best_iteration: 0,
        best_score: 0.0,
        diagnostic_level: action.diagnostic_level,
        message_kind: action.message_kind,
        response_stamp_ns: 0,
        response_position: [0.0; 3],
        response_orientation: [0.0; 4],
        response_covariance: [0.0; 36],
    }
}

fn search_summary_is_valid(input: &AwNdtAlignServiceSearchSummaryInput) -> bool {
    input.particles_requested >= 0
        && input.particles_evaluated >= 0
        && input.marker_publish_count >= 0
        && input.cloud_publish_count >= 0
        && input.best_iteration >= 0
}

fn make_search_summary_trace_event(
    input: &AwNdtAlignServiceSearchSummaryInput,
) -> AwNdtAlignServiceTraceEvent {
    let valid = search_summary_is_valid(input);
    AwNdtAlignServiceTraceEvent {
        kind: NDT_ALIGN_TRACE_EVENT_SEARCH_SUMMARY,
        status: if valid {
            NDT_ALIGN_SERVICE_STATUS_ALIGNED
        } else {
            NDT_ALIGN_SERVICE_STATUS_INVALID_INPUT
        },
        success: byte(valid),
        reliable: byte(valid && input.reliable_score_threshold < input.best_score),
        should_align: 0,
        valid: byte(valid),
        score_available: byte(valid),
        score: input.best_score,
        reliable_score_threshold: input.reliable_score_threshold,
        particles_requested: input.particles_requested,
        particles_evaluated: input.particles_evaluated,
        marker_publish_count: input.marker_publish_count,
        cloud_publish_count: input.cloud_publish_count,
        best_iteration: input.best_iteration,
        best_score: input.best_score,
        diagnostic_level: NDT_ALIGN_SERVICE_DIAGNOSTIC_OK,
        message_kind: NDT_ALIGN_SERVICE_MESSAGE_NONE,
        response_stamp_ns: 0,
        response_position: [0.0; 3],
        response_orientation: [0.0; 4],
        response_covariance: [0.0; 36],
    }
}

fn make_response_trace_event(input: &AwNdtAlignServiceResponse) -> AwNdtAlignServiceTraceEvent {
    AwNdtAlignServiceTraceEvent {
        kind: NDT_ALIGN_TRACE_EVENT_RESPONSE,
        status: input.status,
        success: input.success,
        reliable: input.reliable,
        should_align: 0,
        valid: input.valid,
        score_available: 0,
        score: 0.0,
        reliable_score_threshold: 0.0,
        particles_requested: 0,
        particles_evaluated: 0,
        marker_publish_count: 0,
        cloud_publish_count: 0,
        best_iteration: 0,
        best_score: 0.0,
        diagnostic_level: NDT_ALIGN_SERVICE_DIAGNOSTIC_OK,
        message_kind: NDT_ALIGN_SERVICE_MESSAGE_NONE,
        response_stamp_ns: input.stamp_ns,
        response_position: input.position,
        response_orientation: input.orientation,
        response_covariance: input.covariance,
    }
}

#[expect(
    unsafe_code,
    reason = "C ABI trace buffer boundary; pointer validity and capacity are caller contract, bounds checked before write"
)]
fn write_trace_event(trace: *mut AwNdtAlignServiceTrace, event: &AwNdtAlignServiceTraceEvent) {
    if trace.is_null() {
        return;
    }
    // SAFETY: non-null per the check; caller guarantees a valid, aligned trace header.
    let trace_ref = unsafe { &mut *trace };
    if trace_ref.events.is_null() || trace_ref.capacity == 0 {
        trace_ref.overflowed = 1;
        return;
    }
    if trace_ref.len >= trace_ref.capacity {
        trace_ref.overflowed = 1;
        return;
    }
    let Some(next_len) = trace_ref.len.checked_add(1_usize) else {
        trace_ref.overflowed = 1;
        return;
    };
    if next_len > trace_ref.capacity {
        trace_ref.overflowed = 1;
        return;
    }
    // SAFETY: events is non-null and len < capacity, so writing one event at events + len is in-bounds
    // for the caller-provided buffer. The trace owns the mutable buffer for the duration of the call.
    unsafe {
        trace_ref.events.add(trace_ref.len).write(*event);
    }
    trace_ref.len = next_len;
}

fn append_trace_event(
    trace: *mut AwNdtAlignServiceTrace,
    input: &AwNdtAlignServiceInput,
    decision: AwNdtAlignServiceDecision,
) {
    write_trace_event(trace, &make_trace_event(input, decision));
}

#[must_use]
pub fn decide_align_service(input: &AwNdtAlignServiceInput) -> AwNdtAlignServiceDecision {
    let Some(transform_ok) = flag(input.transform_initial_pose_ok) else {
        return invalid_decision();
    };
    let Some(map_ok) = flag(input.map_points_ok) else {
        return invalid_decision();
    };
    let Some(sensor_ok) = flag(input.sensor_points_ok) else {
        return invalid_decision();
    };
    let Some(score_available) = flag(input.align_score_available) else {
        return invalid_decision();
    };

    if !transform_ok {
        return decision(
            NDT_ALIGN_SERVICE_STATUS_TRANSFORM_UNAVAILABLE,
            false,
            false,
            false,
        );
    }
    if !map_ok {
        return decision(
            NDT_ALIGN_SERVICE_STATUS_MAP_UNAVAILABLE,
            false,
            false,
            false,
        );
    }
    if !sensor_ok {
        return decision(
            NDT_ALIGN_SERVICE_STATUS_SENSOR_UNAVAILABLE,
            false,
            false,
            false,
        );
    }
    if !score_available {
        return decision(NDT_ALIGN_SERVICE_STATUS_READY_TO_ALIGN, false, false, true);
    }

    decision(
        NDT_ALIGN_SERVICE_STATUS_ALIGNED,
        true,
        input.reliable_score_threshold < input.align_score,
        false,
    )
}

#[must_use]
pub fn assemble_aligned_response(
    input: &AwNdtAlignServiceAlignedInput,
) -> AwNdtAlignServiceResponse {
    let decision_input = AwNdtAlignServiceInput {
        transform_initial_pose_ok: 1,
        map_points_ok: 1,
        sensor_points_ok: 1,
        align_score_available: 1,
        align_score: input.align_score,
        reliable_score_threshold: input.reliable_score_threshold,
    };
    let result = decide_align_service(&decision_input);
    AwNdtAlignServiceResponse {
        status: result.status,
        success: result.success,
        reliable: result.reliable,
        valid: result.valid,
        stamp_ns: input.stamp_ns,
        position: input.position,
        orientation: input.orientation,
        covariance: input.request_covariance,
    }
}

#[must_use]
pub fn gate_action_from_decision(
    decision: AwNdtAlignServiceDecision,
) -> AwNdtAlignServiceGateAction {
    let (diagnostic_level, message_kind) = match decision.status {
        NDT_ALIGN_SERVICE_STATUS_TRANSFORM_UNAVAILABLE => (
            NDT_ALIGN_SERVICE_DIAGNOSTIC_ERROR,
            NDT_ALIGN_SERVICE_MESSAGE_TRANSFORM_UNAVAILABLE,
        ),
        NDT_ALIGN_SERVICE_STATUS_MAP_UNAVAILABLE => (
            NDT_ALIGN_SERVICE_DIAGNOSTIC_WARN,
            NDT_ALIGN_SERVICE_MESSAGE_MAP_UNAVAILABLE,
        ),
        NDT_ALIGN_SERVICE_STATUS_SENSOR_UNAVAILABLE => (
            NDT_ALIGN_SERVICE_DIAGNOSTIC_WARN,
            NDT_ALIGN_SERVICE_MESSAGE_SENSOR_UNAVAILABLE,
        ),
        _ => (
            NDT_ALIGN_SERVICE_DIAGNOSTIC_OK,
            NDT_ALIGN_SERVICE_MESSAGE_NONE,
        ),
    };
    AwNdtAlignServiceGateAction {
        status: decision.status,
        success: decision.success,
        reliable: decision.reliable,
        should_align: decision.should_align,
        valid: decision.valid,
        diagnostic_level,
        message_kind,
    }
}

#[must_use]
pub fn evaluate_align_service_gate(input: &AwNdtAlignServiceInput) -> AwNdtAlignServiceGateAction {
    gate_action_from_decision(decide_align_service(input))
}

pub fn append_search_summary_trace(
    input: &AwNdtAlignServiceSearchSummaryInput,
    trace: *mut AwNdtAlignServiceTrace,
) {
    write_trace_event(trace, &make_search_summary_trace_event(input));
}

pub fn append_response_trace(
    input: &AwNdtAlignServiceResponse,
    trace: *mut AwNdtAlignServiceTrace,
) {
    write_trace_event(trace, &make_response_trace_event(input));
}

/// Decide the deterministic align-service branch/response state and optionally append one semantic
/// trace event.
///
/// # Safety
/// `input` must point to a valid, aligned [`AwNdtAlignServiceInput`] and `out` to a valid, aligned,
/// writable [`AwNdtAlignServiceDecision`]. `trace` may be null; otherwise it must point to a valid,
/// aligned [`AwNdtAlignServiceTrace`] whose `events` field is either null or points to `capacity`
/// writable [`AwNdtAlignServiceTraceEvent`] slots. If `input` or `out` is null, this function is a
/// no-op and no trace event is written.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers and u8 boolean bit patterns validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_node_decide_align_service_traced(
    input: *const AwNdtAlignServiceInput,
    trace: *mut AwNdtAlignServiceTrace,
    out: *mut AwNdtAlignServiceDecision,
) {
    if input.is_null() || out.is_null() {
        return;
    }
    // SAFETY: non-null per the check; caller guarantees a valid, aligned input struct, read once.
    let input_ref = unsafe { &*input };
    let result = decide_align_service(input_ref);
    append_trace_event(trace, input_ref, result);
    // SAFETY: `out` is non-null per the check and points to a writable decision per the contract.
    unsafe {
        *out = result;
    }
}

/// Decide the deterministic align-service branch/response state.
///
/// # Safety
/// `input` must point to a valid, aligned [`AwNdtAlignServiceInput`] and `out` to a valid, aligned,
/// writable [`AwNdtAlignServiceDecision`]. If either pointer is null, this function is a no-op.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; compatibility wrapper over the traced FFI"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_node_decide_align_service(
    input: *const AwNdtAlignServiceInput,
    out: *mut AwNdtAlignServiceDecision,
) {
    // SAFETY: this wrapper preserves the same pointer contract and passes a null trace pointer.
    unsafe {
        autoware_ndt_scan_matcher_rs_node_decide_align_service_traced(
            input,
            core::ptr::null_mut(),
            out,
        );
    }
}

/// Assemble the successful align-service response payload after the align search has completed.
///
/// # Safety
/// `input` must point to a valid, aligned [`AwNdtAlignServiceAlignedInput`] and `out` to a valid,
/// aligned, writable [`AwNdtAlignServiceResponse`]. If either pointer is null, this function is a
/// no-op.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers validated before reading/writing POD response structs"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_node_assemble_align_service_response(
    input: *const AwNdtAlignServiceAlignedInput,
    out: *mut AwNdtAlignServiceResponse,
) {
    if input.is_null() || out.is_null() {
        return;
    }
    // SAFETY: non-null per the check; caller guarantees a valid, aligned input struct, read once.
    let input_ref = unsafe { &*input };
    let result = assemble_aligned_response(input_ref);
    // SAFETY: `out` is non-null per the check and points to writable response storage.
    unsafe {
        *out = result;
    }
}

/// Evaluate the deterministic align-service gate action and optionally append one semantic decision
/// trace event.
///
/// # Safety
/// `input` must point to a valid, aligned [`AwNdtAlignServiceInput`] and `out` to a valid, aligned,
/// writable [`AwNdtAlignServiceGateAction`]. `trace` follows the same caller-owned buffer contract as
/// [`autoware_ndt_scan_matcher_rs_node_decide_align_service_traced`]. If `input` or `out` is null,
/// this function is a no-op and no trace event is written.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers and u8 boolean bit patterns validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_node_evaluate_align_service_gate(
    input: *const AwNdtAlignServiceInput,
    trace: *mut AwNdtAlignServiceTrace,
    out: *mut AwNdtAlignServiceGateAction,
) {
    if input.is_null() || out.is_null() {
        return;
    }
    // SAFETY: non-null per the check; caller guarantees a valid, aligned input struct, read once.
    let input_ref = unsafe { &*input };
    let decision = decide_align_service(input_ref);
    append_trace_event(trace, input_ref, decision);
    let action = gate_action_from_decision(decision);
    // SAFETY: `out` is non-null per the check and points to writable action storage.
    unsafe {
        *out = action;
    }
}

/// Append one semantic summary event for the existing C++ align-service TPE/search loop.
///
/// # Safety
/// `input` must point to a valid, aligned [`AwNdtAlignServiceSearchSummaryInput`]. `trace` may be
/// null; otherwise it must point to a valid, aligned [`AwNdtAlignServiceTrace`] whose `events` field
/// is either null or points to `capacity` writable [`AwNdtAlignServiceTraceEvent`] slots. If `input`
/// is null, this function is a no-op.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointer and signed count validity are checked before trace append"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_node_append_align_service_search_summary_trace(
    input: *const AwNdtAlignServiceSearchSummaryInput,
    trace: *mut AwNdtAlignServiceTrace,
) {
    if input.is_null() {
        return;
    }
    // SAFETY: non-null per the check; caller guarantees a valid, aligned input struct, read once.
    let input_ref = unsafe { &*input };
    append_search_summary_trace(input_ref, trace);
}

/// Append one semantic response-summary event for the align-service response payload.
///
/// # Safety
/// `input` must point to a valid, aligned [`AwNdtAlignServiceResponse`]. `trace` follows the same
/// caller-owned buffer contract as [`autoware_ndt_scan_matcher_rs_node_decide_align_service_traced`].
/// If `input` is null, this function is a no-op.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; response pointer is checked before trace append"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_node_append_align_service_response_trace(
    input: *const AwNdtAlignServiceResponse,
    trace: *mut AwNdtAlignServiceTrace,
) {
    if input.is_null() {
        return;
    }
    // SAFETY: non-null per the check; caller guarantees a valid, aligned response struct, read once.
    let input_ref = unsafe { &*input };
    append_response_trace(input_ref, trace);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn input(
        transform_ok: u8,
        map_ok: u8,
        sensor_ok: u8,
        score_available: u8,
        score: f64,
    ) -> AwNdtAlignServiceInput {
        AwNdtAlignServiceInput {
            transform_initial_pose_ok: transform_ok,
            map_points_ok: map_ok,
            sensor_points_ok: sensor_ok,
            align_score_available: score_available,
            align_score: score,
            reliable_score_threshold: 4.0,
        }
    }

    fn expect_decision(
        got: AwNdtAlignServiceDecision,
        status: i32,
        success: u8,
        reliable: u8,
        should_align: u8,
        valid: u8,
    ) {
        assert_eq!(got.status, status);
        assert_eq!(got.success, success);
        assert_eq!(got.reliable, reliable);
        assert_eq!(got.should_align, should_align);
        assert_eq!(got.valid, valid);
    }

    fn expect_event(
        got: &AwNdtAlignServiceTraceEvent,
        input: &AwNdtAlignServiceInput,
        decision: AwNdtAlignServiceDecision,
    ) {
        assert_eq!(got.kind, NDT_ALIGN_TRACE_EVENT_DECISION);
        assert_eq!(got.status, decision.status);
        assert_eq!(got.success, decision.success);
        assert_eq!(got.reliable, decision.reliable);
        assert_eq!(got.should_align, decision.should_align);
        assert_eq!(got.valid, decision.valid);
        assert_eq!(got.score_available, score_available_byte(input));
        assert_eq!(got.score.to_bits(), input.align_score.to_bits());
        assert_eq!(
            got.reliable_score_threshold.to_bits(),
            input.reliable_score_threshold.to_bits()
        );
        assert_eq!(got.particles_requested, 0);
        assert_eq!(got.particles_evaluated, 0);
        assert_eq!(got.marker_publish_count, 0);
        assert_eq!(got.cloud_publish_count, 0);
        assert_eq!(got.best_iteration, 0);
        assert_eq!(got.best_score.to_bits(), 0.0_f64.to_bits());
        let action = gate_action_from_decision(decision);
        assert_eq!(got.diagnostic_level, action.diagnostic_level);
        assert_eq!(got.message_kind, action.message_kind);
        assert_eq!(got.response_stamp_ns, 0);
        expect_f64_array_bits_eq(&got.response_position, &[0.0; 3]);
        expect_f64_array_bits_eq(&got.response_orientation, &[0.0; 4]);
        expect_f64_array_bits_eq(&got.response_covariance, &[0.0; 36]);
    }

    fn search_summary(
        particles_requested: i64,
        particles_evaluated: i64,
        marker_publish_count: i64,
        cloud_publish_count: i64,
        best_iteration: i32,
        best_score: f64,
        reliable_score_threshold: f64,
    ) -> AwNdtAlignServiceSearchSummaryInput {
        AwNdtAlignServiceSearchSummaryInput {
            particles_requested,
            particles_evaluated,
            marker_publish_count,
            cloud_publish_count,
            best_iteration,
            best_score,
            reliable_score_threshold,
        }
    }

    fn expect_search_summary_event(
        got: &AwNdtAlignServiceTraceEvent,
        input: &AwNdtAlignServiceSearchSummaryInput,
        status: i32,
        success: u8,
        reliable: u8,
        valid: u8,
        score_available: u8,
    ) {
        assert_eq!(got.kind, NDT_ALIGN_TRACE_EVENT_SEARCH_SUMMARY);
        assert_eq!(got.status, status);
        assert_eq!(got.success, success);
        assert_eq!(got.reliable, reliable);
        assert_eq!(got.should_align, 0);
        assert_eq!(got.valid, valid);
        assert_eq!(got.score_available, score_available);
        assert_eq!(got.score.to_bits(), input.best_score.to_bits());
        assert_eq!(
            got.reliable_score_threshold.to_bits(),
            input.reliable_score_threshold.to_bits()
        );
        assert_eq!(got.particles_requested, input.particles_requested);
        assert_eq!(got.particles_evaluated, input.particles_evaluated);
        assert_eq!(got.marker_publish_count, input.marker_publish_count);
        assert_eq!(got.cloud_publish_count, input.cloud_publish_count);
        assert_eq!(got.best_iteration, input.best_iteration);
        assert_eq!(got.best_score.to_bits(), input.best_score.to_bits());
        assert_eq!(got.diagnostic_level, NDT_ALIGN_SERVICE_DIAGNOSTIC_OK);
        assert_eq!(got.message_kind, NDT_ALIGN_SERVICE_MESSAGE_NONE);
        assert_eq!(got.response_stamp_ns, 0);
        expect_f64_array_bits_eq(&got.response_position, &[0.0; 3]);
        expect_f64_array_bits_eq(&got.response_orientation, &[0.0; 4]);
        expect_f64_array_bits_eq(&got.response_covariance, &[0.0; 36]);
    }

    fn expect_response_event(got: &AwNdtAlignServiceTraceEvent, input: &AwNdtAlignServiceResponse) {
        assert_eq!(got.kind, NDT_ALIGN_TRACE_EVENT_RESPONSE);
        assert_eq!(got.status, input.status);
        assert_eq!(got.success, input.success);
        assert_eq!(got.reliable, input.reliable);
        assert_eq!(got.should_align, 0);
        assert_eq!(got.valid, input.valid);
        assert_eq!(got.score_available, 0);
        assert_eq!(got.score.to_bits(), 0.0_f64.to_bits());
        assert_eq!(got.reliable_score_threshold.to_bits(), 0.0_f64.to_bits());
        assert_eq!(got.particles_requested, 0);
        assert_eq!(got.particles_evaluated, 0);
        assert_eq!(got.marker_publish_count, 0);
        assert_eq!(got.cloud_publish_count, 0);
        assert_eq!(got.best_iteration, 0);
        assert_eq!(got.best_score.to_bits(), 0.0_f64.to_bits());
        assert_eq!(got.diagnostic_level, NDT_ALIGN_SERVICE_DIAGNOSTIC_OK);
        assert_eq!(got.message_kind, NDT_ALIGN_SERVICE_MESSAGE_NONE);
        assert_eq!(got.response_stamp_ns, input.stamp_ns);
        expect_f64_array_bits_eq(&got.response_position, &input.position);
        expect_f64_array_bits_eq(&got.response_orientation, &input.orientation);
        expect_f64_array_bits_eq(&got.response_covariance, &input.covariance);
    }

    fn expect_f64_array_bits_eq<const N: usize>(got: &[f64; N], expected: &[f64; N]) {
        for (got_value, expected_value) in got.iter().zip(expected.iter()) {
            assert_eq!(got_value.to_bits(), expected_value.to_bits());
        }
    }

    #[expect(unsafe_code, reason = "test exercises the C ABI entry point directly")]
    fn ffi_decide(input: *const AwNdtAlignServiceInput, out: *mut AwNdtAlignServiceDecision) {
        // SAFETY: tests pass either null pointers to verify the no-op contract or pointers to local
        // valid, aligned structs that live for the duration of the call.
        unsafe {
            autoware_ndt_scan_matcher_rs_node_decide_align_service(input, out);
        }
    }

    #[expect(
        unsafe_code,
        reason = "test exercises the traced C ABI entry point directly"
    )]
    fn ffi_decide_traced(
        input: *const AwNdtAlignServiceInput,
        trace: *mut AwNdtAlignServiceTrace,
        out: *mut AwNdtAlignServiceDecision,
    ) {
        // SAFETY: tests pass null pointers to verify no-op contracts or local valid, aligned structs
        // and buffers that live for the duration of the call.
        unsafe {
            autoware_ndt_scan_matcher_rs_node_decide_align_service_traced(input, trace, out);
        }
    }

    #[expect(
        unsafe_code,
        reason = "test exercises the gate action C ABI entry point directly"
    )]
    fn ffi_evaluate_gate(
        input: *const AwNdtAlignServiceInput,
        trace: *mut AwNdtAlignServiceTrace,
        out: *mut AwNdtAlignServiceGateAction,
    ) {
        // SAFETY: tests pass null pointers to verify no-op contracts or local valid, aligned structs
        // and buffers that live for the duration of the call.
        unsafe {
            autoware_ndt_scan_matcher_rs_node_evaluate_align_service_gate(input, trace, out);
        }
    }

    #[expect(
        unsafe_code,
        reason = "test exercises the search summary trace C ABI entry point directly"
    )]
    fn ffi_append_search_summary_trace(
        input: *const AwNdtAlignServiceSearchSummaryInput,
        trace: *mut AwNdtAlignServiceTrace,
    ) {
        // SAFETY: tests pass null pointers to verify no-op contracts or local valid, aligned structs
        // and buffers that live for the duration of the call.
        unsafe {
            autoware_ndt_scan_matcher_rs_node_append_align_service_search_summary_trace(
                input, trace,
            );
        }
    }

    #[expect(
        unsafe_code,
        reason = "test exercises the response trace C ABI entry point directly"
    )]
    fn ffi_append_response_trace(
        input: *const AwNdtAlignServiceResponse,
        trace: *mut AwNdtAlignServiceTrace,
    ) {
        // SAFETY: tests pass null pointers to verify no-op contracts or local valid, aligned structs
        // and buffers that live for the duration of the call.
        unsafe {
            autoware_ndt_scan_matcher_rs_node_append_align_service_response_trace(input, trace);
        }
    }

    fn gate_action(
        status: i32,
        success: u8,
        reliable: u8,
        should_align: u8,
        valid: u8,
        diagnostic_level: i32,
        message_kind: i32,
    ) -> AwNdtAlignServiceGateAction {
        AwNdtAlignServiceGateAction {
            status,
            success,
            reliable,
            should_align,
            valid,
            diagnostic_level,
            message_kind,
        }
    }

    fn expect_gate_action(got: AwNdtAlignServiceGateAction, expected: AwNdtAlignServiceGateAction) {
        assert_eq!(got, expected);
    }

    #[test]
    fn gate_action_maps_failure_messages_and_levels() {
        expect_gate_action(
            evaluate_align_service_gate(&input(0, 1, 1, 0, 0.0)),
            gate_action(
                NDT_ALIGN_SERVICE_STATUS_TRANSFORM_UNAVAILABLE,
                0,
                0,
                0,
                1,
                NDT_ALIGN_SERVICE_DIAGNOSTIC_ERROR,
                NDT_ALIGN_SERVICE_MESSAGE_TRANSFORM_UNAVAILABLE,
            ),
        );
        expect_gate_action(
            evaluate_align_service_gate(&input(1, 0, 1, 0, 0.0)),
            gate_action(
                NDT_ALIGN_SERVICE_STATUS_MAP_UNAVAILABLE,
                0,
                0,
                0,
                1,
                NDT_ALIGN_SERVICE_DIAGNOSTIC_WARN,
                NDT_ALIGN_SERVICE_MESSAGE_MAP_UNAVAILABLE,
            ),
        );
        expect_gate_action(
            evaluate_align_service_gate(&input(1, 1, 0, 0, 0.0)),
            gate_action(
                NDT_ALIGN_SERVICE_STATUS_SENSOR_UNAVAILABLE,
                0,
                0,
                0,
                1,
                NDT_ALIGN_SERVICE_DIAGNOSTIC_WARN,
                NDT_ALIGN_SERVICE_MESSAGE_SENSOR_UNAVAILABLE,
            ),
        );
    }

    #[test]
    fn gate_action_ready_aligned_and_invalid_have_no_message() {
        expect_gate_action(
            evaluate_align_service_gate(&input(1, 1, 1, 0, 0.0)),
            gate_action(
                NDT_ALIGN_SERVICE_STATUS_READY_TO_ALIGN,
                0,
                0,
                1,
                1,
                NDT_ALIGN_SERVICE_DIAGNOSTIC_OK,
                NDT_ALIGN_SERVICE_MESSAGE_NONE,
            ),
        );
        expect_gate_action(
            evaluate_align_service_gate(&input(1, 1, 1, 1, 4.5)),
            gate_action(
                NDT_ALIGN_SERVICE_STATUS_ALIGNED,
                1,
                1,
                0,
                1,
                NDT_ALIGN_SERVICE_DIAGNOSTIC_OK,
                NDT_ALIGN_SERVICE_MESSAGE_NONE,
            ),
        );
        expect_gate_action(
            evaluate_align_service_gate(&input(9, 1, 1, 0, 0.0)),
            gate_action(
                NDT_ALIGN_SERVICE_STATUS_INVALID_INPUT,
                0,
                0,
                0,
                0,
                NDT_ALIGN_SERVICE_DIAGNOSTIC_OK,
                NDT_ALIGN_SERVICE_MESSAGE_NONE,
            ),
        );
    }

    #[test]
    fn traced_gate_action_appends_existing_decision_event() {
        let input_ok = input(1, 0, 1, 0, 0.0);
        let mut out = AwNdtAlignServiceGateAction::default();
        let mut events = [AwNdtAlignServiceTraceEvent::default(); 1];
        let mut trace = AwNdtAlignServiceTrace {
            events: events.as_mut_ptr(),
            capacity: events.len(),
            len: 0,
            overflowed: 0,
        };

        ffi_evaluate_gate(&raw const input_ok, &raw mut trace, &raw mut out);

        assert_eq!(trace.len, 1);
        assert_eq!(trace.overflowed, 0);
        expect_gate_action(
            out,
            gate_action(
                NDT_ALIGN_SERVICE_STATUS_MAP_UNAVAILABLE,
                0,
                0,
                0,
                1,
                NDT_ALIGN_SERVICE_DIAGNOSTIC_WARN,
                NDT_ALIGN_SERVICE_MESSAGE_MAP_UNAVAILABLE,
            ),
        );
        let decision = decide_align_service(&input_ok);
        expect_event(&events[0], &input_ok, decision);
        assert_eq!(
            events[0].diagnostic_level,
            NDT_ALIGN_SERVICE_DIAGNOSTIC_WARN
        );
        assert_eq!(
            events[0].message_kind,
            NDT_ALIGN_SERVICE_MESSAGE_MAP_UNAVAILABLE
        );
    }

    #[test]
    fn traced_gate_action_failure_events_include_diagnostic_metadata() {
        let cases = [
            (
                input(0, 1, 1, 0, 0.0),
                NDT_ALIGN_SERVICE_DIAGNOSTIC_ERROR,
                NDT_ALIGN_SERVICE_MESSAGE_TRANSFORM_UNAVAILABLE,
            ),
            (
                input(1, 0, 1, 0, 0.0),
                NDT_ALIGN_SERVICE_DIAGNOSTIC_WARN,
                NDT_ALIGN_SERVICE_MESSAGE_MAP_UNAVAILABLE,
            ),
            (
                input(1, 1, 0, 0, 0.0),
                NDT_ALIGN_SERVICE_DIAGNOSTIC_WARN,
                NDT_ALIGN_SERVICE_MESSAGE_SENSOR_UNAVAILABLE,
            ),
        ];

        for (input_case, diagnostic_level, message_kind) in cases {
            let mut out = AwNdtAlignServiceGateAction::default();
            let mut events = [AwNdtAlignServiceTraceEvent::default(); 1];
            let mut trace = AwNdtAlignServiceTrace {
                events: events.as_mut_ptr(),
                capacity: events.len(),
                len: 0,
                overflowed: 0,
            };

            ffi_evaluate_gate(&raw const input_case, &raw mut trace, &raw mut out);

            assert_eq!(trace.len, 1);
            assert_eq!(trace.overflowed, 0);
            assert_eq!(events[0].diagnostic_level, diagnostic_level);
            assert_eq!(events[0].message_kind, message_kind);
            assert_eq!(out.diagnostic_level, diagnostic_level);
            assert_eq!(out.message_kind, message_kind);
        }
    }

    #[test]
    fn gate_action_ffi_null_is_noop() {
        let input_ok = input(1, 1, 1, 0, 0.0);
        let mut out = AwNdtAlignServiceGateAction {
            status: 99,
            success: 1,
            reliable: 1,
            should_align: 1,
            valid: 1,
            diagnostic_level: 2,
            message_kind: 3,
        };

        ffi_evaluate_gate(core::ptr::null(), core::ptr::null_mut(), &raw mut out);
        assert_eq!(out.status, 99);
        ffi_evaluate_gate(
            &raw const input_ok,
            core::ptr::null_mut(),
            core::ptr::null_mut(),
        );
    }

    #[test]
    fn transform_failure_stops_before_align() {
        expect_decision(
            decide_align_service(&input(0, 1, 1, 0, 0.0)),
            NDT_ALIGN_SERVICE_STATUS_TRANSFORM_UNAVAILABLE,
            0,
            0,
            0,
            1,
        );
    }

    #[test]
    fn missing_map_stops_before_align() {
        expect_decision(
            decide_align_service(&input(1, 0, 1, 0, 0.0)),
            NDT_ALIGN_SERVICE_STATUS_MAP_UNAVAILABLE,
            0,
            0,
            0,
            1,
        );
    }

    #[test]
    fn missing_sensor_stops_before_align() {
        expect_decision(
            decide_align_service(&input(1, 1, 0, 0, 0.0)),
            NDT_ALIGN_SERVICE_STATUS_SENSOR_UNAVAILABLE,
            0,
            0,
            0,
            1,
        );
    }

    #[test]
    fn all_gates_pass_without_score_is_ready() {
        expect_decision(
            decide_align_service(&input(1, 1, 1, 0, 0.0)),
            NDT_ALIGN_SERVICE_STATUS_READY_TO_ALIGN,
            0,
            0,
            1,
            1,
        );
    }

    #[test]
    fn aligned_reliable_uses_strict_threshold_less_than_score() {
        expect_decision(
            decide_align_service(&input(1, 1, 1, 1, 4.1)),
            NDT_ALIGN_SERVICE_STATUS_ALIGNED,
            1,
            1,
            0,
            1,
        );
    }

    #[test]
    fn aligned_unreliable_when_score_does_not_exceed_threshold() {
        expect_decision(
            decide_align_service(&input(1, 1, 1, 1, 4.0)),
            NDT_ALIGN_SERVICE_STATUS_ALIGNED,
            1,
            0,
            0,
            1,
        );
    }

    #[test]
    fn aligned_nan_score_is_unreliable() {
        expect_decision(
            decide_align_service(&input(1, 1, 1, 1, f64::NAN)),
            NDT_ALIGN_SERVICE_STATUS_ALIGNED,
            1,
            0,
            0,
            1,
        );
    }

    #[test]
    fn invalid_flag_returns_invalid_input() {
        expect_decision(
            decide_align_service(&input(2, 1, 1, 0, 0.0)),
            NDT_ALIGN_SERVICE_STATUS_INVALID_INPUT,
            0,
            0,
            0,
            0,
        );
    }

    #[test]
    fn ffi_null_is_noop_and_invalid_flag_writes_invalid() {
        let input_ok = input(1, 1, 1, 0, 0.0);
        let mut out = AwNdtAlignServiceDecision {
            status: 99,
            success: 1,
            reliable: 1,
            should_align: 1,
            valid: 1,
        };
        ffi_decide(core::ptr::null(), &raw mut out);
        assert_eq!(out.status, 99);
        ffi_decide(&raw const input_ok, core::ptr::null_mut());

        let bad = input(1, 1, 3, 0, 0.0);
        ffi_decide(&raw const bad, &raw mut out);
        expect_decision(out, NDT_ALIGN_SERVICE_STATUS_INVALID_INPUT, 0, 0, 0, 0);
    }

    #[test]
    fn traced_and_untraced_decisions_match() {
        let input_ok = input(1, 1, 1, 1, 4.5);
        let mut traced = AwNdtAlignServiceDecision::default();
        let mut untraced = AwNdtAlignServiceDecision::default();
        ffi_decide(&raw const input_ok, &raw mut untraced);
        ffi_decide_traced(&raw const input_ok, core::ptr::null_mut(), &raw mut traced);
        assert_eq!(traced, untraced);
    }

    #[test]
    fn traced_decision_appends_one_semantic_event() {
        let input_ok = input(1, 1, 1, 1, 4.5);
        let mut out = AwNdtAlignServiceDecision::default();
        let mut events = [AwNdtAlignServiceTraceEvent::default(); 2];
        let mut trace = AwNdtAlignServiceTrace {
            events: events.as_mut_ptr(),
            capacity: events.len(),
            len: 0,
            overflowed: 0,
        };

        ffi_decide_traced(&raw const input_ok, &raw mut trace, &raw mut out);

        assert_eq!(trace.len, 1);
        assert_eq!(trace.overflowed, 0);
        expect_decision(out, NDT_ALIGN_SERVICE_STATUS_ALIGNED, 1, 1, 0, 1);
        expect_event(&events[0], &input_ok, out);
    }

    #[test]
    fn traced_invalid_input_appends_invalid_event() {
        let bad = input(1, 2, 1, 0, 7.0);
        let mut out = AwNdtAlignServiceDecision::default();
        let mut events = [AwNdtAlignServiceTraceEvent::default(); 1];
        let mut trace = AwNdtAlignServiceTrace {
            events: events.as_mut_ptr(),
            capacity: events.len(),
            len: 0,
            overflowed: 0,
        };

        ffi_decide_traced(&raw const bad, &raw mut trace, &raw mut out);

        assert_eq!(trace.len, 1);
        assert_eq!(trace.overflowed, 0);
        expect_decision(out, NDT_ALIGN_SERVICE_STATUS_INVALID_INPUT, 0, 0, 0, 0);
        expect_event(&events[0], &bad, out);
    }

    #[test]
    fn trace_null_event_buffer_or_full_buffer_sets_overflow() {
        let input_ok = input(1, 1, 1, 0, 0.0);
        let mut out = AwNdtAlignServiceDecision::default();
        let mut null_trace = AwNdtAlignServiceTrace {
            events: core::ptr::null_mut(),
            capacity: 1,
            len: 0,
            overflowed: 0,
        };
        ffi_decide_traced(&raw const input_ok, &raw mut null_trace, &raw mut out);
        assert_eq!(null_trace.len, 0);
        assert_eq!(null_trace.overflowed, 1);

        let mut events = [AwNdtAlignServiceTraceEvent::default(); 1];
        let mut full_trace = AwNdtAlignServiceTrace {
            events: events.as_mut_ptr(),
            capacity: events.len(),
            len: events.len(),
            overflowed: 0,
        };
        ffi_decide_traced(&raw const input_ok, &raw mut full_trace, &raw mut out);
        assert_eq!(full_trace.len, events.len());
        assert_eq!(full_trace.overflowed, 1);
    }

    fn aligned_input(score: f64, threshold: f64) -> AwNdtAlignServiceAlignedInput {
        let mut covariance = [0.0; 36];
        covariance[0] = 1.0;
        covariance[7] = 2.0;
        covariance[14] = 3.0;
        covariance[21] = 4.0;
        covariance[28] = 5.0;
        covariance[35] = 6.0;
        AwNdtAlignServiceAlignedInput {
            stamp_ns: 123,
            position: [1.0, 2.0, 3.0],
            orientation: [0.1, 0.2, 0.3, 0.9],
            request_covariance: covariance,
            align_score: score,
            reliable_score_threshold: threshold,
        }
    }

    #[expect(
        unsafe_code,
        reason = "test exercises the response assembly C ABI entry point directly"
    )]
    fn ffi_assemble_response(
        input: *const AwNdtAlignServiceAlignedInput,
        out: *mut AwNdtAlignServiceResponse,
    ) {
        // SAFETY: tests pass either null pointers to verify the no-op contract or valid local POD
        // structs that live for the duration of the call.
        unsafe {
            autoware_ndt_scan_matcher_rs_node_assemble_align_service_response(input, out);
        }
    }

    #[test]
    fn search_summary_trace_appends_semantic_event() {
        let summary = search_summary(64, 64, 4, 64, 12, 5.5, 4.0);
        let mut events = [AwNdtAlignServiceTraceEvent::default(); 1];
        let mut trace = AwNdtAlignServiceTrace {
            events: events.as_mut_ptr(),
            capacity: events.len(),
            len: 0,
            overflowed: 0,
        };

        append_search_summary_trace(&summary, &raw mut trace);

        assert_eq!(trace.len, 1);
        assert_eq!(trace.overflowed, 0);
        expect_search_summary_event(
            &events[0],
            &summary,
            NDT_ALIGN_SERVICE_STATUS_ALIGNED,
            1,
            1,
            1,
            1,
        );
    }

    #[test]
    fn search_summary_trace_records_invalid_negative_counts() {
        let summary = search_summary(64, -1, 4, 64, 12, 5.5, 4.0);
        let mut events = [AwNdtAlignServiceTraceEvent::default(); 1];
        let mut trace = AwNdtAlignServiceTrace {
            events: events.as_mut_ptr(),
            capacity: events.len(),
            len: 0,
            overflowed: 0,
        };

        append_search_summary_trace(&summary, &raw mut trace);

        assert_eq!(trace.len, 1);
        assert_eq!(trace.overflowed, 0);
        expect_search_summary_event(
            &events[0],
            &summary,
            NDT_ALIGN_SERVICE_STATUS_INVALID_INPUT,
            0,
            0,
            0,
            0,
        );
    }

    #[test]
    fn search_summary_trace_null_and_full_buffers_are_safe() {
        let summary = search_summary(64, 64, 4, 64, 12, 5.5, 4.0);

        ffi_append_search_summary_trace(core::ptr::null(), core::ptr::null_mut());
        ffi_append_search_summary_trace(&raw const summary, core::ptr::null_mut());

        let mut events = [AwNdtAlignServiceTraceEvent::default(); 1];
        let mut full_trace = AwNdtAlignServiceTrace {
            events: events.as_mut_ptr(),
            capacity: events.len(),
            len: events.len(),
            overflowed: 0,
        };
        ffi_append_search_summary_trace(&raw const summary, &raw mut full_trace);
        assert_eq!(full_trace.len, events.len());
        assert_eq!(full_trace.overflowed, 1);

        let mut null_events = AwNdtAlignServiceTrace {
            events: core::ptr::null_mut(),
            capacity: 1,
            len: 0,
            overflowed: 0,
        };
        ffi_append_search_summary_trace(&raw const summary, &raw mut null_events);
        assert_eq!(null_events.len, 0);
        assert_eq!(null_events.overflowed, 1);
    }

    #[test]
    fn response_trace_appends_response_payload_event() {
        let response = assemble_aligned_response(&aligned_input(4.5, 4.0));
        let mut events = [AwNdtAlignServiceTraceEvent::default(); 1];
        let mut trace = AwNdtAlignServiceTrace {
            events: events.as_mut_ptr(),
            capacity: events.len(),
            len: 0,
            overflowed: 0,
        };

        append_response_trace(&response, &raw mut trace);

        assert_eq!(trace.len, 1);
        assert_eq!(trace.overflowed, 0);
        expect_response_event(&events[0], &response);
    }

    #[test]
    fn response_trace_null_and_full_buffers_are_safe() {
        let response = assemble_aligned_response(&aligned_input(4.5, 4.0));

        ffi_append_response_trace(core::ptr::null(), core::ptr::null_mut());
        ffi_append_response_trace(&raw const response, core::ptr::null_mut());

        let mut events = [AwNdtAlignServiceTraceEvent::default(); 1];
        let mut full_trace = AwNdtAlignServiceTrace {
            events: events.as_mut_ptr(),
            capacity: events.len(),
            len: events.len(),
            overflowed: 0,
        };
        ffi_append_response_trace(&raw const response, &raw mut full_trace);
        assert_eq!(full_trace.len, events.len());
        assert_eq!(full_trace.overflowed, 1);

        let mut null_events = AwNdtAlignServiceTrace {
            events: core::ptr::null_mut(),
            capacity: 1,
            len: 0,
            overflowed: 0,
        };
        ffi_append_response_trace(&raw const response, &raw mut null_events);
        assert_eq!(null_events.len, 0);
        assert_eq!(null_events.overflowed, 1);
    }

    #[test]
    fn aligned_response_copies_pose_covariance_and_reliability() {
        let input_ok = aligned_input(4.5, 4.0);
        let response = assemble_aligned_response(&input_ok);

        assert_eq!(response.status, NDT_ALIGN_SERVICE_STATUS_ALIGNED);
        assert_eq!(response.success, 1);
        assert_eq!(response.reliable, 1);
        assert_eq!(response.valid, 1);
        assert_eq!(response.stamp_ns, input_ok.stamp_ns);
        expect_f64_array_bits_eq(&response.position, &input_ok.position);
        expect_f64_array_bits_eq(&response.orientation, &input_ok.orientation);
        expect_f64_array_bits_eq(&response.covariance, &input_ok.request_covariance);
    }

    #[test]
    fn aligned_response_threshold_equality_and_nan_are_unreliable() {
        let equal_response = assemble_aligned_response(&aligned_input(4.0, 4.0));
        assert_eq!(equal_response.status, NDT_ALIGN_SERVICE_STATUS_ALIGNED);
        assert_eq!(equal_response.success, 1);
        assert_eq!(equal_response.reliable, 0);
        assert_eq!(equal_response.valid, 1);

        let nan_response = assemble_aligned_response(&aligned_input(f64::NAN, 4.0));
        assert_eq!(nan_response.status, NDT_ALIGN_SERVICE_STATUS_ALIGNED);
        assert_eq!(nan_response.success, 1);
        assert_eq!(nan_response.reliable, 0);
        assert_eq!(nan_response.valid, 1);
    }

    #[test]
    fn aligned_response_ffi_null_is_noop() {
        let input_ok = aligned_input(4.5, 4.0);
        let mut out = AwNdtAlignServiceResponse {
            status: 99,
            success: 1,
            reliable: 1,
            valid: 1,
            stamp_ns: 9,
            position: [9.0, 8.0, 7.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
            covariance: [1.0; 36],
        };

        ffi_assemble_response(core::ptr::null(), &raw mut out);
        assert_eq!(out.status, 99);
        ffi_assemble_response(&raw const input_ok, core::ptr::null_mut());

        ffi_assemble_response(&raw const input_ok, &raw mut out);
        assert_eq!(out.status, NDT_ALIGN_SERVICE_STATUS_ALIGNED);
        assert_eq!(out.success, 1);
        assert_eq!(out.reliable, 1);
        expect_f64_array_bits_eq(&out.covariance, &input_ok.request_covariance);
    }
}
