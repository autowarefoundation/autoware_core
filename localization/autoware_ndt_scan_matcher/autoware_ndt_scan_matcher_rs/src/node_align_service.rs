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

/// One semantic trace event emitted by the align-service decision FFI.
///
/// This intentionally records semantic decision fields only, not ROS message bytes, wall-clock time,
/// memory addresses, or raw logs.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
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
    }
}

#[expect(
    unsafe_code,
    reason = "C ABI trace buffer boundary; pointer validity and capacity are caller contract, bounds checked before write"
)]
fn append_trace_event(
    trace: *mut AwNdtAlignServiceTrace,
    input: &AwNdtAlignServiceInput,
    decision: AwNdtAlignServiceDecision,
) {
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
        trace_ref
            .events
            .add(trace_ref.len)
            .write(make_trace_event(input, decision));
    }
    trace_ref.len = next_len;
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
        got: AwNdtAlignServiceTraceEvent,
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
        expect_event(events[0], &input_ok, out);
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
        expect_event(events[0], &bad, out);
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
