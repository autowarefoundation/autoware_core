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

/// Decide the deterministic align-service branch/response state.
///
/// # Safety
/// `input` must point to a valid, aligned [`AwNdtAlignServiceInput`] and `out` to a valid, aligned,
/// writable [`AwNdtAlignServiceDecision`]. If either pointer is null, this function is a no-op.
#[expect(
    unsafe_code,
    reason = "C ABI boundary; pointers and u8 boolean bit patterns validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_node_decide_align_service(
    input: *const AwNdtAlignServiceInput,
    out: *mut AwNdtAlignServiceDecision,
) {
    if input.is_null() || out.is_null() {
        return;
    }
    // SAFETY: non-null per the check; caller guarantees a valid, aligned input struct, read once.
    let input_ref = unsafe { &*input };
    let result = decide_align_service(input_ref);
    // SAFETY: `out` is non-null per the check and points to a writable decision per the contract.
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

    #[expect(unsafe_code, reason = "test exercises the C ABI entry point directly")]
    fn ffi_decide(input: *const AwNdtAlignServiceInput, out: *mut AwNdtAlignServiceDecision) {
        // SAFETY: tests pass either null pointers to verify the no-op contract or pointers to local
        // valid, aligned structs that live for the duration of the call.
        unsafe {
            autoware_ndt_scan_matcher_rs_node_decide_align_service(input, out);
        }
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
}
