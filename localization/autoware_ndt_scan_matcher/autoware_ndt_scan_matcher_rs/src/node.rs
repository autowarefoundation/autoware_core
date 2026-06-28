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

/// The ROS I/O / node-state operations a migrated Rust callback needs, as C function pointers over an
/// opaque context. Built and owned C++-side (the trampolines cast `ctx` back to `NDTScanMatcher*`);
/// every pointer + `ctx` must stay valid for the duration of the call. Field order must match the C
/// `AwNdtHost` struct.
#[repr(C)]
pub struct NdtHost {
    ctx: *mut c_void,
    /// Set the node's activation flag (`is_activated_`).
    set_activated: extern "C" fn(*mut c_void, bool),
    /// Clear the initial-pose interpolation buffer (`initial_pose_buffer_`).
    clear_initial_pose_buffer: extern "C" fn(*mut c_void),
}

/// Migrated body of `service_trigger_node`: set the activation flag, and on enable clear the
/// initial-pose buffer (so stale poses don't survive a re-activation). The C++ wrapper keeps the
/// diagnostics around this core.
///
/// # Safety
/// `host` is a valid [`NdtHost`] (or null → no-op) whose function pointers and `ctx` outlive the call.
#[expect(
    unsafe_code,
    reason = "C ABI host-interface boundary; validated per rust-c-ffi-safety"
)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn autoware_ndt_scan_matcher_rs_node_on_trigger(
    host: *const NdtHost,
    activate: bool,
) {
    if host.is_null() {
        return;
    }
    // SAFETY: non-null per the check; caller guarantees a valid host whose ctx/fn-pointers are live.
    let h = unsafe { &*host };
    (h.set_activated)(h.ctx, activate);
    if activate {
        (h.clear_initial_pose_buffer)(h.ctx);
    }
}

/// Inputs to the NDT convergence decision: one alignment result's relevant scalars plus the
/// `score_estimation` params. `oscillation_num` is precomputed by the caller (it is already the
/// `count_oscillation` port, so this fn stays focused on the convergence decision).
#[derive(Clone, Copy, Debug)]
pub struct ConvergenceInput {
    /// Iterations the NDT optimizer actually ran.
    pub iteration_num: i32,
    /// The optimizer's iteration cap (`getMaximumIterations`).
    pub max_iterations: i32,
    /// Consecutive direction-inversion count over the iteration trajectory.
    pub oscillation_num: i32,
    /// Transform-probability score of the final pose.
    pub transform_probability: f64,
    /// Nearest-voxel transformation likelihood of the final pose.
    pub nearest_voxel_transformation_likelihood: f64,
    /// Which score gates convergence: `0` = transform probability, `1` = nearest-voxel likelihood.
    pub converged_param_type: i32,
    /// Convergence threshold for the transform-probability score.
    pub converged_param_transform_probability: f64,
    /// Convergence threshold for the nearest-voxel-likelihood score.
    pub converged_param_nearest_voxel_transformation_likelihood: f64,
}

/// The convergence verdict plus the sub-flags the C++ side needs to drive its diagnostics.
#[expect(
    clippy::struct_excessive_bools,
    reason = "1:1 mirror of the C++ callback's independent diagnostic sub-flags (each reported \
              separately to /diagnostics); an enum/state-machine would break that mapping"
)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ConvergenceVerdict {
    /// `false` when `converged_param_type` is unknown — the C++ caller then emits an ERROR
    /// diagnostic and aborts the callback (the other fields are unset/`false`).
    pub valid_param_type: bool,
    /// The optimizer stopped before hitting its iteration cap.
    pub is_ok_iteration_num: bool,
    /// Oscillation count exceeded the threshold (a local-minimum stop still counts as converged).
    pub is_local_optimal_solution_oscillation: bool,
    /// The selected score cleared its threshold.
    pub is_ok_score: bool,
    /// Final verdict: `(is_ok_iteration_num || is_local_optimal_solution_oscillation) && is_ok_score`.
    pub is_converged: bool,
    /// The score that was compared against the threshold (selected by `converged_param_type`).
    pub score: f64,
    /// The threshold the score was compared against.
    pub score_threshold: f64,
}

/// Threshold above which the iteration trajectory is treated as oscillating in a local minimum.
/// Mirrors the `constexpr int oscillation_num_threshold = 10` in `callback_sensor_points_main`.
const OSCILLATION_NUM_THRESHOLD: i32 = 10;

/// Pure NDT convergence decision, ported verbatim from `callback_sensor_points_main`. No allocation,
/// no panic (matches on the `i32` discriminant, no indexing/slicing) and O(1) — RT-clean.
#[must_use]
pub fn evaluate_convergence(input: &ConvergenceInput) -> ConvergenceVerdict {
    let is_ok_iteration_num = input.iteration_num < input.max_iterations;
    let is_local_optimal_solution_oscillation = input.oscillation_num > OSCILLATION_NUM_THRESHOLD;
    let (valid_param_type, score, score_threshold) = match input.converged_param_type {
        // ConvergedParamType::TRANSFORM_PROBABILITY
        0 => (
            true,
            input.transform_probability,
            input.converged_param_transform_probability,
        ),
        // ConvergedParamType::NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD
        1 => (
            true,
            input.nearest_voxel_transformation_likelihood,
            input.converged_param_nearest_voxel_transformation_likelihood,
        ),
        // Unknown type: C++ emits an ERROR diagnostic and returns false from the callback.
        _ => {
            return ConvergenceVerdict {
                valid_param_type: false,
                is_ok_iteration_num,
                is_local_optimal_solution_oscillation,
                ..ConvergenceVerdict::default()
            };
        }
    };
    let is_ok_score = score > score_threshold;
    let is_converged =
        (is_ok_iteration_num || is_local_optimal_solution_oscillation) && is_ok_score;
    ConvergenceVerdict {
        valid_param_type,
        is_ok_iteration_num,
        is_local_optimal_solution_oscillation,
        is_ok_score,
        is_converged,
        score,
        score_threshold,
    }
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

/// C ABI mirror of [`ConvergenceVerdict`] (same field order). `bool` is a 1-byte, C-ABI-stable type.
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

#[cfg(test)]
#[allow(
    unsafe_code,
    clippy::borrow_as_ptr,
    clippy::float_cmp,
    clippy::allow_attributes,
    reason = "test code: invoking the C ABI entry with a mock host; exact-equal float asserts on deterministic passthrough"
)]
mod tests {
    use super::*;
    use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

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
    fn converges_when_iterations_ok_and_score_ok() {
        let v = evaluate_convergence(&base());
        assert!(v.valid_param_type);
        assert!(v.is_ok_iteration_num);
        assert!(!v.is_local_optimal_solution_oscillation);
        assert!(v.is_ok_score);
        assert!(v.is_converged);
        assert_eq!(v.score, 3.0);
        assert_eq!(v.score_threshold, 2.0);
    }

    #[test]
    fn not_converged_when_score_below_threshold() {
        // iterations fine, but TP=1.0 is NOT > 2.0 → fails the score gate.
        let v = evaluate_convergence(&ConvergenceInput {
            transform_probability: 1.0,
            ..base()
        });
        assert!(v.is_ok_iteration_num);
        assert!(!v.is_ok_score);
        assert!(!v.is_converged);
    }

    #[test]
    fn iteration_limit_alone_blocks_convergence() {
        // Hit the iteration cap (30 == 30, not <), no oscillation override → not converged
        // even though the score is fine.
        let v = evaluate_convergence(&ConvergenceInput {
            iteration_num: 30,
            ..base()
        });
        assert!(!v.is_ok_iteration_num);
        assert!(!v.is_local_optimal_solution_oscillation);
        assert!(v.is_ok_score);
        assert!(!v.is_converged);
    }

    #[test]
    fn oscillation_overrides_iteration_limit() {
        // Iteration cap reached, but oscillation_num 11 > 10 lets a good score still converge.
        let v = evaluate_convergence(&ConvergenceInput {
            iteration_num: 30,
            oscillation_num: 11,
            ..base()
        });
        assert!(!v.is_ok_iteration_num);
        assert!(v.is_local_optimal_solution_oscillation);
        assert!(v.is_ok_score);
        assert!(v.is_converged);
    }

    #[test]
    fn oscillation_boundary_is_strictly_greater_than_ten() {
        // 10 is NOT > 10 (so with the iteration cap hit, no convergence)...
        let at = evaluate_convergence(&ConvergenceInput {
            iteration_num: 30,
            oscillation_num: 10,
            ..base()
        });
        assert!(!at.is_local_optimal_solution_oscillation);
        assert!(!at.is_converged);
        // ...11 IS > 10.
        let over = evaluate_convergence(&ConvergenceInput {
            iteration_num: 30,
            oscillation_num: 11,
            ..base()
        });
        assert!(over.is_local_optimal_solution_oscillation);
        assert!(over.is_converged);
    }

    #[test]
    fn nvtl_param_type_selects_nvtl_score_and_threshold() {
        // converged_param_type = NVTL → score is the nvtl field, gated by the nvtl threshold;
        // the TP fields are ignored. nvtl=5.0 > 4.0 converges; the TP value would be irrelevant.
        let v = evaluate_convergence(&ConvergenceInput {
            converged_param_type: NVTL,
            transform_probability: 0.0,
            converged_param_transform_probability: 100.0,
            ..base()
        });
        assert!(v.valid_param_type);
        assert_eq!(v.score, 5.0);
        assert_eq!(v.score_threshold, 4.0);
        assert!(v.is_ok_score);
        assert!(v.is_converged);
    }

    #[test]
    fn unknown_param_type_is_invalid() {
        let v = evaluate_convergence(&ConvergenceInput {
            converged_param_type: 2,
            ..base()
        });
        assert!(!v.valid_param_type);
        // The two flags computed before the dispatch are still populated; the rest stay false.
        assert!(v.is_ok_iteration_num);
        assert!(!v.is_local_optimal_solution_oscillation);
        assert!(!v.is_ok_score);
        assert!(!v.is_converged);
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

    // The mock host records calls in process-global state, so the two trigger sequences below run in
    // ONE sequential test (parallel tests would race the flags).
    static ACTIVATED: AtomicBool = AtomicBool::new(false);
    static CLEARS: AtomicU32 = AtomicU32::new(0);

    extern "C" fn rec_set_activated(_ctx: *mut c_void, activate: bool) {
        ACTIVATED.store(activate, Ordering::SeqCst);
    }
    extern "C" fn rec_clear(_ctx: *mut c_void) {
        CLEARS.fetch_add(1, Ordering::SeqCst);
    }

    #[test]
    fn on_trigger_sets_flag_and_clears_only_on_activate() {
        let host = NdtHost {
            ctx: core::ptr::null_mut(),
            set_activated: rec_set_activated,
            clear_initial_pose_buffer: rec_clear,
        };

        // activate: flag set true, buffer cleared once.
        // SAFETY: `host` is a valid NdtHost living for the call.
        unsafe { autoware_ndt_scan_matcher_rs_node_on_trigger(&host, true) };
        assert!(ACTIVATED.load(Ordering::SeqCst));
        assert_eq!(CLEARS.load(Ordering::SeqCst), 1);

        // deactivate: flag set false, buffer NOT cleared again.
        // SAFETY: as above.
        unsafe { autoware_ndt_scan_matcher_rs_node_on_trigger(&host, false) };
        assert!(!ACTIVATED.load(Ordering::SeqCst));
        assert_eq!(CLEARS.load(Ordering::SeqCst), 1);

        // null host is a no-op (no panic).
        // SAFETY: passing null is explicitly handled.
        unsafe { autoware_ndt_scan_matcher_rs_node_on_trigger(core::ptr::null(), true) };
        assert_eq!(CLEARS.load(Ordering::SeqCst), 1);
    }
}
