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

//! The NDT convergence decision — pure, `no_std`, allocation-free.
//!
//! Ported verbatim from `callback_sensor_points_main`. Lives here (not in the std-gated
//! [`crate::node`] FFI module) so the portable [`crate::scan_matcher::ScanMatcher`] can produce the
//! same verdict on bare-metal. The ROS side reuses it via the `Aw*` `#[repr(C)]` mirrors + the
//! `extern "C"` shim in [`crate::node`].

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

#[cfg(test)]
// exact-equal float asserts on deterministic passthrough of the score/threshold inputs.
#[allow(clippy::float_cmp, clippy::allow_attributes, reason = "test code")]
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
}
