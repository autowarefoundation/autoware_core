// Copyright 2015-2019 Autoware Foundation
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

#ifndef SCAN_MATCH_EVALUATION_HPP_
#define SCAN_MATCH_EVALUATION_HPP_

#include <autoware/ndt_scan_matcher/diagnostics_report.hpp>
#include <autoware/ndt_scan_matcher/hyper_parameters.hpp>
#include <autoware/ndt_scan_matcher/ndt_omp/ndt_struct.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <optional>
#include <vector>

namespace autoware::ndt_scan_matcher
{

/** \brief Verdict of evaluating one NDT alignment result against the convergence criteria. */
struct ScanMatchEvaluation
{
  /// Whether the alignment is accepted as converged, and so whether the resulting pose is
  /// published as a valid estimate.
  bool is_converged{false};
  /// Whether the score alone cleared its threshold. Reported separately because the node logs a
  /// throttled warning for it; deriving it again at the call site would duplicate the decision.
  bool is_ok_score{false};
  /// The score the verdict was made on, selected by `converged_param_type`, and the threshold it
  /// was compared against. The node includes both in that warning.
  double score{0.0};
  double score_threshold{0.0};
};

/**
 * \brief Decide whether an NDT alignment result is good enough to publish.
 *
 * Pure decision logic: no ROS node, clock, TF or publisher is involved, so this is unit-testable
 * without spinning a node. Diagnostics are accumulated into \p diagnostics in the same order the
 * node previously added them, which keeps the published `/diagnostics` content unchanged.
 *
 * \param ndt_result            the alignment result to judge.
 * \param max_iterations        the NDT's iteration limit, i.e. `getMaximumIterations()`.
 * \param transformation_array  the per-iteration poses, used to detect oscillation around a local
 *                              minimum. The caller already builds these for the marker publish.
 * \param param                 the score-estimation parameters selecting score type and threshold.
 * \param diagnostics           accumulates the key/values and status for this evaluation.
 *
 * \return the verdict, or std::nullopt when `param.converged_param_type` holds an unknown value.
 *         In that case an ERROR is recorded in \p diagnostics and the caller must abort the
 *         scan-matching callback without publishing a pose.
 */
std::optional<ScanMatchEvaluation> evaluate_scan_match_result(
  const pclomp::NdtResult & ndt_result, int max_iterations,
  const std::vector<geometry_msgs::msg::Pose> & transformation_array,
  const HyperParameters::ScoreEstimation & param, DiagnosticsReport & diagnostics);

}  // namespace autoware::ndt_scan_matcher

#endif  // SCAN_MATCH_EVALUATION_HPP_
