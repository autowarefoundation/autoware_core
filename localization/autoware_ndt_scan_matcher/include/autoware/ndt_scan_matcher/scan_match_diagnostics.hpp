// Copyright 2026 Autoware Foundation
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

#ifndef AUTOWARE__NDT_SCAN_MATCHER__SCAN_MATCH_DIAGNOSTICS_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__SCAN_MATCH_DIAGNOSTICS_HPP_

#include "diagnostics_report.hpp"

#include <vector>

namespace autoware::ndt_scan_matcher
{

struct ScanMatchDiagnosticsConfig
{
  double initial_to_result_distance_tolerance_m;
  double critical_upper_bound_exe_time_ms;
};

// Values observed while matching one scan. The convergence verdicts (is_ok_*) are decided by the
// scan matching itself and are only reported here, never recomputed.
struct ScanMatchDiagnosticValues
{
  int iteration_num{};
  int maximum_iterations{};
  bool is_ok_iteration_num{};
  int oscillation_num{};
  bool is_local_optimal_solution_oscillation{};
  float transform_probability{};
  float nearest_voxel_transformation_likelihood{};
  std::vector<float> transform_probability_array;
  std::vector<float> nearest_voxel_transformation_likelihood_array;
  // When false, the scan was rejected at this point and the fields below are not reported.
  bool is_known_converged_param_type{true};
  double score{};
  double score_threshold{};
  bool is_ok_score{};
  double distance_initial_to_result{};
  float execution_time_ms{};
};

// Builds the scan matching part of the `scan_matching_status` diagnostics from the observed values.
class ScanMatchDiagnostics
{
public:
  explicit ScanMatchDiagnostics(const ScanMatchDiagnosticsConfig & config);
  [[nodiscard]] DiagnosticsReport make_report(const ScanMatchDiagnosticValues & values) const;

private:
  ScanMatchDiagnosticsConfig config_;
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__SCAN_MATCH_DIAGNOSTICS_HPP_
