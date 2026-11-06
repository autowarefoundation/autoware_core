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

#include <autoware/ndt_scan_matcher/scan_match_diagnostics.hpp>

#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::ndt_scan_matcher
{
namespace
{
// Reports the first-to-last change of a per-iteration score array, or warns when the array does
// not hold one entry per iteration plus the initial one.
void report_score_array(
  const std::string & name, const std::vector<float> & array, const int iteration_num,
  DiagnosticsReport & report)
{
  if (static_cast<int>(array.size()) != iteration_num + 1) {
    // only publish warning to /diagnostics, not skip publishing pose
    std::stringstream message;
    message << name << "_array size is not equal to iteration_num + 1."
            << " " << name << "_array size: " << array.size()
            << ", iteration_num: " << iteration_num;
    report.update_level_and_message(DiagnosticLevel::WARN, message.str());
    return;
  }
  const float diff = array.back() - array.front();
  report.add_key_value({name + "_diff", static_cast<double>(diff)});
  report.add_key_value({name + "_before", static_cast<double>(array.front())});
}
}  // namespace

ScanMatchDiagnostics::ScanMatchDiagnostics(const ScanMatchDiagnosticsConfig & config)
: config_(config)
{
}

DiagnosticsReport ScanMatchDiagnostics::make_report(const ScanMatchDiagnosticValues & values) const
{
  DiagnosticsReport report;

  report.add_key_value({"iteration_num", static_cast<int64_t>(values.iteration_num)});
  if (!values.is_ok_iteration_num) {
    std::stringstream message;
    message << "The number of iterations has reached its upper limit. The number of iterations: "
            << values.iteration_num << ", Limit: " << values.maximum_iterations << ".";
    report.update_level_and_message(DiagnosticLevel::WARN, message.str());
  }

  report.add_key_value(
    {"local_optimal_solution_oscillation_num", static_cast<int64_t>(values.oscillation_num)});
  if (values.is_local_optimal_solution_oscillation) {
    report.update_level_and_message(
      DiagnosticLevel::WARN, "There is a possibility of oscillation in a local minimum");
  }

  report.add_key_value(
    {"transform_probability", static_cast<double>(values.transform_probability)});
  report.add_key_value(
    {"nearest_voxel_transformation_likelihood",
     static_cast<double>(values.nearest_voxel_transformation_likelihood)});

  if (!values.is_known_converged_param_type) {
    report.update_level_and_message(
      DiagnosticLevel::ERROR,
      "Unknown converged param type. Please check `score_estimation.converged_param_type`");
    return report;
  }

  report_score_array(
    "transform_probability", values.transform_probability_array, values.iteration_num, report);
  report_score_array(
    "nearest_voxel_transformation_likelihood",
    values.nearest_voxel_transformation_likelihood_array, values.iteration_num, report);

  if (!values.is_ok_score) {
    std::stringstream message;
    message << "Score is below the threshold. Score: " << values.score
            << ", Threshold: " << values.score_threshold;
    report.update_level_and_message(DiagnosticLevel::WARN, message.str());
  }

  report.add_key_value({"distance_initial_to_result", values.distance_initial_to_result});
  if (values.distance_initial_to_result > config_.initial_to_result_distance_tolerance_m) {
    std::stringstream message;
    message << "distance_initial_to_result is too large (" << values.distance_initial_to_result
            << " [m]).";
    report.update_level_and_message(DiagnosticLevel::WARN, message.str());
  }

  report.add_key_value({"execution_time", static_cast<double>(values.execution_time_ms)});
  if (values.execution_time_ms > config_.critical_upper_bound_exe_time_ms) {
    std::stringstream message;
    message << "NDT exe time is too long (took " << values.execution_time_ms << " [ms]).";
    report.update_level_and_message(DiagnosticLevel::WARN, message.str());
  }

  return report;
}

}  // namespace autoware::ndt_scan_matcher
