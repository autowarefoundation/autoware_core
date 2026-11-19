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

#include "scan_match_evaluation.hpp"

#include "ndt_scan_matcher_helper.hpp"

#include <cstdint>
#include <optional>
#include <sstream>
#include <vector>

namespace autoware::ndt_scan_matcher
{

std::optional<ScanMatchEvaluation> evaluate_scan_match_result(
  const pclomp::NdtResult & ndt_result, const int max_iterations,
  const std::vector<geometry_msgs::msg::Pose> & transformation_array,
  const HyperParameters::ScoreEstimation & param, DiagnosticsReport & diagnostics)
{
  // check iteration_num
  diagnostics.add_key_value({"iteration_num", static_cast<int64_t>(ndt_result.iteration_num)});
  const bool is_ok_iteration_num = (ndt_result.iteration_num < max_iterations);
  if (!is_ok_iteration_num) {
    std::stringstream message;
    message << "The number of iterations has reached its upper limit. The number of iterations: "
            << ndt_result.iteration_num << ", Limit: " << max_iterations << ".";
    diagnostics.update_level_and_message(DiagnosticLevel::WARN, message.str());
  }

  // check local_optimal_solution_oscillation_num
  constexpr int oscillation_num_threshold = 10;
  const int oscillation_num = count_oscillation(transformation_array);
  diagnostics.add_key_value(
    {"local_optimal_solution_oscillation_num", static_cast<int64_t>(oscillation_num)});
  const bool is_local_optimal_solution_oscillation = (oscillation_num > oscillation_num_threshold);
  if (is_local_optimal_solution_oscillation) {
    std::stringstream message;
    message << "There is a possibility of oscillation in a local minimum";
    diagnostics.update_level_and_message(DiagnosticLevel::WARN, message.str());
  }

  // check score
  diagnostics.add_key_value(
    {"transform_probability", static_cast<double>(ndt_result.transform_probability)});
  diagnostics.add_key_value(
    {"nearest_voxel_transformation_likelihood",
     static_cast<double>(ndt_result.nearest_voxel_transformation_likelihood)});
  double score = 0.0;
  double score_threshold = 0.0;
  if (param.converged_param_type == ConvergedParamType::TRANSFORM_PROBABILITY) {
    score = ndt_result.transform_probability;
    score_threshold = param.converged_param_transform_probability;
  } else if (
    param.converged_param_type == ConvergedParamType::NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD) {
    score = ndt_result.nearest_voxel_transformation_likelihood;
    score_threshold = param.converged_param_nearest_voxel_transformation_likelihood;
  } else {
    std::stringstream message;
    message << "Unknown converged param type. Please check `score_estimation.converged_param_type`";
    diagnostics.update_level_and_message(DiagnosticLevel::ERROR, message.str());
    return std::nullopt;
  }

  // check score diff
  const std::vector<float> & tp_array = ndt_result.transform_probability_array;
  if (static_cast<int>(tp_array.size()) != ndt_result.iteration_num + 1) {
    // only publish warning to /diagnostics, not skip publishing pose
    std::stringstream message;
    message << "transform_probability_array size is not equal to iteration_num + 1."
            << " transform_probability_array size: " << tp_array.size()
            << ", iteration_num: " << ndt_result.iteration_num;
    diagnostics.update_level_and_message(DiagnosticLevel::WARN, message.str());
  } else {
    const float diff = tp_array.back() - tp_array.front();
    diagnostics.add_key_value({"transform_probability_diff", static_cast<double>(diff)});
    diagnostics.add_key_value(
      {"transform_probability_before", static_cast<double>(tp_array.front())});
  }
  const std::vector<float> & nvtl_array = ndt_result.nearest_voxel_transformation_likelihood_array;
  if (static_cast<int>(nvtl_array.size()) != ndt_result.iteration_num + 1) {
    // only publish warning to /diagnostics, not skip publishing pose
    std::stringstream message;
    message
      << "nearest_voxel_transformation_likelihood_array size is not equal to iteration_num + 1."
      << " nearest_voxel_transformation_likelihood_array size: " << nvtl_array.size()
      << ", iteration_num: " << ndt_result.iteration_num;
    diagnostics.update_level_and_message(DiagnosticLevel::WARN, message.str());
  } else {
    const float diff = nvtl_array.back() - nvtl_array.front();
    diagnostics.add_key_value(
      {"nearest_voxel_transformation_likelihood_diff", static_cast<double>(diff)});
    diagnostics.add_key_value(
      {"nearest_voxel_transformation_likelihood_before", static_cast<double>(nvtl_array.front())});
  }

  ScanMatchEvaluation evaluation;
  evaluation.score = score;
  evaluation.score_threshold = score_threshold;
  evaluation.is_ok_score = (score > score_threshold);
  if (!evaluation.is_ok_score) {
    std::stringstream message;
    message << "Score is below the threshold. Score: " << score
            << ", Threshold: " << score_threshold;
    diagnostics.update_level_and_message(DiagnosticLevel::WARN, message.str());
  }

  // check is_converged
  evaluation.is_converged =
    (is_ok_iteration_num || is_local_optimal_solution_oscillation) && evaluation.is_ok_score;

  return evaluation;
}

}  // namespace autoware::ndt_scan_matcher
