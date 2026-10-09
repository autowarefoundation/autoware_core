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

#include "../src/scan_match_evaluation.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <variant>
#include <vector>

namespace autoware::ndt_scan_matcher
{
namespace
{

/// A pose sequence that walks in a straight line, so `count_oscillation` reports zero.
std::vector<geometry_msgs::msg::Pose> make_monotonic_poses(const std::size_t num_poses)
{
  std::vector<geometry_msgs::msg::Pose> poses(num_poses);
  for (std::size_t i = 0; i < num_poses; ++i) {
    poses[i].position.x = static_cast<double>(i);
  }
  return poses;
}

/// A pose sequence that reverses direction on every step, so every step from the third on counts
/// as an inversion. `num_poses` poses therefore yield `num_poses - 2` consecutive oscillations.
std::vector<geometry_msgs::msg::Pose> make_oscillating_poses(const std::size_t num_poses)
{
  std::vector<geometry_msgs::msg::Pose> poses(num_poses);
  for (std::size_t i = 0; i < num_poses; ++i) {
    poses[i].position.x = (i % 2 == 0) ? 0.0 : 1.0;
  }
  return poses;
}

/// An NDT result whose score arrays are consistent with `iteration_num`, i.e. the shape the NDT
/// produces when nothing has gone wrong.
pclomp::NdtResult make_ndt_result(
  const int iteration_num, const float transform_probability,
  const float nearest_voxel_transformation_likelihood)
{
  pclomp::NdtResult result{};
  result.iteration_num = iteration_num;
  result.transform_probability = transform_probability;
  result.nearest_voxel_transformation_likelihood = nearest_voxel_transformation_likelihood;
  // Both arrays hold one entry per iteration plus the initial value. Ramp them so that
  // back() - front() is a distinctive, easily asserted number.
  result.transform_probability_array.resize(iteration_num + 1);
  result.nearest_voxel_transformation_likelihood_array.resize(iteration_num + 1);
  for (int i = 0; i <= iteration_num; ++i) {
    result.transform_probability_array[i] = 1.0f + static_cast<float>(i);
    result.nearest_voxel_transformation_likelihood_array[i] = 2.0f + static_cast<float>(i);
  }
  return result;
}

HyperParameters::ScoreEstimation make_param(
  const ConvergedParamType type, const double tp_threshold, const double nvtl_threshold)
{
  HyperParameters::ScoreEstimation param{};
  param.converged_param_type = type;
  param.converged_param_transform_probability = tp_threshold;
  param.converged_param_nearest_voxel_transformation_likelihood = nvtl_threshold;
  return param;
}

std::vector<std::string> keys_of(const DiagnosticsReport & report)
{
  std::vector<std::string> keys;
  keys.reserve(report.key_values.size());
  for (const auto & key_value : report.key_values) {
    keys.push_back(key_value.key);
  }
  return keys;
}

bool has_key(const DiagnosticsReport & report, const std::string & key)
{
  const auto keys = keys_of(report);
  return std::find(keys.begin(), keys.end(), key) != keys.end();
}

double double_value_of(const DiagnosticsReport & report, const std::string & key)
{
  for (const auto & key_value : report.key_values) {
    if (key_value.key == key) {
      return std::get<double>(key_value.value);
    }
  }
  ADD_FAILURE() << "no such key: " << key;
  return 0.0;
}

int64_t int_value_of(const DiagnosticsReport & report, const std::string & key)
{
  for (const auto & key_value : report.key_values) {
    if (key_value.key == key) {
      return std::get<int64_t>(key_value.value);
    }
  }
  ADD_FAILURE() << "no such key: " << key;
  return 0;
}

}  // namespace

// The happy path: the solver stopped before the iteration limit and the score cleared its
// threshold, so the pose is published. The key set and its order are asserted here because the
// characterization suite pins exactly this sequence on /diagnostics.
TEST(ScanMatchEvaluation, ConvergesWhenIterationsAndScoreAreOk)  // NOLINT
{
  DiagnosticsReport report;
  const auto evaluation = evaluate_scan_match_result(
    make_ndt_result(5, 4.0f, 3.0f), 30, make_monotonic_poses(6),
    make_param(ConvergedParamType::TRANSFORM_PROBABILITY, 1.0, 1.0), report);

  ASSERT_TRUE(evaluation.has_value());
  EXPECT_TRUE(evaluation->is_converged);
  EXPECT_TRUE(evaluation->is_ok_score);
  EXPECT_DOUBLE_EQ(evaluation->score, 4.0);
  EXPECT_DOUBLE_EQ(evaluation->score_threshold, 1.0);

  EXPECT_EQ(report.level, DiagnosticLevel::OK);
  EXPECT_TRUE(report.message.empty());
  EXPECT_EQ(
    keys_of(report),
    (std::vector<std::string>{
      "iteration_num", "local_optimal_solution_oscillation_num", "transform_probability",
      "nearest_voxel_transformation_likelihood", "transform_probability_diff",
      "transform_probability_before", "nearest_voxel_transformation_likelihood_diff",
      "nearest_voxel_transformation_likelihood_before"}));

  EXPECT_EQ(int_value_of(report, "iteration_num"), 5);
  EXPECT_EQ(int_value_of(report, "local_optimal_solution_oscillation_num"), 0);
  EXPECT_DOUBLE_EQ(double_value_of(report, "transform_probability"), 4.0);
  // The arrays ramp by 1.0 per iteration, so the diff over 5 iterations is 5.0.
  EXPECT_DOUBLE_EQ(double_value_of(report, "transform_probability_diff"), 5.0);
  EXPECT_DOUBLE_EQ(double_value_of(report, "transform_probability_before"), 1.0);
  EXPECT_DOUBLE_EQ(double_value_of(report, "nearest_voxel_transformation_likelihood_before"), 2.0);
}

// A score below its threshold vetoes convergence on its own, however well the solver behaved.
TEST(ScanMatchEvaluation, DoesNotConvergeWhenScoreIsBelowThreshold)  // NOLINT
{
  DiagnosticsReport report;
  const auto evaluation = evaluate_scan_match_result(
    make_ndt_result(5, 0.5f, 3.0f), 30, make_monotonic_poses(6),
    make_param(ConvergedParamType::TRANSFORM_PROBABILITY, 1.0, 1.0), report);

  ASSERT_TRUE(evaluation.has_value());
  EXPECT_FALSE(evaluation->is_converged);
  EXPECT_FALSE(evaluation->is_ok_score);
  EXPECT_EQ(report.level, DiagnosticLevel::WARN);
  EXPECT_NE(report.message.find("Score is below the threshold."), std::string::npos);
}

// Frozen behavior worth stating explicitly: hitting the iteration limit does NOT by itself block
// convergence when the solver was oscillating, because the two are combined with `||`. Oscillation
// is treated as "it was never going to settle", so the limit is excused.
TEST(ScanMatchEvaluation, ConvergesAtIterationLimitWhenOscillating)  // NOLINT
{
  DiagnosticsReport report;
  // 14 poses give 12 consecutive inversions, above the threshold of 10.
  const auto evaluation = evaluate_scan_match_result(
    make_ndt_result(30, 4.0f, 3.0f), 30, make_oscillating_poses(14),
    make_param(ConvergedParamType::TRANSFORM_PROBABILITY, 1.0, 1.0), report);

  ASSERT_TRUE(evaluation.has_value());
  EXPECT_EQ(int_value_of(report, "local_optimal_solution_oscillation_num"), 12);
  EXPECT_TRUE(evaluation->is_converged);

  // Both conditions are still reported, in the order they are checked.
  EXPECT_EQ(report.level, DiagnosticLevel::WARN);
  const auto limit_pos =
    report.message.find("The number of iterations has reached its upper limit");
  const auto oscillation_pos = report.message.find("possibility of oscillation in a local minimum");
  ASSERT_NE(limit_pos, std::string::npos);
  ASSERT_NE(oscillation_pos, std::string::npos);
  EXPECT_LT(limit_pos, oscillation_pos);
}

// Without oscillation, the same iteration limit does block convergence.
TEST(ScanMatchEvaluation, DoesNotConvergeAtIterationLimitWithoutOscillation)  // NOLINT
{
  DiagnosticsReport report;
  const auto evaluation = evaluate_scan_match_result(
    make_ndt_result(30, 4.0f, 3.0f), 30, make_monotonic_poses(31),
    make_param(ConvergedParamType::TRANSFORM_PROBABILITY, 1.0, 1.0), report);

  ASSERT_TRUE(evaluation.has_value());
  EXPECT_EQ(int_value_of(report, "local_optimal_solution_oscillation_num"), 0);
  EXPECT_FALSE(evaluation->is_converged);
  EXPECT_TRUE(evaluation->is_ok_score);
}

// `converged_param_type` selects which score is judged, and against which threshold.
TEST(ScanMatchEvaluation, SelectsNearestVoxelLikelihoodWhenConfigured)  // NOLINT
{
  DiagnosticsReport report;
  // The transform probability would fail its threshold; the likelihood passes its own. Only the
  // configured one may be consulted.
  const auto evaluation = evaluate_scan_match_result(
    make_ndt_result(5, 0.1f, 3.0f), 30, make_monotonic_poses(6),
    make_param(ConvergedParamType::NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD, 1.0, 2.0), report);

  ASSERT_TRUE(evaluation.has_value());
  EXPECT_DOUBLE_EQ(evaluation->score, 3.0);
  EXPECT_DOUBLE_EQ(evaluation->score_threshold, 2.0);
  EXPECT_TRUE(evaluation->is_converged);
}

// A `converged_param_type` outside the enum aborts the evaluation: there is no score to judge, so
// no verdict is returned and the caller must skip publishing. The keys gathered before the abort
// are still reported.
TEST(ScanMatchEvaluation, ReturnsNulloptOnUnknownConvergedParamType)  // NOLINT
{
  DiagnosticsReport report;
  const auto evaluation = evaluate_scan_match_result(
    make_ndt_result(5, 4.0f, 3.0f), 30, make_monotonic_poses(6),
    make_param(static_cast<ConvergedParamType>(99), 1.0, 1.0), report);

  EXPECT_FALSE(evaluation.has_value());
  EXPECT_EQ(report.level, DiagnosticLevel::ERROR);
  EXPECT_NE(report.message.find("Unknown converged param type."), std::string::npos);
  EXPECT_EQ(
    keys_of(report), (std::vector<std::string>{
                       "iteration_num", "local_optimal_solution_oscillation_num",
                       "transform_probability", "nearest_voxel_transformation_likelihood"}));
}

// A score array out of step with iteration_num is reported but does not veto the pose: the diff
// keys are simply omitted. Both arrays are checked independently, in order.
TEST(ScanMatchEvaluation, WarnsButStillConvergesWhenScoreArraySizesMismatch)  // NOLINT
{
  auto ndt_result = make_ndt_result(5, 4.0f, 3.0f);
  ndt_result.transform_probability_array.pop_back();
  ndt_result.nearest_voxel_transformation_likelihood_array.pop_back();

  DiagnosticsReport report;
  const auto evaluation = evaluate_scan_match_result(
    ndt_result, 30, make_monotonic_poses(6),
    make_param(ConvergedParamType::TRANSFORM_PROBABILITY, 1.0, 1.0), report);

  ASSERT_TRUE(evaluation.has_value());
  EXPECT_TRUE(evaluation->is_converged);
  EXPECT_EQ(report.level, DiagnosticLevel::WARN);

  EXPECT_FALSE(has_key(report, "transform_probability_diff"));
  EXPECT_FALSE(has_key(report, "transform_probability_before"));
  EXPECT_FALSE(has_key(report, "nearest_voxel_transformation_likelihood_diff"));
  EXPECT_FALSE(has_key(report, "nearest_voxel_transformation_likelihood_before"));

  const auto tp_pos = report.message.find("transform_probability_array size is not equal");
  const auto nvtl_pos =
    report.message.find("nearest_voxel_transformation_likelihood_array size is not equal");
  ASSERT_NE(tp_pos, std::string::npos);
  ASSERT_NE(nvtl_pos, std::string::npos);
  EXPECT_LT(tp_pos, nvtl_pos);
}

}  // namespace autoware::ndt_scan_matcher
