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

#include "autoware/ndt_scan_matcher/scan_match_diagnostics.hpp"

#include <gtest/gtest.h>

#include <string>
#include <variant>
#include <vector>

namespace
{
using autoware::ndt_scan_matcher::DiagnosticLevel;
using autoware::ndt_scan_matcher::DiagnosticsReport;
using autoware::ndt_scan_matcher::ScanMatchDiagnosticValues;
using autoware::ndt_scan_matcher::ScanMatchDiagnostics;
using autoware::ndt_scan_matcher::ScanMatchDiagnosticsConfig;

ScanMatchDiagnosticsConfig make_default_config()
{
  ScanMatchDiagnosticsConfig config{};
  config.initial_to_result_distance_tolerance_m = 3.0;
  config.critical_upper_bound_exe_time_ms = 100.0;
  return config;
}

// A converged scan match that passes every check.
ScanMatchDiagnosticValues make_healthy_values()
{
  ScanMatchDiagnosticValues values;
  values.iteration_num = 3;
  values.maximum_iterations = 30;
  values.is_ok_iteration_num = true;
  values.oscillation_num = 0;
  values.is_local_optimal_solution_oscillation = false;
  values.transform_probability = 3.0f;
  values.nearest_voxel_transformation_likelihood = 2.5f;
  values.transform_probability_array = {2.0f, 2.5f, 2.75f, 3.0f};
  values.nearest_voxel_transformation_likelihood_array = {1.5f, 2.0f, 2.25f, 2.5f};
  values.is_known_converged_param_type = true;
  values.score = 2.5;
  values.score_threshold = 2.3;
  values.is_ok_score = true;
  values.distance_initial_to_result = 0.1;
  values.execution_time_ms = 10.0f;
  return values;
}

std::vector<std::string> keys_of(const DiagnosticsReport & report)
{
  std::vector<std::string> keys;
  for (const auto & key_value : report.key_values) {
    keys.push_back(key_value.key);
  }
  return keys;
}

double double_value_of(const DiagnosticsReport & report, const std::string & key)
{
  for (const auto & key_value : report.key_values) {
    if (key_value.key == key) {
      return std::get<double>(key_value.value);
    }
  }
  ADD_FAILURE() << "key not found: " << key;
  return 0.0;
}
}  // namespace

TEST(ScanMatchDiagnostics, ReportsOkWhenAllChecksPass)  // NOLINT
{
  const ScanMatchDiagnostics diagnostics(make_default_config());

  const DiagnosticsReport report = diagnostics.make_report(make_healthy_values());

  EXPECT_EQ(report.level, DiagnosticLevel::OK);
  EXPECT_TRUE(report.message.empty());
}

TEST(ScanMatchDiagnostics, ReportsAllScanMatchingKeysInOrder)  // NOLINT
{
  const ScanMatchDiagnostics diagnostics(make_default_config());

  const DiagnosticsReport report = diagnostics.make_report(make_healthy_values());

  const std::vector<std::string> expected{
    "iteration_num",
    "local_optimal_solution_oscillation_num",
    "transform_probability",
    "nearest_voxel_transformation_likelihood",
    "transform_probability_diff",
    "transform_probability_before",
    "nearest_voxel_transformation_likelihood_diff",
    "nearest_voxel_transformation_likelihood_before",
    "distance_initial_to_result",
    "execution_time"};
  EXPECT_EQ(keys_of(report), expected);
}

TEST(ScanMatchDiagnostics, WarnsWhenIterationNumReachesLimit)  // NOLINT
{
  const ScanMatchDiagnostics diagnostics(make_default_config());
  auto values = make_healthy_values();
  values.is_ok_iteration_num = false;

  const DiagnosticsReport report = diagnostics.make_report(values);

  EXPECT_EQ(report.level, DiagnosticLevel::WARN);
}

TEST(ScanMatchDiagnostics, WarnsWhenOscillationIsDetected)  // NOLINT
{
  const ScanMatchDiagnostics diagnostics(make_default_config());
  auto values = make_healthy_values();
  values.is_local_optimal_solution_oscillation = true;

  const DiagnosticsReport report = diagnostics.make_report(values);

  EXPECT_EQ(report.level, DiagnosticLevel::WARN);
}

TEST(ScanMatchDiagnostics, WarnsWhenScoreIsBelowThreshold)  // NOLINT
{
  const ScanMatchDiagnostics diagnostics(make_default_config());
  auto values = make_healthy_values();
  values.is_ok_score = false;

  const DiagnosticsReport report = diagnostics.make_report(values);

  EXPECT_EQ(report.level, DiagnosticLevel::WARN);
}

TEST(ScanMatchDiagnostics, ReportsErrorWhenConvergedParamTypeIsUnknown)  // NOLINT
{
  const ScanMatchDiagnostics diagnostics(make_default_config());
  auto values = make_healthy_values();
  values.is_known_converged_param_type = false;

  const DiagnosticsReport report = diagnostics.make_report(values);

  EXPECT_EQ(report.level, DiagnosticLevel::ERROR);
}

TEST(ScanMatchDiagnostics, StopsReportingKeysWhenConvergedParamTypeIsUnknown)  // NOLINT
{
  const ScanMatchDiagnostics diagnostics(make_default_config());
  auto values = make_healthy_values();
  values.is_known_converged_param_type = false;

  const DiagnosticsReport report = diagnostics.make_report(values);

  const std::vector<std::string> expected{
    "iteration_num", "local_optimal_solution_oscillation_num", "transform_probability",
    "nearest_voxel_transformation_likelihood"};
  EXPECT_EQ(keys_of(report), expected);
}

TEST(ScanMatchDiagnostics, ReportsFirstToLastChangeOfTransformProbability)  // NOLINT
{
  const ScanMatchDiagnostics diagnostics(make_default_config());

  const DiagnosticsReport report = diagnostics.make_report(make_healthy_values());

  EXPECT_DOUBLE_EQ(double_value_of(report, "transform_probability_diff"), 1.0);
  EXPECT_DOUBLE_EQ(double_value_of(report, "transform_probability_before"), 2.0);
}

TEST(ScanMatchDiagnostics, WarnsWhenScoreArraySizeDoesNotMatchIterationNum)  // NOLINT
{
  const ScanMatchDiagnostics diagnostics(make_default_config());
  auto values = make_healthy_values();
  values.transform_probability_array = {2.0f, 3.0f};

  const DiagnosticsReport report = diagnostics.make_report(values);

  EXPECT_EQ(report.level, DiagnosticLevel::WARN);
}

TEST(ScanMatchDiagnostics, WarnsWhenDistanceInitialToResultExceedsTolerance)  // NOLINT
{
  const ScanMatchDiagnostics diagnostics(make_default_config());
  auto values = make_healthy_values();
  values.distance_initial_to_result = 3.5;

  const DiagnosticsReport report = diagnostics.make_report(values);

  EXPECT_EQ(report.level, DiagnosticLevel::WARN);
}

TEST(ScanMatchDiagnostics, ReportsOkWhenDistanceInitialToResultEqualsTolerance)  // NOLINT
{
  const ScanMatchDiagnostics diagnostics(make_default_config());
  auto values = make_healthy_values();
  values.distance_initial_to_result = 3.0;

  const DiagnosticsReport report = diagnostics.make_report(values);

  EXPECT_EQ(report.level, DiagnosticLevel::OK);
}

TEST(ScanMatchDiagnostics, WarnsWhenExecutionTimeExceedsUpperBound)  // NOLINT
{
  const ScanMatchDiagnostics diagnostics(make_default_config());
  auto values = make_healthy_values();
  values.execution_time_ms = 150.0f;

  const DiagnosticsReport report = diagnostics.make_report(values);

  EXPECT_EQ(report.level, DiagnosticLevel::WARN);
}
