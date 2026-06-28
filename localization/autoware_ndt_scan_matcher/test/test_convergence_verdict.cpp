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

// Differential test (node port N1): the NDT convergence decision migrated to Rust
// (autoware_ndt_scan_matcher_rs_node_evaluate_convergence) must match the original C++ logic from
// callback_sensor_points_main bit-for-bit. The logic is pure integer/double comparisons, so the
// verdict (flags + selected score/threshold) is exactly equal -- compared with EXPECT_EQ, not a
// tolerance. The C++ reference below is a verbatim transcription of the callback's gate logic.

#include <autoware/ndt_scan_matcher/hyper_parameters.hpp>

#include "autoware_ndt_scan_matcher_rs.h"

#include <gtest/gtest.h>

#include <vector>

namespace
{
using autoware::ndt_scan_matcher::ConvergedParamType;

// The Rust FFI encodes converged_param_type as a plain int; pin that mapping to the C++ enum.
static_assert(static_cast<int>(ConvergedParamType::TRANSFORM_PROBABILITY) == 0);
static_assert(static_cast<int>(ConvergedParamType::NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD) == 1);

// Verbatim transcription of the C++ convergence gate from callback_sensor_points_main. Returns the
// same fields the Rust verdict carries. `valid_param_type == false` mirrors the "unknown type ->
// ERROR + return false" branch (the callback aborts; the remaining fields are then unused).
AwConvergenceVerdict cpp_reference(const AwConvergenceInput & in)
{
  AwConvergenceVerdict v{};
  constexpr int oscillation_num_threshold = 10;
  v.is_ok_iteration_num = (in.iteration_num < in.max_iterations);
  v.is_local_optimal_solution_oscillation = (in.oscillation_num > oscillation_num_threshold);
  if (in.converged_param_type == static_cast<int>(ConvergedParamType::TRANSFORM_PROBABILITY)) {
    v.valid_param_type = true;
    v.score = in.transform_probability;
    v.score_threshold = in.converged_param_transform_probability;
  } else if (
    in.converged_param_type ==
    static_cast<int>(ConvergedParamType::NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD)) {
    v.valid_param_type = true;
    v.score = in.nearest_voxel_transformation_likelihood;
    v.score_threshold = in.converged_param_nearest_voxel_transformation_likelihood;
  } else {
    v.valid_param_type = false;
    return v;
  }
  v.is_ok_score = (v.score > v.score_threshold);
  v.is_converged =
    (v.is_ok_iteration_num || v.is_local_optimal_solution_oscillation) && v.is_ok_score;
  return v;
}

void expect_match(const AwConvergenceInput & in)
{
  const AwConvergenceVerdict expected = cpp_reference(in);
  AwConvergenceVerdict got{};
  autoware_ndt_scan_matcher_rs_node_evaluate_convergence(&in, &got);
  EXPECT_EQ(got.valid_param_type, expected.valid_param_type);
  EXPECT_EQ(got.is_ok_iteration_num, expected.is_ok_iteration_num);
  EXPECT_EQ(
    got.is_local_optimal_solution_oscillation, expected.is_local_optimal_solution_oscillation);
  EXPECT_EQ(got.is_ok_score, expected.is_ok_score);
  EXPECT_EQ(got.is_converged, expected.is_converged);
  EXPECT_EQ(got.score, expected.score);
  EXPECT_EQ(got.score_threshold, expected.score_threshold);
}

constexpr int kTp = 0;    // ConvergedParamType::TRANSFORM_PROBABILITY
constexpr int kNvtl = 1;  // ConvergedParamType::NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD
}  // namespace

// A grid over both param types, the iteration / oscillation / score gates, and an invalid type.
TEST(ConvergenceVerdict, MatchesCppAcrossGrid)  // NOLINT
{
  const int it_cases[] = {5, 30, 31};                   // below / at / above the cap of 30
  const int osc_cases[] = {0, 10, 11};                  // below / at / above the threshold of 10
  const int type_cases[] = {kTp, kNvtl, 2};             // TP / NVTL / unknown
  const double tp_cases[] = {1.0, 3.0};                 // vs the 2.0 TP threshold
  const double nvtl_cases[] = {3.0, 5.0};               // vs the 4.0 NVTL threshold

  for (const int it : it_cases) {
    for (const int osc : osc_cases) {
      for (const int type : type_cases) {
        for (const double tp : tp_cases) {
          for (const double nvtl : nvtl_cases) {
            AwConvergenceInput in{};
            in.iteration_num = it;
            in.max_iterations = 30;
            in.oscillation_num = osc;
            in.transform_probability = tp;
            in.nearest_voxel_transformation_likelihood = nvtl;
            in.converged_param_type = type;
            in.converged_param_transform_probability = 2.0;
            in.converged_param_nearest_voxel_transformation_likelihood = 4.0;
            expect_match(in);
          }
        }
      }
    }
  }
}

// Float -> double widening of the f32 scores is exact, so the verdict still matches at fractional
// values straddling the threshold.
TEST(ConvergenceVerdict, MatchesCppAtFractionalScores)  // NOLINT
{
  AwConvergenceInput in{};
  in.iteration_num = 5;
  in.max_iterations = 30;
  in.oscillation_num = 0;
  in.converged_param_type = kTp;
  in.converged_param_transform_probability = 2.5;
  in.converged_param_nearest_voxel_transformation_likelihood = 4.0;
  for (const float tp : {2.4999F, 2.5F, 2.5001F}) {
    in.transform_probability = static_cast<double>(tp);
    in.nearest_voxel_transformation_likelihood = 0.0;
    expect_match(in);
  }
}

// A null output pointer must be a no-op (no crash, no write).
TEST(ConvergenceVerdict, NullOutputIsNoop)  // NOLINT
{
  AwConvergenceInput in{};
  in.max_iterations = 30;
  in.converged_param_type = kTp;
  autoware_ndt_scan_matcher_rs_node_evaluate_convergence(&in, nullptr);
  autoware_ndt_scan_matcher_rs_node_evaluate_convergence(nullptr, nullptr);
  SUCCEED();
}
