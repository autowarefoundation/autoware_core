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

// Phase 7A: the align-service deterministic gate/response decision lives in Rust. These tests pin
// the C ABI from the C++ side before the later TPE/search migration changes the align body.

#include "autoware_ndt_scan_matcher_rs.h"

#include <gtest/gtest.h>

#include <cstdint>

namespace
{
AwNdtAlignServiceInput make_input(
  const bool transform_ok, const bool map_ok, const bool sensor_ok,
  const bool score_available, const double score)
{
  AwNdtAlignServiceInput input{};
  input.transform_initial_pose_ok = transform_ok ? 1U : 0U;
  input.map_points_ok = map_ok ? 1U : 0U;
  input.sensor_points_ok = sensor_ok ? 1U : 0U;
  input.align_score_available = score_available ? 1U : 0U;
  input.align_score = score;
  input.reliable_score_threshold = 4.0;
  return input;
}

AwNdtAlignServiceDecision decide(const AwNdtAlignServiceInput & input)
{
  AwNdtAlignServiceDecision decision{};
  autoware_ndt_scan_matcher_rs_node_decide_align_service(&input, &decision);
  return decision;
}

void expect_decision(
  const AwNdtAlignServiceDecision & got, const int32_t status, const uint8_t success,
  const uint8_t reliable, const uint8_t should_align, const uint8_t valid)
{
  EXPECT_EQ(got.status, status);
  EXPECT_EQ(got.success, success);
  EXPECT_EQ(got.reliable, reliable);
  EXPECT_EQ(got.should_align, should_align);
  EXPECT_EQ(got.valid, valid);
}
}  // namespace

TEST(NdtAlignServiceDecision, GateFailuresStopBeforeAlign)  // NOLINT
{
  expect_decision(
    decide(make_input(false, true, true, false, 0.0)),
    NDT_ALIGN_SERVICE_STATUS_TRANSFORM_UNAVAILABLE, 0U, 0U, 0U, 1U);
  expect_decision(
    decide(make_input(true, false, true, false, 0.0)),
    NDT_ALIGN_SERVICE_STATUS_MAP_UNAVAILABLE, 0U, 0U, 0U, 1U);
  expect_decision(
    decide(make_input(true, true, false, false, 0.0)),
    NDT_ALIGN_SERVICE_STATUS_SENSOR_UNAVAILABLE, 0U, 0U, 0U, 1U);
}

TEST(NdtAlignServiceDecision, ReadyToAlignWhenAllGatesPassWithoutScore)  // NOLINT
{
  expect_decision(
    decide(make_input(true, true, true, false, 0.0)),
    NDT_ALIGN_SERVICE_STATUS_READY_TO_ALIGN, 0U, 0U, 1U, 1U);
}

TEST(NdtAlignServiceDecision, AlignedReliabilityMatchesCppStrictLessThanSemantics)  // NOLINT
{
  expect_decision(
    decide(make_input(true, true, true, true, 4.1)),
    NDT_ALIGN_SERVICE_STATUS_ALIGNED, 1U, 1U, 0U, 1U);
  expect_decision(
    decide(make_input(true, true, true, true, 4.0)),
    NDT_ALIGN_SERVICE_STATUS_ALIGNED, 1U, 0U, 0U, 1U);
}

TEST(NdtAlignServiceDecision, InvalidFlagAndNullPointersAreSafe)  // NOLINT
{
  AwNdtAlignServiceInput invalid = make_input(true, true, true, false, 0.0);
  invalid.sensor_points_ok = 2U;
  expect_decision(
    decide(invalid), NDT_ALIGN_SERVICE_STATUS_INVALID_INPUT, 0U, 0U, 0U, 0U);

  AwNdtAlignServiceDecision unchanged{};
  unchanged.status = 99;
  autoware_ndt_scan_matcher_rs_node_decide_align_service(nullptr, &unchanged);
  EXPECT_EQ(unchanged.status, 99);

  const AwNdtAlignServiceInput valid = make_input(true, true, true, false, 0.0);
  autoware_ndt_scan_matcher_rs_node_decide_align_service(&valid, nullptr);
}
