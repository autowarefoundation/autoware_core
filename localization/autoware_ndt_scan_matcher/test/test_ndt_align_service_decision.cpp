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

// The align-service deterministic gate/response decision and its abstract semantic
// trace live in Rust. These tests pin the C ABI from the C++ side before the later TPE/search
// migration changes the align body.

#include "autoware_ndt_scan_matcher_rs.h"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstddef>

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

AwNdtAlignServiceDecision decide_traced(
  const AwNdtAlignServiceInput & input, AwNdtAlignServiceTrace * trace)
{
  AwNdtAlignServiceDecision decision{};
  autoware_ndt_scan_matcher_rs_node_decide_align_service_traced(&input, trace, &decision);
  return decision;
}

AwNdtAlignServiceDecision decide_like_cpp_helper(
  const bool transform_ok, const bool map_ok, const bool sensor_ok,
  const bool score_available, const double score, const double threshold,
  AwNdtAlignServiceTrace * trace = nullptr)
{
  AwNdtAlignServiceInput input{};
  input.transform_initial_pose_ok = transform_ok ? 1U : 0U;
  input.map_points_ok = map_ok ? 1U : 0U;
  input.sensor_points_ok = sensor_ok ? 1U : 0U;
  input.align_score_available = score_available ? 1U : 0U;
  input.align_score = score;
  input.reliable_score_threshold = threshold;
  AwNdtAlignServiceDecision decision{};
  autoware_ndt_scan_matcher_rs_node_decide_align_service_traced(&input, trace, &decision);
  return decision;
}

AwNdtAlignServiceGateAction evaluate_gate_like_cpp_helper(
  const bool transform_ok, const bool map_ok, const bool sensor_ok,
  const bool score_available, const double score, const double threshold,
  AwNdtAlignServiceTrace * trace = nullptr)
{
  AwNdtAlignServiceInput input{};
  input.transform_initial_pose_ok = transform_ok ? 1U : 0U;
  input.map_points_ok = map_ok ? 1U : 0U;
  input.sensor_points_ok = sensor_ok ? 1U : 0U;
  input.align_score_available = score_available ? 1U : 0U;
  input.align_score = score;
  input.reliable_score_threshold = threshold;
  AwNdtAlignServiceGateAction action{};
  autoware_ndt_scan_matcher_rs_node_evaluate_align_service_gate(&input, trace, &action);
  return action;
}

AwNdtAlignServiceAlignedInput make_aligned_input(const double score, const double threshold)
{
  AwNdtAlignServiceAlignedInput input{};
  input.stamp_ns = 123;
  input.position[0] = 1.0;
  input.position[1] = 2.0;
  input.position[2] = 3.0;
  input.orientation[0] = 0.1;
  input.orientation[1] = 0.2;
  input.orientation[2] = 0.3;
  input.orientation[3] = 0.9;
  input.request_covariance[0] = 1.0;
  input.request_covariance[7] = 2.0;
  input.request_covariance[14] = 3.0;
  input.request_covariance[21] = 4.0;
  input.request_covariance[28] = 5.0;
  input.request_covariance[35] = 6.0;
  input.align_score = score;
  input.reliable_score_threshold = threshold;
  return input;
}

AwNdtAlignServiceResponse assemble_response(const AwNdtAlignServiceAlignedInput & input)
{
  AwNdtAlignServiceResponse response{};
  autoware_ndt_scan_matcher_rs_node_assemble_align_service_response(&input, &response);
  return response;
}

AwNdtAlignServiceSearchSummaryInput make_search_summary(
  const int64_t particles_requested, const int64_t particles_evaluated,
  const int64_t marker_publish_count, const int64_t cloud_publish_count,
  const int32_t best_iteration, const double best_score, const double threshold)
{
  AwNdtAlignServiceSearchSummaryInput input{};
  input.particles_requested = particles_requested;
  input.particles_evaluated = particles_evaluated;
  input.marker_publish_count = marker_publish_count;
  input.cloud_publish_count = cloud_publish_count;
  input.best_iteration = best_iteration;
  input.best_score = best_score;
  input.reliable_score_threshold = threshold;
  return input;
}

uint8_t score_available_byte(const AwNdtAlignServiceInput & input)
{
  return input.align_score_available == 1U ? 1U : 0U;
}

AwNdtAlignServiceTraceEvent reference_event(
  const AwNdtAlignServiceInput & input, const AwNdtAlignServiceDecision & decision)
{
  AwNdtAlignServiceTraceEvent event{};
  event.kind = NDT_ALIGN_TRACE_EVENT_DECISION;
  event.status = decision.status;
  event.success = decision.success;
  event.reliable = decision.reliable;
  event.should_align = decision.should_align;
  event.valid = decision.valid;
  event.score_available = score_available_byte(input);
  event.score = input.align_score;
  event.reliable_score_threshold = input.reliable_score_threshold;
  event.particles_requested = 0;
  event.particles_evaluated = 0;
  event.marker_publish_count = 0;
  event.cloud_publish_count = 0;
  event.best_iteration = 0;
  event.best_score = 0.0;
  if (decision.status == NDT_ALIGN_SERVICE_STATUS_TRANSFORM_UNAVAILABLE) {
    event.diagnostic_level = NDT_ALIGN_SERVICE_DIAGNOSTIC_ERROR;
    event.message_kind = NDT_ALIGN_SERVICE_MESSAGE_TRANSFORM_UNAVAILABLE;
  } else if (decision.status == NDT_ALIGN_SERVICE_STATUS_MAP_UNAVAILABLE) {
    event.diagnostic_level = NDT_ALIGN_SERVICE_DIAGNOSTIC_WARN;
    event.message_kind = NDT_ALIGN_SERVICE_MESSAGE_MAP_UNAVAILABLE;
  } else if (decision.status == NDT_ALIGN_SERVICE_STATUS_SENSOR_UNAVAILABLE) {
    event.diagnostic_level = NDT_ALIGN_SERVICE_DIAGNOSTIC_WARN;
    event.message_kind = NDT_ALIGN_SERVICE_MESSAGE_SENSOR_UNAVAILABLE;
  } else {
    event.diagnostic_level = NDT_ALIGN_SERVICE_DIAGNOSTIC_OK;
    event.message_kind = NDT_ALIGN_SERVICE_MESSAGE_NONE;
  }
  event.response_stamp_ns = 0;
  return event;
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

void expect_gate_action(
  const AwNdtAlignServiceGateAction & got, const int32_t status, const uint8_t success,
  const uint8_t reliable, const uint8_t should_align, const uint8_t valid,
  const int32_t diagnostic_level, const int32_t message_kind)
{
  EXPECT_EQ(got.status, status);
  EXPECT_EQ(got.success, success);
  EXPECT_EQ(got.reliable, reliable);
  EXPECT_EQ(got.should_align, should_align);
  EXPECT_EQ(got.valid, valid);
  EXPECT_EQ(got.diagnostic_level, diagnostic_level);
  EXPECT_EQ(got.message_kind, message_kind);
}

void expect_event(
  const AwNdtAlignServiceTraceEvent & got, const AwNdtAlignServiceTraceEvent & expected)
{
  EXPECT_EQ(got.kind, expected.kind);
  EXPECT_EQ(got.status, expected.status);
  EXPECT_EQ(got.success, expected.success);
  EXPECT_EQ(got.reliable, expected.reliable);
  EXPECT_EQ(got.should_align, expected.should_align);
  EXPECT_EQ(got.valid, expected.valid);
  EXPECT_EQ(got.score_available, expected.score_available);
  EXPECT_EQ(got.score, expected.score);
  EXPECT_EQ(got.reliable_score_threshold, expected.reliable_score_threshold);
  EXPECT_EQ(got.particles_requested, expected.particles_requested);
  EXPECT_EQ(got.particles_evaluated, expected.particles_evaluated);
  EXPECT_EQ(got.marker_publish_count, expected.marker_publish_count);
  EXPECT_EQ(got.cloud_publish_count, expected.cloud_publish_count);
  EXPECT_EQ(got.best_iteration, expected.best_iteration);
  EXPECT_EQ(got.best_score, expected.best_score);
  EXPECT_EQ(got.diagnostic_level, expected.diagnostic_level);
  EXPECT_EQ(got.message_kind, expected.message_kind);
  EXPECT_EQ(got.response_stamp_ns, expected.response_stamp_ns);
  for (std::size_t i = 0; i < 3U; ++i) {
    EXPECT_EQ(got.response_position[i], expected.response_position[i]);
  }
  for (std::size_t i = 0; i < 4U; ++i) {
    EXPECT_EQ(got.response_orientation[i], expected.response_orientation[i]);
  }
  for (std::size_t i = 0; i < 36U; ++i) {
    EXPECT_EQ(got.response_covariance[i], expected.response_covariance[i]);
  }
}

void expect_search_summary_event(
  const AwNdtAlignServiceTraceEvent & got, const AwNdtAlignServiceSearchSummaryInput & expected,
  const int32_t status, const uint8_t success, const uint8_t reliable, const uint8_t valid,
  const uint8_t score_available)
{
  EXPECT_EQ(got.kind, NDT_ALIGN_TRACE_EVENT_SEARCH_SUMMARY);
  EXPECT_EQ(got.status, status);
  EXPECT_EQ(got.success, success);
  EXPECT_EQ(got.reliable, reliable);
  EXPECT_EQ(got.should_align, 0U);
  EXPECT_EQ(got.valid, valid);
  EXPECT_EQ(got.score_available, score_available);
  EXPECT_EQ(got.score, expected.best_score);
  EXPECT_EQ(got.reliable_score_threshold, expected.reliable_score_threshold);
  EXPECT_EQ(got.particles_requested, expected.particles_requested);
  EXPECT_EQ(got.particles_evaluated, expected.particles_evaluated);
  EXPECT_EQ(got.marker_publish_count, expected.marker_publish_count);
  EXPECT_EQ(got.cloud_publish_count, expected.cloud_publish_count);
  EXPECT_EQ(got.best_iteration, expected.best_iteration);
  EXPECT_EQ(got.best_score, expected.best_score);
  EXPECT_EQ(got.diagnostic_level, NDT_ALIGN_SERVICE_DIAGNOSTIC_OK);
  EXPECT_EQ(got.message_kind, NDT_ALIGN_SERVICE_MESSAGE_NONE);
  EXPECT_EQ(got.response_stamp_ns, 0);
}

void expect_response_event(
  const AwNdtAlignServiceTraceEvent & got, const AwNdtAlignServiceResponse & expected)
{
  EXPECT_EQ(got.kind, NDT_ALIGN_TRACE_EVENT_RESPONSE);
  EXPECT_EQ(got.status, expected.status);
  EXPECT_EQ(got.success, expected.success);
  EXPECT_EQ(got.reliable, expected.reliable);
  EXPECT_EQ(got.should_align, 0U);
  EXPECT_EQ(got.valid, expected.valid);
  EXPECT_EQ(got.score_available, 0U);
  EXPECT_EQ(got.score, 0.0);
  EXPECT_EQ(got.reliable_score_threshold, 0.0);
  EXPECT_EQ(got.particles_requested, 0);
  EXPECT_EQ(got.particles_evaluated, 0);
  EXPECT_EQ(got.marker_publish_count, 0);
  EXPECT_EQ(got.cloud_publish_count, 0);
  EXPECT_EQ(got.best_iteration, 0);
  EXPECT_EQ(got.best_score, 0.0);
  EXPECT_EQ(got.diagnostic_level, NDT_ALIGN_SERVICE_DIAGNOSTIC_OK);
  EXPECT_EQ(got.message_kind, NDT_ALIGN_SERVICE_MESSAGE_NONE);
  EXPECT_EQ(got.response_stamp_ns, expected.stamp_ns);
  for (std::size_t i = 0; i < 3U; ++i) {
    EXPECT_EQ(got.response_position[i], expected.position[i]);
  }
  for (std::size_t i = 0; i < 4U; ++i) {
    EXPECT_EQ(got.response_orientation[i], expected.orientation[i]);
  }
  for (std::size_t i = 0; i < 36U; ++i) {
    EXPECT_EQ(got.response_covariance[i], expected.covariance[i]);
  }
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

TEST(NdtAlignServiceDecision, TracedAndCompatibilityWrappersMatch)  // NOLINT
{
  const AwNdtAlignServiceInput input = make_input(true, true, true, true, 4.5);
  const AwNdtAlignServiceDecision untraced = decide(input);
  const AwNdtAlignServiceDecision traced = decide_traced(input, nullptr);
  EXPECT_EQ(traced.status, untraced.status);
  EXPECT_EQ(traced.success, untraced.success);
  EXPECT_EQ(traced.reliable, untraced.reliable);
  EXPECT_EQ(traced.should_align, untraced.should_align);
  EXPECT_EQ(traced.valid, untraced.valid);
}

TEST(NdtAlignServiceDecision, TracedDecisionAppendsSemanticEvent)  // NOLINT
{
  const AwNdtAlignServiceInput input = make_input(true, true, true, true, 4.5);
  std::array<AwNdtAlignServiceTraceEvent, 2> events{};
  AwNdtAlignServiceTrace trace{};
  trace.events = events.data();
  trace.capacity = events.size();
  trace.len = 0;
  trace.overflowed = 0U;

  const AwNdtAlignServiceDecision decision = decide_traced(input, &trace);

  ASSERT_EQ(trace.len, 1U);
  EXPECT_EQ(trace.overflowed, 0U);
  expect_decision(decision, NDT_ALIGN_SERVICE_STATUS_ALIGNED, 1U, 1U, 0U, 1U);
  expect_event(events[0], reference_event(input, decision));
}

TEST(NdtAlignServiceDecision, ProductionShapedHelperUsesTracedFfiWithNullTrace)  // NOLINT
{
  AwNdtAlignServiceInput ready_input = make_input(true, true, true, false, 0.0);
  ready_input.reliable_score_threshold = 2.5;
  const AwNdtAlignServiceDecision ready_reference = decide(ready_input);
  const AwNdtAlignServiceDecision ready_helper =
    decide_like_cpp_helper(true, true, true, false, 0.0, 2.5);
  EXPECT_EQ(ready_helper.status, ready_reference.status);
  EXPECT_EQ(ready_helper.success, ready_reference.success);
  EXPECT_EQ(ready_helper.reliable, ready_reference.reliable);
  EXPECT_EQ(ready_helper.should_align, ready_reference.should_align);
  EXPECT_EQ(ready_helper.valid, ready_reference.valid);

  AwNdtAlignServiceInput aligned_input = make_input(true, true, true, true, 3.0);
  aligned_input.reliable_score_threshold = 2.5;
  const AwNdtAlignServiceDecision aligned_reference = decide(aligned_input);
  const AwNdtAlignServiceDecision aligned_helper =
    decide_like_cpp_helper(true, true, true, true, 3.0, 2.5);
  EXPECT_EQ(aligned_helper.status, aligned_reference.status);
  EXPECT_EQ(aligned_helper.success, aligned_reference.success);
  EXPECT_EQ(aligned_helper.reliable, aligned_reference.reliable);
  EXPECT_EQ(aligned_helper.should_align, aligned_reference.should_align);
  EXPECT_EQ(aligned_helper.valid, aligned_reference.valid);
}

TEST(NdtAlignServiceDecision, GateActionMapsMessagesAndDiagnosticLevels)  // NOLINT
{
  expect_gate_action(
    evaluate_gate_like_cpp_helper(false, true, true, false, 0.0, 4.0),
    NDT_ALIGN_SERVICE_STATUS_TRANSFORM_UNAVAILABLE, 0U, 0U, 0U, 1U,
    NDT_ALIGN_SERVICE_DIAGNOSTIC_ERROR, NDT_ALIGN_SERVICE_MESSAGE_TRANSFORM_UNAVAILABLE);
  expect_gate_action(
    evaluate_gate_like_cpp_helper(true, false, true, false, 0.0, 4.0),
    NDT_ALIGN_SERVICE_STATUS_MAP_UNAVAILABLE, 0U, 0U, 0U, 1U,
    NDT_ALIGN_SERVICE_DIAGNOSTIC_WARN, NDT_ALIGN_SERVICE_MESSAGE_MAP_UNAVAILABLE);
  expect_gate_action(
    evaluate_gate_like_cpp_helper(true, true, false, false, 0.0, 4.0),
    NDT_ALIGN_SERVICE_STATUS_SENSOR_UNAVAILABLE, 0U, 0U, 0U, 1U,
    NDT_ALIGN_SERVICE_DIAGNOSTIC_WARN, NDT_ALIGN_SERVICE_MESSAGE_SENSOR_UNAVAILABLE);
}

TEST(NdtAlignServiceDecision, GateActionReadyAlignedAndInvalidHaveNoMessage)  // NOLINT
{
  expect_gate_action(
    evaluate_gate_like_cpp_helper(true, true, true, false, 0.0, 4.0),
    NDT_ALIGN_SERVICE_STATUS_READY_TO_ALIGN, 0U, 0U, 1U, 1U,
    NDT_ALIGN_SERVICE_DIAGNOSTIC_OK, NDT_ALIGN_SERVICE_MESSAGE_NONE);
  expect_gate_action(
    evaluate_gate_like_cpp_helper(true, true, true, true, 4.5, 4.0),
    NDT_ALIGN_SERVICE_STATUS_ALIGNED, 1U, 1U, 0U, 1U, NDT_ALIGN_SERVICE_DIAGNOSTIC_OK,
    NDT_ALIGN_SERVICE_MESSAGE_NONE);

  AwNdtAlignServiceInput invalid = make_input(true, true, true, false, 0.0);
  invalid.transform_initial_pose_ok = 7U;
  AwNdtAlignServiceGateAction action{};
  autoware_ndt_scan_matcher_rs_node_evaluate_align_service_gate(&invalid, nullptr, &action);
  expect_gate_action(
    action, NDT_ALIGN_SERVICE_STATUS_INVALID_INPUT, 0U, 0U, 0U, 0U,
    NDT_ALIGN_SERVICE_DIAGNOSTIC_OK, NDT_ALIGN_SERVICE_MESSAGE_NONE);
}

TEST(NdtAlignServiceDecision, ProductionShapedGateHelperMatchesDecisionAndTrace)  // NOLINT
{
  AwNdtAlignServiceInput input = make_input(true, false, true, false, 0.0);
  input.reliable_score_threshold = 2.5;
  const AwNdtAlignServiceDecision decision = decide(input);
  std::array<AwNdtAlignServiceTraceEvent, 1> events{};
  AwNdtAlignServiceTrace trace{};
  trace.events = events.data();
  trace.capacity = events.size();
  trace.len = 0U;
  trace.overflowed = 0U;

  const AwNdtAlignServiceGateAction action =
    evaluate_gate_like_cpp_helper(true, false, true, false, 0.0, 2.5, &trace);

  EXPECT_EQ(action.status, decision.status);
  EXPECT_EQ(action.success, decision.success);
  EXPECT_EQ(action.reliable, decision.reliable);
  EXPECT_EQ(action.should_align, decision.should_align);
  EXPECT_EQ(action.valid, decision.valid);
  ASSERT_EQ(trace.len, 1U);
  EXPECT_EQ(trace.overflowed, 0U);
  expect_event(events[0], reference_event(input, decision));
}

TEST(NdtAlignServiceDecision, GateActionNullPointersAreSafe)  // NOLINT
{
  const AwNdtAlignServiceInput input = make_input(true, true, true, false, 0.0);
  AwNdtAlignServiceGateAction unchanged{};
  unchanged.status = 99;
  unchanged.success = 1U;
  unchanged.reliable = 1U;
  unchanged.should_align = 1U;
  unchanged.valid = 1U;

  autoware_ndt_scan_matcher_rs_node_evaluate_align_service_gate(nullptr, nullptr, &unchanged);
  EXPECT_EQ(unchanged.status, 99);

  autoware_ndt_scan_matcher_rs_node_evaluate_align_service_gate(&input, nullptr, nullptr);
}

TEST(NdtAlignServiceDecision, ProductionShapedHelperCanAppendDecisionTrace)  // NOLINT
{
  std::array<AwNdtAlignServiceTraceEvent, 2> ready_events{};
  AwNdtAlignServiceTrace ready_trace{};
  ready_trace.events = ready_events.data();
  ready_trace.capacity = ready_events.size();
  ready_trace.len = 0U;
  ready_trace.overflowed = 0U;
  AwNdtAlignServiceInput ready_input = make_input(true, true, true, false, 0.0);
  ready_input.reliable_score_threshold = 2.5;

  const AwNdtAlignServiceDecision ready_decision =
    decide_like_cpp_helper(true, true, true, false, 0.0, 2.5, &ready_trace);

  ASSERT_EQ(ready_trace.len, 1U);
  EXPECT_EQ(ready_trace.overflowed, 0U);
  expect_decision(ready_decision, NDT_ALIGN_SERVICE_STATUS_READY_TO_ALIGN, 0U, 0U, 1U, 1U);
  expect_event(ready_events[0], reference_event(ready_input, ready_decision));

  std::array<AwNdtAlignServiceTraceEvent, 2> aligned_events{};
  AwNdtAlignServiceTrace aligned_trace{};
  aligned_trace.events = aligned_events.data();
  aligned_trace.capacity = aligned_events.size();
  aligned_trace.len = 0U;
  aligned_trace.overflowed = 0U;
  AwNdtAlignServiceInput aligned_input = make_input(true, true, true, true, 3.0);
  aligned_input.reliable_score_threshold = 2.5;

  const AwNdtAlignServiceDecision aligned_decision =
    decide_like_cpp_helper(true, true, true, true, 3.0, 2.5, &aligned_trace);

  ASSERT_EQ(aligned_trace.len, 1U);
  EXPECT_EQ(aligned_trace.overflowed, 0U);
  expect_decision(aligned_decision, NDT_ALIGN_SERVICE_STATUS_ALIGNED, 1U, 1U, 0U, 1U);
  expect_event(aligned_events[0], reference_event(aligned_input, aligned_decision));
}


TEST(NdtAlignServiceDecision, SearchSummaryTraceAppendsSemanticEvent)  // NOLINT
{
  const AwNdtAlignServiceSearchSummaryInput input =
    make_search_summary(64, 64, 4, 64, 12, 5.5, 4.0);
  std::array<AwNdtAlignServiceTraceEvent, 1> events{};
  AwNdtAlignServiceTrace trace{};
  trace.events = events.data();
  trace.capacity = events.size();
  trace.len = 0U;
  trace.overflowed = 0U;

  autoware_ndt_scan_matcher_rs_node_append_align_service_search_summary_trace(&input, &trace);

  ASSERT_EQ(trace.len, 1U);
  EXPECT_EQ(trace.overflowed, 0U);
  expect_search_summary_event(
    events[0], input, NDT_ALIGN_SERVICE_STATUS_ALIGNED, 1U, 1U, 1U, 1U);
}

TEST(NdtAlignServiceDecision, SearchSummaryTraceInvalidCountsAreExplicit)  // NOLINT
{
  const AwNdtAlignServiceSearchSummaryInput input =
    make_search_summary(64, -1, 4, 64, 12, 5.5, 4.0);
  std::array<AwNdtAlignServiceTraceEvent, 1> events{};
  AwNdtAlignServiceTrace trace{};
  trace.events = events.data();
  trace.capacity = events.size();
  trace.len = 0U;
  trace.overflowed = 0U;

  autoware_ndt_scan_matcher_rs_node_append_align_service_search_summary_trace(&input, &trace);

  ASSERT_EQ(trace.len, 1U);
  EXPECT_EQ(trace.overflowed, 0U);
  expect_search_summary_event(
    events[0], input, NDT_ALIGN_SERVICE_STATUS_INVALID_INPUT, 0U, 0U, 0U, 0U);
}

TEST(NdtAlignServiceDecision, SearchSummaryTraceNullAndFullBuffersAreSafe)  // NOLINT
{
  const AwNdtAlignServiceSearchSummaryInput input =
    make_search_summary(64, 64, 4, 64, 12, 5.5, 4.0);
  autoware_ndt_scan_matcher_rs_node_append_align_service_search_summary_trace(nullptr, nullptr);
  autoware_ndt_scan_matcher_rs_node_append_align_service_search_summary_trace(&input, nullptr);

  std::array<AwNdtAlignServiceTraceEvent, 1> events{};
  AwNdtAlignServiceTrace full_trace{};
  full_trace.events = events.data();
  full_trace.capacity = events.size();
  full_trace.len = events.size();
  full_trace.overflowed = 0U;
  autoware_ndt_scan_matcher_rs_node_append_align_service_search_summary_trace(&input, &full_trace);
  EXPECT_EQ(full_trace.len, events.size());
  EXPECT_EQ(full_trace.overflowed, 1U);

  AwNdtAlignServiceTrace null_events{};
  null_events.events = nullptr;
  null_events.capacity = 1U;
  null_events.len = 0U;
  null_events.overflowed = 0U;
  autoware_ndt_scan_matcher_rs_node_append_align_service_search_summary_trace(&input, &null_events);
  EXPECT_EQ(null_events.len, 0U);
  EXPECT_EQ(null_events.overflowed, 1U);
}


TEST(NdtAlignServiceDecision, ResponseTraceAppendsResponsePayloadEvent)  // NOLINT
{
  const AwNdtAlignServiceResponse response = assemble_response(make_aligned_input(4.5, 4.0));
  std::array<AwNdtAlignServiceTraceEvent, 1> events{};
  AwNdtAlignServiceTrace trace{};
  trace.events = events.data();
  trace.capacity = events.size();
  trace.len = 0U;
  trace.overflowed = 0U;

  autoware_ndt_scan_matcher_rs_node_append_align_service_response_trace(&response, &trace);

  ASSERT_EQ(trace.len, 1U);
  EXPECT_EQ(trace.overflowed, 0U);
  expect_response_event(events[0], response);
}

TEST(NdtAlignServiceDecision, ResponseTraceNullAndFullBuffersAreSafe)  // NOLINT
{
  const AwNdtAlignServiceResponse response = assemble_response(make_aligned_input(4.5, 4.0));
  autoware_ndt_scan_matcher_rs_node_append_align_service_response_trace(nullptr, nullptr);
  autoware_ndt_scan_matcher_rs_node_append_align_service_response_trace(&response, nullptr);

  std::array<AwNdtAlignServiceTraceEvent, 1> events{};
  AwNdtAlignServiceTrace full_trace{};
  full_trace.events = events.data();
  full_trace.capacity = events.size();
  full_trace.len = events.size();
  full_trace.overflowed = 0U;
  autoware_ndt_scan_matcher_rs_node_append_align_service_response_trace(&response, &full_trace);
  EXPECT_EQ(full_trace.len, events.size());
  EXPECT_EQ(full_trace.overflowed, 1U);

  AwNdtAlignServiceTrace null_events{};
  null_events.events = nullptr;
  null_events.capacity = 1U;
  null_events.len = 0U;
  null_events.overflowed = 0U;
  autoware_ndt_scan_matcher_rs_node_append_align_service_response_trace(&response, &null_events);
  EXPECT_EQ(null_events.len, 0U);
  EXPECT_EQ(null_events.overflowed, 1U);
}

TEST(NdtAlignServiceDecision, ResponseAssemblyCopiesPoseCovarianceAndReliability)  // NOLINT
{
  const AwNdtAlignServiceAlignedInput input = make_aligned_input(4.5, 4.0);

  const AwNdtAlignServiceResponse response = assemble_response(input);

  EXPECT_EQ(response.status, NDT_ALIGN_SERVICE_STATUS_ALIGNED);
  EXPECT_EQ(response.success, 1U);
  EXPECT_EQ(response.reliable, 1U);
  EXPECT_EQ(response.valid, 1U);
  EXPECT_EQ(response.stamp_ns, input.stamp_ns);
  for (std::size_t i = 0; i < 3U; ++i) {
    EXPECT_EQ(response.position[i], input.position[i]);
  }
  for (std::size_t i = 0; i < 4U; ++i) {
    EXPECT_EQ(response.orientation[i], input.orientation[i]);
  }
  for (std::size_t i = 0; i < 36U; ++i) {
    EXPECT_EQ(response.covariance[i], input.request_covariance[i]);
  }
}

TEST(NdtAlignServiceDecision, ResponseAssemblyMatchesDecisionWrapper)  // NOLINT
{
  AwNdtAlignServiceInput decision_input = make_input(true, true, true, true, 4.0);
  decision_input.reliable_score_threshold = 4.0;
  const AwNdtAlignServiceDecision decision = decide(decision_input);

  const AwNdtAlignServiceResponse response = assemble_response(make_aligned_input(4.0, 4.0));

  EXPECT_EQ(response.status, decision.status);
  EXPECT_EQ(response.success, decision.success);
  EXPECT_EQ(response.reliable, decision.reliable);
  EXPECT_EQ(response.valid, decision.valid);
}

TEST(NdtAlignServiceDecision, ResponseAssemblyNullPointersAreSafe)  // NOLINT
{
  const AwNdtAlignServiceAlignedInput input = make_aligned_input(4.5, 4.0);
  AwNdtAlignServiceResponse unchanged{};
  unchanged.status = 99;
  unchanged.success = 1U;
  unchanged.reliable = 1U;
  unchanged.valid = 1U;

  autoware_ndt_scan_matcher_rs_node_assemble_align_service_response(nullptr, &unchanged);
  EXPECT_EQ(unchanged.status, 99);

  autoware_ndt_scan_matcher_rs_node_assemble_align_service_response(&input, nullptr);
}

TEST(NdtAlignServiceDecision, TracedInvalidInputAndOverflowAreExplicit)  // NOLINT
{
  AwNdtAlignServiceInput invalid = make_input(true, false, true, false, 7.0);
  invalid.map_points_ok = 3U;
  std::array<AwNdtAlignServiceTraceEvent, 1> events{};
  AwNdtAlignServiceTrace trace{};
  trace.events = events.data();
  trace.capacity = events.size();
  trace.len = 0;
  trace.overflowed = 0U;

  const AwNdtAlignServiceDecision decision = decide_traced(invalid, &trace);

  ASSERT_EQ(trace.len, 1U);
  EXPECT_EQ(trace.overflowed, 0U);
  expect_decision(decision, NDT_ALIGN_SERVICE_STATUS_INVALID_INPUT, 0U, 0U, 0U, 0U);
  expect_event(events[0], reference_event(invalid, decision));

  AwNdtAlignServiceTrace full_trace{};
  full_trace.events = events.data();
  full_trace.capacity = events.size();
  full_trace.len = events.size();
  full_trace.overflowed = 0U;
  static_cast<void>(decide_traced(make_input(true, true, true, false, 0.0), &full_trace));
  EXPECT_EQ(full_trace.len, events.size());
  EXPECT_EQ(full_trace.overflowed, 1U);

  AwNdtAlignServiceTrace null_events{};
  null_events.events = nullptr;
  null_events.capacity = 1U;
  null_events.len = 0U;
  null_events.overflowed = 0U;
  static_cast<void>(decide_traced(make_input(true, true, true, false, 0.0), &null_events));
  EXPECT_EQ(null_events.len, 0U);
  EXPECT_EQ(null_events.overflowed, 1U);
}
