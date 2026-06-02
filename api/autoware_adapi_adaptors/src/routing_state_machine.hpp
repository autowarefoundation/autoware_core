// Copyright 2025 The Autoware Contributors
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

#ifndef ROUTING_STATE_MACHINE_HPP_
#define ROUTING_STATE_MACHINE_HPP_

#include <cstdint>

namespace autoware::adapi_adaptors
{

/// Number of timer ticks to wait before issuing a request so that consecutive
/// goals and checkpoints are merged into a single request.
/// At 5 Hz this corresponds to (g_routing_request_delay_count - 1) / 5 = 0.4 seconds.
constexpr int g_routing_request_delay_count = 3;

/// The action the routing timer should take on a single tick.
enum class RoutingAction : std::uint8_t {
  /// Do nothing this tick (still within the merge window, or a service call is in flight).
  None,
  /// Clear the current route before a new one can be set.
  CallClear,
  /// Send the pending SetRoutePoints request.
  CallRoute,
};

/// Result of a single routing timer tick: which action to take and the updated counter.
struct RoutingDecision
{
  RoutingAction action;
  int number_of_requests_for_timing_control;
};

/// Pure merge-window state-machine for RoutingAdaptor::on_timer.
///
/// Given the current merge-window counter, whether a service call is in flight, and whether
/// the route state is UNSET, decide the next action and the updated counter. This isolates
/// the branching from the ROS node so it can be unit-tested directly.
///
/// @param number_of_requests_for_timing_control current merge-window counter (0 means idle).
/// @param calling_service true while a previous async service call has not completed.
/// @param state_is_unset true when the route state equals RouteState::UNSET.
inline RoutingDecision decide_routing_action(
  int number_of_requests_for_timing_control, bool calling_service, bool state_is_unset)
{
  // Wait a moment to combine consecutive goals and checkpoints into a single request.
  if (
    0 < number_of_requests_for_timing_control &&
    number_of_requests_for_timing_control < g_routing_request_delay_count) {
    ++number_of_requests_for_timing_control;
  }
  if (number_of_requests_for_timing_control != g_routing_request_delay_count) {
    return {RoutingAction::None, number_of_requests_for_timing_control};
  }

  if (calling_service) {
    return {RoutingAction::None, number_of_requests_for_timing_control};
  }

  if (!state_is_unset) {
    return {RoutingAction::CallClear, number_of_requests_for_timing_control};
  }
  return {RoutingAction::CallRoute, 0};
}

}  // namespace autoware::adapi_adaptors

#endif  // ROUTING_STATE_MACHINE_HPP_
