// Copyright 2025 Tier IV, Inc.
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

#ifndef STOP_LINE_UTIL_HPP_
#define STOP_LINE_UTIL_HPP_
#define EIGEN_MPL2_ONLY

#include "autoware/behavior_velocity_planner_common/utilization/util.hpp"
#include "autoware/trajectory/path_point_with_lane_id.hpp"

#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/LineString.h>

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner::stop_line_utils
{

/// @brief Trajectory type shared by both the legacy and experimental stop-line scenes.
using Trajectory = autoware::experimental::trajectory::Trajectory<
  autoware_internal_planning_msgs::msg::PathPointWithLaneId>;

/// @brief State machine state of the stop-line module.
/// @details Shared between the legacy and experimental implementations so the state-machine logic
/// can be factored into a single set of pure free functions.
enum class State { APPROACH, STOPPED, START };

/// @brief Result of @ref advance_state describing which transition (if any) occurred.
/// @details The module wrappers use this to reproduce their exact log output without duplicating
/// the decision logic.
enum class StateTransition {
  APPROACH_STAY,          ///< Stayed in APPROACH (not yet stopped / too far).
  APPROACH_TO_STOPPED,    ///< Transitioned APPROACH -> STOPPED.
  STOPPED_TIME_RECOVERY,  ///< In STOPPED but stopped_time had no value; it was recovered.
  STOPPED_STAY,           ///< Stayed in STOPPED (duration not yet reached).
  STOPPED_TO_START,       ///< Transitioned STOPPED -> START.
  START                   ///< Stayed in START.
};

/// @brief Outcome of @ref advance_state.
struct StateTransitionResult
{
  StateTransition transition;  ///< Which transition occurred.
  /// Stop duration (seconds) computed in the STOPPED branch; only meaningful for STOPPED_STAY /
  /// STOPPED_TO_START, where it is the value logged by the caller before any reset.
  double stop_duration{0.0};
};

/// @brief Check whether @p connected_lanelet_ids and @p point_lane_ids share any id.
/// @details Replaces the hand-rolled sorted-set intersection check that used to be copy-pasted
/// between the legacy and experimental scenes. The invariant side is passed as the (typically
/// pre-built) @p connected_lanelet_ids vector and the per-point ids are scanned linearly, so no
/// temporary @c std::set is allocated per call.
bool has_intersection(
  const lanelet::Ids & connected_lanelet_ids, const std::vector<int64_t> & point_lane_ids);

/// @brief Compute the ego arc length and (optional) stop-point arc length on @p trajectory.
/// @param trajectory Current trajectory.
/// @param left_bound Left bound used to extend the stop line.
/// @param right_bound Right bound used to extend the stop line.
/// @param stop_line Raw stop-line geometry.
/// @param ego_pose Current pose of the vehicle.
/// @param state Current state of the stop-line module.
/// @param connected_lanelet_ids Lanelet ids connected to the stop line (loop invariant).
/// @param base_link2front Distance from base link to vehicle front.
/// @param stop_margin Stop margin to the stop line.
/// @return Pair of ego arc length and optional stop-point arc length.
std::pair<double, std::optional<double>> compute_ego_and_stop_point(
  const Trajectory & trajectory, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound,
  const lanelet::ConstLineString3d & stop_line, const geometry_msgs::msg::Pose & ego_pose,
  const State & state, const lanelet::Ids & connected_lanelet_ids, const double base_link2front,
  const double stop_margin);

/// @brief Advance the stop-line state machine.
/// @param state In/out current state.
/// @param stopped_time In/out time when the vehicle stopped.
/// @param now Current time.
/// @param distance_to_stop_point Distance to the stop point.
/// @param is_vehicle_stopped Whether the vehicle is stopped.
/// @param hold_stop_margin_distance Distance threshold for transitioning to STOPPED.
/// @param required_stop_duration_sec Required stop duration before transitioning to START.
/// @return The transition that occurred (and stop duration), so the caller can reproduce its log
/// output.
StateTransitionResult advance_state(
  State & state, std::optional<rclcpp::Time> & stopped_time, const rclcpp::Time & now,
  const double distance_to_stop_point, const bool is_vehicle_stopped,
  const double hold_stop_margin_distance, const double required_stop_duration_sec);

}  // namespace autoware::behavior_velocity_planner::stop_line_utils

#endif  // STOP_LINE_UTIL_HPP_
