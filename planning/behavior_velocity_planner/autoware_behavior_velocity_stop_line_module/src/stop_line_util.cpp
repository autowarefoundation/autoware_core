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

#include "stop_line_util.hpp"

#include "autoware/trajectory/utils/closest.hpp"
#include "autoware/trajectory/utils/crossed.hpp"

#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner::stop_line_utils
{

bool has_intersection(
  const lanelet::Ids & connected_lanelet_ids, const std::vector<int64_t> & point_lane_ids)
{
  return std::any_of(
    point_lane_ids.begin(), point_lane_ids.end(), [&connected_lanelet_ids](const int64_t id) {
      return std::find(connected_lanelet_ids.begin(), connected_lanelet_ids.end(), id) !=
             connected_lanelet_ids.end();
    });
}

std::pair<double, std::optional<double>> compute_ego_and_stop_point(
  const Trajectory & trajectory, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound,
  const lanelet::ConstLineString3d & stop_line, const geometry_msgs::msg::Pose & ego_pose,
  const State & state, const lanelet::Ids & connected_lanelet_ids, const double base_link2front,
  const double stop_margin)
{
  const double ego_s = autoware::experimental::trajectory::closest(trajectory, ego_pose);
  std::optional<double> stop_point_s;

  switch (state) {
    case State::APPROACH: {
      const LineString2d extended_stop_line = planning_utils::extendSegmentToBounds(
        lanelet::utils::to2D(stop_line).basicLineString(), left_bound, right_bound);

      // Calculate intersection with stop line
      const auto trajectory_stop_line_intersection =
        autoware::experimental::trajectory::crossed_with_constraint(
          trajectory, extended_stop_line,
          [&connected_lanelet_ids](
            const autoware_internal_planning_msgs::msg::PathPointWithLaneId & point) {
            return has_intersection(connected_lanelet_ids, point.lane_ids);
          });

      // If no collision found, do nothing
      if (trajectory_stop_line_intersection.size() == 0) {
        stop_point_s = std::nullopt;
        break;
      }

      stop_point_s = trajectory_stop_line_intersection.at(0) -
                     (base_link2front + stop_margin);  // consider vehicle length and stop margin

      if (*stop_point_s < 0.0) {
        stop_point_s = std::nullopt;
      }
      break;
    }

    case State::STOPPED: {
      stop_point_s = ego_s;
      break;
    }

    case State::START: {
      stop_point_s = std::nullopt;
      break;
    }
  }
  return {ego_s, stop_point_s};
}

StateTransitionResult advance_state(
  State & state, std::optional<rclcpp::Time> & stopped_time, const rclcpp::Time & now,
  const double distance_to_stop_point, const bool is_vehicle_stopped,
  const double hold_stop_margin_distance, const double required_stop_duration_sec)
{
  switch (state) {
    case State::APPROACH: {
      if (distance_to_stop_point < hold_stop_margin_distance && is_vehicle_stopped) {
        state = State::STOPPED;
        stopped_time = now;
        return {StateTransition::APPROACH_TO_STOPPED};
      }
      return {StateTransition::APPROACH_STAY};
    }
    case State::STOPPED: {
      if (!stopped_time.has_value()) {
        stopped_time = now;
        return {StateTransition::STOPPED_TIME_RECOVERY};
      }
      const double stop_duration = (now - *stopped_time).seconds();
      if (stop_duration > required_stop_duration_sec) {
        state = State::START;
        stopped_time.reset();
        return {StateTransition::STOPPED_TO_START, stop_duration};
      }
      return {StateTransition::STOPPED_STAY, stop_duration};
    }
    case State::START: {
      return {StateTransition::START};
    }
  }
  return {StateTransition::START};
}

}  // namespace autoware::behavior_velocity_planner::stop_line_utils
