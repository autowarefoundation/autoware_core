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

#ifndef AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__VELOCITY_PLANNING_RESULT_HPP_
#define AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__VELOCITY_PLANNING_RESULT_HPP_

#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit_clear_command.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <optional>
#include <vector>

namespace autoware::motion_velocity_planner
{
struct SlowdownInterval
{
  SlowdownInterval(
    const geometry_msgs::msg::Point & from_, const geometry_msgs::msg::Point & to_,
    const double vel)
  : from{from_}, to{to_}, velocity{vel}
  {
  }
  geometry_msgs::msg::Point from;
  geometry_msgs::msg::Point to;
  double velocity{};
};
struct VelocityPlanningResult
{
  std::vector<geometry_msgs::msg::Point> stop_points;
  std::vector<SlowdownInterval> slowdown_intervals;
  std::optional<autoware_internal_planning_msgs::msg::VelocityLimit> velocity_limit{std::nullopt};
  std::optional<autoware_internal_planning_msgs::msg::VelocityLimitClearCommand>
    velocity_limit_clear_command{std::nullopt};
};

/// @brief modify a trajectory with the given velocity planning result
void add_planning_results_to_trajectory(
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const VelocityPlanningResult & planning_result);
/// @brief insert a stop point in the given trajectory
void insert_stop(
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const geometry_msgs::msg::Point & stop_point);
/// @brief insert a slowdown interval in the given trajectory
void insert_slowdown(
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const autoware::motion_velocity_planner::SlowdownInterval & slowdown_interval);
}  // namespace autoware::motion_velocity_planner

#endif  // AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__VELOCITY_PLANNING_RESULT_HPP_
