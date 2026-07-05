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

#ifndef MISSION_PLANNER__REROUTE_SAFETY_HPP_
#define MISSION_PLANNER__REROUTE_SAFETY_HPP_

#include <rclcpp/logger.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>

#include <lanelet2_core/LaneletMap.h>

namespace autoware::mission_planner
{

/**
 * @brief pure, dependency-injected reroute-safety check.
 *
 * Decides whether switching from @p original_route to @p target_route is safe while the ego
 * vehicle is moving. The length of road that the two routes share ahead of the ego (from the ego
 * position to the goal of the target route) must exceed a safety margin that grows with the
 * current velocity. This is the node-independent core of MissionPlanner::check_reroute_safety;
 * the node method forwards to it after validating its own odometry / map members.
 *
 * @param original_route route the ego is currently following.
 * @param target_route candidate route to switch to.
 * @param lanelet_map lanelet map providing the primitives referenced by both routes.
 * @param current_velocity longitudinal velocity of the ego [m/s].
 * @param reroute_time_threshold time horizon used to scale the velocity-dependent safety length
 * [s].
 * @param minimum_reroute_length lower bound of the safety length [m].
 * @param logger logger used for the diagnostic messages emitted on the failure branches.
 * @return true if the reroute is safe (or the vehicle is effectively stopped), false otherwise.
 */
bool check_reroute_safety(
  const autoware_planning_msgs::msg::LaneletRoute & original_route,
  const autoware_planning_msgs::msg::LaneletRoute & target_route,
  const lanelet::LaneletMapConstPtr & lanelet_map, const double current_velocity,
  const double reroute_time_threshold, const double minimum_reroute_length,
  const rclcpp::Logger & logger);

}  // namespace autoware::mission_planner

#endif  // MISSION_PLANNER__REROUTE_SAFETY_HPP_
