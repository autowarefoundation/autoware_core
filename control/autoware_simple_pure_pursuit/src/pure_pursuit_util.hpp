// Copyright 2024 TIER IV, Inc.
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

#ifndef PURE_PURSUIT_UTIL_HPP_
#define PURE_PURSUIT_UTIL_HPP_

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <vector>

namespace autoware::control::simple_pure_pursuit
{
using autoware_planning_msgs::msg::TrajectoryPoint;

/// @brief Compute the speed-dependent lookahead distance.
/// @param lookahead_gain gain multiplied by the target longitudinal velocity
/// @param target_longitudinal_vel target longitudinal velocity [m/s]
/// @param lookahead_min_distance minimum lookahead distance [m]
/// @return lookahead distance [m]
double calc_lookahead_distance(
  const double lookahead_gain, const double target_longitudinal_vel,
  const double lookahead_min_distance);

/// @brief Compute the longitudinal acceleration command from a proportional controller.
/// @param speed_proportional_gain proportional gain
/// @param target_longitudinal_vel target longitudinal velocity [m/s]
/// @param current_longitudinal_vel current longitudinal velocity [m/s]
/// @return acceleration command [m/s^2]
double calc_longitudinal_acceleration(
  const double speed_proportional_gain, const double target_longitudinal_vel,
  const double current_longitudinal_vel);

/// @brief Compute the position of the rear-axle center from the base-link pose.
/// @param base_link_pose base-link pose (orientation interpreted as planar yaw)
/// @param wheel_base wheel base [m]
/// @return rear-axle center position
geometry_msgs::msg::Point calc_rear_position(
  const geometry_msgs::msg::Pose & base_link_pose, const double wheel_base);

/// @brief Compute the position of the rear-axle center from the base-link pose using a
/// precomputed yaw. Equivalent to the overload above with @p yaw set to the pose's planar yaw,
/// but avoids recomputing the yaw when it is already available at the call site.
/// @param base_link_pose base-link pose (only its position is used)
/// @param wheel_base wheel base [m]
/// @param yaw vehicle heading (yaw) [rad]
/// @return rear-axle center position
geometry_msgs::msg::Point calc_rear_position(
  const geometry_msgs::msg::Pose & base_link_pose, const double wheel_base, const double yaw);

/// @brief Search the first trajectory point at or beyond the lookahead distance from the
/// rear-axle center, starting from the closest trajectory point. Falls back to the last
/// trajectory point when none reaches the lookahead distance.
/// @param points trajectory points (must be non-empty)
/// @param closest_traj_point_idx index of the closest trajectory point
/// @param rear_position rear-axle center position
/// @param lookahead_distance lookahead distance [m]
/// @return const reference to the selected lookahead trajectory point within @p points (the
/// reference is valid for as long as @p points and its elements remain alive)
const TrajectoryPoint & find_lookahead_point(
  const std::vector<TrajectoryPoint> & points, const size_t closest_traj_point_idx,
  const geometry_msgs::msg::Point & rear_position, const double lookahead_distance);

/// @brief Compute the steering tire angle from the pure-pursuit geometry.
/// @param rear_position rear-axle center position
/// @param lookahead_position lookahead point position
/// @param vehicle_heading vehicle heading (yaw) [rad]
/// @param wheel_base wheel base [m]
/// @param lookahead_distance lookahead distance [m]
/// @return steering tire angle [rad]
double calc_steering_tire_angle(
  const geometry_msgs::msg::Point & rear_position,
  const geometry_msgs::msg::Point & lookahead_position, const double vehicle_heading,
  const double wheel_base, const double lookahead_distance);

}  // namespace autoware::control::simple_pure_pursuit

#endif  // PURE_PURSUIT_UTIL_HPP_
