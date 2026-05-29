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

#include "pure_pursuit_util.hpp"

#include <autoware_utils_geometry/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::control::simple_pure_pursuit
{

double calc_lookahead_distance(
  const double lookahead_gain, const double target_longitudinal_vel,
  const double lookahead_min_distance)
{
  return lookahead_gain * target_longitudinal_vel + lookahead_min_distance;
}

double calc_longitudinal_acceleration(
  const double speed_proportional_gain, const double target_longitudinal_vel,
  const double current_longitudinal_vel)
{
  return speed_proportional_gain * (target_longitudinal_vel - current_longitudinal_vel);
}

geometry_msgs::msg::Point calc_rear_position(
  const geometry_msgs::msg::Pose & base_link_pose, const double wheel_base)
{
  const double vehicle_heading = autoware_utils_geometry::get_rpy(base_link_pose.orientation).z;
  return calc_rear_position(base_link_pose, wheel_base, vehicle_heading);
}

geometry_msgs::msg::Point calc_rear_position(
  const geometry_msgs::msg::Pose & base_link_pose, const double wheel_base, const double yaw)
{
  return autoware_utils_geometry::create_point(
    base_link_pose.position.x - wheel_base / 2.0 * std::cos(yaw),
    base_link_pose.position.y - wheel_base / 2.0 * std::sin(yaw), base_link_pose.position.z);
}

TrajectoryPoint find_lookahead_point(
  const std::vector<TrajectoryPoint> & points, const size_t closest_traj_point_idx,
  const geometry_msgs::msg::Point & rear_position, const double lookahead_distance)
{
  auto lookahead_point_itr = std::find_if(
    points.begin() + static_cast<std::ptrdiff_t>(closest_traj_point_idx), points.end(),
    [&](const TrajectoryPoint & point) {
      return autoware_utils_geometry::calc_distance2d(point.pose.position, rear_position) >=
             lookahead_distance;
    });
  if (lookahead_point_itr == points.end()) {
    lookahead_point_itr = points.end() - 1;
  }
  return *lookahead_point_itr;
}

double calc_steering_tire_angle(
  const geometry_msgs::msg::Point & rear_position,
  const geometry_msgs::msg::Point & lookahead_position, const double vehicle_heading,
  const double wheel_base, const double lookahead_distance)
{
  const double alpha =
    std::atan2(lookahead_position.y - rear_position.y, lookahead_position.x - rear_position.x) -
    vehicle_heading;
  return std::atan2(2.0 * wheel_base * std::sin(alpha), lookahead_distance);
}

}  // namespace autoware::control::simple_pure_pursuit
