// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__TRAJECTORY__UTILS__LATERAL_METRICS_HPP_
#define AUTOWARE__TRAJECTORY__UTILS__LATERAL_METRICS_HPP_

#include "autoware/trajectory/detail/types.hpp"
#include "autoware/trajectory/forward.hpp"
#include "autoware_utils_geometry/geometry.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace autoware::experimental::trajectory
{

inline double calc_azimuth_angle(
  const geometry_msgs::msg::Point & p_from, const geometry_msgs::msg::Point & p_to)
{
  const double dx = p_to.x - p_from.x;
  const double dy = p_to.y - p_from.y;
  return std::atan2(dy, dx);
}

inline double normalize_radian(const double rad, const double min_rad = -M_PI)
{
  const auto max_rad = min_rad + 2 * M_PI;

  const auto value = std::fmod(rad, 2 * M_PI);
  if (min_rad <= value && value < max_rad) {
    return value;
  }

  return value - std::copysign(2 * M_PI, value);
}

template <class TrajectoryPointType>
double compute_lateral_distance(
  const Trajectory<TrajectoryPointType> & trajectory, const geometry_msgs::msg::Point & p_target,
  const double target_s)
{
  const auto p_front = trajectory.compute(target_s);

  const auto traj_tangent_yaw = autoware_utils_geometry::get_rpy(p_front).z;
  const Eigen::Vector3d segment_vec{std::cos(traj_tangent_yaw), std::sin(traj_tangent_yaw), 0.0};
  const Eigen::Vector3d target_vec{
    p_target.x - p_front.position.x, p_target.y - p_front.position.y, 0.0};

  const Eigen::Vector3d cross_vec = segment_vec.cross(target_vec);

  return std::fabs(cross_vec(2) / segment_vec.norm());
}

template <class TrajectoryPointType>
bool is_left_side(
  const Trajectory<TrajectoryPointType> & trajectory,
  const geometry_msgs::msg::Point & target_point, const double target_s)
{
  const auto traj_pose = trajectory.compute(target_s);
  const double tangent_yaw = autoware_utils_geometry::get_rpy(traj_pose).z;

  const double target_yaw = calc_azimuth_angle(traj_pose.position, target_point);

  const double diff_yaw = normalize_radian(target_yaw - tangent_yaw);

  if (0 < diff_yaw) {
    return true;
  }
  return false;
}

}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__UTILS__LATERAL_METRICS_HPP_
