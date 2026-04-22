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

#include "autoware/trajectory/threshold.hpp"

#include "autoware_utils_geometry/geometry.hpp"

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Scalar.hpp>

#include <lanelet2_core/primitives/Point.h>

#include <cmath>

namespace autoware::experimental::trajectory
{

bool is_almost_same(const double s1, const double s2)
{
  return std::fabs(s1 - s2) < k_points_minimum_dist_threshold;
}

bool is_almost_same(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  return autoware_utils_geometry::calc_distance3d(p1, p2) < k_epsilon_distance;
}

bool is_almost_same(
  const geometry_msgs::msg::Quaternion & q1, const geometry_msgs::msg::Quaternion & q2)
{
  tf2::Quaternion tf_q1(q1.x, q1.y, q1.z, q1.w);
  tf2::Quaternion tf_q2(q2.x, q2.y, q2.z, q2.w);
  tf_q1.normalize();
  tf_q2.normalize();

  const double dot = tf_q1.dot(tf_q2);
  return std::abs(std::abs(dot) - 1.0) < k_epsilon_angle;
}

bool is_almost_same(const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2)
{
  return autoware_utils_geometry::calc_distance3d(p1, p2) < k_epsilon_distance;
}

bool is_almost_same(
  const autoware_planning_msgs::msg::PathPoint & p1,
  const autoware_planning_msgs::msg::PathPoint & p2)
{
  return autoware_utils_geometry::calc_distance3d(p1, p2) < k_epsilon_distance;
}

bool is_almost_same(
  const autoware_planning_msgs::msg::TrajectoryPoint & p1,
  const autoware_planning_msgs::msg::TrajectoryPoint & p2)
{
  return autoware_utils_geometry::calc_distance3d(p1, p2) < k_epsilon_distance;
}

bool is_almost_same(
  const autoware_internal_planning_msgs::msg::PathPointWithLaneId & p1,
  const autoware_internal_planning_msgs::msg::PathPointWithLaneId & p2)
{
  return autoware_utils_geometry::calc_distance3d(p1, p2) < k_epsilon_distance;
}

bool is_almost_same(const lanelet::ConstPoint3d & p1, const lanelet::ConstPoint3d & p2)
{
  return std::hypot(p1.x() - p2.x(), p1.y() - p2.y(), p1.z() - p2.z()) < k_epsilon_distance;
}
}  // namespace autoware::experimental::trajectory
