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

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/trajectory/detail/types.hpp"
#include "autoware/trajectory/pose.hpp"
#include "autoware/trajectory/threshold.hpp"
#include "autoware/trajectory/utils/find_nearest.hpp"
#include "autoware/trajectory/utils/lateral_metrics.hpp"

#include <gtest/gtest.h>

#include <fstream>
#include <limits>
#include <string>
#include <vector>

namespace
{
using autoware::experimental::trajectory::Trajectory;
using autoware_utils_geometry::calc_squared_distance2d;
using autoware_utils_geometry::create_point;
using autoware_utils_geometry::create_quaternion_from_rpy;

static Trajectory<geometry_msgs::msg::Pose> build_horizontal_line(
  const size_t num_points, const double interval)
{
  std::vector<geometry_msgs::msg::Pose> raw_poses;
  raw_poses.reserve(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    double x = static_cast<double>(i) * interval;
    double y = 0;
    geometry_msgs::msg::Pose p;
    p.position = create_point(x, y, 0.0);
    p.orientation = create_quaternion_from_rpy(0.0, 0.0, 0.0);
    raw_poses.push_back(p);
  }

  auto traj = Trajectory<geometry_msgs::msg::Pose>::Builder{}.build(raw_poses);
  return traj.value();
}

static Trajectory<geometry_msgs::msg::Pose> build_parabolic_trajectory(
  const size_t num_points, const double interval, const bool reverse = false)
{
  std::vector<geometry_msgs::msg::Pose> raw_poses;
  raw_poses.reserve(num_points);

  double half = static_cast<double>(num_points - 1) / 2.0;
  for (size_t i = 0; i < num_points; ++i) {
    double x;
    double y;
    double yaw;
    if (!reverse) {
      x = (static_cast<double>(i) - half) * interval;
      y = x * x;                       // Parabola: y = x^2
      yaw = std::atan2(2.0 * x, 1.0);  // Tangent direction
    } else {
      x = (-1 * static_cast<double>(i) + half) * interval;
      y = x * x;                              // Parabola: y = x^2
      yaw = std::atan2(2.0 * x, 1.0) - M_PI;  // Tangent direction
    }
    geometry_msgs::msg::Pose p;
    p.position = create_point(x, y, 0.0);
    p.orientation = create_quaternion_from_rpy(0.0, 0.0, yaw);
    raw_poses.push_back(p);
  }

  auto traj = Trajectory<geometry_msgs::msg::Pose>::Builder{}.build(raw_poses);
  return traj.value();
}

// Test 1: Simple test on horizontal line
TEST(lateral_metrics, horizontalLine)
{
  auto traj = build_horizontal_line(10, 1.0);
  geometry_msgs::msg::Point target_point;
  target_point.x = 5;
  target_point.y = 5;
  target_point.z = 0;

  const auto nearest_s = autoware::experimental::trajectory::find_nearest_index(traj, target_point);
  auto result =
    autoware::experimental::trajectory::compute_lateral_distance(traj, target_point, nearest_s);

  ASSERT_EQ(result, 5.0) << "It should be exactly 5.0!";

  auto on_left = autoware::experimental::trajectory::is_left_side(traj, target_point, nearest_s);
  ASSERT_TRUE(on_left);
}

// Test 2: Parabolic Trajectory (not at nearest index)
TEST(lateral_metrics, parabolicTrajectoryNotNearest)
{
  auto traj = build_parabolic_trajectory(11, 0.5);
  geometry_msgs::msg::Point target_point;
  target_point.x = 1;
  target_point.y = 0.5;
  target_point.z = 0;

  const auto nearest_s = autoware::experimental::trajectory::find_nearest_index(traj, target_point);
  // test s that is moved slightly from nearest_s
  const auto target_s = nearest_s - 0.5;

  const auto longitudinal_offset =
    autoware_utils_geometry::calc_longitudinal_deviation(traj.compute(target_s), target_point);
  const auto vertical_point =
    autoware_utils_geometry::calc_offset_pose(traj.compute(target_s), longitudinal_offset, 0, 0, 0);

  const auto check_distance =
    autoware_utils_geometry::calc_distance2d(target_point, vertical_point);
  // Note: x= 0.479068, y = 0.230792 gives distance of 0.166013 (but it's not correct because of
  // rounded value)
  auto result =
    autoware::experimental::trajectory::compute_lateral_distance(traj, target_point, target_s);

  EXPECT_EQ(result, check_distance);

  auto on_left = autoware::experimental::trajectory::is_left_side(traj, target_point, nearest_s);
  ASSERT_FALSE(on_left);
}

}  // namespace
