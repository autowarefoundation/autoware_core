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

#include "autoware/trajectory/threshold.hpp"
#include "autoware/trajectory/utils/pretty_build.hpp"
#include "autoware/trajectory/utils/velocity.hpp"
#include "autoware_utils_geometry/geometry.hpp"

#include <gtest/gtest.h>

#include <vector>

using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_utils_geometry::create_quaternion_from_yaw;
using geometry_msgs::build;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;

static TrajectoryPoint make_trajectory_point(double x, double y, double vx, double yaw = 0.0)
{
  TrajectoryPoint point;
  point.pose = build<Pose>()
                 .position(build<Point>().x(x).y(y).z(0.0))
                 .orientation(create_quaternion_from_yaw(yaw));
  point.longitudinal_velocity_mps = vx;
  point.lateral_velocity_mps = 0.0;
  point.heading_rate_rps = 0.0;
  return point;
}

namespace autoware::experimental::trajectory
{

TEST(search_zero_velocity_position, found_at_exact_base_point)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(make_trajectory_point(0.0, 0.0, 10.0));
  points.push_back(make_trajectory_point(1.0, 0.0, 5.0));
  points.push_back(make_trajectory_point(2.0, 0.0, 0.0));  // zero velocity here
  points.push_back(make_trajectory_point(3.0, 0.0, -5.0));

  auto traj = Trajectory<TrajectoryPoint>::Builder().build(points).value();

  auto result = search_zero_velocity_position(traj, 0.0, traj.length());
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result.value(), traj.length() * 2.0 / 3.0, k_zero_velocity_threshold);
}

TEST(search_zero_velocity_position, found_at_zero_crossing)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(make_trajectory_point(0.0, 0.0, 10.0));
  points.push_back(make_trajectory_point(1.0, 0.0, 5.0));
  points.push_back(make_trajectory_point(2.0, 0.0, -5.0));  // crosses zero between previous
  points.push_back(make_trajectory_point(3.0, 0.0, -10.0));

  auto traj = Trajectory<TrajectoryPoint>::Builder().build(points).value();

  auto result = search_zero_velocity_position(traj, 0.0, traj.length());
  ASSERT_TRUE(result.has_value());
  EXPECT_GT(result.value(), 0.0);
  EXPECT_LT(result.value(), traj.length());
}

TEST(search_zero_velocity_position, not_found_monotonic_positive)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(make_trajectory_point(0.0, 0.0, 10.0));
  points.push_back(make_trajectory_point(1.0, 0.0, 8.0));
  points.push_back(make_trajectory_point(2.0, 0.0, 5.0));
  points.push_back(make_trajectory_point(3.0, 0.0, 3.0));

  auto traj = Trajectory<TrajectoryPoint>::Builder().build(points).value();

  auto result = search_zero_velocity_position(traj, 0.0, traj.length());
  EXPECT_FALSE(result.has_value());
}

TEST(search_zero_velocity_position, not_found_monotonic_negative)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(make_trajectory_point(0.0, 0.0, -10.0));
  points.push_back(make_trajectory_point(1.0, 0.0, -8.0));
  points.push_back(make_trajectory_point(2.0, 0.0, -5.0));
  points.push_back(make_trajectory_point(3.0, 0.0, -3.0));

  auto traj = Trajectory<TrajectoryPoint>::Builder().build(points).value();

  auto result = search_zero_velocity_position(traj, 0.0, traj.length());
  EXPECT_FALSE(result.has_value());
}

TEST(search_zero_velocity_position, found_at_first_point)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(make_trajectory_point(0.0, 0.0, 0.0));
  points.push_back(make_trajectory_point(1.0, 0.0, 5.0));
  points.push_back(make_trajectory_point(2.0, 0.0, 10.0));
  points.push_back(make_trajectory_point(3.0, 0.0, 10.0));

  auto traj = Trajectory<TrajectoryPoint>::Builder().build(points).value();

  auto result = search_zero_velocity_position(traj, 0.0, traj.length());
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result.value(), 0.0, k_zero_velocity_threshold);
}

TEST(search_zero_velocity_position, found_at_last_point)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(make_trajectory_point(0.0, 0.0, 2.0));
  points.push_back(make_trajectory_point(1.0, 0.0, 5.0));
  points.push_back(make_trajectory_point(2.0, 0.0, 4.0));
  points.push_back(make_trajectory_point(3.0, 0.0, 0.0));

  auto traj = Trajectory<TrajectoryPoint>::Builder().build(points).value();

  auto result = search_zero_velocity_position(traj, 0.0, traj.length());
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result.value(), traj.length(), k_zero_velocity_threshold);
}

TEST(search_zero_velocity_position, found_with_interval)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(make_trajectory_point(0.0, 0.0, 10.0));
  points.push_back(make_trajectory_point(1.0, 0.0, -10.0));
  points.push_back(make_trajectory_point(2.0, 0.0, -15.0));
  points.push_back(make_trajectory_point(3.0, 0.0, -20.0));

  auto traj = Trajectory<TrajectoryPoint>::Builder().build(points).value();

  // Test with full range (no distance limits)
  auto result_full = search_zero_velocity_position(traj, 0.0, traj.length());
  ASSERT_TRUE(result_full.has_value());
  EXPECT_GT(result_full.value(), 0.0);
  EXPECT_LE(result_full.value(), traj.length());

  // Test with distance interval [0.0, 1.5m] - should find zero crossing
  auto result_interval = search_zero_velocity_position(traj, 0.0, 1.5);
  ASSERT_TRUE(result_interval.has_value());
  EXPECT_LE(result_interval.value(), 1.5);

  // Test with distance interval [2.0, 3.0m] - should not find zero crossing in this range
  auto result_no_crossing = search_zero_velocity_position(traj, 2.0, 3.0);
  EXPECT_FALSE(result_no_crossing.has_value());
}

}  // namespace autoware::experimental::trajectory
