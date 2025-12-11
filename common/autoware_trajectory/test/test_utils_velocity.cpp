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

#include "autoware/trajectory/utils/pretty_build.hpp"
#include "autoware/trajectory/utils/velocity.hpp"
#include "autoware_utils_geometry/geometry.hpp"

#include <gtest/gtest.h>

using autoware::experimental::trajectory::searchZeroVelocityPosition;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_utils_geometry::create_quaternion_from_yaw;
using geometry_msgs::build;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;

// Helper to create a trajectory point with position and velocity
static TrajectoryPoint make_trajectory_point(
  double x, double y, double vx, double yaw = 0.0)
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

TEST(searchZeroVelocityPosition, found_at_exact_base_point)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(make_trajectory_point(0.0, 0.0, 10.0));
  points.push_back(make_trajectory_point(1.0, 0.0, 5.0));
  points.push_back(make_trajectory_point(2.0, 0.0, 0.0));  // zero velocity here
  points.push_back(make_trajectory_point(3.0, 0.0, -5.0));

  auto traj = autoware::experimental::trajectory::Trajectory<TrajectoryPoint>::Builder()
                .build(points)
                .value();

  auto result = searchZeroVelocityPosition(traj);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result.value(), traj.length() * 2.0 / 3.0, 0.1);
}

TEST(searchZeroVelocityPosition, found_at_zero_crossing)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(make_trajectory_point(0.0, 0.0, 10.0));
  points.push_back(make_trajectory_point(1.0, 0.0, 5.0));
  points.push_back(make_trajectory_point(2.0, 0.0, -5.0));  // crosses zero between previous
  points.push_back(make_trajectory_point(3.0, 0.0, -10.0));

  auto traj = autoware::experimental::trajectory::Trajectory<TrajectoryPoint>::Builder()
                .build(points)
                .value();

  auto result = searchZeroVelocityPosition(traj);
  ASSERT_TRUE(result.has_value());
  EXPECT_GT(result.value(), 0.0);
  EXPECT_LT(result.value(), traj.length());
}

TEST(searchZeroVelocityPosition, not_found_monotonic_positive)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(make_trajectory_point(0.0, 0.0, 10.0));
  points.push_back(make_trajectory_point(1.0, 0.0, 8.0));
  points.push_back(make_trajectory_point(2.0, 0.0, 5.0));
  points.push_back(make_trajectory_point(3.0, 0.0, 3.0));

  auto traj = autoware::experimental::trajectory::Trajectory<TrajectoryPoint>::Builder()
                .build(points)
                .value();

  auto result = searchZeroVelocityPosition(traj);
  EXPECT_FALSE(result.has_value());
}

TEST(searchZeroVelocityPosition, not_found_monotonic_negative)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(make_trajectory_point(0.0, 0.0, -10.0));
  points.push_back(make_trajectory_point(1.0, 0.0, -8.0));
  points.push_back(make_trajectory_point(2.0, 0.0, -5.0));
  points.push_back(make_trajectory_point(3.0, 0.0, -3.0));

  auto traj = autoware::experimental::trajectory::Trajectory<TrajectoryPoint>::Builder()
                .build(points)
                .value();

  auto result = searchZeroVelocityPosition(traj);
  EXPECT_FALSE(result.has_value());
}

TEST(searchZeroVelocityPosition, found_at_first_point)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(make_trajectory_point(0.0, 0.0, 0.0));
  points.push_back(make_trajectory_point(1.0, 0.0, 5.0));
  points.push_back(make_trajectory_point(2.0, 0.0, 10.0));
  points.push_back(make_trajectory_point(3.0, 0.0, 10.0));

  auto traj = autoware::experimental::trajectory::Trajectory<TrajectoryPoint>::Builder()
                .build(points)
                .value();

  auto result = searchZeroVelocityPosition(traj);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result.value(), 0.0, 1e-6);
}

TEST(searchZeroVelocityPosition, refined_with_binary_search)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(make_trajectory_point(0.0, 0.0, 10.0));
  points.push_back(make_trajectory_point(1.0, 0.0, -10.0));
  points.push_back(make_trajectory_point(2.0, 0.0, -15.0));
  points.push_back(make_trajectory_point(3.0, 0.0, -20.0));

  auto traj = autoware::experimental::trajectory::Trajectory<TrajectoryPoint>::Builder()
                .build(points)
                .value();

  auto result1 = searchZeroVelocityPosition(traj, 5);
  auto result2 = searchZeroVelocityPosition(traj, 10);
  auto result3 = searchZeroVelocityPosition(traj, 20);

  ASSERT_TRUE(result1.has_value());
  ASSERT_TRUE(result2.has_value());
  ASSERT_TRUE(result3.has_value());

  double dist_to_zero_1 = std::abs(traj.longitudinal_velocity_mps().compute(result1.value()));
  double dist_to_zero_2 = std::abs(traj.longitudinal_velocity_mps().compute(result2.value()));
  double dist_to_zero_3 = std::abs(traj.longitudinal_velocity_mps().compute(result3.value()));

  EXPECT_GE(dist_to_zero_1, dist_to_zero_2);
  EXPECT_GE(dist_to_zero_2, dist_to_zero_3);
}
