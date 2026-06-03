// Copyright 2024 Tier IV, Inc.
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

#include "../src/pure_pursuit_util.hpp"

#include <autoware_utils_geometry/geometry.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace autoware::control::simple_pure_pursuit
{
namespace
{
geometry_msgs::msg::Point make_point(const double x, const double y, const double z = 0.0)
{
  return autoware_utils_geometry::create_point(x, y, z);
}

geometry_msgs::msg::Pose make_pose(const double x, const double y, const double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position = autoware_utils_geometry::create_point(x, y, 0.0);
  pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw);
  return pose;
}

TrajectoryPoint make_traj_point(const double x, const double y)
{
  TrajectoryPoint p;
  p.pose.position = make_point(x, y);
  return p;
}
}  // namespace

TEST(PurePursuitUtil, calc_rear_position)
{
  {  // heading 0: rear axle is offset along -x
    const auto rear = calc_rear_position(make_pose(10.0, 5.0, 0.0), 2.0);
    EXPECT_DOUBLE_EQ(rear.x, 9.0);
    EXPECT_DOUBLE_EQ(rear.y, 5.0);
  }
  {  // heading pi/2: rear axle is offset along -y
    const auto rear = calc_rear_position(make_pose(10.0, 5.0, M_PI / 2.0), 4.0);
    EXPECT_NEAR(rear.x, 10.0, 1e-9);
    EXPECT_NEAR(rear.y, 3.0, 1e-9);
  }
}

TEST(PurePursuitUtil, find_lookahead_point_normal)
{
  const std::vector<TrajectoryPoint> points{
    make_traj_point(0.0, 0.0), make_traj_point(1.0, 0.0), make_traj_point(2.0, 0.0),
    make_traj_point(3.0, 0.0)};

  // first point at or beyond 1.5 m from rear (0, 0) is (2, 0)
  const auto rear_position = make_point(0.0, 0.0);
  const auto & p = find_lookahead_point(points, 0, rear_position, 1.5);
  EXPECT_DOUBLE_EQ(p.pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(p.pose.position.y, 0.0);
}

TEST(PurePursuitUtil, find_lookahead_point_fallback_to_last)
{
  const std::vector<TrajectoryPoint> points{
    make_traj_point(0.0, 0.0), make_traj_point(1.0, 0.0), make_traj_point(2.0, 0.0)};

  // no point reaches a 100 m lookahead distance -> clamp to the last point (2, 0)
  const auto rear_position = make_point(0.0, 0.0);
  const auto & p = find_lookahead_point(points, 0, rear_position, 100.0);
  EXPECT_DOUBLE_EQ(p.pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(p.pose.position.y, 0.0);
}

TEST(PurePursuitUtil, calc_steering_tire_angle_straight)
{
  // lookahead point straight ahead -> alpha is 0 -> no steering
  const double steering =
    calc_steering_tire_angle(make_point(0.0, 0.0), make_point(2.0, 0.0), 0.0, 2.0, 2.0);
  EXPECT_DOUBLE_EQ(steering, 0.0);
}

TEST(PurePursuitUtil, calc_steering_tire_angle_curved)
{
  // rear=(0,0), lookahead=(1,1), heading=0, wheel_base=2, lookahead_distance=sqrt(2)
  // alpha = atan2(1, 1) - 0 = pi/4
  // steering = atan2(2 * 2 * sin(pi/4), sqrt(2)) = atan2(2*sqrt(2), sqrt(2)) = atan(2)
  const double lookahead_distance = std::sqrt(2.0);
  const double steering = calc_steering_tire_angle(
    make_point(0.0, 0.0), make_point(1.0, 1.0), 0.0, 2.0, lookahead_distance);
  EXPECT_NEAR(steering, std::atan(2.0), 1e-9);
  EXPECT_GT(steering, 0.0);
}

}  // namespace autoware::control::simple_pure_pursuit
