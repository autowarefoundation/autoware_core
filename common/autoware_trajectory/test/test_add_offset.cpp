// Copyright 2026 TIER IV, Inc.
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

#include "autoware/trajectory/trajectory_point.hpp"
#include "autoware/trajectory/utils/add_offset.hpp"

#include <autoware_utils_geometry/geometry.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_utils_geometry::create_quaternion_from_yaw;
using geometry_msgs::build;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;

namespace autoware::experimental::trajectory
{

static TrajectoryPoint make_trajectory_point(double x, double y, double yaw, double vx = 1.0)
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

TEST(add_offset, forward_offset_shifts_along_heading)
{
  // Straight trajectory along +x axis
  std::vector<TrajectoryPoint> points;
  points.push_back(make_trajectory_point(0.0, 0.0, 0.0));
  points.push_back(make_trajectory_point(1.0, 0.0, 0.0));
  points.push_back(make_trajectory_point(2.0, 0.0, 0.0));

  auto traj = Trajectory<TrajectoryPoint>::Builder().build(points).value();
  const double forward_offset = 2.0;  // 2m forward

  auto offset_traj = add_offset(traj, forward_offset, 0.0);

  // Each point should be shifted 2m in +x direction (heading is 0)
  EXPECT_NEAR(offset_traj.compute(0.0).pose.position.x, 2.0, 1e-6);
  EXPECT_NEAR(offset_traj.compute(0.0).pose.position.y, 0.0, 1e-6);
  EXPECT_NEAR(offset_traj.compute(traj.length()).pose.position.x, 4.0, 1e-6);
  EXPECT_NEAR(offset_traj.compute(traj.length()).pose.position.y, 0.0, 1e-6);
}

TEST(add_offset, lateral_offset_shifts_to_left)
{
  // Straight trajectory along +x axis
  std::vector<TrajectoryPoint> points;
  points.push_back(make_trajectory_point(0.0, 0.0, 0.0));
  points.push_back(make_trajectory_point(1.0, 0.0, 0.0));
  points.push_back(make_trajectory_point(2.0, 0.0, 0.0));

  auto traj = Trajectory<TrajectoryPoint>::Builder().build(points).value();
  const double lateral_offset = 1.5;  // 1.5m to the left

  auto offset_traj = add_offset(traj, 0.0, lateral_offset);

  // Each point should be shifted 1.5m in +y direction (left when heading is 0)
  EXPECT_NEAR(offset_traj.compute(0.0).pose.position.x, 0.0, 1e-6);
  EXPECT_NEAR(offset_traj.compute(0.0).pose.position.y, 1.5, 1e-6);
  EXPECT_NEAR(offset_traj.compute(traj.length()).pose.position.x, 2.0, 1e-6);
  EXPECT_NEAR(offset_traj.compute(traj.length()).pose.position.y, 1.5, 1e-6);
}

TEST(add_offset, combined_offset_respects_heading_rotation)
{
  // Trajectory with 90 degree heading (+y direction)
  std::vector<TrajectoryPoint> points;
  const double yaw_90 = M_PI_2;
  points.push_back(make_trajectory_point(0.0, 0.0, yaw_90));
  points.push_back(make_trajectory_point(0.0, 1.0, yaw_90));
  points.push_back(make_trajectory_point(0.0, 2.0, yaw_90));

  auto traj = Trajectory<TrajectoryPoint>::Builder().build(points).value();
  const double forward_offset = 1.0;  // 1m forward (in vehicle frame = +y in global)
  const double lateral_offset = 0.5;  // 0.5m left (in vehicle frame = -x in global)

  auto offset_traj = add_offset(traj, forward_offset, lateral_offset);

  // At heading=90deg: forward offset -> +y, lateral offset -> -x
  EXPECT_NEAR(offset_traj.compute(0.0).pose.position.x, -0.5, 1e-6);
  EXPECT_NEAR(offset_traj.compute(0.0).pose.position.y, 1.0, 1e-6);
  EXPECT_NEAR(offset_traj.compute(traj.length()).pose.position.x, -0.5, 1e-6);
  EXPECT_NEAR(offset_traj.compute(traj.length()).pose.position.y, 3.0, 1e-6);
}

TEST(add_offset, preserves_trajectory_length)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(make_trajectory_point(0.0, 0.0, 0.0));
  points.push_back(make_trajectory_point(1.0, 0.0, 0.0));
  points.push_back(make_trajectory_point(2.0, 0.0, 0.0));

  auto traj = Trajectory<TrajectoryPoint>::Builder().build(points).value();
  auto offset_traj = add_offset(traj, 1.0, 0.5);

  // Length should be preserved for a straight trajectory
  EXPECT_NEAR(offset_traj.length(), traj.length(), 1e-6);
}

TEST(add_offset, zero_offset_returns_identical_trajectory)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(make_trajectory_point(0.0, 0.0, 0.0));
  points.push_back(make_trajectory_point(1.0, 0.0, 0.0));
  points.push_back(make_trajectory_point(2.0, 0.0, 0.0));

  auto traj = Trajectory<TrajectoryPoint>::Builder().build(points).value();
  auto offset_traj = add_offset(traj, 0.0, 0.0);

  // Points should be identical
  for (const auto s : traj.get_underlying_bases()) {
    const auto orig = traj.compute(s);
    const auto offset = offset_traj.compute(s);
    EXPECT_NEAR(orig.pose.position.x, offset.pose.position.x, 1e-6);
    EXPECT_NEAR(orig.pose.position.y, offset.pose.position.y, 1e-6);
    EXPECT_NEAR(orig.pose.position.z, offset.pose.position.z, 1e-6);
  }
}

TEST(add_offset, negative_lateral_offset_shifts_right)
{
  // Straight trajectory along +x axis
  std::vector<TrajectoryPoint> points;
  points.push_back(make_trajectory_point(0.0, 0.0, 0.0));
  points.push_back(make_trajectory_point(1.0, 0.0, 0.0));
  points.push_back(make_trajectory_point(2.0, 0.0, 0.0));

  auto traj = Trajectory<TrajectoryPoint>::Builder().build(points).value();
  const double lateral_offset = -1.0;  // 1m to the right

  auto offset_traj = add_offset(traj, 0.0, lateral_offset);

  // Each point should be shifted 1m in -y direction (right when heading is 0)
  EXPECT_NEAR(offset_traj.compute(0.0).pose.position.y, -1.0, 1e-6);
  EXPECT_NEAR(offset_traj.compute(traj.length()).pose.position.y, -1.0, 1e-6);
}

}  // namespace autoware::experimental::trajectory
