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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <cmath>
#include <vector>

using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_utils_geometry::create_quaternion_from_rpy;
using geometry_msgs::build;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;

namespace autoware::experimental::trajectory
{

static TrajectoryPoint make_trajectory_point(
  double x, double y, double z = 0.0, double roll = 0.0, double pitch = 0.0, double yaw = 0.0)
{
  TrajectoryPoint point;
  point.pose = build<Pose>()
                 .position(build<Point>().x(x).y(y).z(z))
                 .orientation(create_quaternion_from_rpy(roll, pitch, yaw));
  return point;
}

static void expect_offset_at(
  const Trajectory<TrajectoryPoint> & trajectory, const double s, const double offset_x,
  const double offset_y, const double offset_z = 0.0, const double tolerance = 1e-6)
{
  const auto original = trajectory.compute(s);
  const auto offset_trajectory = add_offset(trajectory, offset_x, offset_y, offset_z);
  const auto shifted = offset_trajectory.compute(s);

  tf2::Quaternion orientation(
    original.pose.orientation.x, original.pose.orientation.y, original.pose.orientation.z,
    original.pose.orientation.w);
  orientation.normalize();
  const auto expected_offset =
    tf2::quatRotate(orientation, tf2::Vector3(offset_x, offset_y, offset_z));

  EXPECT_NEAR(shifted.pose.position.x, original.pose.position.x + expected_offset.x(), tolerance);
  EXPECT_NEAR(shifted.pose.position.y, original.pose.position.y + expected_offset.y(), tolerance);
  EXPECT_NEAR(shifted.pose.position.z, original.pose.position.z + expected_offset.z(), tolerance);
}

TEST(AddOffset, ForwardOffsetShiftsAlongHeading)
{
  // Straight trajectory along +x axis
  std::vector<TrajectoryPoint> points;
  points.emplace_back(make_trajectory_point(0.0, 0.0));
  points.emplace_back(make_trajectory_point(1.0, 0.0));
  points.emplace_back(make_trajectory_point(2.0, 0.0));

  auto traj = Trajectory<TrajectoryPoint>::Builder().build(points).value();
  const double forward_offset = 2.0;  // 2m forward

  expect_offset_at(traj, 0.0, forward_offset, 0.0);
  expect_offset_at(traj, traj.length(), forward_offset, 0.0);
}

TEST(AddOffset, LateralOffsetShiftsToLeft)
{
  // Straight trajectory along +x axis
  std::vector<TrajectoryPoint> points;
  points.emplace_back(make_trajectory_point(0.0, 0.0));
  points.emplace_back(make_trajectory_point(1.0, 0.0));
  points.emplace_back(make_trajectory_point(2.0, 0.0));

  auto traj = Trajectory<TrajectoryPoint>::Builder().build(points).value();
  const double lateral_offset = 1.5;  // 1.5m to the left

  expect_offset_at(traj, 0.0, 0.0, lateral_offset);
  expect_offset_at(traj, traj.length(), 0.0, lateral_offset);
}

TEST(AddOffset, CombinedOffsetRespectsHeadingRotation)
{
  // Trajectory with 90 degree heading (+y direction)
  std::vector<TrajectoryPoint> points;
  const double yaw_90 = M_PI_2;
  points.emplace_back(make_trajectory_point(0.0, 0.0, 0.0, 0.0, 0.0, yaw_90));
  points.emplace_back(make_trajectory_point(0.0, 1.0, 0.0, 0.0, 0.0, yaw_90));
  points.emplace_back(make_trajectory_point(0.0, 2.0, 0.0, 0.0, 0.0, yaw_90));

  auto traj = Trajectory<TrajectoryPoint>::Builder().build(points).value();
  const double forward_offset = 1.0;  // 1m forward (in vehicle frame = +y in global)
  const double lateral_offset = 0.5;  // 0.5m left (in vehicle frame = -x in global)

  expect_offset_at(traj, 0.0, forward_offset, lateral_offset);
  expect_offset_at(traj, traj.length(), forward_offset, lateral_offset);
}

TEST(AddOffset, PreservesTrajectoryLength)
{
  std::vector<TrajectoryPoint> points;
  points.emplace_back(make_trajectory_point(0.0, 0.0));
  points.emplace_back(make_trajectory_point(1.0, 0.0));
  points.emplace_back(make_trajectory_point(2.0, 0.0));

  auto traj = Trajectory<TrajectoryPoint>::Builder().build(points).value();
  auto offset_traj = add_offset(traj, 1.0, 0.5);

  // Length should be preserved for a straight trajectory
  EXPECT_NEAR(offset_traj.length(), traj.length(), 1e-6);
}

TEST(AddOffset, ZeroOffsetReturnsIdenticalTrajectory)
{
  std::vector<TrajectoryPoint> points;
  points.emplace_back(make_trajectory_point(0.0, 0.0));
  points.emplace_back(make_trajectory_point(1.0, 0.0));
  points.emplace_back(make_trajectory_point(2.0, 0.0));

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

TEST(AddOffset, NegativeLateralOffsetShiftsRight)
{
  // Straight trajectory along +x axis
  std::vector<TrajectoryPoint> points;
  points.emplace_back(make_trajectory_point(0.0, 0.0));
  points.emplace_back(make_trajectory_point(1.0, 0.0));
  points.emplace_back(make_trajectory_point(2.0, 0.0));

  auto traj = Trajectory<TrajectoryPoint>::Builder().build(points).value();
  const double lateral_offset = -1.0;  // 1m to the right

  expect_offset_at(traj, 0.0, 0.0, lateral_offset);
  expect_offset_at(traj, traj.length(), 0.0, lateral_offset);
}

TEST(AddOffset, SinglePointCombinedOffset)
{
  // Single point trajectory at (1.0, 2.0, 0.5) with 45-degree yaw and 30-degree pitch
  const double yaw = M_PI_4;        // 45 degrees
  const double pitch = M_PI / 6.0;  // 30 degrees
  std::vector<TrajectoryPoint> points;
  points.emplace_back(make_trajectory_point(1.0, 2.0, 0.5, 0.0, pitch, yaw));

  auto traj = Trajectory<TrajectoryPoint>::Builder().build(points).value();

  const double offset_x = 3.0;  // 3m forward in vehicle frame
  const double offset_y = 1.0;  // 1m left in vehicle frame
  const double offset_z = 0.5;  // 0.5m up in vehicle frame

  auto offset_traj = add_offset(traj, offset_x, offset_y, offset_z);

  expect_offset_at(traj, 0.0, offset_x, offset_y, offset_z);

  // Trajectory should have exactly one underlying base
  EXPECT_EQ(offset_traj.get_underlying_bases().size(), 1u);

  // Length of a single-point trajectory is zero
  EXPECT_NEAR(offset_traj.length(), 0.0, 1e-9);
}

TEST(AddOffset, VerticalOffsetShiftsZAtZeroPitch)
{
  // Flat trajectory (zero pitch) along +x axis: offset_z should translate directly into global z
  std::vector<TrajectoryPoint> points;
  points.emplace_back(make_trajectory_point(0.0, 0.0));
  points.emplace_back(make_trajectory_point(1.0, 0.0));
  points.emplace_back(make_trajectory_point(2.0, 0.0));

  auto traj = Trajectory<TrajectoryPoint>::Builder().build(points).value();
  const double offset_z = 1.5;  // 1.5m up in vehicle frame

  expect_offset_at(traj, 0.0, 0.0, 0.0, offset_z);
  expect_offset_at(traj, traj.length(), 0.0, 0.0, offset_z);
}

TEST(AddOffset, ForwardOffsetOnPitchedTrajectoryShiftsZ)
{
  // Pitched trajectory (nose-up 30 deg, heading 0): forward offset_x should lift the z
  const double pitch = M_PI / 6.0;  // 30 degrees nose-up
  std::vector<TrajectoryPoint> points;
  points.emplace_back(make_trajectory_point(0.0, 0.0, 0.0, 0.0, pitch));
  points.emplace_back(make_trajectory_point(1.0, 0.0, 0.0, 0.0, pitch));
  points.emplace_back(make_trajectory_point(2.0, 0.0, 0.0, 0.0, pitch));

  auto traj = Trajectory<TrajectoryPoint>::Builder().build(points).value();
  const double offset_x = 2.0;  // 2m forward in vehicle frame

  expect_offset_at(traj, 0.0, offset_x, 0.0, 0.0);
}

TEST(AddOffset, ExplicitZeroOffsetZMatchesDefaultArgument)
{
  // Verify that omitting offset_z behaves exactly the same as passing 0.0 explicitly.
  std::vector<TrajectoryPoint> points;
  points.emplace_back(make_trajectory_point(0.0, 0.0, 1.0, 0.0, 0.1));
  points.emplace_back(make_trajectory_point(1.0, 0.0, 1.0, 0.0, 0.1));
  points.emplace_back(make_trajectory_point(2.0, 0.0, 1.0, 0.0, 0.1));

  auto traj = Trajectory<TrajectoryPoint>::Builder().build(points).value();
  auto default_offset_traj = add_offset(traj, 1.0, 0.5);
  auto explicit_offset_traj = add_offset(traj, 1.0, 0.5, 0.0);

  for (const auto s : traj.get_underlying_bases()) {
    const auto default_offset = default_offset_traj.compute(s);
    const auto explicit_offset = explicit_offset_traj.compute(s);
    EXPECT_NEAR(default_offset.pose.position.x, explicit_offset.pose.position.x, 1e-6);
    EXPECT_NEAR(default_offset.pose.position.y, explicit_offset.pose.position.y, 1e-6);
    EXPECT_NEAR(default_offset.pose.position.z, explicit_offset.pose.position.z, 1e-6);
  }
}

TEST(AddOffset, LateralOffsetOnRolledTrajectoryShiftsZ)
{
  // With positive roll, a left offset should also move upward in the global frame.
  const double roll = M_PI / 6.0;  // 30 degrees
  std::vector<TrajectoryPoint> points;
  points.emplace_back(make_trajectory_point(0.0, 0.0, 0.0, roll, 0.0, 0.0));
  points.emplace_back(make_trajectory_point(1.0, 0.0, 0.0, roll, 0.0, 0.0));
  points.emplace_back(make_trajectory_point(2.0, 0.0, 0.0, roll, 0.0, 0.0));

  auto traj = Trajectory<TrajectoryPoint>::Builder().build(points).value();
  const double offset_y = 2.0;

  auto offset_traj = add_offset(traj, 0.0, offset_y, 0.0);

  expect_offset_at(traj, 0.0, 0.0, offset_y, 0.0);
}

TEST(add_offset, single_point_combined_offset)
{
  // Single point trajectory at (1.0, 2.0) with 45-degree heading
  const double yaw = M_PI_4;  // 45 degrees
  std::vector<TrajectoryPoint> points;
  points.push_back(make_trajectory_point(1.0, 2.0, yaw));

  auto traj = Trajectory<TrajectoryPoint>::Builder().build(points).value();

  const double offset_x = 3.0;  // 3m forward in vehicle frame
  const double offset_y = 1.0;  // 1m left in vehicle frame

  auto offset_traj = add_offset(traj, offset_x, offset_y);

  // Compute expected global offsets:
  // global_offset_x = cos(45deg)*3 - sin(45deg)*1 = (sqrt(2)/2)*(3-1) = sqrt(2)
  // global_offset_y = sin(45deg)*3 + cos(45deg)*1 = (sqrt(2)/2)*(3+1) = 2*sqrt(2)
  const double expected_x = 1.0 + std::cos(yaw) * offset_x - std::sin(yaw) * offset_y;
  const double expected_y = 2.0 + std::sin(yaw) * offset_x + std::cos(yaw) * offset_y;

  const auto result = offset_traj.compute(0.0);
  EXPECT_NEAR(result.pose.position.x, expected_x, 1e-6);
  EXPECT_NEAR(result.pose.position.y, expected_y, 1e-6);
  EXPECT_NEAR(result.pose.position.z, 0.0, 1e-6);

  // Trajectory should have exactly one underlying base
  EXPECT_EQ(offset_traj.get_underlying_bases().size(), 1u);

  // Length of a single-point trajectory is zero
  EXPECT_NEAR(offset_traj.length(), 0.0, 1e-9);
}

}  // namespace autoware::experimental::trajectory
