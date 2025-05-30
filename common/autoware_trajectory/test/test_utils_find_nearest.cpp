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

#include "autoware/trajectory/utils/find_nearest.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace
{
using autoware_planning_msgs::msg::Trajectory;
using autoware_utils_geometry::create_point;
using autoware_utils_geometry::create_quaternion_from_rpy;

geometry_msgs::msg::Pose create_pose(
  double x, double y, double z, double roll, double pitch, double yaw)
{
  geometry_msgs::msg::Pose p;
  p.position = create_point(x, y, z);
  p.orientation = create_quaternion_from_rpy(roll, pitch, yaw);
  return p;
}

template <class T>
T generate_test_trajectory(
  const size_t num_points, const double point_interval, const double vel = 0.0,
  const double init_theta = 0.0, const double delta_theta = 0.0)
{
  using Point = typename T::_points_type::value_type;

  T traj;
  for (size_t i = 0; i < num_points; ++i) {
    const double theta = init_theta + i * delta_theta;
    const double x = i * point_interval * std::cos(theta);
    const double y = i * point_interval * std::sin(theta);

    Point p;
    p.pose = create_pose(x, y, 0.0, 0.0, 0.0, theta);
    p.longitudinal_velocity_mps = vel;
    traj.points.push_back(p);
  }

  return traj;
}

TEST(trajectory, find_nearest_index_Pos_StraightTrajectory)
{
  using autoware::experimental::trajectory::find_nearest_index;

  const auto traj = generate_test_trajectory<Trajectory>(10, 1.0);

  // Empty
  try {
    [[maybe_unused]] auto retval =
      find_nearest_index(Trajectory{}.points, geometry_msgs::msg::Point{});
    FAIL() << "Expected std::invalid_argument exception, but no exception was thrown.";
  } catch (const std::invalid_argument &) {
    SUCCEED();
  } catch (...) {
    FAIL() << "Expected std::invalid_argument exception, but a different exception was thrown.";
  }

  // Start point
  EXPECT_EQ(find_nearest_index(traj.points, create_point(0.0, 0.0, 0.0)), 0U);

  // End point
  EXPECT_EQ(find_nearest_index(traj.points, create_point(9.0, 0.0, 0.0)), 9U);

  // Boundary conditions
  EXPECT_EQ(find_nearest_index(traj.points, create_point(0.5, 0.0, 0.0)), 0U);
  EXPECT_EQ(find_nearest_index(traj.points, create_point(0.51, 0.0, 0.0)), 1U);

  // Point before start point
  EXPECT_EQ(find_nearest_index(traj.points, create_point(-4.0, 5.0, 0.0)), 0U);

  // Point after end point
  EXPECT_EQ(find_nearest_index(traj.points, create_point(100.0, -3.0, 0.0)), 9U);

  // Random cases
  EXPECT_EQ(find_nearest_index(traj.points, create_point(2.4, 1.3, 0.0)), 2U);
  EXPECT_EQ(find_nearest_index(traj.points, create_point(4.0, 0.0, 0.0)), 4U);
}

TEST(trajectory, find_nearest_index_Pos_CurvedTrajectory)
{
  using autoware::experimental::trajectory::find_nearest_index;

  const auto traj = generate_test_trajectory<Trajectory>(10, 1.0, 0.0, 0.0, 0.1);

  // Random cases
  EXPECT_EQ(find_nearest_index(traj.points, create_point(5.1, 3.4, 0.0)), 6U);
}

TEST(trajectory, find_nearest_index_Pose_NoThreshold)
{
  using autoware::experimental::trajectory::find_nearest_index;

  const auto traj = generate_test_trajectory<Trajectory>(10, 1.0);

  // Empty
  EXPECT_FALSE(find_nearest_index(Trajectory{}.points, geometry_msgs::msg::Pose{}, {}));

  // Start point
  EXPECT_EQ(*find_nearest_index(traj.points, create_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)), 0U);

  // End point
  EXPECT_EQ(*find_nearest_index(traj.points, create_pose(9.0, 0.0, 0.0, 0.0, 0.0, 0.0)), 9U);

  // Boundary conditions
  EXPECT_EQ(*find_nearest_index(traj.points, create_pose(0.5, 0.0, 0.0, 0.0, 0.0, 0.0)), 0U);
  EXPECT_EQ(*find_nearest_index(traj.points, create_pose(0.51, 0.0, 0.0, 0.0, 0.0, 0.0)), 1U);

  // Point before start point
  EXPECT_EQ(*find_nearest_index(traj.points, create_pose(-4.0, 5.0, 0.0, 0.0, 0.0, 0.0)), 0U);

  // Point after end point
  EXPECT_EQ(*find_nearest_index(traj.points, create_pose(100.0, -3.0, 0.0, 0.0, 0.0, 0.0)), 9U);
}

TEST(trajectory, find_nearest_index_Pose_DistThreshold)
{
  using autoware::experimental::trajectory::find_nearest_index;

  const auto traj = generate_test_trajectory<Trajectory>(10, 1.0);

  // Out of threshold
  EXPECT_FALSE(find_nearest_index(traj.points, create_pose(3.0, 0.6, 0.0, 0.0, 0.0, 0.0), 0.5));

  // On threshold
  EXPECT_EQ(*find_nearest_index(traj.points, create_pose(3.0, 0.5, 0.0, 0.0, 0.0, 0.0), 0.5), 3U);

  // Within threshold
  EXPECT_EQ(*find_nearest_index(traj.points, create_pose(3.0, 0.4, 0.0, 0.0, 0.0, 0.0), 0.5), 3U);
}

TEST(trajectory, find_nearest_index_Pose_YawThreshold)
{
  using autoware::experimental::trajectory::find_nearest_index;

  const auto traj = generate_test_trajectory<Trajectory>(10, 1.0);
  const auto max_d = std::numeric_limits<double>::max();

  // Out of threshold
  EXPECT_FALSE(
    find_nearest_index(traj.points, create_pose(3.0, 0.0, 0.0, 0.0, 0.0, 1.1), max_d, 1.0));

  // On threshold
  EXPECT_EQ(
    *find_nearest_index(traj.points, create_pose(3.0, 0.0, 0.0, 0.0, 0.0, 1.0), max_d, 1.0), 3U);

  // Within threshold
  EXPECT_EQ(
    *find_nearest_index(traj.points, create_pose(3.0, 0.0, 0.0, 0.0, 0.0, 0.9), max_d, 1.0), 3U);
}

TEST(trajectory, find_nearest_index_Pose_DistAndYawThreshold)
{
  using autoware::experimental::trajectory::find_nearest_index;

  const auto traj = generate_test_trajectory<Trajectory>(10, 1.0);

  // Random cases
  EXPECT_EQ(
    *find_nearest_index(traj.points, create_pose(2.4, 1.3, 0.0, 0.0, 0.0, 0.3), 2.0, 0.4), 2U);
  EXPECT_EQ(
    *find_nearest_index(traj.points, create_pose(4.1, 0.3, 0.0, 0.0, 0.0, -0.8), 0.5, 1.0), 4U);
  EXPECT_EQ(
    *find_nearest_index(traj.points, create_pose(8.5, -0.5, 0.0, 0.0, 0.0, 0.0), 1.0, 0.1), 8U);
}
}  // namespace
