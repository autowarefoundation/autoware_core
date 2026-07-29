// Copyright 2026 TIER IV
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

#include "../src/path_to_trajectory.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <vector>

namespace
{
autoware_planning_msgs::msg::PathPoint make_path_point(
  const double x, const double y, const float longitudinal_velocity, const float lateral_velocity,
  const float heading_rate)
{
  autoware_planning_msgs::msg::PathPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.position.z = 0.0;
  point.pose.orientation.w = 1.0;
  point.longitudinal_velocity_mps = longitudinal_velocity;
  point.lateral_velocity_mps = lateral_velocity;
  point.heading_rate_rps = heading_rate;
  return point;
}

// Builds a PathPoint from raw pose + kinematics values
autoware_planning_msgs::msg::PathPoint make_boundary_path_point(
  const double x, const double y, const double z, const double qx, const double qy, const double qz,
  const double qw, const float longitudinal_velocity, const float lateral_velocity,
  const float heading_rate)
{
  autoware_planning_msgs::msg::PathPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.position.z = z;
  point.pose.orientation.x = qx;
  point.pose.orientation.y = qy;
  point.pose.orientation.z = qz;
  point.pose.orientation.w = qw;
  point.longitudinal_velocity_mps = longitudinal_velocity;
  point.lateral_velocity_mps = lateral_velocity;
  point.heading_rate_rps = heading_rate;
  return point;
}

autoware_planning_msgs::msg::Path make_path(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points)
{
  autoware_planning_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.points = points;
  return path;
}
}  // namespace

namespace autoware::planning_topic_converter
{

TEST(PathToTrajectoryConvert, ConvertsPathPointsToTrajectoryPoints_BoundaryValues)
{
  constexpr double d_low = std::numeric_limits<double>::lowest();
  constexpr double d_zero = 0.0;
  constexpr double d_max = std::numeric_limits<double>::max();
  constexpr float f_low = std::numeric_limits<float>::lowest();
  constexpr float f_zero = 0.0f;
  constexpr float f_max = std::numeric_limits<float>::max();

  const auto path = make_path({
    make_boundary_path_point(d_low, d_low, d_low, d_low, d_low, d_low, d_low, f_low, f_low, f_low),
    make_boundary_path_point(
      d_zero, d_zero, d_zero, d_zero, d_zero, d_zero, d_zero, f_zero, f_zero, f_zero),
    make_boundary_path_point(d_max, d_max, d_max, d_max, d_max, d_max, d_max, f_max, f_max, f_max),
  });

  // Act
  const auto trajectory = path_to_trajectory::convert(path);

  // Assert
  // Header is copied verbatim from the input path.
  EXPECT_EQ(trajectory.header.frame_id, "map");

  ASSERT_EQ(trajectory.points.size(), path.points.size());
  for (size_t i = 0; i < path.points.size(); ++i) {
    const auto & path_point = path.points[i];
    const auto & traj_point = trajectory.points[i];

    // Pose is mapped directly without any transformation.
    EXPECT_DOUBLE_EQ(traj_point.pose.position.x, path_point.pose.position.x);
    EXPECT_DOUBLE_EQ(traj_point.pose.position.y, path_point.pose.position.y);
    EXPECT_DOUBLE_EQ(traj_point.pose.position.z, path_point.pose.position.z);
    EXPECT_DOUBLE_EQ(traj_point.pose.orientation.x, path_point.pose.orientation.x);
    EXPECT_DOUBLE_EQ(traj_point.pose.orientation.y, path_point.pose.orientation.y);
    EXPECT_DOUBLE_EQ(traj_point.pose.orientation.z, path_point.pose.orientation.z);
    EXPECT_DOUBLE_EQ(traj_point.pose.orientation.w, path_point.pose.orientation.w);

    // Longitudinal/lateral velocity and heading rate are mapped directly.
    EXPECT_FLOAT_EQ(traj_point.longitudinal_velocity_mps, path_point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(traj_point.lateral_velocity_mps, path_point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(traj_point.heading_rate_rps, path_point.heading_rate_rps);

    // The conversion does not compute timing, acceleration or steering, so these
    // TrajectoryPoint-only fields stay at their default-constructed value.
    EXPECT_EQ(traj_point.time_from_start.sec, 0);
    EXPECT_EQ(traj_point.time_from_start.nanosec, 0u);
    EXPECT_FLOAT_EQ(traj_point.acceleration_mps2, 0.0f);
    EXPECT_FLOAT_EQ(traj_point.front_wheel_angle_rad, 0.0f);
    EXPECT_FLOAT_EQ(traj_point.rear_wheel_angle_rad, 0.0f);
  }
}

TEST(PathToTrajectoryConvert, EmptyPathProducesEmptyTrajectory)
{
  // Arrange
  const auto path = make_path({});

  // Act
  const auto trajectory = path_to_trajectory::convert(path);

  // Assert
  EXPECT_EQ(trajectory.header.frame_id, "map");
  EXPECT_TRUE(trajectory.points.empty());
}

TEST(PathToTrajectoryConvert, SinglePointIsConverted)
{
  // Arrange
  const auto path = make_path({make_path_point(5.0, 6.0, 4.0, 0.2, -0.3)});

  // Act
  const auto trajectory = path_to_trajectory::convert(path);

  // Assert
  ASSERT_EQ(trajectory.points.size(), 1u);
  EXPECT_DOUBLE_EQ(trajectory.points[0].pose.position.x, 5.0);
  EXPECT_DOUBLE_EQ(trajectory.points[0].pose.position.y, 6.0);
  EXPECT_FLOAT_EQ(trajectory.points[0].longitudinal_velocity_mps, 4.0f);
  EXPECT_FLOAT_EQ(trajectory.points[0].lateral_velocity_mps, 0.2f);
  EXPECT_FLOAT_EQ(trajectory.points[0].heading_rate_rps, -0.3f);
}

}  // namespace autoware::planning_topic_converter
