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

#include "autoware/trajectory/path_point_with_lane_id.hpp"
#include "autoware/trajectory/trajectory_point.hpp"

#include <gtest/gtest.h>

#include <vector>

TEST(Point, PointFailure)
{
  auto pose = [](double x, double y) -> geometry_msgs::msg::Point {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    return p;
  };

  std::vector<geometry_msgs::msg::Point> points = {pose(0.49, 0.59), pose(0.61, 1.22)};

  using autoware::experimental::trajectory::Trajectory;

  auto trajectory = Trajectory<geometry_msgs::msg::Point>();
  const auto result = trajectory.build(points);
  EXPECT_EQ(result.has_value(), false);
}

TEST(Pose, PoseFailure)
{
  auto pose = [](double x, double y) -> geometry_msgs::msg::Pose {
    geometry_msgs::msg::Pose p;
    p.position.x = x;
    p.position.y = y;
    p.position.z = 0.0;
    return p;
  };

  std::vector<geometry_msgs::msg::Pose> points = {pose(0.49, 0.59), pose(0.61, 1.22)};

  using autoware::experimental::trajectory::Trajectory;

  auto trajectory = Trajectory<geometry_msgs::msg::Pose>();
  const auto result = trajectory.build(points);
  EXPECT_EQ(result.has_value(), false);
}

TEST(PathPoint, PathPointFailure)
{
  auto pose = [](double x, double y) -> autoware_planning_msgs::msg::PathPoint {
    autoware_planning_msgs::msg::PathPoint p;
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.longitudinal_velocity_mps = 1.0;
    return p;
  };

  std::vector<autoware_planning_msgs::msg::PathPoint> points = {pose(0.49, 0.59), pose(0.61, 1.22)};

  using autoware::experimental::trajectory::Trajectory;

  auto trajectory = Trajectory<autoware_planning_msgs::msg::PathPoint>();
  const auto result = trajectory.build(points);
  EXPECT_EQ(result.has_value(), false);
}

TEST(PathPointWithLaneId, PathPointWithLaneIdFailure)
{
  auto pose = [](double x, double y) -> autoware_internal_planning_msgs::msg::PathPointWithLaneId {
    autoware_internal_planning_msgs::msg::PathPointWithLaneId p;
    p.point.pose.position.x = x;
    p.point.pose.position.y = y;
    p.point.longitudinal_velocity_mps = 1.0;
    p.lane_ids = {1, 2};
    return p;
  };

  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> points = {
    pose(0.49, 0.59), pose(0.61, 1.22)};

  using autoware::experimental::trajectory::Trajectory;

  auto trajectory = Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>();
  const auto result = trajectory.build(points);
  EXPECT_EQ(result.has_value(), false);
}

TEST(TrajectoryPoint, TrajectoryPointFailure)
{
  auto pose = [](double x, double y) -> autoware_planning_msgs::msg::TrajectoryPoint {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.position.y = y;
    return point;
  };

  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> points = {
    pose(0.49, 0.59), pose(0.61, 1.22)};

  using autoware::experimental::trajectory::Trajectory;

  auto trajectory = Trajectory<autoware_planning_msgs::msg::TrajectoryPoint>();
  const auto result = trajectory.build(points);
  EXPECT_EQ(result.has_value(), false);
}
