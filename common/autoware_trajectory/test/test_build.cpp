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

#include "autoware/trajectory/path_point.hpp"
#include "autoware/trajectory/point.hpp"
#include "autoware/trajectory/pose.hpp"

#include <autoware_planning_msgs/msg/path_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <gtest/gtest.h>

#include <vector>

namespace
{
template <typename PointType>
void expect_build_success_with_single_point(const PointType & point)
{
  using autoware::experimental::trajectory::Trajectory;

  auto trajectory = Trajectory<PointType>();
  const auto result = trajectory.build(std::vector<PointType>{point});
  EXPECT_TRUE(result.has_value());
}

geometry_msgs::msg::Point make_point(const double x, const double y)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  return point;
}

geometry_msgs::msg::Pose make_pose(const double x, const double y)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.0;
  return pose;
}

autoware_planning_msgs::msg::PathPoint make_path_point(const double x, const double y)
{
  autoware_planning_msgs::msg::PathPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.longitudinal_velocity_mps = 1.0;
  return point;
}
}  // namespace

TEST(BuildFallback, point_single_point_succeeds)
{
  expect_build_success_with_single_point(make_point(0.49, 0.59));
}

TEST(BuildFallback, pose_single_point_succeeds)
{
  expect_build_success_with_single_point(make_pose(0.49, 0.59));
}

TEST(BuildFallback, path_point_single_point_succeeds)
{
  expect_build_success_with_single_point(make_path_point(0.49, 0.59));
}
