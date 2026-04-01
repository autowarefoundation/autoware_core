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

#include "autoware/trajectory/utils/pretty_build.hpp"
#include "autoware_utils_geometry/geometry.hpp"

#include <autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_planning_msgs/msg/path_point.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <gtest/gtest.h>

#include <vector>

using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_utils_geometry::create_quaternion_from_yaw;
using autoware_utils_geometry::get_rpy;
using geometry_msgs::build;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;

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

PathPointWithLaneId make_path_point_with_lane_id(const double x, const double y, const double yaw)
{
  PathPointWithLaneId point;
  point.point.pose = build<Pose>()
                       .position(build<Point>().x(x).y(y).z(0.0))
                       .orientation(create_quaternion_from_yaw(yaw));
  point.point.longitudinal_velocity_mps = 10.0;
  point.point.lateral_velocity_mps = 0.5;
  point.point.heading_rate_rps = 0.5;
  point.lane_ids = std::vector<std::int64_t>{1};
  return point;
}

autoware_planning_msgs::msg::TrajectoryPoint make_trajectory_point(const double x, const double y)
{
  autoware_planning_msgs::msg::TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
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

TEST(BuildFallback, path_point_with_lane_id_single_point_succeeds)
{
  auto point = make_path_point_with_lane_id(0.49, 0.59, 0.0);
  point.lane_ids = {1, 2};
  expect_build_success_with_single_point(point);
}

TEST(BuildFallback, trajectory_point_single_point_succeeds)
{
  expect_build_success_with_single_point(make_trajectory_point(0.49, 0.59));
}

TEST(PrettyBuild, builds_from_two_points_with_default_interpolator)
{
  const std::vector<PathPointWithLaneId> points{
    make_path_point_with_lane_id(1.0, 1.0, 0.0),
    make_path_point_with_lane_id(2.0, 2.0, M_PI / 2.0)};

  auto trajectory_opt = autoware::experimental::trajectory::pretty_build(points);
  ASSERT_TRUE(trajectory_opt.has_value());

  auto & trajectory = trajectory_opt.value();
  EXPECT_EQ(trajectory.get_underlying_bases().size(), 2);
  EXPECT_FLOAT_EQ(trajectory.length(), std::sqrt(2.0));

  trajectory.align_orientation_with_trajectory_direction();
  for (const auto s : trajectory.get_underlying_bases()) {
    EXPECT_FLOAT_EQ(trajectory.azimuth(s), get_rpy(trajectory.compute(s).point.pose.orientation).z);
  }
}

TEST(PrettyBuild, builds_from_three_points_with_default_interpolator)
{
  const std::vector<PathPointWithLaneId> points{
    make_path_point_with_lane_id(1.0, 1.0, 0.0), make_path_point_with_lane_id(0.7, 0.3, 0.0),
    make_path_point_with_lane_id(2.0, 2.0, M_PI / 2.0)};

  auto trajectory_opt = autoware::experimental::trajectory::pretty_build(points);
  ASSERT_TRUE(trajectory_opt.has_value());

  auto & trajectory = trajectory_opt.value();
  EXPECT_EQ(trajectory.get_underlying_bases().size(), 3);

  trajectory.align_orientation_with_trajectory_direction();
  for (const auto s : trajectory.get_underlying_bases()) {
    EXPECT_FLOAT_EQ(trajectory.azimuth(s), get_rpy(trajectory.compute(s).point.pose.orientation).z);
  }
}

TEST(PrettyBuild, builds_from_four_points_with_akima)
{
  const std::vector<PathPointWithLaneId> points{
    make_path_point_with_lane_id(1.0, 1.0, 0.0), make_path_point_with_lane_id(1.5, 0.5, 0.0),
    make_path_point_with_lane_id(2.0, 2.0, M_PI / 2.0),
    make_path_point_with_lane_id(2.5, 3.0, M_PI / 2.0)};

  auto trajectory_opt = autoware::experimental::trajectory::pretty_build(points, true);
  ASSERT_TRUE(trajectory_opt.has_value());

  auto & trajectory = trajectory_opt.value();
  EXPECT_EQ(trajectory.get_underlying_bases().size(), 4);

  trajectory.align_orientation_with_trajectory_direction();
  for (const auto s : trajectory.get_underlying_bases()) {
    EXPECT_FLOAT_EQ(trajectory.azimuth(s), get_rpy(trajectory.compute(s).point.pose.orientation).z);
  }
}

TEST(PrettyBuild, builds_from_single_point_with_default_fallback)
{
  const std::vector<PathPointWithLaneId> points{make_path_point_with_lane_id(1.0, 1.0, 0.0)};

  auto trajectory_opt = autoware::experimental::trajectory::pretty_build(points);
  ASSERT_TRUE(trajectory_opt.has_value());
  EXPECT_EQ(trajectory_opt->get_underlying_bases().size(), 1);
}

TEST(PrettyBuild, builds_from_single_point_with_akima_fallback)
{
  const std::vector<PathPointWithLaneId> points{make_path_point_with_lane_id(1.0, 1.0, 0.0)};

  auto trajectory_opt = autoware::experimental::trajectory::pretty_build(points, true);
  ASSERT_TRUE(trajectory_opt.has_value());
  EXPECT_EQ(trajectory_opt->get_underlying_bases().size(), 1);
}
