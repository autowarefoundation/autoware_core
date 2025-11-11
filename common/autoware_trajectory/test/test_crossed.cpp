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

#include "autoware/trajectory/utils/crossed.hpp"
#include "autoware/trajectory/utils/pretty_build.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>

#include <gtest/gtest.h>

#include <vector>
using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_utils_geometry::create_quaternion_from_yaw;
using geometry_msgs::build;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Quaternion;

using autoware_utils_geometry::LineString2d;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;

TEST(crossed, linestring)
{
  std::vector<PathPointWithLaneId> points;
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(0.0).y(0.0).z(0.0))
                         .orientation(create_quaternion_from_yaw(0.0));
    point.point.longitudinal_velocity_mps = 10.0;
    point.point.lateral_velocity_mps = 0.5;
    point.point.heading_rate_rps = 0.5;
    point.lane_ids = std::vector<std::int64_t>{1};
    points.push_back(point);
  }
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(4.0).y(4.0).z(0.0))
                         .orientation(create_quaternion_from_yaw(M_PI / 2.0));
    point.point.longitudinal_velocity_mps = 20.0;
    point.point.lateral_velocity_mps = 1.0;
    point.point.heading_rate_rps = 1.0;
    point.lane_ids = std::vector<std::int64_t>{2};
    points.push_back(point);
  }
  const auto points4_result = autoware::experimental::trajectory::detail::populate4(points);
  const auto & points4 = points4_result.value();

  const auto trajectory_opt = autoware::experimental::trajectory::pretty_build(points4);
  EXPECT_EQ(trajectory_opt.has_value(), true);
  const auto & trajectory = trajectory_opt.value();
  EXPECT_EQ(trajectory.get_underlying_bases().size(), 4);

  {
    const LineString2d line{Point2d{4.0, 0.0}, Point2d{0.0, 4.0}};
    const auto crossed_line = autoware::experimental::trajectory::crossed(trajectory, line);
    EXPECT_EQ(crossed_line.size(), 1);
    EXPECT_FLOAT_EQ(crossed_line.front(), 2.0 * std::sqrt(2.0));
  }

  {
    const LineString2d line{Point2d{3.0, 0.0}, Point2d{3.0, 4.0}};
    const auto crossed_line = autoware::experimental::trajectory::crossed(trajectory, line);
    EXPECT_EQ(crossed_line.size(), 1);
    EXPECT_FLOAT_EQ(crossed_line.front(), 3.0 * std::sqrt(2.0));
  }
}

TEST(crossed, open_polygon)
{
  std::vector<PathPointWithLaneId> points;
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(0.0).y(0.0).z(0.0))
                         .orientation(create_quaternion_from_yaw(0.0));
    point.point.longitudinal_velocity_mps = 10.0;
    point.point.lateral_velocity_mps = 0.5;
    point.point.heading_rate_rps = 0.5;
    point.lane_ids = std::vector<std::int64_t>{1};
    points.push_back(point);
  }
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(4.0).y(4.0).z(0.0))
                         .orientation(create_quaternion_from_yaw(M_PI / 2.0));
    point.point.longitudinal_velocity_mps = 20.0;
    point.point.lateral_velocity_mps = 1.0;
    point.point.heading_rate_rps = 1.0;
    point.lane_ids = std::vector<std::int64_t>{2};
    points.push_back(point);
  }
  const auto points4_result = autoware::experimental::trajectory::detail::populate4(points);
  const auto & points4 = points4_result.value();

  const auto trajectory_opt = autoware::experimental::trajectory::pretty_build(points4);
  EXPECT_EQ(trajectory_opt.has_value(), true);
  const auto & trajectory = trajectory_opt.value();
  EXPECT_EQ(trajectory.get_underlying_bases().size(), 4);

  const std::vector<Point2d> open_polygon{
    Point2d{1.0, 1.0}, Point2d{3.0, 1.0}, Point2d{3.0, 3.0}, Point2d{1.0, 3.0}};
  {
    const auto crossed_line =
      autoware::experimental::trajectory::crossed_with_polygon(trajectory, open_polygon);
    EXPECT_EQ(crossed_line.size(), 2);
    EXPECT_FLOAT_EQ(crossed_line.front(), 1.0 * std::sqrt(2.0));
    EXPECT_FLOAT_EQ(crossed_line.back(), 3.0 * std::sqrt(2.0));
  }
  {
    const auto crossed_line = autoware::experimental::trajectory::crossed_with_polygon(
      trajectory, open_polygon, 1.0, trajectory.length() - 1.0);
    EXPECT_EQ(crossed_line.size(), 2);
    EXPECT_FLOAT_EQ(crossed_line.front(), 1.0 * std::sqrt(2.0));
    EXPECT_FLOAT_EQ(crossed_line.back(), 3.0 * std::sqrt(2.0));
  }
  {
    const auto crossed_line = autoware::experimental::trajectory::crossed_with_polygon(
      trajectory, open_polygon, 2.0 * std::sqrt(2.0) - 0.5, 2.0 * std::sqrt(2.0) + 0.5);
    EXPECT_EQ(crossed_line.size(), 0);
  }
  {
    const auto crossed_line = autoware::experimental::trajectory::crossed_with_polygon(
      trajectory, open_polygon, 0.0, 2.0 * std::sqrt(2.0) - 0.5);
    EXPECT_EQ(crossed_line.size(), 1);
    EXPECT_EQ(crossed_line.front(), std::sqrt(2.0));
  }
  {
    const auto crossed_line = autoware::experimental::trajectory::crossed_with_polygon(
      trajectory, open_polygon, 2.0 * std::sqrt(2.0) + 0.5);
    EXPECT_EQ(crossed_line.size(), 1);
    EXPECT_EQ(crossed_line.front(), 3 * std::sqrt(2.0));
  }
}

TEST(crossed, closed_polygon)
{
  std::vector<PathPointWithLaneId> points;
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(0.0).y(0.0).z(0.0))
                         .orientation(create_quaternion_from_yaw(0.0));
    point.point.longitudinal_velocity_mps = 10.0;
    point.point.lateral_velocity_mps = 0.5;
    point.point.heading_rate_rps = 0.5;
    point.lane_ids = std::vector<std::int64_t>{1};
    points.push_back(point);
  }
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(4.0).y(4.0).z(0.0))
                         .orientation(create_quaternion_from_yaw(M_PI / 2.0));
    point.point.longitudinal_velocity_mps = 20.0;
    point.point.lateral_velocity_mps = 1.0;
    point.point.heading_rate_rps = 1.0;
    point.lane_ids = std::vector<std::int64_t>{2};
    points.push_back(point);
  }
  const auto points4_result = autoware::experimental::trajectory::detail::populate4(points);
  const auto & points4 = points4_result.value();

  const auto trajectory_opt = autoware::experimental::trajectory::pretty_build(points4);
  EXPECT_EQ(trajectory_opt.has_value(), true);
  const auto & trajectory = trajectory_opt.value();
  EXPECT_EQ(trajectory.get_underlying_bases().size(), 4);

  const std::vector<Point2d> open_polygon{
    Point2d{1.0, 1.0}, Point2d{3.0, 1.0}, Point2d{3.0, 3.0}, Point2d{1.0, 3.0}, Point2d{1.0, 1.0}};
  {
    const auto crossed_line =
      autoware::experimental::trajectory::crossed_with_polygon(trajectory, open_polygon);
    EXPECT_EQ(crossed_line.size(), 2);
    EXPECT_FLOAT_EQ(crossed_line.front(), 1.0 * std::sqrt(2.0));
    EXPECT_FLOAT_EQ(crossed_line.back(), 3.0 * std::sqrt(2.0));
  }
  {
    const auto crossed_line = autoware::experimental::trajectory::crossed_with_polygon(
      trajectory, open_polygon, 1.0, trajectory.length() - 1.0);
    EXPECT_EQ(crossed_line.size(), 2);
    EXPECT_FLOAT_EQ(crossed_line.front(), 1.0 * std::sqrt(2.0));
    EXPECT_FLOAT_EQ(crossed_line.back(), 3.0 * std::sqrt(2.0));
  }
  {
    const auto crossed_line = autoware::experimental::trajectory::crossed_with_polygon(
      trajectory, open_polygon, 2.0 * std::sqrt(2.0) - 0.5, 2.0 * std::sqrt(2.0) + 0.5);
    EXPECT_EQ(crossed_line.size(), 0);
  }
  {
    const auto crossed_line = autoware::experimental::trajectory::crossed_with_polygon(
      trajectory, open_polygon, 0.0, 2.0 * std::sqrt(2.0) - 0.5);
    EXPECT_EQ(crossed_line.size(), 1);
    EXPECT_EQ(crossed_line.front(), std::sqrt(2.0));
  }
  {
    const auto crossed_line = autoware::experimental::trajectory::crossed_with_polygon(
      trajectory, open_polygon, 2.0 * std::sqrt(2.0) + 0.5);
    EXPECT_EQ(crossed_line.size(), 1);
    EXPECT_EQ(crossed_line.front(), 3 * std::sqrt(2.0));
  }
}

TEST(crossed, post_condition_001)
{
  std::vector<PathPointWithLaneId> points;
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(0.0).y(0.0).z(0.0))
                         .orientation(create_quaternion_from_yaw(0.0));
    point.point.longitudinal_velocity_mps = 10.0;
    point.point.lateral_velocity_mps = 0.5;
    point.point.heading_rate_rps = 0.5;
    point.lane_ids = std::vector<std::int64_t>{1};
    points.push_back(point);
  }
  {
    PathPointWithLaneId point;
    point.point.pose = build<Pose>()
                         .position(build<Point>().x(4.0).y(4.0).z(0.0))
                         .orientation(create_quaternion_from_yaw(M_PI / 2.0));
    point.point.longitudinal_velocity_mps = 20.0;
    point.point.lateral_velocity_mps = 1.0;
    point.point.heading_rate_rps = 1.0;
    point.lane_ids = std::vector<std::int64_t>{2};
    points.push_back(point);
  }
  const auto points4_result = autoware::experimental::trajectory::detail::populate4(points);
  const auto & points4 = points4_result.value();

  const auto trajectory_opt = autoware::experimental::trajectory::pretty_build(points4);
  EXPECT_EQ(trajectory_opt.has_value(), true);
  const auto & trajectory = trajectory_opt.value();
  EXPECT_EQ(trajectory.get_underlying_bases().size(), 4);

  const std::vector<Point2d> open_polygon{
    Point2d{1.0, 1.0}, Point2d{3.0, 1.0}, Point2d{3.0, 3.0}, Point2d{1.0, 3.0}, Point2d{1.0, 1.0}};
  {
    const auto crossed_line =
      autoware::experimental::trajectory::crossed_with_polygon(trajectory, open_polygon);
    EXPECT_EQ(crossed_line.size(), 2);
    EXPECT_FLOAT_EQ(crossed_line.front(), 1.0 * std::sqrt(2.0));
    EXPECT_FLOAT_EQ(crossed_line.back(), 3.0 * std::sqrt(2.0));
    ASSERT_TRUE(crossed_line.front() < crossed_line.back());
  }
  {
    const auto crossed_line = autoware::experimental::trajectory::crossed_with_polygon(
      trajectory, open_polygon, 1.0, trajectory.length() - 1.0);
    EXPECT_EQ(crossed_line.size(), 2);
    EXPECT_FLOAT_EQ(crossed_line.front(), 1.0 * std::sqrt(2.0));
    EXPECT_FLOAT_EQ(crossed_line.back(), 3.0 * std::sqrt(2.0));
    ASSERT_TRUE(crossed_line.front() < crossed_line.back());
  }
}
