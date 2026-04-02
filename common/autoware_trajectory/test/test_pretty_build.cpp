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

#include <rclcpp/duration.hpp>

#include <autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp>

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
autoware_planning_msgs::msg::TrajectoryPoint make_temporal_trajectory_point(
  const double x, const double y, const double time_from_start)
{
  autoware_planning_msgs::msg::TrajectoryPoint point;
  point.pose = build<Pose>()
                 .position(build<Point>().x(x).y(y).z(0.0))
                 .orientation(create_quaternion_from_yaw(0.0));
  point.longitudinal_velocity_mps = 10.0;
  point.lateral_velocity_mps = 0.5;
  point.heading_rate_rps = 0.5;
  point.acceleration_mps2 = 0.1;
  point.front_wheel_angle_rad = 0.0;
  point.rear_wheel_angle_rad = 0.0;
  point.time_from_start = rclcpp::Duration::from_seconds(time_from_start);
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

}  // namespace

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

TEST(PrettyBuild, rejects_single_point_with_default_interpolator)
{
  const std::vector<PathPointWithLaneId> points{make_path_point_with_lane_id(1.0, 1.0, 0.0)};

  auto trajectory_opt = autoware::experimental::trajectory::pretty_build(points);
  EXPECT_FALSE(trajectory_opt.has_value());
}

TEST(PrettyBuild, builds_from_single_point_with_akima_fallback)
{
  const std::vector<PathPointWithLaneId> points{make_path_point_with_lane_id(1.0, 1.0, 0.0)};

  auto trajectory_opt = autoware::experimental::trajectory::pretty_build(points, true);
  ASSERT_TRUE(trajectory_opt.has_value());
  EXPECT_EQ(trajectory_opt->get_underlying_bases().size(), 1);
}

TEST(pretty_build_temporal, from_2_cubic)
{
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> points{
    make_temporal_trajectory_point(0.0, 0.0, 0.0), make_temporal_trajectory_point(2.0, 0.0, 2.0)};

  auto trajectory_opt = autoware::experimental::trajectory::pretty_build_temporal(points);
  ASSERT_TRUE(trajectory_opt.has_value());

  const auto & trajectory = trajectory_opt.value();
  EXPECT_EQ(trajectory.get_underlying_time_bases().size(), 2);
  EXPECT_NEAR(trajectory.length(), 2.0, 1e-6);
  EXPECT_NEAR(trajectory.duration(), 2.0, 1e-6);

  const auto middle_point = trajectory.compute_from_time(1.0);
  EXPECT_NEAR(middle_point.pose.position.x, 1.0, 1e-6);
  EXPECT_NEAR(rclcpp::Duration(middle_point.time_from_start).seconds(), 1.0, 1e-6);
}

TEST(pretty_build_temporal, from_4_akima)
{
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> points{
    make_temporal_trajectory_point(0.0, 0.0, 0.0), make_temporal_trajectory_point(1.0, 0.5, 1.0),
    make_temporal_trajectory_point(2.0, 2.0, 2.5), make_temporal_trajectory_point(3.0, 3.0, 4.0)};

  auto trajectory_opt = autoware::experimental::trajectory::pretty_build_temporal(points, true);
  ASSERT_TRUE(trajectory_opt.has_value());

  const auto & trajectory = trajectory_opt.value();
  EXPECT_EQ(trajectory.get_underlying_time_bases().size(), 4);
  EXPECT_NEAR(trajectory.start_time(), 0.0, 1e-6);
  EXPECT_NEAR(trajectory.end_time(), 4.0, 1e-6);
}

TEST(pretty_build_temporal, fail_because_points_are_too_few)
{
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> points{
    make_temporal_trajectory_point(0.0, 0.0, 0.0)};

  const auto trajectory_opt = autoware::experimental::trajectory::pretty_build_temporal(points);
  EXPECT_FALSE(trajectory_opt.has_value());
}
