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

TEST(PrettyBuild, BuildsFromTwoPointsWithDefaultInterpolator)
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

TEST(PrettyBuild, BuildsFromThreePointsWithDefaultInterpolator)
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

TEST(PrettyBuild, BuildsFromFourPointsWithAkima)
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

TEST(PrettyBuild, RejectsSinglePointWithDefaultInterpolator)
{
  const std::vector<PathPointWithLaneId> points{make_path_point_with_lane_id(1.0, 1.0, 0.0)};

  auto trajectory_opt = autoware::experimental::trajectory::pretty_build(points);
  EXPECT_FALSE(trajectory_opt.has_value());
}

TEST(PrettyBuild, BuildsFromSinglePointWithAkimaFallback)
{
  const std::vector<PathPointWithLaneId> points{make_path_point_with_lane_id(1.0, 1.0, 0.0)};

  auto trajectory_opt = autoware::experimental::trajectory::pretty_build(points, true);
  ASSERT_TRUE(trajectory_opt.has_value());
  EXPECT_EQ(trajectory_opt->get_underlying_bases().size(), 1);
}
