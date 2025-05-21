// Copyright 2024 TIER IV, Inc.
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
#include "autoware/trajectory/utils/closest.hpp"
#include "autoware/trajectory/utils/crossed.hpp"
#include "autoware/trajectory/utils/curvature_utils.hpp"
#include "autoware/trajectory/utils/pretty_build.hpp"
#include "autoware_utils_geometry/geometry.hpp"
#include "lanelet2_core/primitives/LineString.h"

#include <gtest/gtest.h>

#include <vector>

using Trajectory =
  autoware::experimental::trajectory::Trajectory<autoware_planning_msgs::msg::TrajectoryPoint>;

autoware_planning_msgs::msg::TrajectoryPoint trajectory_point(double x, double y)
{
  autoware_planning_msgs::msg::TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  return point;
}

TEST(TrajectoryCreatorTestForTrajectoryPoint, create)
{
  {
    std::vector<autoware_planning_msgs::msg::TrajectoryPoint> points{trajectory_point(0.00, 0.00)};
    auto trajectory = Trajectory::Builder{}.build(points);
    ASSERT_TRUE(!trajectory);
  }
  {
    std::vector<autoware_planning_msgs::msg::TrajectoryPoint> points{
      trajectory_point(0.00, 0.00), trajectory_point(0.81, 1.68), trajectory_point(1.65, 2.98),
      trajectory_point(3.30, 4.01)};
    auto trajectory = Trajectory::Builder{}.build(points);
    ASSERT_TRUE(trajectory);
  }
}

class TrajectoryTestForTrajectoryPoint : public ::testing::Test
{
public:
  std::optional<Trajectory> trajectory;

  void SetUp() override
  {
    std::vector<autoware_planning_msgs::msg::TrajectoryPoint> points{
      trajectory_point(0.00, 0.00), trajectory_point(0.81, 1.68), trajectory_point(1.65, 2.98),
      trajectory_point(3.30, 4.01), trajectory_point(4.70, 4.52), trajectory_point(6.49, 5.20),
      trajectory_point(8.11, 6.07), trajectory_point(8.76, 7.23), trajectory_point(9.36, 8.74),
      trajectory_point(10.0, 10.0)};

    const auto result = Trajectory::Builder{}.build(points);
    ASSERT_TRUE(result);
    trajectory.emplace(result.value());
  }
};

TEST_F(TrajectoryTestForTrajectoryPoint, compute)
{
  double length = trajectory->length();

  trajectory->longitudinal_velocity_mps()
    .range(trajectory->length() / 3.0, trajectory->length())
    .set(10.0);
  auto point = trajectory->compute(length / 2.0);

  EXPECT_LT(0, point.pose.position.x);
  EXPECT_LT(point.pose.position.x, 10);

  EXPECT_LT(0, point.pose.position.y);
  EXPECT_LT(point.pose.position.y, 10);
}

TEST_F(TrajectoryTestForTrajectoryPoint, manipulate_velocity)
{
  trajectory->longitudinal_velocity_mps() = 10.0;
  trajectory->longitudinal_velocity_mps()
    .range(trajectory->length() / 3, 2.0 * trajectory->length() / 3)
    .set(5.0);
  auto point1 = trajectory->compute(0.0);
  auto point2 = trajectory->compute(trajectory->length() / 2.0);
  auto point3 = trajectory->compute(trajectory->length());

  EXPECT_EQ(10.0, point1.longitudinal_velocity_mps);
  EXPECT_EQ(5.0, point2.longitudinal_velocity_mps);
  EXPECT_EQ(10.0, point3.longitudinal_velocity_mps);
}

TEST_F(TrajectoryTestForTrajectoryPoint, direction)
{
  double dir = trajectory->azimuth(0.0);
  EXPECT_LT(0, dir);
  EXPECT_LT(dir, M_PI / 2);
}

TEST_F(TrajectoryTestForTrajectoryPoint, curvature)
{
  double curvature_val = trajectory->curvature(0.0);
  EXPECT_LT(-1.0, curvature_val);
  EXPECT_LT(curvature_val, 1.0);
}

TEST_F(TrajectoryTestForTrajectoryPoint, restore)
{
  using autoware::experimental::trajectory::Trajectory;
  trajectory->longitudinal_velocity_mps().range(4.0, trajectory->length()).set(5.0);
  auto points = trajectory->restore(0);
  EXPECT_EQ(11, points.size());
}

TEST_F(TrajectoryTestForTrajectoryPoint, crossed)
{
  lanelet::LineString2d line_string;
  line_string.push_back(lanelet::Point3d(lanelet::InvalId, 0.0, 10.0, 0.0));
  line_string.push_back(lanelet::Point3d(lanelet::InvalId, 10.0, 0.0, 0.0));

  auto crossed_point = autoware::experimental::trajectory::crossed(*trajectory, line_string);
  ASSERT_EQ(crossed_point.size(), 1);

  EXPECT_LT(0.0, crossed_point.at(0));
  EXPECT_LT(crossed_point.at(0), trajectory->length());
}

TEST_F(TrajectoryTestForTrajectoryPoint, closest)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 5.0;
  pose.position.y = 5.0;

  auto closest_pose =
    trajectory->compute(autoware::experimental::trajectory::closest(*trajectory, pose));

  double distance = std::hypot(
    closest_pose.pose.position.x - pose.position.x, closest_pose.pose.position.y - pose.position.y);

  EXPECT_LT(distance, 3.0);
}

TEST_F(TrajectoryTestForTrajectoryPoint, crop)
{
  double length = trajectory->length();

  auto start_point_expect = trajectory->compute(length / 3.0);
  auto end_point_expect = trajectory->compute(length / 3.0 + 1.0);

  trajectory->crop(length / 3.0, 1.0);

  EXPECT_EQ(trajectory->length(), 1.0);

  auto start_point_actual = trajectory->compute(0.0);
  auto end_point_actual = trajectory->compute(trajectory->length());

  EXPECT_EQ(start_point_expect.pose.position.x, start_point_actual.pose.position.x);
  EXPECT_EQ(start_point_expect.pose.position.y, start_point_actual.pose.position.y);

  EXPECT_EQ(end_point_expect.pose.position.x, end_point_actual.pose.position.x);
  EXPECT_EQ(end_point_expect.pose.position.y, end_point_actual.pose.position.y);
}

TEST_F(TrajectoryTestForTrajectoryPoint, max_curvature)
{
  double max_curvature = autoware::experimental::trajectory::max_curvature(*trajectory);
  EXPECT_LT(0, max_curvature);
}

TEST_F(TrajectoryTestForTrajectoryPoint, pretty_build_from_3_cubic)
{
  using autoware_utils_geometry::get_rpy;

  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> points{
    trajectory_point(1.0, 1.0), trajectory_point(2.0, 2.0)};
  auto trajectory_opt = autoware::experimental::trajectory::pretty_build(points);
  EXPECT_EQ(trajectory_opt.has_value(), true);

  auto & trajectory = trajectory_opt.value();
  EXPECT_EQ(trajectory.get_underlying_bases().size(), 4);

  trajectory.align_orientation_with_trajectory_direction();
  for (const auto s : trajectory.get_underlying_bases()) {
    EXPECT_FLOAT_EQ(trajectory.azimuth(s), get_rpy(trajectory.compute(s).pose.orientation).z);
  }
}
