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
#include "autoware/trajectory/path_point_with_lane_id.hpp"
#include "autoware/trajectory/threshold.hpp"
#include "autoware/trajectory/utils/closest.hpp"
#include "autoware/trajectory/utils/crossed.hpp"
#include "autoware/trajectory/utils/curvature_utils.hpp"
#include "autoware/trajectory/utils/find_if.hpp"
#include "autoware/trajectory/utils/find_intervals.hpp"
#include "autoware_utils_geometry/geometry.hpp"
#include "lanelet2_core/primitives/LineString.h"

#include <geometry_msgs/msg/point.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <utility>
#include <vector>
namespace
{
using Trajectory = autoware::experimental::trajectory::Trajectory<
  autoware_internal_planning_msgs::msg::PathPointWithLaneId>;

autoware_internal_planning_msgs::msg::PathPointWithLaneId path_point_with_lane_id(
  double x, double y, uint8_t lane_id)
{
  autoware_internal_planning_msgs::msg::PathPointWithLaneId point;
  point.point.pose.position.x = x;
  point.point.pose.position.y = y;
  point.lane_ids.emplace_back(lane_id);
  return point;
}

geometry_msgs::msg::Point point(double x, double y)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  return p;
}

void expect_strictly_increasing(const std::vector<double> & bases)
{
  ASSERT_FALSE(bases.empty());
  for (size_t i = 1; i < bases.size(); ++i) {
    EXPECT_LT(bases[i - 1], bases[i]);
  }
}

void check_if_equals(const Trajectory & trajectory1, const Trajectory & trajectory2)
{
  for (double s = 0.0; s <= trajectory1.length(); s += 0.5) {
    auto point1 = trajectory1.compute(s);
    auto point2 = trajectory2.compute(s);

    EXPECT_EQ(point1.point.pose.position.x, point2.point.pose.position.x);
  }

  EXPECT_EQ(
    trajectory1.longitudinal_velocity_mps().start(),
    trajectory2.longitudinal_velocity_mps().start());
  EXPECT_EQ(
    trajectory1.longitudinal_velocity_mps().end(), trajectory2.longitudinal_velocity_mps().end());
  EXPECT_EQ(trajectory1.lateral_velocity_mps().start(), trajectory2.lateral_velocity_mps().start());
  EXPECT_EQ(trajectory1.lateral_velocity_mps().end(), trajectory2.lateral_velocity_mps().end());
  EXPECT_EQ(trajectory1.heading_rate_rps().start(), trajectory2.heading_rate_rps().start());
  EXPECT_EQ(trajectory1.heading_rate_rps().end(), trajectory2.heading_rate_rps().end());
  EXPECT_EQ(trajectory1.lane_ids().start(), trajectory2.lane_ids().start());
  EXPECT_EQ(trajectory1.lane_ids().end(), trajectory2.lane_ids().end());
}
}  // namespace
TEST(TrajectoryCreatorTest, Constructor)
{
  std::vector<geometry_msgs::msg::Point> points{
    point(0.00, 0.00), point(0.81, 1.68), point(1.65, 2.98), point(3.30, 4.01)};
  auto trajectory =
    autoware::experimental::trajectory::Trajectory<geometry_msgs::msg::Point>::Builder{}.build(
      points);
  ASSERT_TRUE(trajectory);
  autoware::experimental::trajectory::Trajectory<geometry_msgs::msg::Pose> trj_pose(*trajectory);
}

TEST(TrajectoryCreatorTest, CreateFromSinglePoint)
{
  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> points{
    path_point_with_lane_id(0.00, 0.00, 0)};
  auto trajectory = Trajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory);
}

TEST(TrajectoryCreatorTest, RestoreSinglePointTrajectory)
{
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> points{
    path_point_with_lane_id(0.00, 0.00, 7)};
  const auto trajectory = Trajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory);

  const auto restored = trajectory->restore();
  ASSERT_EQ(restored.size(), 1UL);
  EXPECT_DOUBLE_EQ(restored.front().point.pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(restored.front().point.pose.position.y, 0.0);
  ASSERT_EQ(restored.front().lane_ids.size(), 1UL);
  EXPECT_EQ(restored.front().lane_ids.front(), 7);
}

TEST(TrajectoryCreatorTest, RestoreCompleteDuplicatePointsTrajectoryPreservesDuplicates)
{
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> points{
    path_point_with_lane_id(1.0, 2.0, 7), path_point_with_lane_id(1.0, 2.0, 7),
    path_point_with_lane_id(1.0, 2.0, 7)};
  const auto trajectory = Trajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory);

  const auto restored = trajectory->restore();
  ASSERT_EQ(restored.size(), 3UL);
  for (const auto & point : restored) {
    EXPECT_DOUBLE_EQ(point.point.pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(point.point.pose.position.y, 2.0);
    ASSERT_EQ(point.lane_ids.size(), 1UL);
    EXPECT_EQ(point.lane_ids.front(), 7);
  }
}

TEST(TrajectoryCreatorTest, RestoreCropToZeroLengthTrajectoryPreservesBoundaries)
{
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> points{
    path_point_with_lane_id(0.0, 0.0, 0), path_point_with_lane_id(1.0, 1.0, 1),
    path_point_with_lane_id(2.0, 2.0, 2), path_point_with_lane_id(3.0, 3.0, 3)};
  auto trajectory = Trajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory);

  const auto cropped_point = trajectory->compute(trajectory->length() / 2.0);
  trajectory->crop(trajectory->length() / 2.0, 0.0);

  const auto restored = trajectory->restore();
  ASSERT_EQ(restored.size(), 2UL);
  for (const auto & point : restored) {
    EXPECT_DOUBLE_EQ(point.point.pose.position.x, cropped_point.point.pose.position.x);
    EXPECT_DOUBLE_EQ(point.point.pose.position.y, cropped_point.point.pose.position.y);
    ASSERT_EQ(point.lane_ids.size(), cropped_point.lane_ids.size());
    EXPECT_EQ(point.lane_ids.front(), cropped_point.lane_ids.front());
  }
}

TEST(TrajectoryCreatorTest, RestoreTinyCroppedTrajectoryPreservesBoundaries)
{
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> points{
    path_point_with_lane_id(0.0, 0.0, 0), path_point_with_lane_id(1.0, 1.0, 1),
    path_point_with_lane_id(2.0, 2.0, 2), path_point_with_lane_id(3.0, 3.0, 3)};
  auto trajectory = Trajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory);

  constexpr double tiny_length =
    autoware::experimental::trajectory::k_points_minimum_dist_threshold / 10.0;
  const double crop_start = trajectory->length() / 2.0;
  const auto cropped_start_point = trajectory->compute(crop_start);
  const auto cropped_end_point = trajectory->compute(crop_start + tiny_length);
  trajectory->crop(crop_start, tiny_length);

  const auto restored = trajectory->restore();
  ASSERT_EQ(restored.size(), 2UL);
  EXPECT_DOUBLE_EQ(
    restored.front().point.pose.position.x, cropped_start_point.point.pose.position.x);
  EXPECT_DOUBLE_EQ(
    restored.front().point.pose.position.y, cropped_start_point.point.pose.position.y);
  ASSERT_EQ(restored.front().lane_ids.size(), cropped_start_point.lane_ids.size());
  EXPECT_EQ(restored.front().lane_ids.front(), cropped_start_point.lane_ids.front());
  EXPECT_DOUBLE_EQ(restored.back().point.pose.position.x, cropped_end_point.point.pose.position.x);
  EXPECT_DOUBLE_EQ(restored.back().point.pose.position.y, cropped_end_point.point.pose.position.y);
  ASSERT_EQ(restored.back().lane_ids.size(), cropped_end_point.lane_ids.size());
  EXPECT_EQ(restored.back().lane_ids.front(), cropped_end_point.lane_ids.front());
  EXPECT_LT(
    std::hypot(
      restored.back().point.pose.position.x - restored.front().point.pose.position.x,
      restored.back().point.pose.position.y - restored.front().point.pose.position.y),
    autoware::experimental::trajectory::k_points_minimum_dist_threshold);
}

TEST(TrajectoryCreatorTest, CreateFromMultiplePoints)
{
  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> points{
    path_point_with_lane_id(0.00, 0.00, 0), path_point_with_lane_id(0.81, 1.68, 0),
    path_point_with_lane_id(1.65, 2.98, 0), path_point_with_lane_id(3.30, 4.01, 1)};
  auto trajectory = Trajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory);
}

TEST(TrajectoryCreatorTest, AlmostSamePointsAreGiven)
{
  const double nano_meter = 1e-9;
  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> points{
    path_point_with_lane_id(0.00, 0.00, 0), path_point_with_lane_id(0.81, 1.68, 0),
    path_point_with_lane_id(1.65, 2.98, 0),
    path_point_with_lane_id(1.65 + nano_meter, 2.98 + nano_meter, 1),
    path_point_with_lane_id(1.65 + 2 * nano_meter, 2.98 + 2 * nano_meter, 1)};
  auto trajectory = Trajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory);
  {
    const auto restored = trajectory->restore();
    EXPECT_EQ(restored.size(), 5);
  }
}

TEST(TrajectoryCreatorTest, CreateFromCompleteDuplicatePointsWithIncreasingBases)
{
  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> points{
    path_point_with_lane_id(1.0, 2.0, 0), path_point_with_lane_id(1.0, 2.0, 0),
    path_point_with_lane_id(1.0, 2.0, 1)};

  const auto trajectory = Trajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory);

  const auto bases = trajectory->get_underlying_bases();
  EXPECT_EQ(bases.size(), points.size());
  expect_strictly_increasing(bases);
}

TEST(TrajectoryCreatorTest, CreateFromPartiallyDuplicatePointsWithIncreasingBases)
{
  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> points{
    path_point_with_lane_id(0.0, 0.0, 0), path_point_with_lane_id(1.0, 1.0, 0),
    path_point_with_lane_id(1.0, 1.0, 1), path_point_with_lane_id(2.0, 2.0, 1)};

  const auto trajectory = Trajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory);

  const auto bases = trajectory->get_underlying_bases();
  EXPECT_EQ(bases.size(), points.size());
  expect_strictly_increasing(bases);
}

class TrajectoryTest : public ::testing::Test
{
public:
  std::optional<Trajectory> trajectory;

  void SetUp() override
  {
    std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> points{
      path_point_with_lane_id(0.00, 0.00, 0), path_point_with_lane_id(0.81, 1.68, 0),
      path_point_with_lane_id(1.65, 2.98, 0), path_point_with_lane_id(3.30, 4.01, 1),
      path_point_with_lane_id(4.70, 4.52, 1), path_point_with_lane_id(6.49, 5.20, 1),
      path_point_with_lane_id(8.11, 6.07, 1), path_point_with_lane_id(8.76, 7.23, 1),
      path_point_with_lane_id(9.36, 8.74, 1), path_point_with_lane_id(10.0, 10.0, 1)};

    const auto result = Trajectory::Builder{}.build(points);
    ASSERT_TRUE(result);
    trajectory.emplace(result.value());
  }
};

TEST_F(TrajectoryTest, Compute)
{
  ASSERT_TRUE(trajectory);

  double length = trajectory->length();

  trajectory->longitudinal_velocity_mps()
    .range(trajectory->length() / 3.0, trajectory->length())
    .set(10.0);
  auto point = trajectory->compute(length / 2.0);

  EXPECT_LT(0, point.point.pose.position.x);
  EXPECT_LT(point.point.pose.position.x, 10);

  EXPECT_LT(0, point.point.pose.position.y);
  EXPECT_LT(point.point.pose.position.y, 10);

  EXPECT_EQ(1, point.lane_ids[0]);
}

TEST_F(TrajectoryTest, ManipulateLongitudinalVelocity)
{
  ASSERT_TRUE(trajectory);
  trajectory->longitudinal_velocity_mps() = 10.0;
  trajectory->longitudinal_velocity_mps()
    .range(trajectory->length() / 3, 2.0 * trajectory->length() / 3)
    .set(5.0);
  auto point1 = trajectory->compute(0.0);
  auto point2 = trajectory->compute(trajectory->length() / 2.0);
  auto point3 = trajectory->compute(trajectory->length());

  EXPECT_FLOAT_EQ(10.0, point1.point.longitudinal_velocity_mps);
  EXPECT_FLOAT_EQ(5.0, point2.point.longitudinal_velocity_mps);
  EXPECT_FLOAT_EQ(10.0, point3.point.longitudinal_velocity_mps);
}

TEST_F(TrajectoryTest, ManipulateLateralVelocity)
{
  ASSERT_TRUE(trajectory);
  trajectory->lateral_velocity_mps()
    .range(trajectory->length() / 3, 2.0 * trajectory->length() / 3)
    .set(5.0);
  trajectory->lateral_velocity_mps()
    .range(trajectory->length() * 0.75, trajectory->length() * 0.9)
    .set(10.0);
  auto point1 = trajectory->compute(0.0);
  auto point2 = trajectory->compute(trajectory->length() / 2.0);
  auto point3 = trajectory->compute(trajectory->length() * 0.8);
  auto point4 = trajectory->compute(trajectory->length());

  EXPECT_FLOAT_EQ(0.0, point1.point.lateral_velocity_mps);
  EXPECT_FLOAT_EQ(5.0, point2.point.lateral_velocity_mps);
  EXPECT_FLOAT_EQ(10.0, point3.point.lateral_velocity_mps);
  EXPECT_FLOAT_EQ(0.0, point4.point.lateral_velocity_mps);
}

TEST_F(TrajectoryTest, ClampLongitudinalVelocity)
{
  ASSERT_TRUE(trajectory);
  trajectory->longitudinal_velocity_mps() = 10.0;
  trajectory->longitudinal_velocity_mps()
    .range(trajectory->length() / 3, 2 * trajectory->length() / 3)
    .set(5.0);
  trajectory->longitudinal_velocity_mps()
    .range(trajectory->length() / 4, 3 * trajectory->length() / 4)
    .clamp(8.0);
  auto point1 = trajectory->compute(0.0);
  auto point2 = trajectory->compute(trajectory->length() / 4.0);
  auto point3 = trajectory->compute(trajectory->length() / 2.0);
  auto point4 = trajectory->compute(3 * trajectory->length() / 4.0);
  auto point5 = trajectory->compute(trajectory->length());

  EXPECT_FLOAT_EQ(10.0, point1.point.longitudinal_velocity_mps);
  EXPECT_FLOAT_EQ(8.0, point2.point.longitudinal_velocity_mps);
  EXPECT_FLOAT_EQ(5.0, point3.point.longitudinal_velocity_mps);
  EXPECT_FLOAT_EQ(8.0, point4.point.longitudinal_velocity_mps);
  EXPECT_FLOAT_EQ(10.0, point5.point.longitudinal_velocity_mps);
}

TEST_F(TrajectoryTest, ManipulateVelocities)
{
  ASSERT_TRUE(trajectory);
  // longitudinal velocity = 1.0 [0.3, 0.7]
  trajectory->longitudinal_velocity_mps()
    .range(trajectory->length() * 0.3, trajectory->length() * 0.7)
    .set(1.0);

  // lateral velocity = 0.5 [0.1, 0.8]
  trajectory->lateral_velocity_mps()
    .range(trajectory->length() * 0.1, trajectory->length() * 0.8)
    .set(0.5);

  // heading rate = 1.0 [0.2, 0.9]
  trajectory->heading_rate_rps()
    .range(trajectory->length() * 0.2, trajectory->length() * 0.9)
    .set(1.0);

  auto point1 = trajectory->compute(0.05);
  auto point2 = trajectory->compute(trajectory->length() * 0.15);
  auto point3 = trajectory->compute(trajectory->length() * 0.25);
  auto point4 = trajectory->compute(trajectory->length() * 0.35);
  auto point5 = trajectory->compute(trajectory->length() * 0.75);
  auto point6 = trajectory->compute(trajectory->length() * 0.85);
  auto point7 = trajectory->compute(trajectory->length() * 0.95);

  {
    EXPECT_FLOAT_EQ(0.0, point1.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.0, point1.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(0.0, point1.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(0.0, point2.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.5, point2.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(0.0, point2.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(0.0, point3.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.5, point3.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(1.0, point3.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(1.0, point4.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.5, point4.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(1.0, point4.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(0.0, point5.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.5, point5.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(1.0, point5.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(0.0, point6.point.longitudinal_velocity_mps);
    // EXPECT_FLOAT_EQ(0.0, point6.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(1.0, point6.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(0.0, point7.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.0, point7.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(0.0, point7.point.heading_rate_rps);
  }
}

TEST_F(TrajectoryTest, ManipulateVelocitiesWithCopyCtor)
{
  ASSERT_TRUE(trajectory);
  Trajectory trajectory2(*trajectory);
  // longitudinal velocity = 1.0 [0.3, 0.7]
  trajectory2.longitudinal_velocity_mps()
    .range(trajectory2.length() * 0.3, trajectory2.length() * 0.7)
    .set(1.0);

  // lateral velocity = 0.5 [0.1, 0.8]
  trajectory2.lateral_velocity_mps()
    .range(trajectory2.length() * 0.1, trajectory2.length() * 0.8)
    .set(0.5);

  // heading rate = 1.0 [0.2, 0.9]
  trajectory2.heading_rate_rps()
    .range(trajectory2.length() * 0.2, trajectory2.length() * 0.9)
    .set(1.0);

  auto point1 = trajectory2.compute(0.05);
  auto point2 = trajectory2.compute(trajectory2.length() * 0.15);
  auto point3 = trajectory2.compute(trajectory2.length() * 0.25);
  auto point4 = trajectory2.compute(trajectory2.length() * 0.35);
  auto point5 = trajectory2.compute(trajectory2.length() * 0.75);
  auto point6 = trajectory2.compute(trajectory2.length() * 0.85);
  auto point7 = trajectory2.compute(trajectory2.length() * 0.95);

  {
    EXPECT_FLOAT_EQ(0.0, point1.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.0, point1.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(0.0, point1.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(0.0, point2.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.5, point2.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(0.0, point2.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(0.0, point3.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.5, point3.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(1.0, point3.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(1.0, point4.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.5, point4.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(1.0, point4.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(0.0, point5.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.5, point5.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(1.0, point5.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(0.0, point6.point.longitudinal_velocity_mps);
    // EXPECT_FLOAT_EQ(0.0, point6.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(1.0, point6.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(0.0, point7.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.0, point7.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(0.0, point7.point.heading_rate_rps);
  }
}

TEST_F(TrajectoryTest, ManipulateVelocitiesWithCopyAssignment)
{
  auto trajectory2 = *trajectory;
  // longitudinal velocity = 1.0 [0.3, 0.7]
  trajectory2.longitudinal_velocity_mps()
    .range(trajectory2.length() * 0.3, trajectory2.length() * 0.7)
    .set(1.0);

  // lateral velocity = 0.5 [0.1, 0.8]
  trajectory2.lateral_velocity_mps()
    .range(trajectory2.length() * 0.1, trajectory2.length() * 0.8)
    .set(0.5);

  // heading rate = 1.0 [0.2, 0.9]
  trajectory2.heading_rate_rps()
    .range(trajectory2.length() * 0.2, trajectory2.length() * 0.9)
    .set(1.0);

  auto point1 = trajectory2.compute(0.05);
  auto point2 = trajectory2.compute(trajectory2.length() * 0.15);
  auto point3 = trajectory2.compute(trajectory2.length() * 0.25);
  auto point4 = trajectory2.compute(trajectory2.length() * 0.35);
  auto point5 = trajectory2.compute(trajectory2.length() * 0.75);
  auto point6 = trajectory2.compute(trajectory2.length() * 0.85);
  auto point7 = trajectory2.compute(trajectory2.length() * 0.95);

  {
    EXPECT_FLOAT_EQ(0.0, point1.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.0, point1.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(0.0, point1.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(0.0, point2.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.5, point2.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(0.0, point2.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(0.0, point3.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.5, point3.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(1.0, point3.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(1.0, point4.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.5, point4.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(1.0, point4.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(0.0, point5.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.5, point5.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(1.0, point5.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(0.0, point6.point.longitudinal_velocity_mps);
    // EXPECT_FLOAT_EQ(0.0, point6.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(1.0, point6.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(0.0, point7.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.0, point7.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(0.0, point7.point.heading_rate_rps);
  }
}

TEST_F(TrajectoryTest, ManipulateVelocitiesWithMoveCtor)
{
  auto trajectory2(std::move(*trajectory));
  // longitudinal velocity = 1.0 [0.3, 0.7]
  trajectory2.longitudinal_velocity_mps()
    .range(trajectory2.length() * 0.3, trajectory2.length() * 0.7)
    .set(1.0);

  // lateral velocity = 0.5 [0.1, 0.8]
  trajectory2.lateral_velocity_mps()
    .range(trajectory2.length() * 0.1, trajectory2.length() * 0.8)
    .set(0.5);

  // heading rate = 1.0 [0.2, 0.9]
  trajectory2.heading_rate_rps()
    .range(trajectory2.length() * 0.2, trajectory2.length() * 0.9)
    .set(1.0);

  auto point1 = trajectory2.compute(0.05);
  auto point2 = trajectory2.compute(trajectory2.length() * 0.15);
  auto point3 = trajectory2.compute(trajectory2.length() * 0.25);
  auto point4 = trajectory2.compute(trajectory2.length() * 0.35);
  auto point5 = trajectory2.compute(trajectory2.length() * 0.75);
  auto point6 = trajectory2.compute(trajectory2.length() * 0.85);
  auto point7 = trajectory2.compute(trajectory2.length() * 0.95);

  {
    EXPECT_FLOAT_EQ(0.0, point1.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.0, point1.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(0.0, point1.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(0.0, point2.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.5, point2.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(0.0, point2.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(0.0, point3.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.5, point3.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(1.0, point3.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(1.0, point4.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.5, point4.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(1.0, point4.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(0.0, point5.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.5, point5.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(1.0, point5.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(0.0, point6.point.longitudinal_velocity_mps);
    // EXPECT_FLOAT_EQ(0.0, point6.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(1.0, point6.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(0.0, point7.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.0, point7.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(0.0, point7.point.heading_rate_rps);
  }
}

TEST_F(TrajectoryTest, ManipulateVelocitiesWithMoveAssignment)
{
  auto trajectory2 = std::move(*trajectory);
  // longitudinal velocity = 1.0 [0.3, 0.7]
  trajectory2.longitudinal_velocity_mps()
    .range(trajectory2.length() * 0.3, trajectory2.length() * 0.7)
    .set(1.0);

  // lateral velocity = 0.5 [0.1, 0.8]
  trajectory2.lateral_velocity_mps()
    .range(trajectory2.length() * 0.1, trajectory2.length() * 0.8)
    .set(0.5);

  // heading rate = 1.0 [0.2, 0.9]
  trajectory2.heading_rate_rps()
    .range(trajectory2.length() * 0.2, trajectory2.length() * 0.9)
    .set(1.0);

  auto point1 = trajectory2.compute(0.05);
  auto point2 = trajectory2.compute(trajectory2.length() * 0.15);
  auto point3 = trajectory2.compute(trajectory2.length() * 0.25);
  auto point4 = trajectory2.compute(trajectory2.length() * 0.35);
  auto point5 = trajectory2.compute(trajectory2.length() * 0.75);
  auto point6 = trajectory2.compute(trajectory2.length() * 0.85);
  auto point7 = trajectory2.compute(trajectory2.length() * 0.95);

  {
    EXPECT_FLOAT_EQ(0.0, point1.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.0, point1.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(0.0, point1.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(0.0, point2.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.5, point2.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(0.0, point2.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(0.0, point3.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.5, point3.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(1.0, point3.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(1.0, point4.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.5, point4.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(1.0, point4.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(0.0, point5.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.5, point5.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(1.0, point5.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(0.0, point6.point.longitudinal_velocity_mps);
    // EXPECT_FLOAT_EQ(0.0, point6.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(1.0, point6.point.heading_rate_rps);
  }
  {
    EXPECT_FLOAT_EQ(0.0, point7.point.longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(0.0, point7.point.lateral_velocity_mps);
    EXPECT_FLOAT_EQ(0.0, point7.point.heading_rate_rps);
  }
}

TEST_F(TrajectoryTest, Direction)
{
  ASSERT_TRUE(trajectory);
  double dir = trajectory->azimuth(0.0);
  EXPECT_LT(0, dir);
  EXPECT_LT(dir, M_PI / 2);
}

TEST_F(TrajectoryTest, Curvature)
{
  ASSERT_TRUE(trajectory);
  double curvature_val = trajectory->curvature(0.0);
  EXPECT_LT(-1.0, curvature_val);
  EXPECT_LT(curvature_val, 1.0);
}

TEST_F(TrajectoryTest, Restore)
{
  ASSERT_TRUE(trajectory);
  using autoware::experimental::trajectory::Trajectory;
  trajectory->longitudinal_velocity_mps().range(4.0, trajectory->length()).set(5.0);
  auto points = trajectory->restore();
  EXPECT_EQ(11, points.size());
}

TEST_F(TrajectoryTest, Crossed)
{
  ASSERT_TRUE(trajectory);
  lanelet::LineString2d line_string;
  line_string.push_back(lanelet::Point3d(lanelet::InvalId, 0.0, 10.0, 0.0));
  line_string.push_back(lanelet::Point3d(lanelet::InvalId, 10.0, 0.0, 0.0));

  auto crossed_point = autoware::experimental::trajectory::crossed(*trajectory, line_string);
  ASSERT_EQ(crossed_point.size(), 1);

  EXPECT_LT(0.0, crossed_point.at(0));
  EXPECT_LT(crossed_point.at(0), trajectory->length());
}

TEST_F(TrajectoryTest, Closest)
{
  ASSERT_TRUE(trajectory);
  geometry_msgs::msg::Pose pose;
  pose.position.x = 5.0;
  pose.position.y = 5.0;

  auto closest_pose =
    trajectory->compute(autoware::experimental::trajectory::closest(*trajectory, pose));

  double distance = std::hypot(
    closest_pose.point.pose.position.x - pose.position.x,
    closest_pose.point.pose.position.y - pose.position.y);

  EXPECT_LT(distance, 3.0);
}

TEST_F(TrajectoryTest, Crop)
{
  ASSERT_TRUE(trajectory);
  double length = trajectory->length();

  auto start_point_expect = trajectory->compute(length / 3.0);
  auto end_point_expect = trajectory->compute(length / 3.0 + 1.0);

  trajectory->crop(length / 3.0, 1.0);

  EXPECT_FLOAT_EQ(trajectory->length(), 1.0);

  auto start_point_actual = trajectory->compute(0.0);
  auto end_point_actual = trajectory->compute(trajectory->length());

  EXPECT_FLOAT_EQ(
    start_point_expect.point.pose.position.x, start_point_actual.point.pose.position.x);
  EXPECT_FLOAT_EQ(
    start_point_expect.point.pose.position.y, start_point_actual.point.pose.position.y);
  EXPECT_FLOAT_EQ(start_point_expect.lane_ids[0], start_point_actual.lane_ids[0]);

  EXPECT_FLOAT_EQ(end_point_expect.point.pose.position.x, end_point_actual.point.pose.position.x);
  EXPECT_FLOAT_EQ(end_point_expect.point.pose.position.y, end_point_actual.point.pose.position.y);
  EXPECT_EQ(end_point_expect.lane_ids[0], end_point_actual.lane_ids[0]);
}

TEST_F(TrajectoryTest, FindIf)
{
  ASSERT_TRUE(trajectory);
  geometry_msgs::msg::Point base_point;
  base_point.x = 5.0;
  base_point.y = 5.0;
  const auto radius = 3.0;

  {  // first index without binary search
    const auto index = autoware::experimental::trajectory::find_first_index_if(
      *trajectory, [&](const autoware_internal_planning_msgs::msg::PathPointWithLaneId & point) {
        return autoware_utils_geometry::calc_distance2d(point.point.pose.position, base_point) <
               radius;
      });
    ASSERT_TRUE(index.has_value());
    const auto point = trajectory->compute(*index).point.pose.position;
    EXPECT_FLOAT_EQ(point.x, 3.30);
    EXPECT_FLOAT_EQ(point.y, 4.01);
  }

  {  // first index with binary search
    const auto index = autoware::experimental::trajectory::find_first_index_if(
      *trajectory,
      [&](const autoware_internal_planning_msgs::msg::PathPointWithLaneId & point) {
        return autoware_utils_geometry::calc_distance2d(point.point.pose.position, base_point) <
               radius;
      },
      20);
    ASSERT_TRUE(index.has_value());
    const auto point = trajectory->compute(*index).point.pose.position;
    EXPECT_NEAR(autoware_utils_geometry::calc_distance2d(point, base_point), radius, 1e-3);
  }

  {  // last index without binary search
    const auto index = autoware::experimental::trajectory::find_last_index_if(
      *trajectory, [&](const autoware_internal_planning_msgs::msg::PathPointWithLaneId & point) {
        return autoware_utils_geometry::calc_distance2d(point.point.pose.position, base_point) <
               radius;
      });
    ASSERT_TRUE(index.has_value());
    const auto point = trajectory->compute(*index).point.pose.position;
    EXPECT_FLOAT_EQ(point.x, 6.49);
    EXPECT_FLOAT_EQ(point.y, 5.20);
  }

  {  // last index with binary search
    const auto index = autoware::experimental::trajectory::find_last_index_if(
      *trajectory,
      [&](const autoware_internal_planning_msgs::msg::PathPointWithLaneId & point) {
        return autoware_utils_geometry::calc_distance2d(point.point.pose.position, base_point) <
               radius;
      },
      20);
    ASSERT_TRUE(index.has_value());
    const auto point = trajectory->compute(*index).point.pose.position;
    EXPECT_NEAR(autoware_utils_geometry::calc_distance2d(point, base_point), radius, 1e-3);
  }
}

TEST_F(TrajectoryTest, FindInterval)
{
  ASSERT_TRUE(trajectory);
  auto intervals = autoware::experimental::trajectory::find_intervals(
    *trajectory, [](const autoware_internal_planning_msgs::msg::PathPointWithLaneId & point) {
      return point.lane_ids[0] == 1;
    });
  EXPECT_EQ(intervals.size(), 1);
  EXPECT_LT(0, intervals[0].start);
  EXPECT_LT(intervals[0].start, intervals[0].end);
  EXPECT_NEAR(intervals[0].end, trajectory->length(), 0.1);
}

TEST_F(TrajectoryTest, FindIntervalWithBinarySearch)
{
  ASSERT_TRUE(trajectory);
  geometry_msgs::msg::Point base_point;
  base_point.x = 6.0;
  base_point.y = 2.0;
  const double radius = 3.0;

  auto intervals_0 = autoware::experimental::trajectory::find_intervals(
    *trajectory, [&](const autoware_internal_planning_msgs::msg::PathPointWithLaneId & point) {
      return autoware_utils_geometry::calc_distance2d(point.point.pose.position, base_point) <
             radius;
    });
  EXPECT_EQ(intervals_0.size(), 1);

  auto intervals_1 = autoware::experimental::trajectory::find_intervals(
    *trajectory,
    [&](const autoware_internal_planning_msgs::msg::PathPointWithLaneId & point) {
      return autoware_utils_geometry::calc_distance2d(point.point.pose.position, base_point) <
             radius;
    },
    10);

  EXPECT_EQ(intervals_1.size(), 1);

  // interval_1 should be accurate than interval_0
  double interval_0_start_distance = autoware_utils_geometry::calc_distance2d(
    trajectory->compute(intervals_0[0].start).point.pose.position, base_point);
  double interval_0_end_distance = autoware_utils_geometry::calc_distance2d(
    trajectory->compute(intervals_0[0].end).point.pose.position, base_point);
  double interval_1_start_distance = autoware_utils_geometry::calc_distance2d(
    trajectory->compute(intervals_1[0].start).point.pose.position, base_point);
  double interval_1_end_distance = autoware_utils_geometry::calc_distance2d(
    trajectory->compute(intervals_1[0].end).point.pose.position, base_point);
  double interval_0_start_error_decrease =
    std::fabs(interval_0_start_distance - radius) - std::fabs(interval_1_start_distance - radius);
  double interval_0_end_error_decrease =
    std::fabs(interval_0_end_distance - radius) - std::fabs(interval_1_end_distance - radius);
  EXPECT_GT(interval_0_start_error_decrease, 0);
  EXPECT_GT(interval_0_end_error_decrease, 0);
}

TEST_F(TrajectoryTest, MaxCurvature)
{
  ASSERT_TRUE(trajectory);
  double max_curvature = autoware::experimental::trajectory::max_curvature(*trajectory);
  EXPECT_LT(0, max_curvature);
}

TEST_F(TrajectoryTest, GetContainedLaneIds)
{
  ASSERT_TRUE(trajectory);
  auto contained_lane_ids = trajectory->get_contained_lane_ids();
  EXPECT_EQ(2, contained_lane_ids.size());
  EXPECT_EQ(0, contained_lane_ids[0]);
  EXPECT_EQ(1, contained_lane_ids[1]);
}

TEST_F(TrajectoryTest, CopyCtor)
{
  ASSERT_TRUE(trajectory);
  const auto trajectory2(*trajectory);

  check_if_equals(*trajectory, trajectory2);
}

TEST_F(TrajectoryTest, CopyAssignment)
{
  ASSERT_TRUE(trajectory);
  const auto trajectory2 = Trajectory::Builder{}.build(
    {path_point_with_lane_id(0.00, 0.00, 1), path_point_with_lane_id(1.68, 0.81, 1),
     path_point_with_lane_id(2.98, 1.65, 1), path_point_with_lane_id(4.01, 3.30, 0)});

  *trajectory = *trajectory2;

  check_if_equals(*trajectory, *trajectory2);
}

TEST_F(TrajectoryTest, MoveCtor)
{
  ASSERT_TRUE(trajectory);
  const auto trajectory1(*trajectory);
  const auto trajectory2(std::move(*trajectory));

  check_if_equals(trajectory1, trajectory2);
}

TEST_F(TrajectoryTest, MoveAssignment)
{
  ASSERT_TRUE(trajectory);
  auto trajectory1 = Trajectory::Builder{}.build(
    {path_point_with_lane_id(0.00, 0.00, 1), path_point_with_lane_id(1.68, 0.81, 1),
     path_point_with_lane_id(2.98, 1.65, 1), path_point_with_lane_id(4.01, 3.30, 0)});

  const auto trajectory2(*trajectory1);
  trajectory = std::move(*trajectory1);

  check_if_equals(*trajectory, trajectory2);
}
