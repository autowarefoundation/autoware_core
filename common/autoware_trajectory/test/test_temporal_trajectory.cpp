// Copyright 2026 TIER IV, Inc.
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

#include "autoware/trajectory/temporal_trajectory.hpp"
#include "autoware/trajectory/utils/crossed.hpp"
#include "autoware/trajectory/utils/find_intervals.hpp"

#include <rclcpp/duration.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/primitives/LineString.h>

#include <vector>

namespace
{
using autoware::experimental::trajectory::TemporalTrajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

TrajectoryPoint make_point(const double x, const double time_from_start)
{
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = 0.0;
  point.longitudinal_velocity_mps = 1.0F;
  point.time_from_start = rclcpp::Duration::from_seconds(time_from_start);
  return point;
}
}  // namespace

TEST(TemporalTrajectory, ComputeFromTimeAndDistance)
{
  const std::vector<TrajectoryPoint> points{
    make_point(0.0, 0.0), make_point(1.0, 1.0), make_point(2.0, 2.0), make_point(3.0, 4.0)};

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto & trajectory = trajectory_result.value();

  const auto from_time = trajectory.compute_from_time(3.0);
  EXPECT_NEAR(from_time.pose.position.x, 2.5, 1e-6);
  EXPECT_NEAR(rclcpp::Duration(from_time.time_from_start).seconds(), 3.0, 1e-6);

  const auto from_distance = trajectory.compute_from_distance(2.5);
  EXPECT_NEAR(from_distance.pose.position.x, 2.5, 1e-6);
  EXPECT_NEAR(rclcpp::Duration(from_distance.time_from_start).seconds(), 3.0, 1e-6);

  const auto time = trajectory.distance_to_time(2.5);
  EXPECT_NEAR(time, 3.0, 1e-6);
  EXPECT_NEAR(trajectory.start_time(), 0.0, 1e-6);
  EXPECT_NEAR(trajectory.end_time(), 4.0, 1e-6);
}

TEST(TemporalTrajectory, BuilderBuildsFromSinglePoint)
{
  const std::vector<TrajectoryPoint> points{make_point(1.0, 2.0)};

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());

  const auto & trajectory = trajectory_result.value();
  EXPECT_NEAR(trajectory.start_time(), 2.0, 1e-6);
  EXPECT_NEAR(trajectory.end_time(), 2.0, 1e-6);
  EXPECT_NEAR(trajectory.duration(), 0.0, 1e-6);

  const auto restored = trajectory.restore();
  ASSERT_EQ(restored.size(), 1U);
  EXPECT_NEAR(restored.front().pose.position.x, 1.0, 1e-6);
  EXPECT_NEAR(rclcpp::Duration(restored.front().time_from_start).seconds(), 2.0, 1e-6);
}

TEST(TemporalTrajectory, BuilderBuildsFromTwoPoints)
{
  const std::vector<TrajectoryPoint> points{make_point(1.0, 2.0), make_point(3.0, 5.0)};

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());

  const auto & trajectory = trajectory_result.value();
  EXPECT_NEAR(trajectory.start_time(), 2.0, 1e-6);
  EXPECT_NEAR(trajectory.end_time(), 5.0, 1e-6);
  EXPECT_NEAR(trajectory.duration(), 3.0, 1e-6);

  const auto point_at_mid_time = trajectory.compute_from_time(3.5);
  EXPECT_NEAR(point_at_mid_time.pose.position.x, 2.0, 1e-6);
  EXPECT_NEAR(rclcpp::Duration(point_at_mid_time.time_from_start).seconds(), 3.5, 1e-6);
}

TEST(TemporalTrajectory, DuplicateTimestampWithDifferentDistanceExtendsTimeBases)
{
  const std::vector<TrajectoryPoint> points{
    make_point(0.0, 0.0), make_point(1.0, 1.0), make_point(2.0, 1.0), make_point(3.0, 2.0)};

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());

  const auto & trajectory = trajectory_result.value();
  const auto time_bases = trajectory.get_underlying_time_bases();

  ASSERT_EQ(time_bases.size(), points.size());
  EXPECT_DOUBLE_EQ(time_bases.at(0), 0.0);
  EXPECT_DOUBLE_EQ(time_bases.at(1), 1.0);
  EXPECT_GT(time_bases.at(2), time_bases.at(1));
  EXPECT_DOUBLE_EQ(time_bases.at(3), 2.0);

  const auto mapped_time = trajectory.distance_to_time(1.5);
  EXPECT_GE(mapped_time, 1.0);
  EXPECT_LE(mapped_time, 2.0);
}

TEST(TemporalTrajectory, CrossedUsesSpatialTrajectory)
{
  const std::vector<TrajectoryPoint> points{
    make_point(0.0, 0.0), make_point(1.0, 1.0), make_point(2.0, 2.0), make_point(3.0, 3.0)};

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto & trajectory = trajectory_result.value();

  lanelet::LineString2d line_string(lanelet::InvalId);
  line_string.push_back(lanelet::Point3d(lanelet::InvalId, 1.5, -1.0, 0.0));
  line_string.push_back(lanelet::Point3d(lanelet::InvalId, 1.5, 1.0, 0.0));

  const auto crossed_points = autoware::experimental::trajectory::crossed(trajectory, line_string);
  ASSERT_EQ(crossed_points.size(), 1U);
  EXPECT_NEAR(crossed_points.front().distance, 1.5, 1e-6);
  EXPECT_NEAR(crossed_points.front().time, 1.5, 1e-6);
}

TEST(TemporalTrajectory, FindIntervalsUsesTimeAxis)
{
  const std::vector<TrajectoryPoint> points{
    make_point(0.0, 0.0), make_point(1.0, 1.0), make_point(2.0, 2.0), make_point(3.0, 3.0)};

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto & trajectory = trajectory_result.value();

  const auto intervals = autoware::experimental::trajectory::find_intervals(
    trajectory, [](const TrajectoryPoint & point) {
      return point.pose.position.x >= 1.0 && point.pose.position.x <= 2.0;
    });

  ASSERT_EQ(intervals.size(), 1U);
  EXPECT_NEAR(intervals.front().start.distance, 1.0, 1e-6);
  EXPECT_NEAR(intervals.front().end.distance, 2.0, 1e-6);
  EXPECT_NEAR(intervals.front().start.time, 1.0, 1e-6);
  EXPECT_NEAR(intervals.front().end.time, 2.0, 1e-6);
}

TEST(TemporalTrajectory, CropTimeRebasesTimeAxis)
{
  const std::vector<TrajectoryPoint> points{
    make_point(0.0, -1.0), make_point(1.0, 0.0), make_point(2.0, 1.0), make_point(3.0, 2.0)};

  auto trajectory = TemporalTrajectory::Builder{}.build(points).value();
  trajectory.crop_time(0.0, 2.0);

  const auto restored = trajectory.restore();
  ASSERT_FALSE(restored.empty());
  EXPECT_EQ(restored.size(), 3U);
  EXPECT_NEAR(rclcpp::Duration(restored.front().time_from_start).seconds(), 0.0, 1e-6);
  EXPECT_NEAR(restored.front().pose.position.x, 1.0, 1e-6);
  EXPECT_NEAR(rclcpp::Duration(restored.back().time_from_start).seconds(), 2.0, 1e-6);
  EXPECT_NEAR(restored.back().pose.position.x, 3.0, 1e-6);
}

TEST(TemporalTrajectory, SetStoplineCollapsesFollowingPoints)
{
  const std::vector<TrajectoryPoint> points{
    make_point(0.0, 0.0), make_point(1.0, 1.0), make_point(2.0, 2.0), make_point(3.0, 3.0)};

  auto trajectory = TemporalTrajectory::Builder{}.build(points).value();
  trajectory.set_stopline(1.5);

  const auto stop_point = trajectory.compute_from_distance(1.5);
  const auto point_after_stop = trajectory.compute_from_time(2.5);
  EXPECT_NEAR(stop_point.pose.position.x, 1.5, 1e-3);
  EXPECT_NEAR(point_after_stop.pose.position.x, stop_point.pose.position.x, 1e-3);
  EXPECT_NEAR(point_after_stop.longitudinal_velocity_mps, 0.0, 1e-6);
}

TEST(TemporalTrajectory, SetStoplineWithTimeExtendsSchedule)
{
  const std::vector<TrajectoryPoint> points{
    make_point(0.0, 0.0), make_point(1.0, 1.0), make_point(2.0, 2.0), make_point(3.0, 3.0)};

  auto trajectory = TemporalTrajectory::Builder{}.build(points).value();
  trajectory.set_stopline(1.5, 1.5);

  const auto stop_point = trajectory.compute_from_time(3.0);
  EXPECT_NEAR(stop_point.pose.position.x, 1.5, 1e-3);
  EXPECT_NEAR(stop_point.longitudinal_velocity_mps, 0.0, 1e-6);
  EXPECT_NEAR(trajectory.duration(), 4.5, 1e-6);

  const auto point_after_stop = trajectory.compute_from_time(4.0);
  EXPECT_GT(point_after_stop.pose.position.x, 1.5);
}

TEST(TemporalTrajectory, DistanceToTimeReturnsFirstStopTime)
{
  const std::vector<TrajectoryPoint> points{
    make_point(0.0, 0.0), make_point(1.0, 1.0), make_point(2.0, 2.0), make_point(3.0, 3.0)};

  auto trajectory = TemporalTrajectory::Builder{}.build(points).value();
  trajectory.set_stopline(1.5, 1.5);

  const auto stop_time = trajectory.distance_to_time(1.5);
  EXPECT_NEAR(stop_time, 1.5, 1e-6);
}

// Test: distance_to_time clamps distance below range (returns start_time)
TEST(TemporalTrajectory, DistanceToTimeClampsBelowRange)
{
  const std::vector<TrajectoryPoint> points{
    make_point(0.0, 0.0), make_point(1.0, 1.0), make_point(2.0, 2.0), make_point(3.0, 3.0)};

  auto trajectory = TemporalTrajectory::Builder{}.build(points).value();

  // Request distance below 0 should return start_time
  const auto time = trajectory.distance_to_time(-1.0);
  EXPECT_NEAR(time, trajectory.start_time(), 1e-6);
}

// Test: distance_to_time clamps distance above range (returns end_time)
TEST(TemporalTrajectory, DistanceToTimeClampsAboveRange)
{
  const std::vector<TrajectoryPoint> points{
    make_point(0.0, 0.0), make_point(1.0, 1.0), make_point(2.0, 2.0), make_point(3.0, 3.0)};

  auto trajectory = TemporalTrajectory::Builder{}.build(points).value();

  // Request distance above length should return end_time
  const auto time = trajectory.distance_to_time(100.0);
  EXPECT_NEAR(time, trajectory.end_time(), 1e-6);
}

// Test: compute_from_distance at trajectory boundaries
TEST(TemporalTrajectory, ComputeFromDistanceAtBoundaries)
{
  const std::vector<TrajectoryPoint> points{
    make_point(0.0, 0.0), make_point(1.0, 1.0), make_point(2.0, 2.0), make_point(3.0, 3.0)};

  auto trajectory = TemporalTrajectory::Builder{}.build(points).value();

  // At s=0
  const auto at_start = trajectory.compute_from_distance(0.0);
  EXPECT_NEAR(at_start.pose.position.x, 0.0, 1e-6);
  EXPECT_NEAR(rclcpp::Duration(at_start.time_from_start).seconds(), 0.0, 1e-6);

  // At s=length
  const auto at_end = trajectory.compute_from_distance(trajectory.length());
  EXPECT_NEAR(at_end.pose.position.x, 3.0, 1e-6);
  EXPECT_NEAR(rclcpp::Duration(at_end.time_from_start).seconds(), 3.0, 1e-6);
}

// Test: compute_from_time at trajectory boundaries
TEST(TemporalTrajectory, ComputeFromTimeAtBoundaries)
{
  const std::vector<TrajectoryPoint> points{
    make_point(0.0, 0.0), make_point(1.0, 1.0), make_point(2.0, 2.0), make_point(3.0, 3.0)};

  auto trajectory = TemporalTrajectory::Builder{}.build(points).value();

  // At t=start_time
  const auto at_start = trajectory.compute_from_time(trajectory.start_time());
  EXPECT_NEAR(at_start.pose.position.x, 0.0, 1e-6);
  EXPECT_NEAR(rclcpp::Duration(at_start.time_from_start).seconds(), 0.0, 1e-6);

  // At t=end_time
  const auto at_end = trajectory.compute_from_time(trajectory.end_time());
  EXPECT_NEAR(at_end.pose.position.x, 3.0, 1e-6);
  EXPECT_NEAR(rclcpp::Duration(at_end.time_from_start).seconds(), 3.0, 1e-6);
}

// Test: restore after crop_time returns cropped trajectory points
TEST(TemporalTrajectory, RestoreAfterCropTime)
{
  const std::vector<TrajectoryPoint> points{
    make_point(0.0, 0.0), make_point(1.0, 1.0), make_point(2.0, 2.0), make_point(3.0, 3.0)};

  auto trajectory = TemporalTrajectory::Builder{}.build(points).value();
  trajectory.crop_time(1.0, 1.0);  // Crop from t=1.0 for duration 1.0

  const auto restored = trajectory.restore();
  ASSERT_FALSE(restored.empty());

  // All restored points should have time_from_start in [1.0, 2.0]
  for (const auto & point : restored) {
    const auto t = rclcpp::Duration(point.time_from_start).seconds();
    EXPECT_GE(t, 1.0 - 1e-6);
    EXPECT_LE(t, 2.0 + 1e-6);
  }
}
