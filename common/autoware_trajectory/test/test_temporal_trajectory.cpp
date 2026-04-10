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

TEST(temporal_trajectory, compute_from_time_and_distance)
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
  ASSERT_TRUE(time.has_value());
  EXPECT_NEAR(*time, 3.0, 1e-6);
  EXPECT_NEAR(trajectory.start_time(), 0.0, 1e-6);
  EXPECT_NEAR(trajectory.end_time(), 4.0, 1e-6);
}

TEST(temporal_trajectory, duplicate_timestamp_with_different_distance_fails)
{
  const std::vector<TrajectoryPoint> points{
    make_point(0.0, 0.0), make_point(1.0, 1.0), make_point(2.0, 1.0), make_point(3.0, 2.0)};

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  EXPECT_FALSE(trajectory_result.has_value());
}

TEST(temporal_trajectory, crossed_uses_spatial_trajectory)
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
  EXPECT_NEAR(crossed_points.front(), 1.5, 1e-6);
}

TEST(temporal_trajectory, crop_time_rebases_time_axis)
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

TEST(temporal_trajectory, set_stopline_collapses_following_points)
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

TEST(temporal_trajectory, set_stopline_with_time_extends_schedule)
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

TEST(temporal_trajectory, distance_to_time_returns_first_stop_time)
{
  const std::vector<TrajectoryPoint> points{
    make_point(0.0, 0.0), make_point(1.0, 1.0), make_point(2.0, 2.0), make_point(3.0, 3.0)};

  auto trajectory = TemporalTrajectory::Builder{}.build(points).value();
  trajectory.set_stopline(1.5, 1.5);

  const auto stop_time = trajectory.distance_to_time(1.5);
  ASSERT_TRUE(stop_time.has_value());
  EXPECT_NEAR(*stop_time, 1.5, 1e-6);
}
