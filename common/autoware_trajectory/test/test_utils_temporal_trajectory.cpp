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

#include "autoware/trajectory/utils/temporal/find_stop_points.hpp"

#include <rclcpp/duration.hpp>

#include <gtest/gtest.h>

#include <vector>

namespace
{
using autoware::experimental::trajectory::find_stop_points;
using autoware::experimental::trajectory::TemporalTrajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

TrajectoryPoint make_point(const double x, const double time_from_start, const float velocity)
{
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = 0.0;
  point.longitudinal_velocity_mps = velocity;
  point.time_from_start = rclcpp::Duration::from_seconds(time_from_start);
  return point;
}
}  // namespace

TEST(find_stop_points, returns_empty_when_no_stop_interval)
{
  const std::vector<TrajectoryPoint> points{
    make_point(0.0, 0.0, 2.0F), make_point(1.0, 1.0, 2.0F), make_point(2.0, 2.0, 1.0F),
    make_point(3.0, 3.0, 0.5F)};

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());

  const auto stop_points = find_stop_points(trajectory_result.value());
  EXPECT_TRUE(stop_points.empty());
}

TEST(find_stop_points, returns_stop_interval_with_distance)
{
  const std::vector<TrajectoryPoint> points{
    make_point(0.0, 0.0, 2.0F), make_point(1.0, 1.0, 1.0F), make_point(2.0, 2.0, 0.0F),
    make_point(2.0, 3.0, 0.0F), make_point(3.0, 4.0, 1.0F)};

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());

  const auto stop_points = find_stop_points(trajectory_result.value());
  ASSERT_EQ(stop_points.size(), 1U);
  EXPECT_NEAR(stop_points.front().start_time, 2.0, 1e-6);
  EXPECT_NEAR(stop_points.front().end_time, 3.0, 1e-6);
  EXPECT_NEAR(stop_points.front().distance, 2.0, 1e-3);
}

TEST(find_stop_points, respects_velocity_threshold)
{
  const std::vector<TrajectoryPoint> points{
    make_point(0.0, 0.0, 2.0F), make_point(1.0, 1.0, 0.2F), make_point(2.0, 2.0, 0.1F),
    make_point(3.0, 3.0, 0.3F)};

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());

  const auto stop_points = find_stop_points(trajectory_result.value(), 0.25);
  ASSERT_EQ(stop_points.size(), 1U);
  EXPECT_NEAR(stop_points.front().start_time, 1.0, 1e-6);
  EXPECT_NEAR(stop_points.front().end_time, 2.0, 1e-6);
}
