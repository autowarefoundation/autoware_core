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

#include <rclcpp/duration.hpp>

#include <gtest/gtest.h>

#include <vector>

namespace
{
using autoware::experimental::trajectory::TemporalTrajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

struct PointParam
{
  double time{};
  double x{};
  float velocity = 1.0F;
};

std::vector<TrajectoryPoint> make_points(const std::initializer_list<PointParam> & inits)
{
  std::vector<TrajectoryPoint> points;
  points.reserve(inits.size());
  for (const auto & init : inits) {
    TrajectoryPoint point;
    point.pose.position.x = init.x;
    point.pose.position.y = 0.0;
    point.longitudinal_velocity_mps = init.velocity;
    point.time_from_start = rclcpp::Duration::from_seconds(init.time);
    points.push_back(point);
  }
  return points;
}
}  // namespace

TEST(TemporalTrajectory, ComputeFromTimeAndDistance)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
    {4.0, 3.0},
  });

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto & trajectory = trajectory_result.value();

  const auto from_time = trajectory.compute_from_time(3.0);
  const auto from_distance = trajectory.compute_from_distance(2.5);
  const auto time = trajectory.distance_to_time(2.5);
  const auto start_time = trajectory.start_time();
  const auto end_time = trajectory.end_time();

  EXPECT_NEAR(from_time.pose.position.x, 2.5, 1e-6);
  EXPECT_NEAR(rclcpp::Duration(from_time.time_from_start).seconds(), 3.0, 1e-6);
  EXPECT_NEAR(from_distance.pose.position.x, 2.5, 1e-6);
  EXPECT_NEAR(rclcpp::Duration(from_distance.time_from_start).seconds(), 3.0, 1e-6);
  EXPECT_NEAR(time, 3.0, 1e-6);
  EXPECT_NEAR(start_time, 0.0, 1e-6);
  EXPECT_NEAR(end_time, 4.0, 1e-6);
}

TEST(TemporalTrajectory, BuilderBuildsFromSinglePoint)
{
  const auto points = make_points({{2.0, 1.0}});

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());

  const auto & trajectory = trajectory_result.value();
  const auto restored = trajectory.restore();
  ASSERT_EQ(restored.size(), 1U);

  EXPECT_NEAR(trajectory.start_time(), 2.0, 1e-6);
  EXPECT_NEAR(trajectory.end_time(), 2.0, 1e-6);
  EXPECT_NEAR(trajectory.duration(), 0.0, 1e-6);
  EXPECT_NEAR(restored.front().pose.position.x, 1.0, 1e-6);
  EXPECT_NEAR(rclcpp::Duration(restored.front().time_from_start).seconds(), 2.0, 1e-6);
}

TEST(TemporalTrajectory, BuilderBuildsFromTwoPoints)
{
  const auto points = make_points({
    {2.0, 1.0},
    {5.0, 3.0},
  });

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());

  const auto & trajectory = trajectory_result.value();
  const auto point_at_mid_time = trajectory.compute_from_time(3.5);

  EXPECT_NEAR(trajectory.start_time(), 2.0, 1e-6);
  EXPECT_NEAR(trajectory.end_time(), 5.0, 1e-6);
  EXPECT_NEAR(trajectory.duration(), 3.0, 1e-6);
  EXPECT_NEAR(point_at_mid_time.pose.position.x, 2.0, 1e-6);
  EXPECT_NEAR(rclcpp::Duration(point_at_mid_time.time_from_start).seconds(), 3.5, 1e-6);
}

TEST(TemporalTrajectory, DuplicateTimestampWithDifferentDistanceExtendsTimeBases)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {1.0, 2.0},
    {2.0, 3.0},
  });

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());

  const auto & trajectory = trajectory_result.value();
  const auto time_bases = trajectory.get_underlying_time_bases();
  const auto mapped_time = trajectory.distance_to_time(1.5);

  ASSERT_EQ(time_bases.size(), points.size());
  EXPECT_DOUBLE_EQ(time_bases.at(0), 0.0);
  EXPECT_DOUBLE_EQ(time_bases.at(1), 1.0);
  EXPECT_GT(time_bases.at(2), time_bases.at(1));
  EXPECT_DOUBLE_EQ(time_bases.at(3), 2.0);
  EXPECT_GE(mapped_time, 1.0);
  EXPECT_LE(mapped_time, 2.0);
}

// Test: distance_to_time throws when distance is below range
TEST(TemporalTrajectory, DistanceToTimeThrowsBelowRange)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
    {3.0, 3.0},
  });

  auto trajectory = TemporalTrajectory::Builder{}.build(points).value();
  EXPECT_THROW(static_cast<void>(trajectory.distance_to_time(-1.0)), std::out_of_range);
}

// Test: distance_to_time throws when distance is above range
TEST(TemporalTrajectory, DistanceToTimeThrowsAboveRange)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
    {3.0, 3.0},
  });

  auto trajectory = TemporalTrajectory::Builder{}.build(points).value();
  EXPECT_THROW(static_cast<void>(trajectory.distance_to_time(100.0)), std::out_of_range);
}

// Test: compute_from_time throws when time is out of range
TEST(TemporalTrajectory, ComputeFromTimeThrowsOutOfRange)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
    {3.0, 3.0},
  });

  auto trajectory = TemporalTrajectory::Builder{}.build(points).value();
  EXPECT_THROW(static_cast<void>(trajectory.compute_from_time(-1.0)), std::out_of_range);
  EXPECT_THROW(static_cast<void>(trajectory.compute_from_time(5.0)), std::out_of_range);
}

// Test: compute_from_distance throws when distance is out of range
TEST(TemporalTrajectory, ComputeFromDistanceThrowsOutOfRange)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
    {3.0, 3.0},
  });

  auto trajectory = TemporalTrajectory::Builder{}.build(points).value();
  EXPECT_THROW(static_cast<void>(trajectory.compute_from_distance(-1.0)), std::out_of_range);
  EXPECT_THROW(static_cast<void>(trajectory.compute_from_distance(100.0)), std::out_of_range);
}

// Test: time_to_distance throws when time is out of range
TEST(TemporalTrajectory, TimeToDistanceThrowsOutOfRange)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
    {3.0, 3.0},
  });

  auto trajectory = TemporalTrajectory::Builder{}.build(points).value();
  EXPECT_THROW(static_cast<void>(trajectory.time_to_distance(-1.0)), std::out_of_range);
  EXPECT_THROW(static_cast<void>(trajectory.time_to_distance(5.0)), std::out_of_range);
}

// Test: compute_from_distance at trajectory boundaries
TEST(TemporalTrajectory, ComputeFromDistanceAtBoundaries)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
    {3.0, 3.0},
  });

  auto trajectory = TemporalTrajectory::Builder{}.build(points).value();

  const auto at_start = trajectory.compute_from_distance(0.0);
  const auto at_end = trajectory.compute_from_distance(trajectory.length());

  EXPECT_NEAR(at_start.pose.position.x, 0.0, 1e-6);
  EXPECT_NEAR(rclcpp::Duration(at_start.time_from_start).seconds(), 0.0, 1e-6);
  EXPECT_NEAR(at_end.pose.position.x, 3.0, 1e-6);
  EXPECT_NEAR(rclcpp::Duration(at_end.time_from_start).seconds(), 3.0, 1e-6);
}

// Test: compute_from_time at trajectory boundaries
TEST(TemporalTrajectory, ComputeFromTimeAtBoundaries)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
    {3.0, 3.0},
  });

  auto trajectory = TemporalTrajectory::Builder{}.build(points).value();

  const auto at_start = trajectory.compute_from_time(trajectory.start_time());
  const auto at_end = trajectory.compute_from_time(trajectory.end_time());

  EXPECT_NEAR(at_start.pose.position.x, 0.0, 1e-6);
  EXPECT_NEAR(rclcpp::Duration(at_start.time_from_start).seconds(), 0.0, 1e-6);
  EXPECT_NEAR(at_end.pose.position.x, 3.0, 1e-6);
  EXPECT_NEAR(rclcpp::Duration(at_end.time_from_start).seconds(), 3.0, 1e-6);
}
