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

#include <cmath>
#include <vector>

namespace
{
using autoware::experimental::trajectory::TemporalTrajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

struct PointParam
{
  double time{};
  double x{};
  double y = 0.0;
  double z = 0.0;
  float velocity = 1.0F;
};

std::vector<TrajectoryPoint> make_points(const std::initializer_list<PointParam> & inits)
{
  std::vector<TrajectoryPoint> points;
  points.reserve(inits.size());
  for (const auto & init : inits) {
    TrajectoryPoint point;
    point.pose.position.x = init.x;
    point.pose.position.y = init.y;
    point.pose.position.z = init.z;
    point.longitudinal_velocity_mps = init.velocity;
    point.time_from_start = rclcpp::Duration::from_seconds(init.time);
    points.push_back(point);
  }
  return points;
}
}  // namespace

TEST(TemporalTrajectory, StartAndEndTime)
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

  EXPECT_NEAR(trajectory.start_time(), 0.0, 1e-6);
  EXPECT_NEAR(trajectory.end_time(), 4.0, 1e-6);
}

TEST(TemporalTrajectory, ComputeFromTime)
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

  const auto result = trajectory.compute_from_time(3.0);
  EXPECT_GE(result.pose.position.x, 2.0);
  EXPECT_LE(result.pose.position.x, 3.0);
  EXPECT_NEAR(rclcpp::Duration(result.time_from_start).seconds(), 3.0, 1e-6);
}

TEST(TemporalTrajectory, ComputeFromDistance)
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

  const auto result = trajectory.compute_from_distance(2.5);
  EXPECT_NEAR(result.pose.position.x, 2.5, 1e-6);
  EXPECT_GE(rclcpp::Duration(result.time_from_start).seconds(), 2.0);
  EXPECT_LE(rclcpp::Duration(result.time_from_start).seconds(), 4.0);
}

TEST(TemporalTrajectory, DistanceToTime)
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

  const auto mapped_time = trajectory.distance_to_time(2.5);
  EXPECT_GE(mapped_time, 2.0);
  EXPECT_LE(mapped_time, 4.0);
}

TEST(TemporalTrajectory, DistanceToTimeAtDuplicatePoints)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
    {3.0, 2.0},
    {4.0, 2.0},
  });

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());

  const auto & trajectory = trajectory_result.value();
  const auto at_stop = trajectory.distance_to_time(2.0);
  EXPECT_NEAR(at_stop, 2.0, 1e-6);
}

TEST(TemporalTrajectory, DistanceToTimeReturnsEndTimeAtStopPoint)
{
  const auto points = make_points({
    {0.0, 0.0, 0.0, 0.0, 1.0F},
    {1.0, 1.0, 0.0, 0.0, 1.0F},
    {2.0, 2.0, 0.0, 0.0, 0.0F},
    {3.0, 2.0, 0.0, 0.0, 0.0F},
    {4.0, 2.0, 0.0, 0.0, 0.0F},
  });

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto & trajectory = trajectory_result.value();

  EXPECT_NEAR(trajectory.distance_to_time(2.0), 2.0, 1e-6);
  EXPECT_NEAR(trajectory.distance_to_time(2.0, true), 4.0, 1e-6);
}

TEST(TemporalTrajectory, ComputeFromTimeDuringDuplicateInterval)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
    {3.0, 2.0},
    {4.0, 3.0},
  });

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto & trajectory = trajectory_result.value();
  const auto at_t2_5 = trajectory.compute_from_time(2.5);
  const auto at_t3_5 = trajectory.compute_from_time(3.5);

  EXPECT_NEAR(at_t2_5.pose.position.x, 2.0, 1e-3);
  EXPECT_GE(at_t3_5.pose.position.x, 2.0);
  EXPECT_LE(at_t3_5.pose.position.x, 3.0);
  EXPECT_GE(at_t3_5.pose.position.x, at_t2_5.pose.position.x);
}

TEST(TemporalTrajectory, ComputeFromDistanceAtDuplicateInterval)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
    {3.0, 2.0},
    {4.0, 3.0},
  });

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto & trajectory = trajectory_result.value();

  EXPECT_NEAR(trajectory.distance_to_time(2.0), 2.0, 1e-6);
}

TEST(TemporalTrajectory, BuildFromSinglePoint)
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

TEST(TemporalTrajectory, BuildFromTwoPoints)
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

// Test: distance_to_time throws when distance is below range
TEST(TemporalTrajectory, DistanceToTimeThrowsBelowRange)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
    {3.0, 3.0},
  });

  auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  auto trajectory = trajectory_result.value();
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

  auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  auto trajectory = trajectory_result.value();
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

  auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  auto trajectory = trajectory_result.value();
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

  auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  auto trajectory = trajectory_result.value();
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

  auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  auto trajectory = trajectory_result.value();
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

  auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  auto trajectory = trajectory_result.value();

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

  auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  auto trajectory = trajectory_result.value();

  const auto at_start = trajectory.compute_from_time(trajectory.start_time());
  const auto at_end = trajectory.compute_from_time(trajectory.end_time());

  EXPECT_NEAR(at_start.pose.position.x, 0.0, 1e-6);
  EXPECT_NEAR(rclcpp::Duration(at_start.time_from_start).seconds(), 0.0, 1e-6);
  EXPECT_NEAR(at_end.pose.position.x, 3.0, 1e-6);
  EXPECT_NEAR(rclcpp::Duration(at_end.time_from_start).seconds(), 3.0, 1e-6);
}

TEST(TemporalTrajectory, AzimuthFromDistance)
{
  const auto points = make_points({
    {0.0, 0.0, 0.0},
    {1.0, 1.0, 1.0},
    {2.0, 2.0, 2.0},
  });

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto & trajectory = trajectory_result.value();

  EXPECT_NEAR(trajectory.azimuth_from_distance(0.5), M_PI / 4.0, M_PI / 10.0);
  EXPECT_NEAR(trajectory.azimuth_from_distance(1.5), M_PI / 4.0, M_PI / 10.0);
}

TEST(TemporalTrajectory, AzimuthFromTime)
{
  const auto points = make_points({
    {0.0, 0.0, 0.0},
    {1.0, 1.0, 1.0},
    {2.0, 2.0, 2.0},
  });

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto & trajectory = trajectory_result.value();

  EXPECT_NEAR(trajectory.azimuth_from_time(0.5), M_PI / 4.0, M_PI / 10.0);
  EXPECT_NEAR(trajectory.azimuth_from_time(1.5), M_PI / 4.0, M_PI / 10.0);
}

TEST(TemporalTrajectory, ElevationFromDistance)
{
  const auto points = make_points({
    {0.0, 0.0, 0.0, 0.0},
    {1.0, 1.0, 0.0, 1.0},
    {2.0, 2.0, 0.0, 2.0},
  });

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto & trajectory = trajectory_result.value();

  const auto e1 = trajectory.elevation_from_distance(0.5);
  const auto e2 = trajectory.elevation_from_distance(1.5);
  EXPECT_NEAR(e1, M_PI / 4.0, M_PI / 10.0);
  EXPECT_NEAR(e2, M_PI / 4.0, M_PI / 10.0);
}

TEST(TemporalTrajectory, ElevationFromTime)
{
  const auto points = make_points({
    {0.0, 0.0, 0.0, 0.0},
    {1.0, 1.0, 0.0, 1.0},
    {2.0, 2.0, 0.0, 2.0},
  });

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto & trajectory = trajectory_result.value();

  const auto e1 = trajectory.elevation_from_time(0.5);
  const auto e2 = trajectory.elevation_from_time(1.5);
  EXPECT_NEAR(e1, M_PI / 4.0, M_PI / 10.0);
  EXPECT_NEAR(e2, M_PI / 4.0, M_PI / 10.0);
}

TEST(TemporalTrajectory, CurvatureFromDistance)
{
  constexpr double radius = 5.0;
  const auto points = make_points({
    {0.0, radius, 0.0},
    {1.0, radius * std::cos(M_PI / 6.0), radius * std::sin(M_PI / 6.0)},
    {2.0, radius * std::cos(M_PI / 3.0), radius * std::sin(M_PI / 3.0)},
    {3.0, 0.0, radius},
  });

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto & trajectory = trajectory_result.value();

  const auto curvature = trajectory.curvature_from_distance(1.5);
  EXPECT_NEAR(curvature, 1.0 / radius, 1e-1);
}

TEST(TemporalTrajectory, CurvatureFromTime)
{
  constexpr double radius = 5.0;
  const auto points = make_points({
    {0.0, radius, 0.0},
    {1.0, radius * std::cos(M_PI / 6.0), radius * std::sin(M_PI / 6.0)},
    {2.0, radius * std::cos(M_PI / 3.0), radius * std::sin(M_PI / 3.0)},
    {3.0, 0.0, radius},
  });

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto & trajectory = trajectory_result.value();

  const auto curvature = trajectory.curvature_from_time(1.5);
  EXPECT_NEAR(curvature, 1.0 / radius, 1e-1);
}
