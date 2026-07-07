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

#include "autoware/trajectory/threshold.hpp"
#include "autoware/trajectory/utils/crop.hpp"
#include "autoware/trajectory/utils/crossed.hpp"
#include "autoware/trajectory/utils/find_intervals.hpp"
#include "autoware/trajectory/utils/max.hpp"
#include "autoware/trajectory/utils/set_stopline.hpp"
#include "autoware/trajectory/utils/set_time_offset.hpp"

#include <rclcpp/duration.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/primitives/LineString.h>

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

TEST(CrossedTemporal, ReturnsCrossedPoint)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
    {3.0, 3.0},
  });

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

TEST(CrossedTemporal, ReturnsCrossedPointDuplicatedPoint)
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

  lanelet::LineString2d line_string(lanelet::InvalId);
  line_string.push_back(lanelet::Point3d(lanelet::InvalId, 2.0, -1.0, 0.0));
  line_string.push_back(lanelet::Point3d(lanelet::InvalId, 2.0, 1.0, 0.0));

  const auto crossed_points = autoware::experimental::trajectory::crossed(trajectory, line_string);
  ASSERT_EQ(crossed_points.size(), 1U);
  EXPECT_NEAR(crossed_points.front().distance, 2, 1e-6);
  EXPECT_NEAR(crossed_points.front().time, 2.0, 1e-6);
}

TEST(FindIntervalsTemporal, ReturnsEmptyWhenNoStopInterval)
{
  const auto points = make_points({
    {0.0, 0.0, 2.0F},
    {1.0, 1.0, 2.0F},
    {2.0, 2.0, 1.0F},
    {3.0, 3.0, 0.5F},
  });

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());

  const auto intervals = autoware::experimental::trajectory::find_intervals(
    trajectory_result.value(), [](const auto & point) {
      return std::abs(point.longitudinal_velocity_mps) <=
             autoware::experimental::trajectory::k_epsilon_velocity;
    });
  EXPECT_TRUE(intervals.empty());
}

TEST(FindIntervalsTemporal, ReturnsStopIntervalWithDistance)
{
  const auto points = make_points({
    {0.0, 0.0, 2.0F},
    {1.0, 1.0, 1.0F},
    {2.0, 2.0, 0.0F},
    {3.0, 2.0, 0.0F},
    {4.0, 3.0, 1.0F},
  });

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());

  const auto intervals = autoware::experimental::trajectory::find_intervals(
    trajectory_result.value(), [](const auto & point) {
      return std::abs(point.longitudinal_velocity_mps) <=
             autoware::experimental::trajectory::k_epsilon_velocity;
    });
  ASSERT_EQ(intervals.size(), 1U);
  EXPECT_NEAR(intervals.front().start.time, 2.0, 1e-6);
  EXPECT_NEAR(intervals.front().end.time, 3.0, 1e-6);
  EXPECT_NEAR(intervals.front().start.distance, 2.0, 1e-3);
  EXPECT_NEAR(intervals.front().end.distance, 2.0, 1e-3);
}

TEST(FindIntervalsTemporal, RespectsVelocityThreshold)
{
  const auto points = make_points({
    {0.0, 0.0, 2.0F},
    {1.0, 1.0, 0.2F},
    {2.0, 2.0, 0.1F},
    {3.0, 3.0, 0.3F},
  });

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());

  const auto intervals = autoware::experimental::trajectory::find_intervals(
    trajectory_result.value(),
    [](const auto & point) { return std::abs(point.longitudinal_velocity_mps) <= 0.25; });
  ASSERT_EQ(intervals.size(), 1U);
  EXPECT_NEAR(intervals.front().start.time, 1.0, 1e-6);
  EXPECT_NEAR(intervals.front().end.time, 2.0, 1e-6);
}

TEST(FindIntervalsTemporal, FindIntervalsByPosition)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
    {3.0, 3.0},
  });

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

TEST(FindIntervalsTemporal, FindIntervalsByTime)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
    {3.0, 3.0},
  });

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());

  const auto intervals = autoware::experimental::trajectory::find_intervals(
    trajectory_result.value(), [](const double t) { return 1.0 <= t && t <= 2.0; });

  ASSERT_EQ(intervals.size(), 1U);
  EXPECT_NEAR(intervals.front().start.time, 1.0, 1e-6);
  EXPECT_NEAR(intervals.front().end.time, 2.0, 1e-6);
  EXPECT_NEAR(intervals.front().start.distance, 1.0, 1e-6);
  EXPECT_NEAR(intervals.front().end.distance, 2.0, 1e-6);
}

TEST(FindIntervalsTemporal, FindsHighCurvatureInterval)
{
  constexpr double radius = 5.0;
  constexpr double curvature_threshold = 0.1;
  constexpr double expected_curve_start_time = 10.0;
  constexpr double expected_curve_end_time = expected_curve_start_time + radius * M_PI_2;

  const auto make_point = [](const double time, const double x, const double y) {
    TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.position.y = y;
    point.longitudinal_velocity_mps = 1.0F;
    point.time_from_start = rclcpp::Duration::from_seconds(time);
    return point;
  };

  std::vector<TrajectoryPoint> points;
  points.reserve(19);

  for (int i = 0; i <= 5; ++i) {
    const double x = -10.0 + 2.0 * static_cast<double>(i);
    points.push_back(make_point(x + 10.0, x, 0.0));
  }

  for (int i = 1; i <= 7; ++i) {
    const double angle = -M_PI_2 + M_PI_2 * static_cast<double>(i) / 8.0;
    const double time = expected_curve_start_time + radius * (angle + M_PI_2);
    points.push_back(make_point(time, radius * std::cos(angle), radius + radius * std::sin(angle)));
  }

  for (int i = 0; i <= 5; ++i) {
    const double y = radius + 2.0 * static_cast<double>(i);
    points.push_back(make_point(expected_curve_end_time + y - radius, radius, y));
  }

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto & trajectory = trajectory_result.value();

  const auto intervals = autoware::experimental::trajectory::find_intervals(
    trajectory,
    [&trajectory](const double t) {
      return std::abs(trajectory.curvature_from_time(t)) >= curvature_threshold;
    },
    20);

  ASSERT_EQ(intervals.size(), 1U);
  EXPECT_NEAR(intervals.front().start.time, expected_curve_start_time, 1.0);
  EXPECT_NEAR(intervals.front().end.time, expected_curve_end_time, 1.0);

  const auto mid_time = 0.5 * (intervals.front().start.time + intervals.front().end.time);
  EXPECT_GE(std::abs(trajectory.curvature_from_time(mid_time)), curvature_threshold);
  EXPECT_LT(std::abs(trajectory.curvature_from_time(2.0)), curvature_threshold);
  EXPECT_LT(
    std::abs(trajectory.curvature_from_time(expected_curve_end_time + 2.0)), curvature_threshold);
}

TEST(MaxTemporal, ReturnsMaxValueAndTimeDistancePair)
{
  const auto points = make_points({
    {0.0, 0.0, 1.0F},
    {1.0, 1.0, 4.0F},
    {2.0, 2.0, 2.0F},
  });

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());

  const auto result = autoware::experimental::trajectory::max(
    trajectory_result.value(), [](autoware_planning_msgs::msg::TrajectoryPoint point) {
      return point.longitudinal_velocity_mps;
    });

  EXPECT_NEAR(result.point.time, 1.0, 1e-6);
  EXPECT_NEAR(result.point.distance, 1.0, 1e-6);
  EXPECT_FLOAT_EQ(result.value, 4.0F);
}

TEST(MaxTemporal, AcceptsTimeEvaluator)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
  });

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());

  const auto result = autoware::experimental::trajectory::max(
    trajectory_result.value(), [](const double t) { return t; });

  EXPECT_NEAR(result.point.time, 2.0, 1e-6);
  EXPECT_NEAR(result.point.distance, 2.0, 1e-6);
  EXPECT_NEAR(result.value, 2.0, 1e-6);
}

TEST(MaxTemporal, SupportsFixedIntervalSearch)
{
  const auto points = make_points({
    {0.0, 0.0},
    {2.0, 2.0},
  });

  const auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());

  const auto result = autoware::experimental::trajectory::max<
    autoware::experimental::trajectory::MaxSearchMethod::FixedInterval>(
    trajectory_result.value(), [](const double t) { return -std::abs(t - 0.5); }, 0.1);

  EXPECT_NEAR(result.point.time, 0.5, 1e-6);
  EXPECT_NEAR(result.point.distance, 0.5, 1e-6);
  EXPECT_NEAR(result.value, 0.0, 1e-6);
}

TEST(CropTemporal, CropTimeRebases)
{
  const auto points = make_points({
    {-1.0, 0.0},
    {0.0, 1.0},
    {1.0, 2.0},
    {2.0, 3.0},
  });

  auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto trajectory = crop_time(trajectory_result.value(), 0.0, 2.0);

  const auto restored = trajectory.restore();
  ASSERT_FALSE(restored.empty());
  EXPECT_EQ(restored.size(), 3U);
  EXPECT_NEAR(rclcpp::Duration(restored.front().time_from_start).seconds(), 0.0, 1e-6);
  EXPECT_NEAR(restored.front().pose.position.x, 1.0, 1e-6);
  EXPECT_NEAR(rclcpp::Duration(restored.back().time_from_start).seconds(), 2.0, 1e-6);
  EXPECT_NEAR(restored.back().pose.position.x, 3.0, 1e-6);
}

TEST(CropTemporal, RestoreAfterCropTime)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
    {3.0, 3.0},
  });

  auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto trajectory = crop_time(trajectory_result.value(), 1.0, 1.0);

  const auto restored = trajectory.restore();
  ASSERT_FALSE(restored.empty());
  ASSERT_EQ(restored.size(), 2U);
  EXPECT_NEAR(rclcpp::Duration(restored.front().time_from_start).seconds(), 1.0, 1e-6);
  EXPECT_NEAR(restored.front().pose.position.x, 1.0, 1e-6);
  EXPECT_NEAR(rclcpp::Duration(restored.back().time_from_start).seconds(), 2.0, 1e-6);
  EXPECT_NEAR(restored.back().pose.position.x, 2.0, 1e-6);
}

TEST(CropTemporal, CropDistance)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
    {3.0, 3.0},
  });

  auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto trajectory = crop_distance(trajectory_result.value(), 1.0, 1.0);

  EXPECT_NEAR(trajectory.start_time(), 1.0, 1e-6);
  EXPECT_NEAR(trajectory.end_time(), 2.0, 1e-6);
  EXPECT_NEAR(trajectory.length(), 1.0, 1e-6);
}

TEST(CropTemporal, RestoreAfterCropDistance)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
    {3.0, 3.0},
  });

  auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto trajectory = crop_distance(trajectory_result.value(), 1.0, 1.0);

  const auto restored = trajectory.restore();
  ASSERT_FALSE(restored.empty());
  ASSERT_EQ(restored.size(), 2U);
  EXPECT_NEAR(rclcpp::Duration(restored.front().time_from_start).seconds(), 1.0, 1e-6);
  EXPECT_NEAR(restored.front().pose.position.x, 1.0, 1e-6);
  EXPECT_NEAR(rclcpp::Duration(restored.back().time_from_start).seconds(), 2.0, 1e-6);
  EXPECT_NEAR(restored.back().pose.position.x, 2.0, 1e-6);
}

TEST(CropTemporal, CropDistanceThrowsOnInvalidStartDistance)
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
  EXPECT_THROW(static_cast<void>(crop_distance(trajectory, -1.0, 1.0)), std::out_of_range);
  EXPECT_THROW(static_cast<void>(crop_distance(trajectory, 4.0, 1.0)), std::out_of_range);
}

TEST(CropTemporal, CropDistanceClampsExcessiveLength)
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
  EXPECT_THROW(static_cast<void>(crop_distance(trajectory, 1.0, -0.1)), std::out_of_range);

  const auto cropped = crop_distance(trajectory, 1.0, 3.0);
  EXPECT_NEAR(cropped.length(), 2.0, 1e-6);
  EXPECT_NEAR(cropped.start_time(), 1.0, 1e-6);
  EXPECT_NEAR(cropped.end_time(), 3.0, 1e-6);
}

TEST(CropTemporal, CropTimeClampsExcessiveDuration)
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
  EXPECT_THROW(static_cast<void>(crop_time(trajectory, 1.0, -0.1)), std::out_of_range);

  const auto cropped = crop_time(trajectory, 1.0, 5.0);
  EXPECT_NEAR(cropped.duration(), 2.0, 1e-6);
  EXPECT_NEAR(cropped.start_time(), 1.0, 1e-6);
  EXPECT_NEAR(cropped.end_time(), 3.0, 1e-6);
}

TEST(CropTemporal, CropDistanceEndsAtStopPointWithPlateau)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 0.0},
    {2.0, 1.0},
    {3.0, 2.0},
    {4.0, 2.0},
    {5.0, 2.0},
  });

  auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto cropped = crop_distance(trajectory_result.value(), 0.0, 2.0);

  EXPECT_NEAR(cropped.length(), 2.0, 1e-3);
  EXPECT_NEAR(cropped.start_time(), 0.0, 1e-6);
  EXPECT_NEAR(cropped.end_time(), 5.0, 1e-6);
}

TEST(SetStoplineTemporal, SetStoplineCollapsesFollowingPoints)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
    {3.0, 3.0},
  });

  auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto trajectory = set_stopline(trajectory_result.value(), 1.5);

  const auto stop_point = trajectory.compute_from_distance(1.5);
  const auto point_after_stop = trajectory.compute_from_time(2.5);
  EXPECT_NEAR(stop_point.pose.position.x, 1.5, 1e-3);
  EXPECT_NEAR(point_after_stop.pose.position.x, stop_point.pose.position.x, 1e-3);
  EXPECT_NEAR(point_after_stop.longitudinal_velocity_mps, 0.0, 1e-6);
}

TEST(SetStoplineTemporal, SetStoplineWithTimeExtendsSchedule)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
    {3.0, 3.0},
  });

  auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto trajectory = insert_stop_duration(trajectory_result.value(), 1.5, 1.5);

  const auto stop_point = trajectory.compute_from_time(3.0);
  EXPECT_NEAR(stop_point.pose.position.x, 1.5, 1e-3);
  EXPECT_NEAR(stop_point.longitudinal_velocity_mps, 0.0, 1e-6);
  EXPECT_NEAR(trajectory.duration(), 4.5, 1e-6);

  const auto point_after_stop = trajectory.compute_from_time(4.0);
  EXPECT_GT(point_after_stop.pose.position.x, 1.5);
}

TEST(SetStoplineTemporal, DistanceToTimeReturnsFirstStopTime)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
    {3.0, 3.0},
  });

  auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto trajectory = insert_stop_duration(trajectory_result.value(), 1.5, 1.5);

  const auto stop_time = trajectory.distance_to_time(1.5);
  EXPECT_NEAR(stop_time, 1.5, 1e-6);
}

TEST(SetStoplineTemporal, DistanceToTimeReturnsEndTimeAfterThreeArgStopline)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
    {3.0, 3.0},
  });

  auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto stopped = insert_stop_duration(trajectory_result.value(), 1.5, 1.5);

  EXPECT_NEAR(stopped.distance_to_time(1.5), 1.5, 1e-6);
  EXPECT_NEAR(stopped.distance_to_time(1.5, true), 3.0, 1e-6);
}

TEST(SetStoplineTemporal, DistanceToTimeReturnsEndTimeAfterTwoArgStopline)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
    {3.0, 3.0},
  });

  auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto stopped = set_stopline(trajectory_result.value(), 1.5);

  EXPECT_NEAR(stopped.distance_to_time(1.5), 1.5, 1e-6);
  EXPECT_NEAR(stopped.distance_to_time(1.5, true), 3.0, 1e-6);
}

TEST(SetStoplineTemporal, TwoArgIdempotent)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
    {3.0, 3.0},
  });

  auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto t1 = set_stopline(trajectory_result.value(), 1.5);
  const auto t2 = set_stopline(t1, 1.5);
  const auto stop_point = t2.compute_from_distance(1.5);

  EXPECT_NEAR(t2.length(), 1.5, 1e-3);
  EXPECT_NEAR(t2.duration(), t1.duration(), 1e-6);
  EXPECT_NEAR(stop_point.pose.position.x, 1.5, 1e-3);
  EXPECT_NEAR(stop_point.longitudinal_velocity_mps, 0.0, 1e-6);
}

TEST(SetStoplineTemporal, ThreeArgAdditive)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
    {3.0, 3.0},
  });

  auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto t1 = insert_stop_duration(trajectory_result.value(), 1.5, 1.5);
  const auto t2 = insert_stop_duration(t1, 1.5, 1.5);
  const auto at_stop = t2.compute_from_time(1.5);
  const auto during_second_plateau = t2.compute_from_time(3.5);
  const auto after_stop = t2.compute_from_time(4.6);

  EXPECT_NEAR(t2.duration(), 6.0, 1e-6);
  EXPECT_NEAR(at_stop.pose.position.x, 1.5, 1e-3);
  EXPECT_NEAR(at_stop.longitudinal_velocity_mps, 0.0, 1e-6);
  EXPECT_NEAR(during_second_plateau.pose.position.x, 1.5, 1e-3);
  EXPECT_NEAR(during_second_plateau.longitudinal_velocity_mps, 0.0, 1e-6);
  EXPECT_GT(after_stop.pose.position.x, 1.5);
}

TEST(SetStoplineTemporal, FindIntervalsAfterTwoArgStopline)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
    {3.0, 3.0},
  });

  auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto trajectory = set_stopline(trajectory_result.value(), 1.5);

  const auto intervals =
    autoware::experimental::trajectory::find_intervals(trajectory, [](const auto & point) {
      return std::abs(point.longitudinal_velocity_mps) <=
             autoware::experimental::trajectory::k_epsilon_velocity;
    });
  ASSERT_EQ(intervals.size(), 1U);
  EXPECT_NEAR(intervals.front().start.time, 1.5, 1e-3);
  EXPECT_NEAR(intervals.front().start.distance, 1.5, 1e-3);
  EXPECT_NEAR(intervals.front().end.time, 3.0, 1e-3);
  EXPECT_NEAR(intervals.front().end.distance, 1.5, 1e-3);
}

TEST(SetStoplineTemporal, FindIntervalsAfterThreeArgStopline)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
    {3.0, 3.0},
  });

  auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto trajectory = insert_stop_duration(trajectory_result.value(), 1.5, 1.5);

  const auto intervals =
    autoware::experimental::trajectory::find_intervals(trajectory, [](const auto & point) {
      return std::abs(point.longitudinal_velocity_mps) <=
             autoware::experimental::trajectory::k_epsilon_velocity;
    });
  ASSERT_EQ(intervals.size(), 1U);
  EXPECT_NEAR(intervals.front().start.time, 1.5, 1e-3);
  EXPECT_NEAR(intervals.front().start.distance, 1.5, 1e-3);
  EXPECT_NEAR(intervals.front().end.time, 3.0, 1e-3);
  EXPECT_NEAR(intervals.front().end.distance, 1.5, 1e-3);
}

TEST(SetStoplineTemporal, FindIntervalsAfterAdditiveStopline)
{
  const auto points = make_points({
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
    {3.0, 3.0},
  });

  auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto t1 = insert_stop_duration(trajectory_result.value(), 1.5, 1.5);
  const auto t2 = insert_stop_duration(t1, 1.5, 1.5);

  const auto intervals =
    autoware::experimental::trajectory::find_intervals(t2, [](const auto & point) {
      return std::abs(point.longitudinal_velocity_mps) <=
             autoware::experimental::trajectory::k_epsilon_velocity;
    });
  ASSERT_EQ(intervals.size(), 1U);
  EXPECT_NEAR(intervals.front().start.time, 1.5, 1e-3);
  EXPECT_NEAR(intervals.front().start.distance, 1.5, 1e-3);
  EXPECT_NEAR(intervals.front().end.time, 4.5, 1e-3);
  EXPECT_NEAR(intervals.front().end.distance, 1.5, 1e-3);
}

TEST(SetTimeOffset, ShiftsNegativeRangeToPositive)
{
  const auto points = make_points({
    {-1.0, -1.0},
    {0.0, 0.0},
    {1.0, 1.0},
    {2.0, 2.0},
  });

  auto trajectory_result = TemporalTrajectory::Builder{}.build(points);
  ASSERT_TRUE(trajectory_result.has_value());
  const auto trajectory =
    autoware::experimental::trajectory::set_time_offset(trajectory_result.value(), -1.0);

  const auto start_time = trajectory.start_time();
  const auto end_time = trajectory.end_time();
  const auto duration = trajectory.duration();
  const auto at_zero = trajectory.compute_from_time(0.0);
  const auto at_mid = trajectory.compute_from_time(1.5);
  const auto at_end = trajectory.compute_from_time(3.0);

  EXPECT_NEAR(start_time, 0.0, 1e-6);
  EXPECT_NEAR(end_time, 3.0, 1e-6);
  EXPECT_NEAR(duration, 3.0, 1e-6);
  EXPECT_NEAR(at_zero.pose.position.x, -1.0, 1e-6);
  EXPECT_NEAR(rclcpp::Duration(at_zero.time_from_start).seconds(), 0.0, 1e-6);
  EXPECT_NEAR(at_mid.pose.position.x, 0.5, 1e-6);
  EXPECT_NEAR(at_end.pose.position.x, 2.0, 1e-6);
}
