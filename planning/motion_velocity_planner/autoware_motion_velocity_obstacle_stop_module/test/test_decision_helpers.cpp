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

#include "decision_helpers.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace autoware::motion_velocity_planner::obstacle_stop_internal
{

TEST(CalcMinimumDistanceToStop, ForwardMotionUsesMinAcc)
{
  // initial_vel >= 0 -> uses min_acc branch: -v^2 / (2 * min_acc)
  // -(2^2) / (2 * -1.0) = 2.0
  EXPECT_DOUBLE_EQ(calc_minimum_distance_to_stop(2.0, 1.0, -1.0), 2.0);
  // -(4^2) / (2 * -2.0) = 4.0
  EXPECT_DOUBLE_EQ(calc_minimum_distance_to_stop(4.0, 1.0, -2.0), 4.0);
}

TEST(CalcMinimumDistanceToStop, ZeroVelocityUsesMinAcc)
{
  // initial_vel == 0 is not < 0, so the min_acc branch is taken and the result is 0.
  EXPECT_DOUBLE_EQ(calc_minimum_distance_to_stop(0.0, 1.0, -1.0), 0.0);
}

TEST(CalcMinimumDistanceToStop, ReverseMotionUsesMaxAcc)
{
  // initial_vel < 0 -> uses max_acc branch: -v^2 / (2 * max_acc)
  // -((-2)^2) / (2 * 1.0) = -2.0
  EXPECT_DOUBLE_EQ(calc_minimum_distance_to_stop(-2.0, 1.0, -1.0), -2.0);
}

TEST(CalcEstimationTime, ZeroDecelerationReturnsHorizon)
{
  // deceleration <= epsilon -> equivalent time is infinity, clamped to the horizon.
  PredictedObject predicted_object;
  predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x = 3.0;
  predicted_object.kinematics.initial_twist_with_covariance.twist.linear.y = 0.0;

  ObstacleFilteringParam param;
  param.outside_obstacle.deceleration = 0.0;
  param.outside_obstacle.estimation_time_horizon = 5.0;

  EXPECT_DOUBLE_EQ(calc_estimation_time(predicted_object, param), 5.0);
}

TEST(CalcEstimationTime, NormalDecelerationConvertsToTime)
{
  // equivalent time = hypot(vx, vy) * 0.5 / deceleration, clamped to [0, horizon].
  PredictedObject predicted_object;
  predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x = 3.0;
  predicted_object.kinematics.initial_twist_with_covariance.twist.linear.y = 4.0;  // hypot = 5.0

  ObstacleFilteringParam param;
  param.outside_obstacle.deceleration = 2.5;
  param.outside_obstacle.estimation_time_horizon = 10.0;

  // 5.0 * 0.5 / 2.5 = 1.0
  EXPECT_DOUBLE_EQ(calc_estimation_time(predicted_object, param), 1.0);
}

TEST(CalcEstimationTime, ClampsToHorizon)
{
  PredictedObject predicted_object;
  predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x = 100.0;
  predicted_object.kinematics.initial_twist_with_covariance.twist.linear.y = 0.0;

  ObstacleFilteringParam param;
  param.outside_obstacle.deceleration = 1.0;
  param.outside_obstacle.estimation_time_horizon = 2.0;

  // 100 * 0.5 / 1.0 = 50.0, clamped to 2.0.
  EXPECT_DOUBLE_EQ(calc_estimation_time(predicted_object, param), 2.0);
}

TEST(CalcXOffsetToBumper, SelectsOffsetByDrivingDirection)
{
  VehicleInfo vehicle_info;
  vehicle_info.max_longitudinal_offset_m = 3.0;
  vehicle_info.min_longitudinal_offset_m = -1.0;

  EXPECT_DOUBLE_EQ(calc_x_offset_to_bumper(true, vehicle_info), 3.0);
  EXPECT_DOUBLE_EQ(calc_x_offset_to_bumper(false, vehicle_info), -1.0);
}

TEST(CalcTimeToReachCollisionPoint, UsesArcLengthAndClampedVelocity)
{
  // Straight trajectory along +x from origin.
  std::vector<TrajectoryPoint> traj_points;
  for (size_t i = 0; i < 21; ++i) {
    TrajectoryPoint p;
    p.pose.position.x = static_cast<double>(i);
    p.pose.position.y = 0.0;
    traj_points.push_back(p);
  }

  Odometry odometry;
  odometry.pose.pose.position.x = 0.0;
  odometry.pose.pose.position.y = 0.0;
  odometry.twist.twist.linear.x = 4.0;

  geometry_msgs::msg::Point collision_point;
  collision_point.x = 20.0;
  collision_point.y = 0.0;

  // dist = |arc_length(20) - x_offset(2)| - margin(0) = 18.0; vel = max(0.1, |4.0|) = 4.0
  // 18.0 / 4.0 = 4.5
  EXPECT_DOUBLE_EQ(
    calc_time_to_reach_collision_point(odometry, collision_point, traj_points, 2.0, 0.0, 0.1), 4.5);
}

TEST(CalcTimeToReachCollisionPoint, ClampsVelocityToMinimum)
{
  std::vector<TrajectoryPoint> traj_points;
  for (size_t i = 0; i < 21; ++i) {
    TrajectoryPoint p;
    p.pose.position.x = static_cast<double>(i);
    p.pose.position.y = 0.0;
    traj_points.push_back(p);
  }

  Odometry odometry;
  odometry.pose.pose.position.x = 0.0;
  odometry.pose.pose.position.y = 0.0;
  odometry.twist.twist.linear.x = 0.0;  // below the minimum

  geometry_msgs::msg::Point collision_point;
  collision_point.x = 20.0;
  collision_point.y = 0.0;

  // dist = |20 - 2| - 0 = 18.0; vel = max(2.0, 0.0) = 2.0 -> 9.0
  EXPECT_DOUBLE_EQ(
    calc_time_to_reach_collision_point(odometry, collision_point, traj_points, 2.0, 0.0, 2.0), 9.0);
}

TEST(CalcBrakingDistAlongTrajectory, SelectsDecelerationByClass)
{
  RSSParam rss;
  rss.pointcloud_deceleration = -1.0;
  rss.no_wheel_objects_deceleration = -2.0;
  rss.two_wheel_objects_deceleration = -4.0;
  rss.vehicle_objects_deceleration = -5.0;
  rss.velocity_offset = 0.0;

  // braking_dist = error_vel^2 * 0.5 / -braking_acc, error_vel = max(lon_vel + offset, 0).
  // POINTCLOUD: -braking_acc = 1.0 -> 10^2 * 0.5 / 1.0 = 50.0
  EXPECT_DOUBLE_EQ(
    calc_braking_dist_along_trajectory(StopObstacleClassification::Type::POINTCLOUD, 10.0, rss),
    50.0);
  // PEDESTRIAN -> no_wheel: -braking_acc = 2.0 -> 10^2 * 0.5 / 2.0 = 25.0
  EXPECT_DOUBLE_EQ(
    calc_braking_dist_along_trajectory(StopObstacleClassification::Type::PEDESTRIAN, 10.0, rss),
    25.0);
  // UNKNOWN -> no_wheel: same as pedestrian
  EXPECT_DOUBLE_EQ(
    calc_braking_dist_along_trajectory(StopObstacleClassification::Type::UNKNOWN, 10.0, rss), 25.0);
  // BICYCLE -> two_wheel: -braking_acc = 4.0 -> 10^2 * 0.5 / 4.0 = 12.5
  EXPECT_DOUBLE_EQ(
    calc_braking_dist_along_trajectory(StopObstacleClassification::Type::BICYCLE, 10.0, rss), 12.5);
  // MOTORCYCLE -> two_wheel: same
  EXPECT_DOUBLE_EQ(
    calc_braking_dist_along_trajectory(StopObstacleClassification::Type::MOTORCYCLE, 10.0, rss),
    12.5);
  // CAR -> vehicle: -braking_acc = 5.0 -> 10^2 * 0.5 / 5.0 = 10.0
  EXPECT_DOUBLE_EQ(
    calc_braking_dist_along_trajectory(StopObstacleClassification::Type::CAR, 10.0, rss), 10.0);
}

TEST(CalcBrakingDistAlongTrajectory, AppliesVelocityOffsetAndClampsToZero)
{
  RSSParam rss;
  rss.vehicle_objects_deceleration = -5.0;
  rss.velocity_offset = 1.0;

  // error_vel = max(9.0 + 1.0, 0) = 10.0 -> 10^2 * 0.5 / 5.0 = 10.0
  EXPECT_DOUBLE_EQ(
    calc_braking_dist_along_trajectory(StopObstacleClassification::Type::CAR, 9.0, rss), 10.0);

  // negative effective velocity is clamped to 0 -> braking_dist = 0
  rss.velocity_offset = 0.0;
  EXPECT_DOUBLE_EQ(
    calc_braking_dist_along_trajectory(StopObstacleClassification::Type::CAR, -3.0, rss), 0.0);
}

TEST(CreatePolygonParam, TrimmingDisabledLeavesNullopt)
{
  ObstacleFilteringParam::TrimTrajectoryParam trim;
  trim.enable_trimming = false;
  trim.min_trajectory_length = 10.0;
  trim.braking_distance_scale_factor = 2.0;

  ObstacleFilteringParam::LateralMarginParam lateral;
  lateral.nominal_margin = 1.0;
  lateral.is_moving_threshold_velocity = 1.0;
  lateral.additional_is_moving_margin = 0.5;
  lateral.additional_is_stop_margin = 0.2;
  lateral.additional_wheel_off_track_scale = 0.3;

  const auto p = create_polygon_param(trim, 4.0, lateral, std::optional<double>(0.0));
  EXPECT_FALSE(p.trimming_length.has_value());
  // object_velocity (0.0) <= threshold (1.0) -> stop margin branch.
  EXPECT_DOUBLE_EQ(p.lateral_margin, 1.0 + 0.2);
  EXPECT_DOUBLE_EQ(p.off_track_scale, 0.3);
}

TEST(CreatePolygonParam, NoBrakingDistanceLeavesNullopt)
{
  ObstacleFilteringParam::TrimTrajectoryParam trim;
  trim.enable_trimming = true;
  trim.min_trajectory_length = 10.0;
  trim.braking_distance_scale_factor = 2.0;

  ObstacleFilteringParam::LateralMarginParam lateral;
  lateral.nominal_margin = 1.0;
  lateral.is_moving_threshold_velocity = 1.0;
  lateral.additional_is_moving_margin = 0.5;
  lateral.additional_is_stop_margin = 0.2;

  const auto p = create_polygon_param(trim, std::nullopt, lateral, std::optional<double>(2.0));
  EXPECT_FALSE(p.trimming_length.has_value());
  // object_velocity (2.0) > threshold (1.0) -> moving margin branch.
  EXPECT_DOUBLE_EQ(p.lateral_margin, 1.0 + 0.5);
}

TEST(CreatePolygonParam, TrimmingEnabledComputesLength)
{
  ObstacleFilteringParam::TrimTrajectoryParam trim;
  trim.enable_trimming = true;
  trim.min_trajectory_length = 10.0;
  trim.braking_distance_scale_factor = 2.0;

  ObstacleFilteringParam::LateralMarginParam lateral;
  lateral.nominal_margin = 1.0;
  lateral.is_moving_threshold_velocity = 1.0;
  lateral.additional_is_moving_margin = 0.5;
  lateral.additional_is_stop_margin = 0.2;

  const auto p = create_polygon_param(trim, 4.0, lateral, std::optional<double>(2.0));
  ASSERT_TRUE(p.trimming_length.has_value());
  // 10.0 + 2.0 * 4.0 = 18.0
  EXPECT_DOUBLE_EQ(p.trimming_length.value(), 18.0);
}

TEST(CreatePolygonParam, NulloptObjectVelocityUsesStopMargin)
{
  ObstacleFilteringParam::TrimTrajectoryParam trim;
  trim.enable_trimming = false;

  ObstacleFilteringParam::LateralMarginParam lateral;
  lateral.nominal_margin = 1.0;
  lateral.is_moving_threshold_velocity = 1.0;
  lateral.additional_is_moving_margin = 0.5;
  lateral.additional_is_stop_margin = 0.2;

  // A nullopt object_velocity compares as not greater than the threshold, so the stop margin is
  // used. This pins the existing std::optional comparison behavior.
  const auto p = create_polygon_param(trim, std::nullopt, lateral, std::nullopt);
  EXPECT_DOUBLE_EQ(p.lateral_margin, 1.0 + 0.2);
}

}  // namespace autoware::motion_velocity_planner::obstacle_stop_internal
