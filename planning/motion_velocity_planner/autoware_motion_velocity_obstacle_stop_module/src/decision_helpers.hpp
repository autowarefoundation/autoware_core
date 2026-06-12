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

#ifndef DECISION_HELPERS_HPP_
#define DECISION_HELPERS_HPP_

#include "parameters.hpp"
#include "type_alias.hpp"
#include "types.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace autoware::motion_velocity_planner::obstacle_stop_internal
{

/// @brief Calculate the minimum distance required to stop from the given initial velocity.
///
/// Uses v^2 / (2 a) with the deceleration limit selected by the sign of the initial velocity:
/// forward motion (initial_vel >= 0) uses min_acc, reverse motion uses max_acc.
inline double calc_minimum_distance_to_stop(
  const double initial_vel, const double max_acc, const double min_acc)
{
  if (initial_vel < 0.0) {
    return -std::pow(initial_vel, 2) / 2.0 / max_acc;
  }

  return -std::pow(initial_vel, 2) / 2.0 / min_acc;
}

/// @brief Convert the object's constant-deceleration assumption into an equivalent
/// constant-velocity estimation time, clamped to [0, estimation_time_horizon].
inline double calc_estimation_time(
  const PredictedObject & predicted_object, const ObstacleFilteringParam & obstacle_filtering_param)
{
  // convert constant deceleration to constant velocity
  // In this feature, we are assuming the pedestrians will decelerate by specified value,
  // hence the travel distance is derived as v^2/2a.
  // However, to maintain the compatibility with the other objects,
  // we have to capsulize this distance information as time with constant velocity assumption.
  // Therefore here we return a value (v^2/2a) / v = v/2a as the equivalent estimation time.
  const auto equivalent_estimation_time = [&]() {
    if (
      obstacle_filtering_param.outside_obstacle.deceleration <=
      std::numeric_limits<double>::epsilon()) {
      return std::numeric_limits<double>::infinity();
    }
    const auto & twist = predicted_object.kinematics.initial_twist_with_covariance.twist;
    return std::hypot(twist.linear.x, twist.linear.y) * 0.5 /
           obstacle_filtering_param.outside_obstacle.deceleration;
  };
  return std::clamp(
    equivalent_estimation_time(), 0.0,
    obstacle_filtering_param.outside_obstacle.estimation_time_horizon);
}

/// @brief Select the longitudinal bumper offset that matches the driving direction.
inline double calc_x_offset_to_bumper(
  const bool is_driving_forward, const VehicleInfo & vehicle_info)
{
  if (is_driving_forward) {
    return vehicle_info.max_longitudinal_offset_m;
  }
  return vehicle_info.min_longitudinal_offset_m;
}

/// @brief Estimate the time for ego to reach the given collision point along the trajectory.
inline double calc_time_to_reach_collision_point(
  const Odometry & odometry, const geometry_msgs::msg::Point & collision_point,
  const std::vector<TrajectoryPoint> & traj_points, const double x_offset_to_bumper,
  const double margin_distance, const double min_velocity_to_reach_collision_point)
{
  const double dist_from_ego_to_obstacle =
    std::abs(
      autoware::motion_utils::calcSignedArcLength(
        traj_points, odometry.pose.pose.position, collision_point) -
      x_offset_to_bumper) -
    margin_distance;
  return dist_from_ego_to_obstacle /
         std::max(min_velocity_to_reach_collision_point, std::abs(odometry.twist.twist.linear.x));
}

/// @brief Calculate the RSS braking distance for the given object class and longitudinal velocity.
// TODO(takagi): refactor this function as same as obstacle_filtering_param
inline double calc_braking_dist_along_trajectory(
  const StopObstacleClassification::Type label, const double lon_vel, const RSSParam & rss_params)
{
  const double braking_acc = [&]() {
    if (label == StopObstacleClassification::Type::POINTCLOUD) {
      return rss_params.pointcloud_deceleration;
    }
    if (
      label == StopObstacleClassification::Type::UNKNOWN ||
      label == StopObstacleClassification::Type::PEDESTRIAN) {
      return rss_params.no_wheel_objects_deceleration;
    }
    if (
      label == StopObstacleClassification::Type::BICYCLE ||
      label == StopObstacleClassification::Type::MOTORCYCLE) {
      return rss_params.two_wheel_objects_deceleration;
    }
    return rss_params.vehicle_objects_deceleration;
  }();
  const double error_considered_vel = std::max(lon_vel + rss_params.velocity_offset, 0.0);
  return error_considered_vel * error_considered_vel * 0.5 / -braking_acc;
}

/// @brief Build the polygon-generation parameters from the trimming, lateral-margin and velocity
/// inputs.
inline PolygonParam create_polygon_param(
  const ObstacleFilteringParam::TrimTrajectoryParam & trim_trajectory_param,
  const std::optional<double> ego_braking_distance,
  const ObstacleFilteringParam::LateralMarginParam & lateral_margin_param,
  const std::optional<double> object_velocity)
{
  PolygonParam p;
  if (!trim_trajectory_param.enable_trimming || !ego_braking_distance.has_value()) {
    p.trimming_length = std::nullopt;
  } else {
    p.trimming_length =
      trim_trajectory_param.min_trajectory_length +
      trim_trajectory_param.braking_distance_scale_factor * ego_braking_distance.value();
  }
  p.lateral_margin = lateral_margin_param.nominal_margin +
                     (object_velocity > lateral_margin_param.is_moving_threshold_velocity
                        ? lateral_margin_param.additional_is_moving_margin
                        : lateral_margin_param.additional_is_stop_margin);
  p.off_track_scale = lateral_margin_param.additional_wheel_off_track_scale;
  return p;
}

}  // namespace autoware::motion_velocity_planner::obstacle_stop_internal

#endif  // DECISION_HELPERS_HPP_
