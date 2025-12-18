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

#ifndef AUTOWARE__TRAJECTORY__UTILS__VELOCITY_HPP_
#define AUTOWARE__TRAJECTORY__UTILS__VELOCITY_HPP_

#include "autoware/trajectory/forward.hpp"
#include "autoware/trajectory/threshold.hpp"

#include <range/v3/all.hpp>

#include "autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp"
#include "autoware_planning_msgs/msg/path_point.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"

#include <cmath>
#include <optional>

namespace autoware::experimental::trajectory
{

/**
 * @brief find zero velocity position in continuous trajectory within a distance interval
 * Searches for the arc-length position where velocity becomes zero in a continuous Trajectory.
 * Uses linear interpolation when velocity changes sign between two points.
 * @param trajectory continuous trajectory to search
 * @param start_distance start arc-length distance of the search interval [m]
 * @param end_distance end arc-length distance of the search interval [m]
 * @return arc-length distance where velocity is zero, or std::nullopt if not found
 */
template <class PointT>
[[nodiscard]] std::optional<double> search_zero_velocity_position(
  const Trajectory<PointT> & trajectory, const double start_distance, const double end_distance)
{
  const auto [bases, velocities] = trajectory.longitudinal_velocity_mps().get_data();

  if (velocities.empty() || start_distance < 0.0 || end_distance < start_distance) {
    return std::nullopt;
  }

  // Check pairs of consecutive points
  for (const auto [curr_distance, next_distance, curr_vel, next_vel] : ranges::views::zip(
         bases, bases | ranges::views::drop(1), velocities, velocities | ranges::views::drop(1))) {
    if (curr_distance < start_distance) {
      continue;
    }
    if (curr_distance > end_distance) {
      break;
    }

    if (std::abs(curr_vel) < k_zero_velocity_threshold) {
      return curr_distance;
    }

    if (next_distance > end_distance) {
      continue;
    }

    const bool sign_change =
      (curr_vel > 0.0 && next_vel < 0.0) || (curr_vel < 0.0 && next_vel > 0.0);

    if (!sign_change) {
      continue;
    }

    const double zero_pos =
      curr_distance - curr_vel * (next_distance - curr_distance) / (next_vel - curr_vel);
    if (zero_pos >= start_distance && zero_pos <= end_distance) {
      return zero_pos;
    }
  }

  const double last_distance = bases.back();
  const double last_vel = velocities.back();
  if (
    last_distance >= start_distance && last_distance <= end_distance &&
    std::abs(last_vel) < k_zero_velocity_threshold) {
    return last_distance;
  }

  return std::nullopt;
}

/**
 * @brief find zero velocity position in continuous trajectory from start distance to trajectory end
 * Searches for the arc-length position where velocity becomes zero starting from start_distance.
 * @param trajectory continuous trajectory to search
 * @param start_distance start arc-length distance of the search [m]
 * @return arc-length distance where velocity is zero, or std::nullopt if not found
 */
template <class PointT>
[[nodiscard]] std::optional<double> search_zero_velocity_position(
  const Trajectory<PointT> & trajectory, const double start_distance)
{
  return search_zero_velocity_position(trajectory, start_distance, trajectory.length());
}

/**
 * @brief find zero velocity position in continuous trajectory over full range
 * Searches for the arc-length position where velocity becomes zero over the entire trajectory.
 * @param trajectory continuous trajectory to search
 * @return arc-length distance where velocity is zero, or std::nullopt if not found
 */
template <class PointT>
[[nodiscard]] std::optional<double> search_zero_velocity_position(
  const Trajectory<PointT> & trajectory)
{
  return search_zero_velocity_position(trajectory, 0.0, trajectory.length());
}

/**
 * @brief Check if velocity has zero in continuous trajectory over full range
 * @param trajectory continuous trajectory to check
 * @return true if velocity is zero anywhere in the trajectory, false otherwise
 */
template <class PointT>
[[nodiscard]] bool has_zero_velocity(const Trajectory<PointT> & trajectory)
{
  return search_zero_velocity_position(trajectory, 0.0, trajectory.length()).has_value();
}

extern template std::optional<double>
search_zero_velocity_position<autoware_planning_msgs::msg::TrajectoryPoint>(
  const Trajectory<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const double start_distance, const double end_distance);

extern template std::optional<double>
search_zero_velocity_position<autoware_planning_msgs::msg::TrajectoryPoint>(
  const Trajectory<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const double start_distance);

extern template std::optional<double>
search_zero_velocity_position<autoware_planning_msgs::msg::TrajectoryPoint>(
  const Trajectory<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory);

extern template bool has_zero_velocity<autoware_planning_msgs::msg::TrajectoryPoint>(
  const Trajectory<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory);

}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__UTILS__VELOCITY_HPP_
