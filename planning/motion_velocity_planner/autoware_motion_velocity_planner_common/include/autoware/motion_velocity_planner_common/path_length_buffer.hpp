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

#pragma once

#include "autoware_utils/geometry/geometry.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <vector>

namespace autoware::motion_velocity_planner::utils
{

struct BufferedStopDistanceItem
{
  double zero_vel_dist;
  StopObstacle stop_obstacle;
  double desired_stop_margin;
  geometry_msgs::msg::Pose stop_pose;

  rclcpp::Time start_time;
  bool is_active;

  BufferedStopDistanceItem(
    const double zero_vel_dist, const StopObstacle & obstacle, const double margin,
    const geometry_msgs::msg::Pose & pose, const rclcpp::Time & time, bool active = false)
  : zero_vel_dist(zero_vel_dist),
    stop_obstacle(obstacle),
    desired_stop_margin(margin),
    stop_pose(pose),
    start_time(time),
    is_active(active)
  {
  }
};

class PathLengthBuffer
{
public:
  PathLengthBuffer() = default;
  PathLengthBuffer(
    double update_distance_threshold, double min_off_duration, double min_on_duration)
  : min_off_duration_(min_off_duration),
    min_on_duration_(min_on_duration),
    update_distance_th_(update_distance_threshold)
  {
  }

  std::optional<BufferedStopDistanceItem> get_nearest_active_item() const
  {
    if (buffer_.empty()) {
      return {};
    }

    auto nearest_item = std::min_element(
      buffer_.begin(), buffer_.end(),
      [](const BufferedStopDistanceItem & a, const BufferedStopDistanceItem & b) {
        if (!a.is_active) return false;
        if (!b.is_active) return true;
        return a.zero_vel_dist < b.zero_vel_dist;
      });

    if (!nearest_item->is_active) {
      return std::nullopt;
    }

    return *nearest_item;
  }

  void update_buffer(
    const double zero_vel_dist, const StopObstacle & stop_obstacle,
    const double desired_stop_margin, const geometry_msgs::msg::Pose & stop_pose,
    std::function<std::optional<double>(const StopObstacle &, const double)> calc_zero_vel_dist,
    const rclcpp::Clock::SharedPtr & clock)
  {
    const rclcpp::Time clock_now = clock->now();

    // Remove items that should be removed
    buffer_.erase(
      std::remove_if(
        buffer_.begin(), buffer_.end(),
        [&](const BufferedStopDistanceItem & buffered_item) {
          const auto duration = (clock_now - buffered_item.start_time).seconds();
          const auto buffered_item_zero_vel_dist = calc_zero_vel_dist(buffered_item.stop_obstacle, buffered_item.desired_stop_margin);

          if (!buffered_item_zero_vel_dist) {
            return true;
          }

          const double rel_dist = std::abs(*buffered_item_zero_vel_dist - zero_vel_dist);

          const bool is_remove = (buffered_item.is_active && (duration > min_off_duration_)) ||
                 (!buffered_item.is_active &&
                  rel_dist > update_distance_th_);

          return is_remove;
        }),
      buffer_.end());

    static constexpr double eps = 1e-3;
    if (buffer_.empty()) {
      buffer_.emplace_back(
        zero_vel_dist, stop_obstacle, desired_stop_margin, stop_pose, clock_now,
        min_on_duration_ < eps);
      return;
    }

    // Update the state of remaining items
    for (auto & buffered_item : buffer_) {
      const auto candidate_zero_vel_dist = calc_zero_vel_dist(buffered_item.stop_obstacle, buffered_item.desired_stop_margin);
      if (!candidate_zero_vel_dist) {
        buffered_item.is_active = false;
        continue;
      }
      if (
        !buffered_item.is_active &&
        (clock_now - buffered_item.start_time).seconds() > min_on_duration_) {
        buffered_item.is_active = true;
        buffered_item.start_time = clock_now;
      }
      buffered_item.zero_vel_dist = *candidate_zero_vel_dist;
    }

    auto nearest_prev_pose_it = buffer_.end();
    auto min_relative_dist = std::numeric_limits<double>::max();
    for (auto it = buffer_.begin(); it < buffer_.end(); ++it) {
      const auto rel_dist = it->zero_vel_dist - zero_vel_dist;
      
      if (std::abs(rel_dist) < update_distance_th_ && rel_dist < min_relative_dist) {
          nearest_prev_pose_it = it;
          min_relative_dist = rel_dist;
      }
    }

    if (nearest_prev_pose_it == buffer_.end()) {
      buffer_.emplace_back(
        zero_vel_dist, stop_obstacle, desired_stop_margin, stop_pose, clock_now,
        min_on_duration_ < eps);
      return;
    }

    if (min_relative_dist > 0) {
      nearest_prev_pose_it->zero_vel_dist = zero_vel_dist;
      nearest_prev_pose_it->stop_obstacle = stop_obstacle;
      nearest_prev_pose_it->desired_stop_margin = desired_stop_margin;
      nearest_prev_pose_it->stop_pose = stop_pose;
    }

    if (nearest_prev_pose_it->is_active) {
      nearest_prev_pose_it->start_time = clock_now;
    }
  }

private:
  std::vector<BufferedStopDistanceItem> buffer_;
  double min_off_duration_;
  double min_on_duration_;
  double update_distance_th_;
};
}  // namespace autoware::motion_velocity_planner::utils
