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

struct BufferedStopDistance
{
  geometry_msgs::msg::Pose stop_pose;
  double arc_length;
  rclcpp::Time start_time;
  bool is_active;

  BufferedStopDistance(geometry_msgs::msg::Pose pose, double arc, rclcpp::Time time, bool active = false)
  : stop_pose(pose), arc_length(arc), start_time(time), is_active(active)
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

  std::optional<double> get_nearest_active_item() const
  {
    if (buffer_.empty()) {
      return {};
    }

    auto nearest_item = std::min_element(
      buffer_.begin(), buffer_.end(),
      [](const BufferedStopDistance & a, const BufferedStopDistance & b) {
        if (!a.is_active) return false;
        if (!b.is_active) return true;
        return a.arc_length < b.arc_length;
      });

    if (!nearest_item->is_active) {
      return std::nullopt;
    }

    return nearest_item->arc_length;
  }

  void update_buffer(const geometry_msgs::msg::Pose& stop_pose, const double stop_pose_arc_length, std::function<double(const geometry_msgs::msg::Pose&)> arc_length_lambda, const rclcpp::Clock::SharedPtr & clock)
  {
    const rclcpp::Time clock_now = clock->now();

    // Remove items that should be removed
    buffer_.erase(
      std::remove_if(
        buffer_.begin(), buffer_.end(),
        [&](const BufferedStopDistance & buffered_item) {
          const auto duration = (clock_now - buffered_item.start_time).seconds();

          const double buffered_item_arc_length = arc_length_lambda(buffered_item.stop_pose);

          return (buffered_item.is_active && duration > min_off_duration_) ||
                 (!buffered_item.is_active &&
                  std::abs(buffered_item_arc_length - stop_pose_arc_length) > update_distance_th_);
        }),
      buffer_.end());

    static constexpr double eps = 1e-3;
    if (buffer_.empty()) {
      buffer_.emplace_back(stop_pose, stop_pose_arc_length, clock_now, min_on_duration_ < eps);
      return;
    }

    // Update the state of remaining items
    for (auto & buffered_item : buffer_) {
      const double buffered_item_arc_length = arc_length_lambda(buffered_item.stop_pose);
      buffered_item.arc_length = buffered_item_arc_length;
      if (
        !buffered_item.is_active &&
        (clock_now - buffered_item.start_time).seconds() > min_on_duration_) {
        buffered_item.is_active = true;
        buffered_item.start_time = clock_now;
      }
    }

    auto nearest_prev_pose_it = std::min_element(
      buffer_.begin(), buffer_.end(),
      [&](const BufferedStopDistance & a, const BufferedStopDistance & b) {
        const auto dist_a = std::abs(a.arc_length - stop_pose_arc_length);
        const auto dist_b = std::abs(b.arc_length - stop_pose_arc_length);
        return dist_a < dist_b;
      });

    if (nearest_prev_pose_it == buffer_.end()) {
      buffer_.emplace_back(stop_pose, stop_pose_arc_length, clock_now, min_on_duration_ < eps);
      return;
    }

    const double min_relative_dist = nearest_prev_pose_it->arc_length - stop_pose_arc_length;

    if (min_relative_dist > 0) {
      nearest_prev_pose_it->arc_length = stop_pose_arc_length;
    }

    if (nearest_prev_pose_it->is_active) {
      nearest_prev_pose_it->start_time = clock_now;
    }
  }

private:
  std::vector<BufferedStopDistance> buffer_;
  double min_off_duration_;
  double min_on_duration_;
  double update_distance_th_;
};
}  // namespace autoware::motion_velocity_planner::utils
