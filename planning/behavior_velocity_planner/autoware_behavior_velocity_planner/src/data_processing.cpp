// Copyright 2026 Autoware Foundation
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

#include "data_processing.hpp"

#include <autoware/behavior_velocity_planner_common/planner_data.hpp>

#include <algorithm>
#include <deque>

namespace autoware::behavior_velocity_planner::data_processing
{
TrafficSignalAggregation apply_traffic_signals(
  const TrafficSignalMap & previous_last_observed,
  const autoware_perception_msgs::msg::TrafficLightGroupArray & msg)
{
  TrafficSignalAggregation result;
  for (const auto & signal : msg.traffic_light_groups) {
    TrafficSignalStamped traffic_signal;
    traffic_signal.stamp = msg.stamp;
    traffic_signal.signal = signal;
    result.raw[signal.traffic_light_group_id] = traffic_signal;
    const bool is_unknown_observation =
      std::any_of(signal.elements.begin(), signal.elements.end(), [](const auto & element) {
        return element.color == autoware_perception_msgs::msg::TrafficLightElement::UNKNOWN;
      });
    // if the observation is UNKNOWN and past observation is available, only update the timestamp
    // and keep the body of the info
    const auto old_data = previous_last_observed.find(signal.traffic_light_group_id);
    if (is_unknown_observation && old_data != previous_last_observed.end()) {
      // copy last observation
      result.last_observed[signal.traffic_light_group_id] = old_data->second;
      // update timestamp
      result.last_observed[signal.traffic_light_group_id].stamp = msg.stamp;
    } else {
      // if (1)the observation is not UNKNOWN or (2)the very first observation is UNKNOWN
      result.last_observed[signal.traffic_light_group_id] = traffic_signal;
    }
  }
  return result;
}

void prune_velocity_buffer(
  std::deque<geometry_msgs::msg::TwistStamped> & velocity_buffer, const rclcpp::Time & now)
{
  while (!velocity_buffer.empty()) {
    // Check oldest data time. Build the comparison time with the same clock type as `now` so the
    // helper stays clock/node-agnostic instead of assuming the default RCL_ROS_TIME.
    const rclcpp::Time s(velocity_buffer.back().header.stamp, now.get_clock_type());
    const auto time_diff =
      now >= s ? now - s : rclcpp::Duration(0, 0);  // Note: negative time throws an exception.

    // Finish when oldest data is newer than threshold
    if (time_diff.seconds() <= PlannerData::velocity_buffer_time_sec) {
      break;
    }

    // Remove old data
    velocity_buffer.pop_back();
  }
}
}  // namespace autoware::behavior_velocity_planner::data_processing
