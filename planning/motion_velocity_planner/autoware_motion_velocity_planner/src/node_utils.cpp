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

#include "node_utils.hpp"

#include <autoware_utils_geometry/geometry.hpp>

#include <algorithm>

namespace autoware::motion_velocity_planner::utils
{
ProcessedTrafficSignals process_traffic_signals(
  const autoware_perception_msgs::msg::TrafficLightGroupArray & msg,
  const TrafficLightIdMap & last_observed_old)
{
  ProcessedTrafficSignals result;
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
    const auto old_data = last_observed_old.find(signal.traffic_light_group_id);
    if (is_unknown_observation && old_data != last_observed_old.end()) {
      // copy last observation
      result.last_observed[signal.traffic_light_group_id] = old_data->second;
      // update timestamp
      result.last_observed[signal.traffic_light_group_id].stamp = msg.stamp;
    } else {
      result.last_observed[signal.traffic_light_group_id] = traffic_signal;
    }
  }
  return result;
}

TrajectoryPoints resample_trajectory_by_min_interval(
  const TrajectoryPoints & trajectory_points, const double min_interval_squared)
{
  TrajectoryPoints resampled;
  // skip points that are too close together to make computation easier
  if (!trajectory_points.empty()) {
    resampled.push_back(trajectory_points.front());
    for (auto i = 1UL; i < trajectory_points.size(); ++i) {
      const auto & p = trajectory_points[i];
      const auto dist_to_prev_point =
        autoware_utils_geometry::calc_squared_distance2d(resampled.back(), p);
      if (dist_to_prev_point > min_interval_squared) {
        resampled.push_back(p);
      }
    }
  }
  return resampled;
}

}  // namespace autoware::motion_velocity_planner::utils
