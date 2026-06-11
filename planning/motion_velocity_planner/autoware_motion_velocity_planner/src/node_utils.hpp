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

#ifndef NODE_UTILS_HPP_
#define NODE_UTILS_HPP_

#include <autoware/motion_velocity_planner_common/planner_data.hpp>

#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <lanelet2_core/Forward.h>

#include <map>
#include <vector>

namespace autoware::motion_velocity_planner::utils
{
using TrajectoryPoints = std::vector<autoware_planning_msgs::msg::TrajectoryPoint>;
using TrafficLightIdMap = std::map<lanelet::Id, TrafficSignalStamped>;

/// @brief result of processing a TrafficLightGroupArray observation
struct ProcessedTrafficSignals
{
  /// @brief raw observation, one entry per traffic light group in the message
  TrafficLightIdMap raw;
  /// @brief last observed signals; for an UNKNOWN observation with a prior entry the previous body
  ///        is kept and only the timestamp is refreshed
  TrafficLightIdMap last_observed;
};

/// @brief turn a raw TrafficLightGroupArray observation into the raw and last-observed maps
/// @param msg incoming traffic light group array
/// @param last_observed_old the previous last-observed map (used to keep the body of UNKNOWN
///        observations that have a prior entry)
/// @return the freshly built raw and last-observed maps
ProcessedTrafficSignals process_traffic_signals(
  const autoware_perception_msgs::msg::TrafficLightGroupArray & msg,
  const TrafficLightIdMap & last_observed_old);

/// @brief decimate a trajectory by dropping points that are closer than a minimum interval
/// @details the first point is always kept; a later point is kept only when its squared 2D distance
///          to the previously kept point exceeds @p min_interval_squared
/// @param trajectory_points input trajectory points
/// @param min_interval_squared squared minimum interval [m^2] between two consecutive kept points
/// @return the decimated trajectory points (empty if the input is empty)
TrajectoryPoints resample_trajectory_by_min_interval(
  const TrajectoryPoints & trajectory_points, double min_interval_squared);

}  // namespace autoware::motion_velocity_planner::utils

#endif  // NODE_UTILS_HPP_
