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

#ifndef DATA_PROCESSING_HPP_
#define DATA_PROCESSING_HPP_

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <lanelet2_core/Forward.h>

#include <deque>
#include <map>

// Package-internal, node-agnostic data-ingestion helpers shared by the legacy and experimental
// BehaviorVelocityPlannerNode variants. These are pure transforms over PlannerData fields so they
// can be unit-tested without spinning a node.
namespace autoware::behavior_velocity_planner::data_processing
{
using TrafficSignalMap = std::map<lanelet::Id, TrafficSignalStamped>;

/**
 * @brief Result of aggregating an incoming traffic-light-group observation.
 *
 * @c raw mirrors the latest message verbatim. @c last_observed applies the
 * "keep last observation on UNKNOWN" contract: when an incoming group is observed as UNKNOWN and a
 * previous observation exists, the previous body is kept and only its timestamp is refreshed.
 */
struct TrafficSignalAggregation
{
  TrafficSignalMap raw;
  TrafficSignalMap last_observed;
};

/**
 * @brief Aggregate an incoming traffic-light-group array into raw and last-observed maps.
 *
 * @param previous_last_observed last-observed map from the previous cycle.
 * @param msg incoming traffic-light-group array.
 * @return the new raw and last-observed maps.
 */
TrafficSignalAggregation apply_traffic_signals(
  const TrafficSignalMap & previous_last_observed,
  const autoware_perception_msgs::msg::TrafficLightGroupArray & msg);

/**
 * @brief Drop velocity-buffer entries older than @c PlannerData::velocity_buffer_time_sec.
 *
 * The oldest entries live at the back of the buffer. Entries whose stamp is in the future relative
 * to @c now (negative time difference) are treated as age 0 and are therefore retained.
 *
 * @param[in,out] velocity_buffer buffer to prune in place (newest at front, oldest at back).
 * @param now reference time used to compute each entry's age.
 */
void prune_velocity_buffer(
  std::deque<geometry_msgs::msg::TwistStamped> & velocity_buffer, const rclcpp::Time & now);
}  // namespace autoware::behavior_velocity_planner::data_processing

#endif  // DATA_PROCESSING_HPP_
