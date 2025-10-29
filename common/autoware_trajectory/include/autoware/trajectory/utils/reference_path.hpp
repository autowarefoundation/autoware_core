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

#ifndef AUTOWARE__TRAJECTORY__UTILS__REFERENCE_PATH_HPP_
#define AUTOWARE__TRAJECTORY__UTILS__REFERENCE_PATH_HPP_

#include "autoware/trajectory/path_point_with_lane_id.hpp"

#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <optional>

namespace autoware::experimental::trajectory
{
/**
 * @brief build reference path that spans specified interval in front of and behind ego in s
 * coordinate
 * @param lanelet_sequence_with_interval consecutive lanelet sequence with interval in s coordinate
 * @param lanelet_map lanelet map
 * @param traffic_rules traffic rules
 * @param waypoint_connection_gradient_from_centerline gradient for connecting centerline and user
 * defined waypoints
 * @return reference path with lane ids (std::nullopt if failed)
 * @note length of Trajectory may not match length of interval
 */
std::optional<Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>
build_reference_path(
  const lanelet::ConstLanelets & lanelet_sequence, const lanelet::LaneletMapConstPtr lanelet_map,
  const lanelet::traffic_rules::TrafficRulesPtr traffic_rules,
  const lanelet::routing::RoutingGraphConstPtr routing_graph,
  const geometry_msgs::msg::Point & ego_position, const lanelet::Id ego_lane_id,
  const double backward_length, const double forward_length,
  const double waypoint_connection_gradient_from_centerline);
}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__UTILS__REFERENCE_PATH_HPP_
