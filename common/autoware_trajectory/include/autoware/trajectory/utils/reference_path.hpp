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

#include "autoware/trajectory/forward.hpp"

#include <tl_expected/expected.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_routing/Forward.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <string>

namespace autoware::experimental::trajectory
{
/**
 * @brief create Trajectory which is backward_length backward and forward_length forward from ego's
 * s coordinate in terms of s coordinate
 * @param connected_lane_sequence consecutive lanelet sequence. it is ok that it intersects with
 * itself
 * @param current_lanelet the lanelet where ego_pose is driving, which is given in order to
 * disambiguate if route_lanelets have self-intersection
 * @param ego_pose ego's current pose
 * @return the s coordinate of start/end is relative from ego by backward_length/forward_length. the
 * length of Trajectory does not match backward_length + forward_length
 */
tl::expected<Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>, std::string>
build_reference_path(
  const lanelet::ConstLanelets & connected_lane_sequence,
  const lanelet::ConstLanelet & current_lanelet, const geometry_msgs::msg::Pose & ego_pose,
  const lanelet::LaneletMapConstPtr lanelet_map,
  const lanelet::routing::RoutingGraphConstPtr routing_graph,
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules, const double forward_length,
  const double backward_length);

}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__UTILS__REFERENCE_PATH_HPP_
