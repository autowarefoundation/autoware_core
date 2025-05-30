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

#ifndef AUTOWARE__LANELET2_UTILS__ROUTE_MANAGER_HPP_
#define AUTOWARE__LANELET2_UTILS__ROUTE_MANAGER_HPP_

#include <autoware/lanelet2_utils/lane_sequence.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/Forward.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <optional>

namespace autoware::experimental::lanelet2_utils
{

class RouteManager
{
public:
  static std::optional<RouteManager> create(
    const autoware_map_msgs::msg::LaneletMapBin & map_msg,
    const autoware_planning_msgs::msg::LaneletRoute & route_msg);

  std::optional<RouteManager> update_current_pose(const geometry_msgs::msg::Pose & current_pose) &&;

private:
  RouteManager(
    lanelet::LaneletMapConstPtr lanelet_map_ptr,
    lanelet::routing::RoutingGraphConstPtr routing_graph_ptr,
    lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr);

  lanelet::LaneletMapConstPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphConstPtr routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;

  lanelet::ConstLanelets all_route_lanelets_;  //<! all route lanelets, the order is not defined
  lanelet::ConstLanelets preferred_lanelets_;

  geometry_msgs::msg::Pose current_pose_;
  lanelet::ConstLanelet current_lanelet_;
};

}  // namespace autoware::experimental::lanelet2_utils

#endif  // AUTOWARE__LANELET2_UTILS__ROUTE_MANAGER_HPP_
