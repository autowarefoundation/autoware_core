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

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware/lanelet2_utils/route_manager.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <optional>
#include <tuple>
#include <utility>

namespace autoware::experimental::lanelet2_utils
{

std::optional<RouteManager> RouteManager::create(
  const autoware_map_msgs::msg::LaneletMapBin & map_msg,
  const autoware_planning_msgs::msg::LaneletRoute & route_msg,
  const geometry_msgs::msg::Pose & initial_pose)
{
  const auto lanelet_map = from_autoware_map_msgs(map_msg);
  if (!lanelet_map) {
    return std::nullopt;
  }
  const auto [routing_graph, traffic_rules] =
    instantiate_routing_graph_and_traffic_rules(lanelet_map);

  lanelet::ConstLanelets all_route_lanelets;
  lanelet::ConstLanelets preferred_lanelets;
  for (const auto & route_segment : route_msg.segments) {
    for (const auto & primitive : route_segment.primitives) {
      const auto id = primitive.id;
      const auto & llt = lanelet_map->laneletLayer.get(id);
      if (id == route_segment.preferred_primitive.id) {
        preferred_lanelets.push_back(llt);
      }
      all_route_lanelets.push_back(llt);
    }
  }

  const auto & first_route_segment = route_msg.segments.front();
  const auto start_lanelet =
    lanelet_map->laneletLayer.get(first_route_segment.preferred_primitive.id);
  const auto & last_route_segment = route_msg.segments.back();
  const auto goal_lanelet =
    lanelet_map->laneletLayer.get(last_route_segment.preferred_primitive.id);
  const auto closest_lanelet_opt = get_closest_lanelet(all_route_lanelets, initial_pose);
  if (!closest_lanelet_opt) {
    return std::nullopt;
  }

  return RouteManager(
    lanelet_map, routing_graph, traffic_rules, std::move(all_route_lanelets),
    std::move(preferred_lanelets), start_lanelet, goal_lanelet, initial_pose,
    closest_lanelet_opt.value());
}

RouteManager::RouteManager(
  lanelet::LaneletMapConstPtr lanelet_map_ptr,
  lanelet::routing::RoutingGraphConstPtr routing_graph_ptr,
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr,
  lanelet::ConstLanelets && all_route_lanelets, lanelet::ConstLanelets && preferred_lanelets,
  const lanelet::ConstLanelet & start_lanelet, const lanelet::ConstLanelet & goal_lanelet,
  const geometry_msgs::msg::Pose & current_pose, lanelet::ConstLanelet current_lanelet)
: lanelet_map_ptr_(lanelet_map_ptr),
  routing_graph_ptr_(routing_graph_ptr),
  traffic_rules_ptr_(traffic_rules_ptr),
  all_route_lanelets_(std::move(all_route_lanelets)),
  preferred_lanelets_(std::move(preferred_lanelets)),
  start_lanelet_(start_lanelet),
  goal_lanelet_(goal_lanelet),
  current_pose_(current_pose),
  current_lanelet_(current_lanelet)
{
}
}  // namespace autoware::experimental::lanelet2_utils
