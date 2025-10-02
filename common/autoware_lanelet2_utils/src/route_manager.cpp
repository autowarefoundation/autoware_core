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
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <memory>
#include <optional>
#include <tuple>
#include <unordered_map>
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
  if (route_msg.segments.empty()) {
    return std::nullopt;
  }

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

  std::unordered_map<lanelet::Id, double> all_route_length_cache;
  for (const auto & route_lanelet : all_route_lanelets) {
    all_route_length_cache[route_lanelet.id()] = lanelet::geometry::length3d(route_lanelet);
  }

  auto route_submap_ptr = std::make_shared<lanelet::LaneletSubmap>();
  for (const auto & route_lanelet : all_route_lanelets) {
    route_submap_ptr->add(
      lanelet::Lanelet(std::const_pointer_cast<lanelet::LaneletData>(route_lanelet.constData())));
  }
  auto route_subgraph_ptr =
    lanelet::routing::RoutingGraph::build(*route_submap_ptr, *traffic_rules);

  return RouteManager(
    lanelet_map, routing_graph, traffic_rules, std::move(all_route_lanelets),
    std::move(all_route_length_cache), std::move(preferred_lanelets), start_lanelet, goal_lanelet,
    initial_pose, closest_lanelet_opt.value(), route_submap_ptr, std::move(route_subgraph_ptr));
}

std::optional<RouteManager> RouteManager::update_current_pose(
  const geometry_msgs::msg::Pose & current_pose, const double dist_threshold_soft,
  const double yaw_threshold_soft, const double search_window) &&
{
  const auto neighbors_seq = get_lanelet_sequence_on_route(search_window, search_window);
  const auto & neighbors = neighbors_seq.as_lanelets();
  if (const auto closest_lane = get_closest_lanelet_within_constraint(
        neighbors, current_pose, dist_threshold_soft, yaw_threshold_soft);
      closest_lane) {
    return RouteManager(
      lanelet_map_ptr_, routing_graph_ptr_, traffic_rules_ptr_, std::move(all_route_lanelets_),
      std::move(all_route_length_cache_), std::move(preferred_lanelets_), start_lanelet_,
      goal_lanelet_, current_pose, closest_lane.value(),
      std::const_pointer_cast<lanelet::LaneletSubmap>(route_submap_ptr_),
      std::const_pointer_cast<lanelet::routing::RoutingGraph>(route_subgraph_ptr_));
  }
  // NOTE(soblin): following line is possible during the execution/transition of
  // swerving/lane_change
  if (const auto closest_lane = get_closest_lanelet(neighbors, current_pose); closest_lane) {
    return RouteManager(
      lanelet_map_ptr_, routing_graph_ptr_, traffic_rules_ptr_, std::move(all_route_lanelets_),
      std::move(all_route_length_cache_), std::move(preferred_lanelets_), start_lanelet_,
      goal_lanelet_, current_pose, closest_lane.value(),
      std::const_pointer_cast<lanelet::LaneletSubmap>(route_submap_ptr_),
      std::const_pointer_cast<lanelet::routing::RoutingGraph>(route_subgraph_ptr_));
  }
  // this line is possible only when `neighbors` is empty or ego position has suddenly jumped
  return std::nullopt;
}

std::optional<RouteManager> RouteManager::commit_lane_change_success(
  const geometry_msgs::msg::Pose & current_pose) &&
{
  if (const auto closest_lane = route_rtree_.get_closest_lanelet(current_pose); closest_lane) {
    return RouteManager(
      lanelet_map_ptr_, routing_graph_ptr_, traffic_rules_ptr_, std::move(all_route_lanelets_),
      std::move(all_route_length_cache_), std::move(preferred_lanelets_), start_lanelet_,
      goal_lanelet_, current_pose, closest_lane.value(),
      std::const_pointer_cast<lanelet::LaneletSubmap>(route_submap_ptr_),
      std::const_pointer_cast<lanelet::routing::RoutingGraph>(route_subgraph_ptr_));
  }
  return std::nullopt;
}

LaneSequence RouteManager::get_lanelet_sequence_on_route(
  const double forward_length, const double backward_length) const
{
  const auto ego_position_2d =
    lanelet::BasicPoint2d{current_pose_.position.x, current_pose_.position.y};
  const double current_lane_entry_to_ego =
    lanelet::geometry::toArcCoordinates(current_lanelet_.centerline2d(), ego_position_2d).length;
  const double ego_to_current_lane_exit =
    lanelet::geometry::length3d(current_lanelet_) - current_lane_entry_to_ego;

  lanelet::ConstLanelets sequence;

  // traverse backward until we exceed backward_length
  for (auto [acc_dist, prev_lanes] = std::make_tuple(
         current_lane_entry_to_ego, route_subgraph_ptr_->previous(current_lanelet_));
       acc_dist < backward_length;) {
    if (prev_lanes.empty()) {
      break;
    }

    // on "route_subgraph_ptr" there should be only one prev_lane which is also a route
    const auto & prev_lane = prev_lanes.front();

    // loop detected
    if (lanelet::utils::contains(sequence, prev_lane)) {
      break;
    }

    sequence.insert(sequence.begin(), prev_lane);
    acc_dist += all_route_length_cache_.at(prev_lane.id());
    prev_lanes = route_subgraph_ptr_->previous(prev_lane);
  }

  sequence.push_back(current_lanelet_);

  // traverse backward until we exceed backward_length
  for (auto [acc_dist, next_lanes] = std::make_tuple(
         ego_to_current_lane_exit, route_subgraph_ptr_->following(current_lanelet_));
       acc_dist < forward_length;) {
    if (next_lanes.empty()) {
      break;
    }

    // on "route_subgraph_ptr" there should be only one next_lane which is also a route
    const auto & next_lane = next_lanes.front();

    // loop detected
    if (lanelet::utils::contains(sequence, next_lane)) {
      break;
    }

    sequence.push_back(next_lane);
    acc_dist += all_route_length_cache_.at(next_lane.id());
    next_lanes = route_subgraph_ptr_->following(next_lane);
  }

  if (const auto seq = LaneSequence::create(sequence, routing_graph_ptr_); seq) {
    return seq.value();
  }
  return LaneSequence(current_lanelet_);
}

LaneSequence RouteManager::get_lanelet_sequence_outward_route(
  const double forward_length, const double backward_length) const
{
  const auto ego_position_2d =
    lanelet::BasicPoint2d{current_pose_.position.x, current_pose_.position.y};
  const double current_lane_entry_to_ego =
    lanelet::geometry::toArcCoordinates(current_lanelet_.centerline2d(), ego_position_2d).length;
  const double ego_to_current_lane_exit =
    lanelet::geometry::length3d(current_lanelet_) - current_lane_entry_to_ego;

  lanelet::ConstLanelets sequence;

  const auto initial_previous_lanes = [&]() {
    const auto route_prev_lanes = route_subgraph_ptr_->previous(current_lanelet_);
    if (route_prev_lanes.empty()) {
      const auto entire_prev_lanes = routing_graph_ptr_->previous(current_lanelet_);
      return entire_prev_lanes;
    }
    return route_prev_lanes;
  }();
  for (auto [acc_dist, prev_lanes] =
         std::make_tuple(current_lane_entry_to_ego, initial_previous_lanes);
       acc_dist < backward_length;) {
    if (prev_lanes.empty()) {
      break;
    }

    const auto prev_lane = prev_lanes.front();

    // loop detected
    if (lanelet::utils::contains(sequence, prev_lane)) {
      break;
    }

    if (const auto route_cache = all_route_length_cache_.find(prev_lane.id());
        route_cache != all_route_length_cache_.end()) {
      // traverse on route lanelets
      sequence.insert(sequence.begin(), prev_lane);
      acc_dist += route_cache->second;
      // continue search on route
      prev_lanes = route_subgraph_ptr_->previous(prev_lane);
      if (prev_lanes.empty()) {
        // this lane is the end of route lanes
        prev_lanes = routing_graph_ptr_->previous(prev_lane);
      }
    } else {
      // query outward route
      sequence.insert(sequence.begin(), prev_lane);
      acc_dist += lanelet::geometry::length3d(prev_lane);
      // continue search outside of route
      prev_lanes = routing_graph_ptr_->previous(prev_lane);
    }
  }

  sequence.push_back(current_lanelet_);

  const auto initial_next_lanes = [&]() {
    const auto route_next_lanes = route_subgraph_ptr_->following(current_lanelet_);
    if (route_next_lanes.empty()) {
      const auto entire_next_lanes = routing_graph_ptr_->following(current_lanelet_);
      return entire_next_lanes;
    }
    return route_next_lanes;
  }();
  for (auto [acc_dist, next_lanes] = std::make_tuple(ego_to_current_lane_exit, initial_next_lanes);
       acc_dist < forward_length;) {
    if (next_lanes.empty()) {
      break;
    }

    const auto next_lane = next_lanes.front();

    // loop detected
    if (lanelet::utils::contains(sequence, next_lane)) {
      break;
    }

    if (const auto route_cache = all_route_length_cache_.find(next_lane.id());
        route_cache != all_route_length_cache_.end()) {
      // traverse on route lanelets
      sequence.push_back(next_lane);
      acc_dist += route_cache->second;
      // continue search on route
      next_lanes = route_subgraph_ptr_->following(next_lane);
      if (next_lanes.empty()) {
        // this lane is the end of route lanes
        next_lanes = routing_graph_ptr_->following(next_lane);
      }
    } else {
      // query outward route
      sequence.push_back(next_lane);
      acc_dist += lanelet::geometry::length3d(next_lane);
      // continue search on route
      next_lanes = routing_graph_ptr_->following(next_lane);
    }
  }

  if (auto seq = LaneSequence::create(sequence, routing_graph_ptr_); seq) {
    return seq.value();
  }
  return LaneSequence(current_lanelet_);
}

std::optional<lanelet::ConstLanelet> RouteManager::get_closest_preferred_route_lanelet(
  const geometry_msgs::msg::Pose & search_pose) const
{
  return preferred_route_rtree_.get_closest_lanelet(search_pose);
}

std::optional<lanelet::ConstLanelet> RouteManager::get_closest_route_lanelet_within_constraints(
  const geometry_msgs::msg::Pose & search_pose, const double dist_threshold,
  const double yaw_threshold) const
{
  return route_rtree_.get_closest_lanelet_within_constraint(
    search_pose, dist_threshold, yaw_threshold);
}

RouteManager::RouteManager(
  lanelet::LaneletMapConstPtr lanelet_map_ptr,
  lanelet::routing::RoutingGraphConstPtr routing_graph_ptr,
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr,
  lanelet::ConstLanelets && all_route_lanelets,
  std::unordered_map<lanelet::Id, double> && all_route_length_cache,
  lanelet::ConstLanelets && preferred_lanelets, const lanelet::ConstLanelet & start_lanelet,
  const lanelet::ConstLanelet & goal_lanelet, const geometry_msgs::msg::Pose & current_pose,
  const lanelet::ConstLanelet & current_lanelet, const lanelet::LaneletSubmapPtr route_submap_ptr,
  const lanelet::routing::RoutingGraphPtr route_subgraph_ptr)
: MapHandler(lanelet_map_ptr, routing_graph_ptr, traffic_rules_ptr),
  all_route_lanelets_(std::move(all_route_lanelets)),
  all_route_length_cache_(std::move(all_route_length_cache)),
  route_rtree_(all_route_lanelets_),
  preferred_lanelets_(std::move(preferred_lanelets)),
  preferred_route_rtree_(preferred_lanelets_),
  start_lanelet_(start_lanelet),
  goal_lanelet_(goal_lanelet),
  current_pose_(current_pose),
  current_lanelet_(current_lanelet),
  route_submap_ptr_(route_submap_ptr),
  route_subgraph_ptr_(route_subgraph_ptr)
{
}

}  // namespace autoware::experimental::lanelet2_utils
