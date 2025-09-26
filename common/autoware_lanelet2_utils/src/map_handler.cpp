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
#include <autoware/lanelet2_utils/kind.hpp>
#include <autoware/lanelet2_utils/map_handler.hpp>
#include <autoware/lanelet2_utils/topology.hpp>
#include <rclcpp/logging.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <set>

namespace
{

std::optional<lanelet::ConstLanelet> get_previous_shoulder_lanelet(
  const lanelet::ConstLanelet & lanelet, const lanelet::LaneletMapConstPtr lanelet_map_ptr)
{
  bool found = false;
  const auto & search_point = lanelet.centerline().front().basicPoint2d();
  const auto previous_lanelet = lanelet_map_ptr->laneletLayer.nearestUntil(
    search_point, [&](const auto & bbox, const auto & ll) {
      if (
        autoware::experimental::lanelet2_utils::is_shoulder_lane(ll) &&
        lanelet::geometry::follows(ll, lanelet))
        found = true;
      // stop search once prev shoulder lanelet is found, or the bbox does not touch the search
      // point
      return found || lanelet::geometry::distance2d(bbox, search_point) > 1e-3;
    });
  if (found && previous_lanelet.has_value()) return *previous_lanelet;
  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> get_following_shoulder_lanelet(
  const lanelet::ConstLanelet & lanelet, const lanelet::LaneletMapConstPtr lanelet_map_ptr)
{
  bool found = false;
  const auto & search_point = lanelet.centerline().back().basicPoint2d();
  const auto next_lanelet = lanelet_map_ptr->laneletLayer.nearestUntil(
    search_point, [&](const auto & bbox, const auto & ll) {
      if (
        autoware::experimental::lanelet2_utils::is_shoulder_lane(ll) &&
        lanelet::geometry::follows(lanelet, ll)) {
        found = true;
      }
      return found || lanelet::geometry::distance2d(bbox, search_point) > 1e-3;
    });
  if (found && next_lanelet.has_value()) {
    return *next_lanelet;
  }
  return std::nullopt;
}

lanelet::ConstLanelets get_shoulder_lanelet_sequence_upto(
  const lanelet::ConstLanelet & lanelet, const lanelet::LaneletMapConstPtr lanelet_map_ptr,
  const double min_length)
{
  double length = 0.0;
  lanelet::ConstLanelets lanelets;
  lanelet::ConstLanelet current_lanelet = lanelet;
  std::set<lanelet::Id> searched_ids{lanelet.id()};
  while (length < min_length) {
    const auto previous_shoulder_lanelet =
      get_previous_shoulder_lanelet(current_lanelet, lanelet_map_ptr);
    if (!previous_shoulder_lanelet) break;

    if (searched_ids.find(previous_shoulder_lanelet->id()) != searched_ids.end()) {
      // loop detected
      break;
    }

    lanelets.insert(lanelets.begin(), *previous_shoulder_lanelet);
    searched_ids.insert(previous_shoulder_lanelet->id());
    length += lanelet::geometry::length3d(*previous_shoulder_lanelet);

    current_lanelet = *previous_shoulder_lanelet;
  }
  return lanelets;
}

lanelet::ConstLanelets get_shoulder_lanelet_sequence_after(
  const lanelet::ConstLanelet & lanelet, const lanelet::LaneletMapConstPtr lanelet_map_ptr,
  const double min_length)
{
  double length = 0.0;
  lanelet::ConstLanelets lanelets;
  lanelet::ConstLanelet current_lanelet = lanelet;
  std::set<lanelet::Id> searched_ids{lanelet.id()};
  while (length < min_length) {
    const auto next_shoulder_lanelet =
      get_following_shoulder_lanelet(current_lanelet, lanelet_map_ptr);
    if (!next_shoulder_lanelet) break;

    if (searched_ids.find(next_shoulder_lanelet->id()) != searched_ids.end()) {
      // loop detected
      break;
    }

    lanelets.push_back(*next_shoulder_lanelet);
    searched_ids.insert(next_shoulder_lanelet->id());
    length += lanelet::geometry::length3d(*next_shoulder_lanelet);

    current_lanelet = *next_shoulder_lanelet;
  }
  return lanelets;
}

}  // namespace

namespace autoware::experimental::lanelet2_utils
{

static constexpr size_t k_normal_bundle_max_size = 10;

std::optional<MapHandler> MapHandler::create(const autoware_map_msgs::msg::LaneletMapBin & map_msg)
{
  const auto lanelet_map = from_autoware_map_msgs(map_msg);
  if (!lanelet_map) {
    return std::nullopt;
  }
  const auto [routing_graph, traffic_rules] =
    instantiate_routing_graph_and_traffic_rules(lanelet_map);

  return MapHandler(lanelet_map, routing_graph, traffic_rules);
}

MapHandler::MapHandler(
  lanelet::LaneletMapConstPtr lanelet_map_ptr,
  lanelet::routing::RoutingGraphConstPtr routing_graph_ptr,
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr)
: lanelet_map_ptr_(lanelet_map_ptr),
  routing_graph_ptr_(routing_graph_ptr),
  traffic_rules_ptr_(traffic_rules_ptr)
{
}

std::optional<lanelet::ConstLanelet> MapHandler::left_lanelet(
  const lanelet::ConstLanelet & lanelet, const bool take_sibling, const ExtraVRU extra_vru) const
{
  if (is_shoulder_lane(lanelet) || is_bicycle_lane(lanelet)) {
    for (const auto & left_lanelet :
         lanelet_map_ptr_->laneletLayer.findUsages(lanelet.leftBound())) {
      if (is_road_lane(left_lanelet)) return left_lanelet;
    }
    return std::nullopt;
  }

  if (extra_vru == ExtraVRU::Shoulder || extra_vru == ExtraVRU::ShoulderAndBicycleLane) {
    if (const auto left_shoulder = left_shoulder_lanelet(lanelet); left_shoulder) {
      return left_shoulder;
    }
  }
  if (extra_vru == ExtraVRU::BicycleLane || extra_vru == ExtraVRU::ShoulderAndBicycleLane) {
    if (const auto left_bicycle_lane = left_bicycle_lanelet(lanelet); left_bicycle_lane) {
      return left_bicycle_lane;
    }
  }

  if (const auto left_lane = routing_graph_ptr_->left(lanelet)) {
    // lane changeable
    return *left_lane;
  }
  if (const auto adjacent_left_lane = routing_graph_ptr_->adjacentLeft(lanelet)) {
    return *adjacent_left_lane;
  }
  if (!take_sibling) {
    return std::nullopt;
  }

  for (const auto & sibling : sibling_lanelets(lanelet, routing_graph_ptr_)) {
    if (lanelet.leftBound().back().id() == sibling.rightBound().back().id()) {
      return sibling;
    }
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> MapHandler::right_lanelet(
  const lanelet::ConstLanelet & lanelet, const bool take_sibling, const ExtraVRU extra_vru) const
{
  if (is_shoulder_lane(lanelet) || is_bicycle_lane(lanelet)) {
    for (const auto & right_lanelet :
         lanelet_map_ptr_->laneletLayer.findUsages(lanelet.rightBound())) {
      if (is_road_lane(right_lanelet)) return right_lanelet;
    }
    return std::nullopt;
  }

  if (extra_vru == ExtraVRU::Shoulder || extra_vru == ExtraVRU::ShoulderAndBicycleLane) {
    if (const auto right_shoulder = right_shoulder_lanelet(lanelet); right_shoulder) {
      return right_shoulder;
    }
  }
  if (extra_vru == ExtraVRU::BicycleLane || extra_vru == ExtraVRU::ShoulderAndBicycleLane) {
    if (const auto right_bicycle_lane = right_bicycle_lanelet(lanelet); right_bicycle_lane) {
      return right_bicycle_lane;
    }
  }

  if (const auto right_lane = routing_graph_ptr_->right(lanelet)) {
    // lane changeable
    return *right_lane;
  }
  if (const auto adjacent_right_lane = routing_graph_ptr_->adjacentRight(lanelet)) {
    return *adjacent_right_lane;
  }
  if (!take_sibling) {
    return std::nullopt;
  }

  for (const auto & sibling : sibling_lanelets(lanelet, routing_graph_ptr_)) {
    if (lanelet.rightBound().back().id() == sibling.leftBound().back().id()) {
      return sibling;
    }
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> MapHandler::leftmost_lanelet(
  const lanelet::ConstLanelet & lanelet, const bool take_sibling, const ExtraVRU extra_vru) const
{
  auto left_lane = left_lanelet(lanelet, take_sibling, extra_vru);
  if (!left_lane) {
    return std::nullopt;
  }
  size_t bundle_size_diagnosis = 0;
  while (bundle_size_diagnosis < k_normal_bundle_max_size) {
    const auto next_left_lane = left_lanelet(left_lane.value(), take_sibling, extra_vru);
    if (!next_left_lane) {
      // reached
      return left_lane;
    }
    left_lane = next_left_lane.value();
    bundle_size_diagnosis++;
  }

  // LCOV_EXCL_START
  RCLCPP_ERROR(
    rclcpp::get_logger("autoware_lanelet2_utility"),
    "You have passed an unrealistic map with a bundle of size>=10");
  return std::nullopt;
  // LCOV_EXCL_STOP
}

std::optional<lanelet::ConstLanelet> MapHandler::rightmost_lanelet(
  const lanelet::ConstLanelet & lanelet, const bool take_sibling, const ExtraVRU extra_vru) const
{
  auto right_lane = right_lanelet(lanelet, take_sibling, extra_vru);
  if (!right_lane) {
    return std::nullopt;
  }
  size_t bundle_size_diagnosis = 0;
  while (bundle_size_diagnosis < k_normal_bundle_max_size) {
    const auto next_right_lane = right_lanelet(right_lane.value(), take_sibling, extra_vru);
    if (!next_right_lane) {
      // reached
      return right_lane;
    }
    right_lane = next_right_lane.value();
    bundle_size_diagnosis++;
  }

  // LCOV_EXCL_START
  RCLCPP_ERROR(
    rclcpp::get_logger("autoware_lanelet2_utility"),
    "You have passed an unrealistic map with a bundle of size>=10");
  return std::nullopt;
  // LCOV_EXCL_STOP
}

lanelet::ConstLanelets MapHandler::left_lanelets(
  const lanelet::ConstLanelet & lanelet, const bool include_opposite,
  const bool invert_opposite_lane) const
{
  lanelet::ConstLanelets lefts{};
  auto left_lane = left_lanelet(lanelet, false, ExtraVRU::RoadOnly);
  size_t bundle_size_diagnosis = 0;
  while (bundle_size_diagnosis < k_normal_bundle_max_size) {
    if (!left_lane) {
      break;
    }
    lefts.push_back(left_lane.value());
    left_lane = left_lanelet(left_lane.value(), false, ExtraVRU::RoadOnly);
    bundle_size_diagnosis++;
  }
  // LCOV_EXCL_START
  if (bundle_size_diagnosis >= k_normal_bundle_max_size) {
    RCLCPP_ERROR(
      rclcpp::get_logger("autoware_lanelet2_utility"),
      "You have passed an unrealistic map with a bundle of size>=10");
    return {};
  }
  // LCOV_EXCL_STOP
  if (lefts.empty()) {
    return {};
  }
  const auto & leftmost = lefts.back();
  if (include_opposite) {
    const auto direct_opposite = left_opposite_lanelet(leftmost, lanelet_map_ptr_);
    if (direct_opposite) {
      const auto opposites = right_lanelets(direct_opposite.value(), false, false);
      lefts.push_back(direct_opposite.value());
      for (const auto & opposite : opposites) {
        lefts.push_back(invert_opposite_lane ? opposite.invert() : opposite);
      }
    }
  }
  return lefts;
}

lanelet::ConstLanelets MapHandler::right_lanelets(
  const lanelet::ConstLanelet & lanelet, const bool include_opposite,
  const bool invert_opposite_lane) const
{
  lanelet::ConstLanelets rights{};
  auto right_lane = right_lanelet(lanelet, false, ExtraVRU::RoadOnly);
  size_t bundle_size_diagnosis = 0;
  while (bundle_size_diagnosis < k_normal_bundle_max_size) {
    if (!right_lane) {
      break;
    }
    rights.push_back(right_lane.value());
    right_lane = right_lanelet(right_lane.value(), false, ExtraVRU::RoadOnly);
    bundle_size_diagnosis++;
  }
  // LCOV_EXCL_START
  if (bundle_size_diagnosis >= k_normal_bundle_max_size) {
    RCLCPP_ERROR(
      rclcpp::get_logger("autoware_lanelet2_utility"),
      "You have passed an unrealistic map with a bundle of size>=10");
    return {};
  }
  // LCOV_EXCL_STOP
  if (rights.empty()) {
    return {};
  }
  const auto & rightmost = rights.back();
  if (include_opposite) {
    const auto direct_opposite = right_opposite_lanelet(rightmost, lanelet_map_ptr_);
    if (direct_opposite) {
      const auto opposites = left_lanelets(direct_opposite.value(), false, false);
      rights.push_back(direct_opposite.value());
      for (const auto & opposite : opposites) {
        rights.push_back(invert_opposite_lane ? opposite.invert() : opposite);
      }
    }
  }
  return rights;
}

lanelet::ConstLanelets MapHandler::get_shoulder_lanelet_sequence(
  const lanelet::ConstLanelet & lanelet, const double backward_distance,
  const double forward_distance) const
{
  if (!is_shoulder_lane(lanelet)) {
    return {};
  }

  const auto forward =
    get_shoulder_lanelet_sequence_after(lanelet, lanelet_map_ptr_, forward_distance);
  const auto backward =
    get_shoulder_lanelet_sequence_upto(lanelet, lanelet_map_ptr_, backward_distance);
  lanelet::ConstLanelets all = backward;

  // loop check
  if (!lanelet::utils::contains(all, lanelet)) {
    all.push_back(lanelet);
  }
  for (const auto & forward_lanelet : forward) {
    if (lanelet::utils::contains(all, forward_lanelet)) {
      break;
    }
    all.push_back(forward_lanelet);
  }

  return all;
}

std::optional<lanelet::ConstLanelet> MapHandler::left_shoulder_lanelet(
  const lanelet::ConstLanelet & lanelet) const
{
  for (const auto & other_lanelet :
       lanelet_map_ptr_->laneletLayer.findUsages(lanelet.leftBound())) {
    if (other_lanelet.rightBound() == lanelet.leftBound() && is_shoulder_lane(other_lanelet))
      return other_lanelet;
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> MapHandler::right_shoulder_lanelet(
  const lanelet::ConstLanelet & lanelet) const
{
  for (const auto & other_lanelet :
       lanelet_map_ptr_->laneletLayer.findUsages(lanelet.rightBound())) {
    if (other_lanelet.leftBound() == lanelet.rightBound() && is_shoulder_lane(other_lanelet))
      return other_lanelet;
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> MapHandler::left_bicycle_lanelet(
  const lanelet::ConstLanelet & lanelet) const
{
  for (const auto & other_lanelet :
       lanelet_map_ptr_->laneletLayer.findUsages(lanelet.leftBound())) {
    if (other_lanelet.rightBound() == lanelet.leftBound() && is_bicycle_lane(other_lanelet))
      return other_lanelet;
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> MapHandler::right_bicycle_lanelet(
  const lanelet::ConstLanelet & lanelet) const
{
  for (const auto & other_lanelet :
       lanelet_map_ptr_->laneletLayer.findUsages(lanelet.rightBound())) {
    if (other_lanelet.leftBound() == lanelet.rightBound() && is_bicycle_lane(other_lanelet))
      return other_lanelet;
  }
  return std::nullopt;
}

}  // namespace autoware::experimental::lanelet2_utils
