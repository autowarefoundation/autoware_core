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

#include <autoware/lanelet2_utils/topology.hpp>
#include <range/v3/all.hpp>
#include <rclcpp/logging.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <vector>

namespace autoware::experimental::lanelet2_utils
{

static constexpr size_t k_normal_bundle_max_size = 10;

std::optional<lanelet::ConstLanelet> left_lanelet(
  const lanelet::ConstLanelet & lanelet, const lanelet::routing::RoutingGraphConstPtr routing_graph)
{
  if (const auto left_lane = routing_graph->left(lanelet)) {
    // lane changeable
    return *left_lane;
  }
  if (const auto adjacent_left_lane = routing_graph->adjacentLeft(lanelet)) {
    return *adjacent_left_lane;
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> right_lanelet(
  const lanelet::ConstLanelet & lanelet, const lanelet::routing::RoutingGraphConstPtr routing_graph)
{
  if (const auto right_lane = routing_graph->right(lanelet)) {
    // lane changeable
    return *right_lane;
  }
  if (const auto adjacent_right_lane = routing_graph->adjacentRight(lanelet)) {
    return *adjacent_right_lane;
  }
  return std::nullopt;
}

/*
std::optional<lanelet::ConstLanelet> left_similar_lanelet(
const lanelet::ConstLanelet & lanelet, const lanelet::LaneletMapConstPtr lanelet_map,
const lanelet::routing::RoutingGraphConstPtr routing_graph);
*/

/*
std::optional<lanelet::ConstLanelet> right_similar_lanelet(
const lanelet::ConstLanelet & lanelet, const lanelet::LaneletMapConstPtr lanelet_map,
const lanelet::routing::RoutingGraphConstPtr routing_graph);
*/

std::optional<lanelet::ConstLanelet> left_opposite_lanelet(
  const lanelet::ConstLanelet & lanelet, const lanelet::LaneletMapConstPtr lanelet_map)
{
  for (const auto & opposite_candidate :
       lanelet_map->laneletLayer.findUsages(lanelet.leftBound().invert())) {
    return opposite_candidate;
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> right_opposite_lanelet(
  const lanelet::ConstLanelet & lanelet, const lanelet::LaneletMapConstPtr lanelet_map)
{
  for (const auto & opposite_candidate :
       lanelet_map->laneletLayer.findUsages(lanelet.rightBound().invert())) {
    return opposite_candidate;
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> leftmost_lanelet(
  const lanelet::ConstLanelet & lanelet, const lanelet::routing::RoutingGraphConstPtr routing_graph)
{
  auto left_lane = left_lanelet(lanelet, routing_graph);
  if (!left_lane) {
    return std::nullopt;
  }
  size_t bundle_size_diagnosis = 0;
  while (bundle_size_diagnosis < k_normal_bundle_max_size) {
    const auto next_left_lane = left_lanelet(left_lane.value(), routing_graph);
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

std::optional<lanelet::ConstLanelet> rightmost_lanelet(
  const lanelet::ConstLanelet & lanelet, const lanelet::routing::RoutingGraphConstPtr routing_graph)
{
  auto right_lane = right_lanelet(lanelet, routing_graph);
  if (!right_lane) {
    return std::nullopt;
  }
  size_t bundle_size_diagnosis = 0;
  while (bundle_size_diagnosis < k_normal_bundle_max_size) {
    const auto next_right_lane = right_lanelet(right_lane.value(), routing_graph);
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

lanelet::ConstLanelets left_lanelets(
  const lanelet::ConstLanelet & lanelet, const lanelet::LaneletMapConstPtr lanelet_map,
  const lanelet::routing::RoutingGraphConstPtr routing_graph, const bool include_opposite,
  const bool invert_opposite_lane)
{
  lanelet::ConstLanelets lefts{};
  auto left_lane = left_lanelet(lanelet, routing_graph);
  size_t bundle_size_diagnosis = 0;
  while (bundle_size_diagnosis < k_normal_bundle_max_size) {
    if (!left_lane) {
      break;
    }
    lefts.push_back(left_lane.value());
    left_lane = left_lanelet(left_lane.value(), routing_graph);
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
    const auto direct_opposite = left_opposite_lanelet(leftmost, lanelet_map);
    if (direct_opposite) {
      const auto opposites =
        right_lanelets(direct_opposite.value(), lanelet_map, routing_graph, false, false);
      lefts.push_back(direct_opposite.value());
      for (const auto & opposite : opposites) {
        lefts.push_back(invert_opposite_lane ? opposite.invert() : opposite);
      }
    }
  }
  return lefts;
}

lanelet::ConstLanelets right_lanelets(
  const lanelet::ConstLanelet & lanelet, const lanelet::LaneletMapConstPtr lanelet_map,
  const lanelet::routing::RoutingGraphConstPtr routing_graph, const bool include_opposite,
  const bool invert_opposite_lane)
{
  lanelet::ConstLanelets rights{};
  auto right_lane = right_lanelet(lanelet, routing_graph);
  size_t bundle_size_diagnosis = 0;
  while (bundle_size_diagnosis < k_normal_bundle_max_size) {
    if (!right_lane) {
      break;
    }
    rights.push_back(right_lane.value());
    right_lane = right_lanelet(right_lane.value(), routing_graph);
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
    const auto direct_opposite = right_opposite_lanelet(rightmost, lanelet_map);
    if (direct_opposite) {
      const auto opposites =
        left_lanelets(direct_opposite.value(), lanelet_map, routing_graph, false, false);
      rights.push_back(direct_opposite.value());
      for (const auto & opposite : opposites) {
        rights.push_back(invert_opposite_lane ? opposite.invert() : opposite);
      }
    }
  }
  return rights;
}

lanelet::ConstLanelets following_lanelets(
  const lanelet::ConstLanelet & lanelet, const lanelet::routing::RoutingGraphConstPtr routing_graph)
{
  return routing_graph->following(lanelet);
}

lanelet::ConstLanelets previous_lanelets(
  const lanelet::ConstLanelet & lanelet, const lanelet::routing::RoutingGraphConstPtr routing_graph)
{
  return routing_graph->previous(lanelet);
}

lanelet::ConstLanelets sibling_lanelets(
  const lanelet::ConstLanelet & lanelet, const lanelet::routing::RoutingGraphConstPtr routing_graph)
{
  lanelet::ConstLanelets siblings;
  for (const auto & previous : previous_lanelets(lanelet, routing_graph)) {
    for (const auto & following : following_lanelets(previous, routing_graph)) {
      if (following.id() != lanelet.id()) {
        siblings.push_back(following);
      }
    }
  }
  return siblings;
}

lanelet::ConstLanelets from_ids(
  const lanelet::LaneletMapConstPtr lanelet_map, const std::vector<lanelet::Id> & ids)
{
  return ids | ranges::views::transform([&](const auto id) {
           return lanelet_map->laneletLayer.get(id);
         }) |
         ranges::to<std::vector>();
}
}  // namespace autoware::experimental::lanelet2_utils
