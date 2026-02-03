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
#include <autoware/lanelet2_utils/topology.hpp>
#include <range/v3/all.hpp>
#include <rclcpp/logging.hpp>

#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <limits>
#include <vector>

namespace
{
std::vector<lanelet::ConstLanelets> get_succeeding_lanelet_sequences_recursive(
  const lanelet::ConstLanelet & current_lanelet,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph, double remaining_length)
{
  std::vector<lanelet::ConstLanelets> succeeding_lanelet_sequences;

  const auto next_lanelets = routing_graph->following(current_lanelet);
  const double current_lanelet_length = lanelet::geometry::length2d(current_lanelet);

  // TODO(sarun-hub): no loop check yet

  // end condition of the recursive function
  if (next_lanelets.empty() || current_lanelet_length >= remaining_length) {
    succeeding_lanelet_sequences.push_back({current_lanelet});
    return succeeding_lanelet_sequences;
  }

  for (const auto & next_lanelet : next_lanelets) {
    // get lanelet sequence after next_lanelet
    auto tmp_lanelet_sequences = get_succeeding_lanelet_sequences_recursive(
      next_lanelet, routing_graph, remaining_length - current_lanelet_length);
    for (auto & tmp_lanelet_sequence : tmp_lanelet_sequences) {
      // fill from bottom to top node (from furthest to closest)
      tmp_lanelet_sequence.push_back(current_lanelet);
      succeeding_lanelet_sequences.push_back(tmp_lanelet_sequence);
    }
  }
  return succeeding_lanelet_sequences;
}

std::vector<lanelet::ConstLanelets> get_preceding_lanelet_sequences_recursive(
  const lanelet::ConstLanelet & current_lanelet,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph, double remaining_length,
  const lanelet::ConstLanelets & exclude_lanelets)
{
  std::vector<lanelet::ConstLanelets> preceding_lanelet_sequences;

  const auto prev_lanelets = routing_graph->previous(current_lanelet);
  const double current_lanelet_length = lanelet::geometry::length2d(current_lanelet);

  // TODO(sarun-hub): no loop check yet

  // end condition of the recursive function
  if (prev_lanelets.empty() || current_lanelet_length >= remaining_length) {
    preceding_lanelet_sequences.push_back({current_lanelet});
    return preceding_lanelet_sequences;
  }

  for (const auto & prev_lanelet : prev_lanelets) {
    if (lanelet::utils::contains(exclude_lanelets, prev_lanelet)) {
      // if prev_lanelet is included in exclude_lanelets,
      // remove prev_lanelet from preceding_lanelet_sequences
      continue;
    }
    // get lanelet sequence after prev_lanelet
    auto tmp_lanelet_sequences = get_preceding_lanelet_sequences_recursive(
      prev_lanelet, routing_graph, remaining_length - current_lanelet_length, exclude_lanelets);
    for (auto & tmp_lanelet_sequence : tmp_lanelet_sequences) {
      // fill from bottom to top node (from furthest to closest)
      tmp_lanelet_sequence.push_back(current_lanelet);
      preceding_lanelet_sequences.push_back(tmp_lanelet_sequence);
    }
  }
  // In case that exclude all prev_lanelets
  if (preceding_lanelet_sequences.empty()) {
    preceding_lanelet_sequences.push_back({current_lanelet});
  }
  return preceding_lanelet_sequences;
}

}  // namespace

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

std::optional<lanelet::ConstLanelets> left_lanelets(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph)
{
  lanelet::ConstLanelets lanelets;
  auto left_lane = left_lanelet(lanelet, routing_graph);
  // If the left lane doesn't exist at the first place.
  if (!left_lane.has_value()) {
    return std::nullopt;
  }
  // If the left lane exists.
  while (left_lane.has_value()) {
    lanelets.push_back(left_lane.value());
    left_lane = left_lanelet(left_lane.value(), routing_graph);
  }
  return lanelets;
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

std::optional<lanelet::ConstLanelets> right_lanelets(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph)
{
  lanelet::ConstLanelets lanelets;
  auto right_lane = right_lanelet(lanelet, routing_graph);
  // If the right lane doesn't exist at the first place.
  if (!right_lane.has_value()) {
    return std::nullopt;
  }
  // If the right lane exists.
  while (right_lane.has_value()) {
    lanelets.push_back(right_lane.value());
    right_lane = right_lanelet(right_lane.value(), routing_graph);
  }
  return lanelets;
}

lanelet::ConstLanelets all_neighbor_lanelets(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph)
{
  lanelet::ConstLanelets lanelets;

  auto left_opt = left_lanelets(lanelet, routing_graph);
  auto right_opt = right_lanelets(lanelet, routing_graph);

  if (!left_opt.has_value() && !right_opt.has_value()) {
    lanelets.push_back(lanelet);
    return lanelets;
  }

  if (left_opt.has_value()) {
    auto left_lls = left_opt.value();
    std::reverse(left_lls.begin(), left_lls.end());
    lanelets.insert(lanelets.end(), left_lls.begin(), left_lls.end());
  }

  lanelets.push_back(lanelet);

  if (right_opt.has_value()) {
    auto right_lls = right_opt.value();
    lanelets.insert(lanelets.end(), right_lls.begin(), right_lls.end());
  }

  return lanelets;
}

lanelet::ConstLanelets lane_changeable_neighbors(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph)
{
  return routing_graph->besides(lanelet);
}

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

lanelet::ConstLanelets get_conflicting_lanelets(
  const lanelet::ConstLanelet & lanelet, const lanelet::routing::RoutingGraphConstPtr & graph)
{
  const auto & llt_or_areas = graph->conflicting(lanelet);
  lanelet::ConstLanelets lanelets;
  lanelets.reserve(llt_or_areas.size());
  for (const auto & l_or_a : llt_or_areas) {
    auto llt_opt = l_or_a.lanelet();
    if (!!llt_opt) {
      lanelets.push_back(llt_opt.get());
    }
  }
  return lanelets;
}

std::vector<lanelet::ConstLanelets> get_succeeding_lanelet_sequences(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph, double length)
{
  std::vector<lanelet::ConstLanelets> succeeding_lanelet_sequences;

  const auto next_lanelets = routing_graph->following(lanelet);
  // start from next_lanelet
  for (const auto & next_lanelet : next_lanelets) {
    // recursive starts
    auto tmp_succeeding_lanelet_sequences =
      get_succeeding_lanelet_sequences_recursive(next_lanelet, routing_graph, length);
    // reverse to get closest to furthest
    for (auto & tmp_lanelet_sequence : tmp_succeeding_lanelet_sequences) {
      std::reverse(tmp_lanelet_sequence.begin(), tmp_lanelet_sequence.end());
    }
    succeeding_lanelet_sequences.insert(
      succeeding_lanelet_sequences.end(), tmp_succeeding_lanelet_sequences.begin(),
      tmp_succeeding_lanelet_sequences.end());
  }
  return succeeding_lanelet_sequences;
}

std::vector<lanelet::ConstLanelets> get_preceding_lanelet_sequences(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph, double length,
  const lanelet::ConstLanelets & exclude_lanelets)
{
  std::vector<lanelet::ConstLanelets> preceding_lanelet_sequences;

  const auto prev_lanelets = routing_graph->previous(lanelet);
  // start from prev_lanelet
  for (const auto & prev_lanelet : prev_lanelets) {
    if (lanelet::utils::contains(exclude_lanelets, prev_lanelet)) {
      // if prev_lanelet is included in exclude_lanelets,
      // remove prev_lanelet from preceding_lanelet_sequences
      continue;
    }
    // recursive starts
    auto tmp_preceding_lanelet_sequences = get_preceding_lanelet_sequences_recursive(
      prev_lanelet, routing_graph, length, exclude_lanelets);
    // does not reverse (order from furthest to closest)
    preceding_lanelet_sequences.insert(
      preceding_lanelet_sequences.end(), tmp_preceding_lanelet_sequences.begin(),
      tmp_preceding_lanelet_sequences.end());
  }
  return preceding_lanelet_sequences;
}

}  // namespace autoware::experimental::lanelet2_utils
