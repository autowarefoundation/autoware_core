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

#ifndef AUTOWARE__LANELET2_UTILS__TOPOLOGY_HPP_
#define AUTOWARE__LANELET2_UTILS__TOPOLOGY_HPP_

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/Forward.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <optional>
#include <vector>

namespace autoware::experimental::lanelet2_utils
{
/**
 * @brief get the left adjacent and same_direction lanelet on the routing graph if exists regardless
 * of lane change permission
 * @param [in] lanelet input lanelet
 * @param [in] routing_graph routing_graph containing `lanelet`
 * @return optional of left adjacent lanelet(nullopt if there is no such adjacent lanelet)
 */
std::optional<lanelet::ConstLanelet> left_lanelet(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::RoutingGraphConstPtr routing_graph);

/**
 * @brief get all the left adjacent and same_direction lanelets on the routing graph if exists
 * regardless of lane change permission
 * @param [in] lanelet input lanelet
 * @param [in] routing_graph routing_graph containing `lanelet`
 * @return optional of all left adjacent lanelets(nullopt if there is no such adjacent lanelet)
 */
std::optional<lanelet::ConstLanelets> left_lanelets(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph);

/**
 * @brief get the right adjacent and same_direction lanelet on the routing graph if exists
 * @param [in] lanelet input lanelet
 * @param [in] routing_graph routing_graph containing `lanelet`
 * @return optional of right adjacent lanelet(nullopt if there is no such adjacent lanelet)
 */
std::optional<lanelet::ConstLanelet> right_lanelet(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::RoutingGraphConstPtr routing_graph);

/**
 * @brief get all the right adjacent and same_direction lanelets on the routing graph if exists
 * regardless of lane change permission
 * @param [in] lanelet input lanelet
 * @param [in] routing_graph routing_graph containing `lanelet`
 * @return optional of all right adjacent lanelets(nullopt if there is no such adjacent lanelet)
 */
std::optional<lanelet::ConstLanelets> right_lanelets(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph);

/**
 * @brief get all adjacent and same_direction lanelets on the routing graph if exists
 * regardless of lane change permission
 * @param [in] lanelet input lanelet
 * @param [in] routing_graph routing_graph containing `lanelet`
 * @return all adjacent lanelets(vector of input lanelet if there is no such adjacent lanelet)
 * @post Output is ordered from **leftmost** lanelet to `lanelet` to **rightmost** lanelet.
 */
lanelet::ConstLanelets all_neighbor_lanelets(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph);

/**
 * @brief get the left adjacent and opposite_direction lanelet on the routing graph if exists
 * @param [in] lanelet input lanelet
 * @param [in] lanelet_map lanelet_map containing `lanelet`
 * @return optional of the left opposite lanelet(nullopt if there is not such opposite lanelet)
 */
std::optional<lanelet::ConstLanelet> left_opposite_lanelet(
  const lanelet::ConstLanelet & lanelet, const lanelet::LaneletMapConstPtr lanelet_map);

/**
 * @brief get the right adjacent and opposite_direction lanelet on the routing graph if exists
 * @param [in] lanelet input lanelet
 * @param [in] routing_graph routing_graph containing `lanelet`
 * @return optional of the right opposite lanelet(nullopt if there is no such opposite lanelet)
 */
std::optional<lanelet::ConstLanelet> right_opposite_lanelet(
  const lanelet::ConstLanelet & lanelet, const lanelet::LaneletMapConstPtr lanelet_map);

/**
 * @brief get the leftmost same_direction lanelet if exists
 * @param [in] lanelet input lanelet
 * @param [in] routing_graph routing_graph containing `lanelet`
 * @return optional of such lanelet(nullopt if there is no such adjacent lanelet)
 */
std::optional<lanelet::ConstLanelet> leftmost_lanelet(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::RoutingGraphConstPtr routing_graph);

std::optional<lanelet::ConstLanelet> rightmost_lanelet(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::RoutingGraphConstPtr routing_graph);

/**
 * @brief get the following lanelets
 * @param [in] lanelet input lanelet
 * @param [in] routing_graph routing_graph containing `lanelet`
 * @return the following lanelets
 */
lanelet::ConstLanelets following_lanelets(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::RoutingGraphConstPtr routing_graph);

/**
 * @brief get the previous lanelets
 * @param [in] lanelet input lanelet
 * @param [in] routing_graph routing_graph containing `lanelet`
 * @return the previous lanelets
 */
lanelet::ConstLanelets previous_lanelets(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::RoutingGraphConstPtr routing_graph);

/**
 * @brief get the sibling lanelets
 * @param [in] lanelet input lanelet
 * @param [in] routing_graph routing_graph containing `lanelet`
 * @return the sibling lanelets excluding `lanelet`
 */
lanelet::ConstLanelets sibling_lanelets(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::RoutingGraphConstPtr routing_graph);

/**
 * @brief get Lanelet instances of the designated ids
 * @param [in] lanelet input lanelet
 * @param [in] routing_graph routing_graph containing `lanelet`
 * @return the list of Lanelets in the same order as `ids`
 */
lanelet::ConstLanelets from_ids(
  const lanelet::LaneletMapConstPtr lanelet_map, const std::vector<lanelet::Id> & ids);

/**
 * @brief get ConstLanelets that has conflict with lanelet in routing graph
 * @param [in] graph routing_graph containing `lanelet`
 * @param [in] lanelet input lanelet
 */
lanelet::ConstLanelets get_conflicting_lanelets(
  const lanelet::ConstLanelet & lanelet, const lanelet::routing::RoutingGraphConstPtr & graph);

/**
 * @brief get adjacent (neighboring) lanelets that allow lane change from input lanelet including
 * itself.
 * @param [in] lanelet input lanelet
 * @param [in] routing_graph routing_graph containing `lanelet`
 * @post returned lanelets are ordered from left to right.
 */
lanelet::ConstLanelets lane_changeable_neighbors(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph);

/**
 * @brief enumerate all succeeding(following) lanelet sequences possible from input lanelet within
 * given length limit. (Also include the last lanelet that exceeds length limit).
 * @param[in] lanelet input lanelet
 * @param[in] routing_graph routing_graph containing `lanelet`
 * @param[in] length length limit
 * @return lanelet sequences that follow input lanelet (does not include input lanelet)
 * @post the lanelet sequences is ordered from closest to furthest.
 */
std::vector<lanelet::ConstLanelets> get_succeeding_lanelet_sequences(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph, double length);

/**
 * @brief enumerate all preceding(previous) lanelet sequences possible leading to input lanelet
 * within given length limit. (Also include the last lanelet that exceeds length limit).
 * @param[in] lanelet input_lanelet
 * @param[in] routing_graph routing_graph containing `lanelet`
 * @param[in] length length limit
 * @param[in] excluding_lanelets to be excluded lanelets
 * @return lanelet sequences that leads to input lanelet (does not include input lanelet)
 * @post the lanelet sequences is ordered from furthest to closest.
 */
std::vector<lanelet::ConstLanelets> get_preceding_lanelet_sequences(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph, double length,
  const lanelet::ConstLanelets & exclude_lanelets = {});

}  // namespace autoware::experimental::lanelet2_utils

#endif  // AUTOWARE__LANELET2_UTILS__TOPOLOGY_HPP_
