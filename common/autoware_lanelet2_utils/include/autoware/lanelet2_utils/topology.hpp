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
 * @brief get the right adjacent and same_direction lanelet on the routing graph if exists
 * @param [in] lanelet input lanelet
 * @param [in] routing_graph routing_graph containing `lanelet`
 * @return optional of right adjacent lanelet(nullopt if there is no such adjacent lanelet)
 */
std::optional<lanelet::ConstLanelet> right_lanelet(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::RoutingGraphConstPtr routing_graph);

/**
 * @brief get left_lanelet() or sibling lanelet. If `lanelet` has turn_direction, search for sibling
 * lanelet is limited to the one with same turn_direction value
 * @param [in] lanelet input lanelet
 * @param [in] routing_graph routing_graph containing `lanelet`
 * @return optional of aforementioned lanelet(nullopt if there is no such lanelet)
 */
/*
std::optional<lanelet::ConstLanelet> left_similar_lanelet(
const lanelet::ConstLanelet & lanelet, const lanelet::LaneletMapConstPtr lanelet_map,
const lanelet::routing::RoutingGraphConstPtr routing_graph);
*/

/**
 * @brief get right_lanelet() or sibling lanelet. If `lanelet` has turn_direction, search for
 * sibling lanelet is limited to the one with same turn_direction value
 * @param [in] lanelet input lanelet
 * @param [in] routing_graph routing_graph containing `lanelet`
 * @return optional of aforementioned lanelet(nullopt if there is no such lanelet)
 */
/*
std::optional<lanelet::ConstLanelet> right_similar_lanelet(
const lanelet::ConstLanelet & lanelet, const lanelet::LaneletMapConstPtr lanelet_map,
const lanelet::routing::RoutingGraphConstPtr routing_graph);
*/

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
 * @brief get the left lanelets which are adjacent to `lanelet`
 * @param [in] lanelet input lanelet
 * @param [in] routing_graph routing_graph containing `lanelet`
 * @param [in] include_opposite flag if opposite_direction lanelet is included
 * @param [in] invert_opposite_lanelet flag if the opposite lanelet in the output is `.inverted()`
 * or not
 * @return the list of lanelets excluding `lanelet` which is ordered in the *hopping* number from
 * `lanelet`
 */
lanelet::ConstLanelets left_lanelets(
  const lanelet::ConstLanelet & lanelet, const lanelet::LaneletMapConstPtr lanelet_map,
  const lanelet::routing::RoutingGraphConstPtr routing_graph, const bool include_opposite = false,
  const bool invert_opposite_lane = false);

/**
 * @brief get the right lanelets which are adjacent to `lanelet`
 * @param [in] lanelet input lanelet
 * @param [in] routing_graph routing_graph containing `lanelet`
 * @param [in] include_opposite flag if opposite_direction lanelet is included
 * @param [in] invert_opposite_lanelet flag if the opposite lanelet in the output is `.inverted()`
 * or not
 * @return the list of lanelets excluding `lanelet` which is ordered in the *hopping* number from
 * `lanelet`
 */
lanelet::ConstLanelets right_lanelets(
  const lanelet::ConstLanelet & lanelet, const lanelet::LaneletMapConstPtr lanelet_map,
  const lanelet::routing::RoutingGraphConstPtr routing_graph, const bool include_opposite = false,
  const bool invert_opposite_lane = false);

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
}  // namespace autoware::experimental::lanelet2_utils

#endif  // AUTOWARE__LANELET2_UTILS__TOPOLOGY_HPP_
