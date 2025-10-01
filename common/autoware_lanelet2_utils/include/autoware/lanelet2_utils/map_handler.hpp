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

#ifndef AUTOWARE__LANELET2_UTILS__MAP_HANDLER_HPP_
#define AUTOWARE__LANELET2_UTILS__MAP_HANDLER_HPP_

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_routing/Forward.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <limits>
#include <optional>

namespace autoware::experimental::lanelet2_utils
{

enum class ExtraVRU : size_t {
  RoadOnly = 0,
  Shoulder = 1,
  BicycleLane = 2,
  ShoulderAndBicycleLane = 3,
};

class MapHandler
{
public:
  /**
   * \defgroup constructors
   */
  /** @{ */
  MapHandler(const MapHandler & other) = delete;
  MapHandler & operator=(const MapHandler & other) = delete;

  /**
   * @brief move construct while keeping invariance
   */
  MapHandler(MapHandler && other) = default;

  /**
   * @brief move construct while keeping invariance
   */
  MapHandler & operator=(MapHandler && other) = default;

  ~MapHandler() = default;

  lanelet::LaneletMapConstPtr lanelet_map_ptr() const { return lanelet_map_ptr_; }

  lanelet::routing::RoutingGraphConstPtr routing_graph_ptr() const { return routing_graph_ptr_; }

  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr() const { return traffic_rules_ptr_; }

  /**
   * @brief create MapHandler
   * @param map_msg raw message
   */
  static std::optional<MapHandler> create(const autoware_map_msgs::msg::LaneletMapBin & map_msg);
  /*@}*/

  /**
   * \defgroup adjacent lane getter considering VRU lanes
   */
  /** @{ */
  /**
   * @brief get the left adjacent and same_direction lanelet on the routing graph if exists
   * regardless of lane change permission
   * @param [in] lanelet input lanelet
   * @param [in] take_sibling if true, sibling lanelet of `lanelet` is searched
   * @param [in] extra_vru if provided, VRU lanes are also searched
   * @return optional of left adjacent lanelet(nullopt if there is no such adjacent lanelet)
   */
  std::optional<lanelet::ConstLanelet> left_lanelet(
    const lanelet::ConstLanelet & lanelet, const bool take_sibling, const ExtraVRU extra_vru) const;

  /**
   * @brief get the right adjacent and same_direction lanelet on the routing graph if exists
   * @param [in] lanelet input lanelet
   * @param [in] take_sibling if true, sibling lanelet of `lanelet` is searched
   * @param [in] extra_vru if provided, VRU lanes are also searched
   * @return optional of right adjacent lanelet(nullopt if there is no such adjacent lanelet)
   */
  std::optional<lanelet::ConstLanelet> right_lanelet(
    const lanelet::ConstLanelet & lanelet, const bool take_sibling, const ExtraVRU extra_vru) const;

  /**
   * @brief get the leftmost same_direction lanelet if exists
   * @param [in] lanelet input lanelet
   * @param [in] take_sibling if true, sibling lanelet of `lanelet` is searched
   * @param [in] extra_vru if provided, VRU lanes are also searched
   * @return optional of such lanelet(nullopt if there is no such adjacent lanelet)
   */
  std::optional<lanelet::ConstLanelet> leftmost_lanelet(
    const lanelet::ConstLanelet & lanelet, const bool take_sibling, const ExtraVRU extra_vru) const;

  /**
   * @brief get the rightmost same_direction lanelet if exists
   * @param [in] lanelet input lanelet
   * @param [in] take_sibling if true, sibling lanelet of `lanelet` is searched
   * @param [in] extra_vru if provided, VRU lanes are also searched
   * @return optional of such lanelet(nullopt if there is no such adjacent lanelet)
   */
  std::optional<lanelet::ConstLanelet> rightmost_lanelet(
    const lanelet::ConstLanelet & lanelet, const bool take_sibling, const ExtraVRU extra_vru) const;

  /**
   * @brief get the left lanelets which are adjacent to `lanelet`
   * @param [in] lanelet input lanelet
   * @param [in] include_opposite flag if opposite_direction lanelet is included
   * @param [in] invert_opposite_lanelet flag if the opposite lanelet in the output is `.inverted()`
   * or not
   * @return the list of lanelets excluding `lanelet` which is ordered in the *hopping* number from
   * `lanelet`
   */
  lanelet::ConstLanelets left_lanelets(
    const lanelet::ConstLanelet & lanelet, const bool include_opposite = false,
    const bool invert_opposite_lane = false) const;

  /**
   * @brief get the right lanelets which are adjacent to `lanelet`
   * @param [in] lanelet input lanelet
   * @param [in] include_opposite flag if opposite_direction lanelet is included
   * @param [in] invert_opposite_lanelet flag if the opposite lanelet in the output is `.inverted()`
   * or not
   * @return the list of lanelets excluding `lanelet` which is ordered in the *hopping* number from
   * `lanelet`
   */
  lanelet::ConstLanelets right_lanelets(
    const lanelet::ConstLanelet & lanelet, const bool include_opposite = false,
    const bool invert_opposite_lane = false) const;

  /*@}*/

  /**
   * \defgroup VRU related operations
   */
  /** @{ */

  /**
   * @brief retrieve lanelets behind/from of `lanelet`, in the order from backward, `lanelet`, to
   * forward
   */
  lanelet::ConstLanelets get_shoulder_lanelet_sequence(
    const lanelet::ConstLanelet & lanelet,
    const double backward_distance = std::numeric_limits<double>::max(),
    const double forward_distance = std::numeric_limits<double>::max()) const;

  std::optional<lanelet::ConstLanelet> left_shoulder_lanelet(
    const lanelet::ConstLanelet & lanelet) const;

  std::optional<lanelet::ConstLanelet> right_shoulder_lanelet(
    const lanelet::ConstLanelet & lanelet) const;

  std::optional<lanelet::ConstLanelet> left_bicycle_lanelet(
    const lanelet::ConstLanelet & lanelet) const;

  std::optional<lanelet::ConstLanelet> right_bicycle_lanelet(
    const lanelet::ConstLanelet & lanelet) const;
  /*@}*/

protected:
  MapHandler(
    lanelet::LaneletMapConstPtr lanelet_map_ptr,
    lanelet::routing::RoutingGraphConstPtr routing_graph_ptr,
    lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr);

  lanelet::LaneletMapConstPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphConstPtr routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
};

}  // namespace autoware::experimental::lanelet2_utils

#endif  // AUTOWARE__LANELET2_UTILS__MAP_HANDLER_HPP_
