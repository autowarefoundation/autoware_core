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
#include <autoware/lanelet2_utils/map_handler.hpp>
#include <autoware/lanelet2_utils/nn_search.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <optional>
#include <unordered_map>

namespace autoware::experimental::lanelet2_utils
{

/**
 * @brief This class handles the driving lane tracking and utility functions for manipulating
 * Lanelet on the route
 * @invariant current_pose IS ASSURED not to be necessarily inside current_lanelet, but to be
 * closest the current_lanelet, especially when ego is avoiding a parked vehicle
 */
class RouteManager : public MapHandler
{
public:
  /**
   * \defgroup constructors
   */
  /** @{ */
  RouteManager(const RouteManager & other) = delete;
  RouteManager & operator=(const RouteManager & other) = delete;

  /**
   * @brief move construct while keeping invariance
   */
  RouteManager(RouteManager && other) = default;

  /**
   * @brief move construct while keeping invariance
   */
  RouteManager & operator=(RouteManager && other) = default;

  virtual ~RouteManager() = default;

  /**
   * @brief create RouteManager
   * @param map_msg raw message
   * @param route_msg raw message
   * @param initial_pose initial_pose
   * @post current_lanelet is determined, but IT IS NOT ASSURED that current_pose is inside
   * current_lanelet.
   */
  static std::optional<RouteManager> create(
    const autoware_map_msgs::msg::LaneletMapBin & map_msg,
    const autoware_planning_msgs::msg::LaneletRoute & route_msg,
    const geometry_msgs::msg::Pose & initial_pose);
  /*@}*/

  /**
   * \defgroup current_route_lanelet related operations
   */
  /** @{ */
  /**
   * @brief update current_pose only within forward/backward route lanelets, and return a new
   * RouteManager if current_pose is valid
   * @param dist_threshold_soft soft constraint for distance from lanelet
   * @param yaw_threshold_soft soft constraint for orientation from lanelet
   * @param search_window [opt, 100] forward/backward length for search
   * @note this function updates current_route_lanelet along its LaneSequence. even if ego is
   * executing avoidance and deviates from reference path, this function tracks
   * current_route_lanelet without considering lane change
   * @attention this can only be called against std::move(*this).
   */
  std::optional<RouteManager> update_current_pose(
    const geometry_msgs::msg::Pose & current_pose, const double dist_threshold_soft,
    const double yaw_threshold_soft, const double search_window = 100) &&;

  /**
   * @brief update current_pose against all route lanelets and return a new RouteManager if
   * current_pose is valid
   * @param search_window [opt, 25] forward/backward length for search
   * @note this functions is aimed to be used only when lane change has just completed
   * @attention this can only be called against std::move(*this)
   */
  std::optional<RouteManager> commit_lane_change_success(
    const geometry_msgs::msg::Pose & current_pose) &&;

  /**
   * @brief return the lanelet to which current_pose is closest among route_lanelets. IT IS NOT
   * ASSURED that current_pose is inside current_lanelet.
   */
  const lanelet::ConstLanelet & current_lanelet() const { return current_lanelet_; }

  /**
   * @brief from current_position, return the list of lanelets from tail(behind of ego) to head at
   * least for given backward/forward length, within the designated route
   * @param forward_length forward length from current_position
   * @param backward_length backward(behind) length from current_position
   * @return LaneSequence
   * @post the output contains at least one lanelet, which is current_lanelet
   */
  LaneSequence get_lanelet_sequence_on_route(
    const double forward_length, const double backward_length) const;

  /**
   * @brief from current_position, return the list of lanelets from tail(behind of ego) to head at
   * least for given backward/forward length, querying lanelets outside of the route as well
   * @param forward_length forward length from current_position
   * @param backward_length backward(behind) length from current_position
   * @note it is undefined which lanelet is chosen in querying lanelet out of route
   * @return LaneSequence
   * @post the output contains at least one lanelet, which is current_lanelet
   */
  LaneSequence get_lanelet_sequence_outward_route(
    const double forward_length, const double backward_length) const;
  /*@}*/

  std::optional<lanelet::ConstLanelet> get_closest_preferred_route_lanelet(
    const geometry_msgs::msg::Pose & search_pose) const;

  std::optional<lanelet::ConstLanelet> get_closest_route_lanelet_within_constraints(
    const geometry_msgs::msg::Pose & search_pose, const double dist_threshold,
    const double yaw_threshold) const;

private:
  RouteManager(
    lanelet::LaneletMapConstPtr lanelet_map_ptr,
    lanelet::routing::RoutingGraphConstPtr routing_graph_ptr,
    lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr,
    lanelet::ConstLanelets && all_route_lanelets,
    std::unordered_map<lanelet::Id, double> && all_route_length_cache,
    lanelet::ConstLanelets && preferred_lanelets, const lanelet::ConstLanelet & start_lanelet,
    const lanelet::ConstLanelet & goal_lanelet, const geometry_msgs::msg::Pose & current_pose,
    const lanelet::ConstLanelet & current_lanelet, const lanelet::LaneletSubmapPtr route_submap_ptr,
    const lanelet::routing::RoutingGraphPtr route_subgraph_ptr);

  lanelet::ConstLanelets all_route_lanelets_;  //<! all route lanelets, the order is not defined
  std::unordered_map<lanelet::Id, double> all_route_length_cache_{};
  LaneletRTree route_rtree_;

  lanelet::ConstLanelets preferred_lanelets_;
  LaneletRTree preferred_route_rtree_;

  lanelet::ConstLanelet start_lanelet_;
  lanelet::ConstLanelet goal_lanelet_;

  geometry_msgs::msg::Pose current_pose_;
  lanelet::ConstLanelet current_lanelet_;

  lanelet::LaneletSubmapConstPtr route_submap_ptr_;
  lanelet::routing::RoutingGraphConstPtr route_subgraph_ptr_;
};

}  // namespace autoware::experimental::lanelet2_utils

#endif  // AUTOWARE__LANELET2_UTILS__ROUTE_MANAGER_HPP_
