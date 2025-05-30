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
  /**
   * @brief create RouteManager
   * @param map_msg raw message
   * @param route_msg raw message
   * @param initial_pose initial_pose
   * @pre route_msg has non-empty RouteSegment
   * @post current_lanelet is determined, but it is not assured that initial_pose is within
   * current_lanelet
   */
  static std::optional<RouteManager> create(
    const autoware_map_msgs::msg::LaneletMapBin & map_msg,
    const autoware_planning_msgs::msg::LaneletRoute & route_msg,
    const geometry_msgs::msg::Pose & initial_pose);

  /**
   * @brief update current_pose only within forward/backward route lanelets, and return a new
   * RouteManager if current_pose is valid
   * @note this function updates current_route_lanelet along its LaneSequence. even if ego is
   * executing avoidance and deviates from reference path, this function tracks
   * current_route_lanelet without considering lane change
   * @attention this can only be called against std::move(*this)
   */
  std::optional<RouteManager> update_current_pose(const geometry_msgs::msg::Pose & current_pose) &&;

  /**
   * @brief update current_pose against all route lanelets and return a new RouteManager if
   * current_pose is valid
   * @note this functions is aimed to be used only when lane change is completed
   * @attention this can only be called against std::move(*this)
   */
  std::optional<RouteManager> update_current_pose_with_lane_change(
    const geometry_msgs::msg::Pose & current_pose) &&;

  const lanelet::ConstLanelet & current_lanelet() const { return current_lanelet_; }

private:
  RouteManager(
    lanelet::LaneletMapConstPtr lanelet_map_ptr,
    lanelet::routing::RoutingGraphConstPtr routing_graph_ptr,
    lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr,
    lanelet::ConstLanelets && all_route_lanelets, lanelet::ConstLanelets && preferred_lanelets,
    const lanelet::ConstLanelet & start_lanelet, const lanelet::ConstLanelet & goal_lanelet,
    const geometry_msgs::msg::Pose & current_pose, lanelet::ConstLanelet current_lanelet);

  lanelet::LaneletMapConstPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphConstPtr routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;

  lanelet::ConstLanelets all_route_lanelets_;  //<! all route lanelets, the order is not defined
  lanelet::ConstLanelets preferred_lanelets_;
  lanelet::ConstLanelet start_lanelet_;
  lanelet::ConstLanelet goal_lanelet_;

  geometry_msgs::msg::Pose current_pose_;
  lanelet::ConstLanelet current_lanelet_;
};

}  // namespace autoware::experimental::lanelet2_utils

#endif  // AUTOWARE__LANELET2_UTILS__ROUTE_MANAGER_HPP_
