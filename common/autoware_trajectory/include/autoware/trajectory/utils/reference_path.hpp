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

#ifndef AUTOWARE__TRAJECTORY__UTILS__REFERENCE_PATH_HPP_
#define AUTOWARE__TRAJECTORY__UTILS__REFERENCE_PATH_HPP_

#include "autoware/trajectory/forward.hpp"
#include "autoware/trajectory/utils/find_intervals.hpp"

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_routing/Forward.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <optional>

namespace autoware::experimental::trajectory
{
template <typename T>
struct ElementWithInterval
{
  T element;
  const Interval interval;
};

template <typename T>
struct ElementWithDistance
{
  T element;
  double distance;
};

struct Waypoint
{
  lanelet::ConstPoint3d point;
  lanelet::Id id;
  std::optional<lanelet::Id> next_id{std::nullopt};  // this is for border point only

  // ctor definition to avoid setting next_id mistakenly
  Waypoint(const lanelet::ConstPoint3d & point, const lanelet::Id & id) : point(point), id(id) {};
};

using LaneletSequenceWithInterval = ElementWithInterval<lanelet::ConstLanelets>;

using LaneletWithDistance = ElementWithDistance<lanelet::ConstLanelet>;
using LaneletsWithDistance = std::vector<LaneletWithDistance>;

/**
 * @brief zip lanelet sequence with accumulated distance (i.e. generate set of pairs of lanelet and
 * accumulated distance)
 * @param lanelet_sequence consecutive lanelet sequence
 * @return lanelet sequence with accumulated distance
 */
LaneletsWithDistance zip_accumulated_distance(const lanelet::ConstLanelets & lanelet_sequence);

/**
 * @brief get position of waypoint in lanelet sequence in s coordinate
 * @param lanelet_sequence_with_acc_dist lanelet sequence with accumulated distance
 * @param waypoint waypoint
 * @return position of waypoint in s coordinate (std::nullopt if waypoint is not in lanelet
 * sequence)
 */
std::optional<double> get_position_in_lanelet_sequence(
  const LaneletsWithDistance & lanelet_sequence_with_acc_dist, const Waypoint & waypoint);

/**
 * @brief extend given lanelet sequence backward/forward so that given interval is within output
 * lanelets
 * @param lanelet_sequence original lanelet sequence
 * @param routing_graph routing graph
 * @param interval interval in s coordinate
 * @return extended lanelet sequence with new interval in s coordinate
 * @post interval.start >= 0, interval.end <= length of lanelet sequence
 */
LaneletSequenceWithInterval extend_lanelet_sequence(
  const lanelet::ConstLanelets & lanelet_sequence,
  const lanelet::routing::RoutingGraphConstPtr routing_graph, const Interval & interval);

/**
 * @brief build reference path that spans backward_length backward and forward_length forward from
 * ego in s coordinate
 * @param lanelet_sequence consecutive lanelet sequence, self-intersections are allowed
 * @param current_lanelet lanelet where ego is driving
 * @param ego_pose ego's current pose
 * @param lanelet_map lanelet map
 * @param routing_graph routing graph
 * @param traffic_rules traffic rules
 * @param forward_length forward length from ego
 * @param backward_length backward length from ego
 * @param waypoint_connection_gradient_from_centerline gradient for connecting centerline and user
 * defined waypoints
 * @return reference path with lane ids (std::nullopt if failed)
 * @note length of Trajectory may not match backward_length + forward_length
 */
std::optional<Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>
build_reference_path(
  const lanelet::ConstLanelets & lanelet_sequence, const lanelet::ConstLanelet & current_lanelet,
  const geometry_msgs::msg::Pose & ego_pose, const lanelet::LaneletMapConstPtr lanelet_map,
  const lanelet::routing::RoutingGraphConstPtr routing_graph,
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules, const double forward_length,
  const double backward_length, const double waypoint_connection_gradient_from_centerline,
  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> * debug_path_points =
    nullptr);
}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__UTILS__REFERENCE_PATH_HPP_
