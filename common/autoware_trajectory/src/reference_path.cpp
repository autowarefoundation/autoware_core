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

#include "autoware/trajectory/reference_path.hpp"

#include "autoware/trajectory/threshold.hpp"
#include "autoware/trajectory/utils/pretty_build.hpp"

#include <autoware/lanelet2_utils/topology.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <range/v3/all.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <set>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware::experimental::trajectory
{

using Waypoint = std::pair<lanelet::ConstPoint3d, lanelet::Id>;
using UserDefinedWaypoints = std::vector<Waypoint>;

struct WaypointsGroupChunk
{
  UserDefinedWaypoints points;  // the points merged from several lanelet UserDefinedWaypoints
                                // that are close to each other
  const double start_s;
  double end_s{};
};

static std::optional<UserDefinedWaypoints> get_user_defined_waypoint(
  const lanelet::ConstLanelet & lanelet, const lanelet::LaneletMapConstPtr & lanelet_map)
{
  if (!lanelet.hasAttribute("waypoints")) {
    return std::nullopt;
  }
  const auto waypoints_id = lanelet.attribute("waypoints").asId().value();
  const auto & waypoints_linestring = lanelet_map->lineStringLayer.get(waypoints_id);

  return waypoints_linestring | ranges::views::transform([&lanelet](const auto & p) {
           return std::make_pair(p, lanelet.id());
         }) |
         ranges::to<std::vector>();
}

/**
 * @brief extend backward/forward the given lanelet_sequence so that s_start, s_end is within
 * the output lanelet sequence
 * @post s_start >= 0.0, s_end <= lanelet::geometry::length3d(lanelet_sequence)
 */
static std::tuple<lanelet::ConstLanelets, double, double> supplement_lanelet_sequence(

  const lanelet::routing::RoutingGraphConstPtr routing_graph,
  lanelet::ConstLanelets lanelet_sequence, double s_start, double s_end)
{
  using autoware::experimental::lanelet2_utils::following_lanelets;
  using autoware::experimental::lanelet2_utils::previous_lanelets;

  std::set<lanelet::Id> visited_prev_lane_ids{lanelet_sequence.front().id()};
  while (s_start < 0.0) {
    const auto previous_lanes = previous_lanelets(lanelet_sequence.front(), routing_graph);
    if (previous_lanes.empty()) {
      s_start = 0.0;
      break;
    }
    // take the longest previous lane to construct underlying lanelets
    const auto longest_previous_lane = *std::max_element(
      previous_lanes.begin(), previous_lanes.end(), [](const auto & lane1, const auto & lane2) {
        return lanelet::geometry::length3d(lane1) < lanelet::geometry::length3d(lane2);
      });
    lanelet_sequence.insert(lanelet_sequence.begin(), longest_previous_lane);
    if (visited_prev_lane_ids.find(longest_previous_lane.id()) != visited_prev_lane_ids.end()) {
      // loop detected
      break;
    }
    visited_prev_lane_ids.insert(longest_previous_lane.id());
    s_start += lanelet::geometry::length3d(longest_previous_lane);
  }

  std::set<lanelet::Id> visited_next_lane_ids{lanelet_sequence.back().id()};
  while (s_end > lanelet::geometry::length3d(lanelet::LaneletSequence(lanelet_sequence))) {
    const auto next_lanes = following_lanelets(lanelet_sequence.back(), routing_graph);
    if (next_lanes.empty()) {
      s_end = lanelet::geometry::length3d(lanelet::LaneletSequence(lanelet_sequence));
      break;
    }
    // take the longest previous lane to construct underlying lanelets
    const auto longest_next_lane = *std::max_element(
      next_lanes.begin(), next_lanes.end(), [](const auto & lane1, const auto & lane2) {
        return lanelet::geometry::length3d(lane1) < lanelet::geometry::length3d(lane2);
      });
    if (visited_next_lane_ids.find(longest_next_lane.id()) != visited_next_lane_ids.end()) {
      // loop detected
      break;
    }
    visited_next_lane_ids.insert(longest_next_lane.id());
    lanelet_sequence.push_back(longest_next_lane);
  }

  return {lanelet_sequence, s_start, s_end};
}

class UserDefinedWaypointsGroup
{
private:
  std::tuple<double, double> compute_smooth_interval(
    const UserDefinedWaypoints & points, const double current_lanelet_distance_from_route_start,
    const lanelet::ConstLanelet & defined_lanelet,
    const double connection_from_default_point_gradient)
  {
    const auto front_frenet_coords = lanelet::geometry::toArcCoordinates(
      defined_lanelet.centerline2d(), lanelet::utils::to2D(std::get<0>(points.front())));
    const double first =
      current_lanelet_distance_from_route_start + front_frenet_coords.length -
      connection_from_default_point_gradient * std::fabs(front_frenet_coords.distance);
    const auto back_frenet_coords = lanelet::geometry::toArcCoordinates(
      defined_lanelet.centerline2d(), lanelet::utils::to2D(std::get<0>(points.back())));
    const double second =
      current_lanelet_distance_from_route_start + back_frenet_coords.length +
      connection_from_default_point_gradient * std::fabs(back_frenet_coords.distance);
    return {first, second};
  }

  std::vector<WaypointsGroupChunk> waypoints_chunks_;

public:
  void add(
    UserDefinedWaypoints && user_defined_waypoints,
    const double current_lanelet_distance_from_route_start,
    const lanelet::ConstLanelet & defined_lanelet, const double group_separation_distance,
    const double connection_from_default_point_gradient)
  {
    if (waypoints_chunks_.empty()) {
      const auto [start, end] = compute_smooth_interval(
        user_defined_waypoints, current_lanelet_distance_from_route_start, defined_lanelet,
        connection_from_default_point_gradient);
      waypoints_chunks_.push_back({std::move(user_defined_waypoints), start, end});
      return;
    }

    if (const auto & last_waypoints = waypoints_chunks_.back().points;
        lanelet::geometry::distance3d(
          last_waypoints.back().first, user_defined_waypoints.front().first) >
        group_separation_distance) {
      const auto [start, end] = compute_smooth_interval(
        user_defined_waypoints, current_lanelet_distance_from_route_start, defined_lanelet,
        connection_from_default_point_gradient);
      waypoints_chunks_.push_back({std::move(user_defined_waypoints), start, end});
      return;
    }

    /*
      '+' are ConstPoint3d

      ----> lane direction

      if last_waypoints and next_waypoints are close, next_waypoints are merged into last_waypoints

      >+--+--+--+--+ last_waypoints --+--+--+--+--+> >+--+--+--+--+ next_waypoints --+--+--+--+--+>
     */
    auto & last_waypoints_mut = waypoints_chunks_.back().points;
    last_waypoints_mut.insert(
      last_waypoints_mut.end(), user_defined_waypoints.begin(), user_defined_waypoints.end());
    const auto [_, new_end] = compute_smooth_interval(
      user_defined_waypoints, current_lanelet_distance_from_route_start, defined_lanelet,
      connection_from_default_point_gradient);
    waypoints_chunks_.back().end_s = new_end;
  }

  std::vector<WaypointsGroupChunk> waypoints_chunks() { return std::move(waypoints_chunks_); }
};

static std::vector<Waypoint> consolidate_user_defined_waypoints_and_native_centerline(
  const std::vector<WaypointsGroupChunk> & waypoints_chunks,
  const lanelet::ConstLanelets & lanelet_sequence, const double s_start, const double s_end)
{
  // reference path generation algorithm
  //
  // Basically, each native_point in centerline of original lanelet_sequence is added to
  // reference_lanelet_points, but if a native_point is within the interval of
  // current_overlapped_chunk_iter, the waypoints on that chunk are added instead.
  // s_start, and s_end are measured by the accumulated distance of native points in
  // lanelet_sequence
  std::vector<Waypoint> reference_lanelet_points;
  auto append_native_reference_point = [&](const auto & point, lanelet::Id id) {
    if (!reference_lanelet_points.empty()) {
      const auto & prev = reference_lanelet_points.back();
      if (is_almost_same(prev.first, point)) {
        return;
      }
    }
    reference_lanelet_points.emplace_back(point, id);
  };
  auto append_custom_waypoint = [&](const Waypoint & point) {
    if (!reference_lanelet_points.empty()) {
      const auto & prev = reference_lanelet_points.back();
      if (is_almost_same(prev.first, point.first)) {
        return;
      }
    }
    reference_lanelet_points.emplace_back(point);
  };
  auto current_overlapped_chunk_iter = waypoints_chunks.begin();

  const auto lanelet_with_acc_dist_sequence = [&]() {
    std::vector<std::pair<lanelet::ConstLanelet, double>> ll_with_dist;
    double acc_dist = 0.0;
    for (const auto & lanelet : lanelet_sequence) {
      ll_with_dist.emplace_back(lanelet, acc_dist);
      acc_dist += lanelet::geometry::length3d(lanelet);
    }
    return ll_with_dist;
  }();

  // flag to indicate that native point iteration has not passed current_overlapped_chunk_iter yet,
  // which is important if current_overlapped_chunk_iter "steps over" to next lanelet
  auto is_overlapping = false;

  for (const auto & lanelet_with_dist : lanelet_with_acc_dist_sequence) {
    const auto & lanelet = lanelet_with_dist.first;
    const auto & centerline = lanelet.centerline();
    const auto this_lanelet_s = lanelet_with_dist.second;
    const auto this_lane_id = lanelet.id();

    bool terminated = false;
    for (auto [native_point_it, native_s] = std::make_tuple(centerline.begin(), this_lanelet_s);
         native_point_it != centerline.end(); native_point_it++) {
      const double prev_native_s = native_s;
      if (native_point_it != centerline.begin()) {
        native_s += lanelet::geometry::distance3d(*std::prev(native_point_it), *native_point_it);
      }

      if (native_s > s_end) {
        append_native_reference_point(*native_point_it, this_lane_id);
        terminated = true;
        break;
      }

      if (
        current_overlapped_chunk_iter == waypoints_chunks.end() ||
        native_s < current_overlapped_chunk_iter->start_s) {
        // native_point haven't reached a waypoint chunk yet
        is_overlapping = false;
        if (
          native_point_it != centerline.begin() && prev_native_s < s_start && s_start <= native_s) {
          // there is a gap between prev_native_it and native_it, so the position at `s_start`
          // needs to be interpolated
          append_native_reference_point(*std::prev(native_point_it), this_lane_id);
        }
        append_native_reference_point(*native_point_it, this_lane_id);
      } else if (native_s <= current_overlapped_chunk_iter->end_s) {
        if (!is_overlapping) {
          // native_point_it reached a waypoint chunk for the first time
          is_overlapping = true;
          // so append all the points in current_overlapped_chunk_iter at once
          const auto & user_defined_points = current_overlapped_chunk_iter->points;
          for (const auto & user_defined_point : user_defined_points) {
            // trim the interval
            append_custom_waypoint(user_defined_point);
          }
        } else {
          // native_point_it is still inside current_overlapped_chunk_iter, so just skip
          // this for-loop can terminate with this block if this Lanelet centerline's last native
          // point is still in current_overlapped_chunk_iter, in which case native points in the
          // next lanelet may continue this block or go to else branch below
        }
      } else {
        current_overlapped_chunk_iter++;
        is_overlapping = false;
        append_native_reference_point(*native_point_it, this_lane_id);
      }
    }
    if (terminated) {
      break;
    }
  }

  // assert("all points in reference_lanelet_points are !is_almost_same")

  // fit reference_lanelet_points to [s_start, s_end]
  if (reference_lanelet_points.size() < 2) {
    return {};
  }
  const auto measure_point_s = [&](const Waypoint & waypoint) {
    const auto & point = waypoint.first;
    const auto & lane_id = waypoint.second;
    const auto [lanelet, acc_dist] = *std::find_if(
      lanelet_with_acc_dist_sequence.begin(), lanelet_with_acc_dist_sequence.end(),
      [&](const auto & ll) { return ll.first.id() == lane_id; });
    return acc_dist +
           lanelet::geometry::toArcCoordinates(lanelet.centerline2d(), point.basicPoint2d()).length;
  };
  {
    const auto p1 = reference_lanelet_points.at(0);
    const auto p2 = reference_lanelet_points.at(1);
    const auto s_p1 = measure_point_s(p1);
    const auto s_p2 = measure_point_s(p2);
    if (s_p1 < s_start && s_start <= s_p2) {
      const auto interpolated_point =
        p1.first.basicPoint() +
        (p2.first.basicPoint() - p1.first.basicPoint()) * (s_start - s_p1) / (s_p2 - s_p1);
      reference_lanelet_points.at(0).first =
        lanelet::ConstPoint3d(lanelet::InvalId, interpolated_point);
    }
  }
  {
    const auto p1 = reference_lanelet_points.at(reference_lanelet_points.size() - 2);
    const auto p2 = reference_lanelet_points.back();
    const auto s_p1 = measure_point_s(p1);
    const auto s_p2 = measure_point_s(p2);
    if (s_end <= s_p1) {
      reference_lanelet_points.erase(std::prev(reference_lanelet_points.end()));
    } else if (s_end < s_p2) {
      // s_p1 < s_p2
      const auto interpolated_point =
        p1.first.basicPoint() +
        (p2.first.basicPoint() - p1.first.basicPoint()) * (s_end - s_p1) / (s_p2 - s_p1);
      reference_lanelet_points.back().first =
        lanelet::ConstPoint3d(lanelet::InvalId, interpolated_point);
    }
  }
  return reference_lanelet_points;
}

std::optional<Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>
build_reference_path(
  const lanelet::ConstLanelets & route_lanelets, const lanelet::ConstLanelet & current_lanelet,
  const geometry_msgs::msg::Pose & ego_pose, const lanelet::LaneletMapConstPtr lanelet_map,
  const lanelet::routing::RoutingGraphConstPtr routing_graph,
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules, const double group_separation_distance,
  const double connection_from_default_point_gradient, const double forward_length,
  const double backward_length)
{
  lanelet::ConstLanelets route_lanelets_before_current;
  bool found_current = false;
  for (const auto & route_lanelet : route_lanelets) {
    if (route_lanelet.id() == current_lanelet.id()) {
      found_current = true;
      break;
    }
    route_lanelets_before_current.push_back(route_lanelet);
  }
  if (!found_current) {
    return std::nullopt;
  }
  const double route_length_before_current =
    route_lanelets_before_current.empty()
      ? 0.0
      : lanelet::geometry::length3d(lanelet::LaneletSequence(route_lanelets_before_current));

  const auto ego_s_current_route =
    route_length_before_current +
    lanelet::geometry::toArcCoordinates(
      lanelet::utils::to2D(current_lanelet.centerline()),
      lanelet::utils::to2D(lanelet::utils::conversion::toLaneletPoint(ego_pose.position)))
      .length;
  const auto [lanelet_sequence, s_start, s_end] = supplement_lanelet_sequence(
    routing_graph, route_lanelets, ego_s_current_route - backward_length,
    ego_s_current_route + forward_length);

  UserDefinedWaypointsGroup waypoint_group;
  for (auto [lanelet, acc_dist] = std::make_tuple(lanelet_sequence.begin(), 0.0);
       lanelet != lanelet_sequence.end(); ++lanelet) {
    if (lanelet != lanelet_sequence.begin()) {
      acc_dist += lanelet::geometry::length3d(*std::prev(lanelet));
    }
    if (auto user_defined_waypoint_opt = get_user_defined_waypoint(*lanelet, lanelet_map);
        user_defined_waypoint_opt) {
      waypoint_group.add(
        std::move(user_defined_waypoint_opt.value()), acc_dist, *lanelet, group_separation_distance,
        connection_from_default_point_gradient);
    }
  }
  const auto waypoints_chunks = waypoint_group.waypoints_chunks();

  const auto reference_lanelet_points = consolidate_user_defined_waypoints_and_native_centerline(
    waypoints_chunks, lanelet_sequence, s_start, s_end);

  const auto path_points_with_lane_ids =
    reference_lanelet_points | ranges::views::transform([&](const auto & waypoint) {
      autoware_internal_planning_msgs::msg::PathPointWithLaneId point;
      // position
      point.point.pose.position = lanelet::utils::conversion::toGeomMsgPt(waypoint.first);
      // longitudinal_velocity
      point.point.longitudinal_velocity_mps =
        traffic_rules->speedLimit(lanelet_map->laneletLayer.get(waypoint.second))
          .speedLimit.value();
      // lane_ids
      point.lane_ids.push_back(waypoint.second);
      return point;
    }) |
    ranges::to<std::vector>();

  if (auto trajectory = pretty_build(path_points_with_lane_ids); trajectory) {
    trajectory->align_orientation_with_trajectory_direction();
    return trajectory;
  }
  return std::nullopt;
}

}  // namespace autoware::experimental::trajectory
