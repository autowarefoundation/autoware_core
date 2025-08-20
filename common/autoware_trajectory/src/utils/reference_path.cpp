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

#include "autoware/trajectory/utils/reference_path.hpp"

#include "autoware/trajectory/threshold.hpp"
#include "autoware/trajectory/utils/pretty_build.hpp"

#include <autoware/lanelet2_utils/topology.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <range/v3/all.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/LaneletSequence.h>

#include <algorithm>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware::experimental::trajectory
{
namespace
{
using Waypoints = std::vector<Waypoint>;
using WaypointsWithInterval = ElementWithInterval<Waypoints>;

template <typename T>
using ElementsWithDistance = std::vector<ElementWithDistance<T>>;

/**
 * @brief get user defined waypoints from lanelet
 * @param lanelet lanelet to get waypoints from
 * @param lanelet_map lanelet map
 * @return user defined waypoints as vector of pairs of point and lanelet id
 */
std::optional<Waypoints> get_user_defined_waypoint(
  const lanelet::ConstLanelet & lanelet, const lanelet::LaneletMapConstPtr & lanelet_map)
{
  if (!lanelet.hasAttribute("waypoints")) {
    return std::nullopt;
  }
  const auto waypoints_id = lanelet.attribute("waypoints").asId().value();
  const auto & waypoints_linestring = lanelet_map->lineStringLayer.get(waypoints_id);

  return waypoints_linestring |
         ranges::views::transform(
           [&lanelet](const lanelet::ConstPoint3d & p) { return Waypoint{p, lanelet.id()}; }) |
         ranges::to<std::vector>();
}

/**
 * @brief compute interval for waypoints to smoothly connect to native centerline
 * @param waypoints waypoints to compute interval for
 * @param defined_lanelet_with_acc_dist lanelet on which waypoints are defined, with accumulated
 * distance
 * @param connection_gradient_from_centerline gradient for connecting centerline and user
 * defined waypoints
 * @return interval in s coordinate
 */
Interval compute_smooth_interval(
  const Waypoints & waypoints, const LaneletWithDistance & defined_lanelet_with_acc_dist,
  const double connection_gradient_from_centerline)
{
  const auto front_arc_coords = lanelet::geometry::toArcCoordinates(
    defined_lanelet_with_acc_dist.element.centerline2d(), waypoints.front().point.basicPoint2d());
  const auto start = defined_lanelet_with_acc_dist.distance + front_arc_coords.length -
                     connection_gradient_from_centerline * std::abs(front_arc_coords.distance);
  const auto back_arc_coords = lanelet::geometry::toArcCoordinates(
    defined_lanelet_with_acc_dist.element.centerline2d(), waypoints.back().point.basicPoint2d());
  const auto end = defined_lanelet_with_acc_dist.distance + back_arc_coords.length +
                   connection_gradient_from_centerline * std::abs(back_arc_coords.distance);
  return {start, end};
}

/**
 * @brief get border point between adjacent lanelets
 * @param segment_across_border segment across border
 * @param next_lanelet next lanelet
 * @return border point (std::nullopt if not found)
 */
std::optional<lanelet::ConstPoint3d> get_border_point(
  const lanelet::BasicLineString3d & segment_across_border,
  const lanelet::ConstLanelet & next_lanelet)
{
  const lanelet::BasicLineString2d border{
    next_lanelet.leftBound().front().basicPoint2d(),
    next_lanelet.rightBound().front().basicPoint2d()};

  lanelet::BasicPoints2d basic_border_point;
  boost::geometry::intersection(
    lanelet::utils::to2D(segment_across_border), border, basic_border_point);
  if (basic_border_point.empty()) {
    return std::nullopt;
  }

  lanelet::Point3d border_point;
  border_point.basicPoint() << basic_border_point.front(), segment_across_border.front().z();
  return lanelet::utils::toConst(border_point);
}

/**
 * @brief append or merge waypoints to waypoint chunks depending on distance to last chunk
 * @param waypoint_chunks waypoint chunks
 * @param waypoints new waypoints
 * @param defined_lanelet_with_acc_dist lanelet on which waypoints are defined, with accumulated
 * distance
 * @param lanelet_map lanelet map
 * @param connection_gradient_from_centerline gradient for connecting centerline and user
 * defined waypoints
 */
void append_or_merge_waypoints_to_chunks(
  std::vector<WaypointsWithInterval> & waypoint_chunks, const Waypoints & waypoints,
  const LaneletWithDistance & defined_lanelet_with_acc_dist,
  const lanelet::LaneletMapConstPtr & lanelet_map, const double connection_gradient_from_centerline)
{
  if (waypoints.empty()) {
    return;
  }

  const auto interval = compute_smooth_interval(
    waypoints, defined_lanelet_with_acc_dist, connection_gradient_from_centerline);
  if (waypoint_chunks.empty() || interval.start > waypoint_chunks.back().interval.end) {
    // current waypoint chunk is not within interval of last chunk, thus create a new chunk
    waypoint_chunks.push_back({waypoints, interval});
    return;
  }

  if (waypoint_chunks.back().element.empty()) {
    return;
  }
  auto last_waypoint_chunk = std::move(waypoint_chunks.back());
  waypoint_chunks.pop_back();

  // remove points that do not belong to their defined lanelet from last_waypoint_chunk
  const auto last_waypoint_defined_lanelet =
    lanelet_map->laneletLayer.get(last_waypoint_chunk.element.front().id);
  last_waypoint_chunk.element.erase(
    std::remove_if(
      last_waypoint_chunk.element.begin(), last_waypoint_chunk.element.end(),
      [&](const Waypoint & waypoint) {
        return !lanelet::geometry::inside(
          last_waypoint_defined_lanelet, waypoint.point.basicPoint2d());
      }),
    last_waypoint_chunk.element.end());
  if (last_waypoint_chunk.element.empty()) {
    return;
  }

  // append border point before merging
  const lanelet::BasicLineString3d segment_across_border{
    last_waypoint_chunk.element.back().point.basicPoint(), waypoints.front().point.basicPoint()};
  if (
    const auto border_point =
      get_border_point(segment_across_border, defined_lanelet_with_acc_dist.element)) {
    if (!is_almost_same(last_waypoint_chunk.element.back().point, *border_point)) {
      last_waypoint_chunk.element.emplace_back(
        *border_point, last_waypoint_chunk.element.back().id);
    }
    last_waypoint_chunk.element.back().next_id = waypoints.front().id;
  }

  // merge waypoints into last_waypoint_chunk
  last_waypoint_chunk.element.insert(
    last_waypoint_chunk.element.end(), waypoints.begin(), waypoints.end());
  const auto [_, end] = compute_smooth_interval(
    waypoints, defined_lanelet_with_acc_dist, connection_gradient_from_centerline);
  waypoint_chunks.push_back(
    {last_waypoint_chunk.element, {last_waypoint_chunk.interval.start, end}});
}

/**
 * @brief merge native centerline and user defined waypoints into one vector of waypoints
 * @param lanelet_sequence consecutive lanelet sequence
 * @param waypoint_chunks user-defined waypoint chunks on lanelet sequence
 * @param path_interval interval of path in s coordinate
 * @return waypoints with user defined waypoints and native centerline points merged
 */
Waypoints merge_native_centerline_and_user_defined_waypoints(
  const LaneletsWithDistance & lanelet_sequence_with_acc_dist,
  const std::vector<WaypointsWithInterval> & user_defined_waypoint_chunks,
  const Interval & path_interval)
{
  // Basically, each native_point in centerline of original lanelet_sequence is added to
  // merged_waypoints, but if native_s is within interval of target_waypoint_chunk, waypoints in
  // that chunk are added instead.

  Waypoints merged_waypoints;
  const auto add_waypoints = [&](const Waypoints & waypoints) {
    for (const auto & waypoint : waypoints) {
      if (!merged_waypoints.empty()) {
        const auto & prev_waypoint = merged_waypoints.back();
        if (is_almost_same(prev_waypoint.point, waypoint.point)) {
          continue;
        }
      }
      merged_waypoints.push_back(waypoint);
    }
  };

  // flag to indicate that native_s is still within interval of target_chunk,
  // which is important if target_chunk "steps over" to next lanelet
  auto is_in_chunk = false;
  auto target_chunk_it = user_defined_waypoint_chunks.begin();

  for (auto lanelet_it = lanelet_sequence_with_acc_dist.begin();
       lanelet_it != lanelet_sequence_with_acc_dist.end(); ++lanelet_it) {
    const auto & [lanelet, lanelet_s] = *lanelet_it;
    const auto & centerline = lanelet.centerline();
    const auto lanelet_id = lanelet.id();

    for (auto [native_point_it, native_s] = std::make_tuple(centerline.begin(), lanelet_s);
         native_point_it != centerline.end(); ++native_point_it) {
      const auto prev_native_s = native_s;
      if (native_point_it != centerline.begin()) {
        native_s += lanelet::geometry::distance3d(*std::prev(native_point_it), *native_point_it);
      }

      if (
        target_chunk_it == user_defined_waypoint_chunks.end() ||
        (native_s < target_chunk_it->interval.start && target_chunk_it->interval.end <= native_s)) {
        // native_s is out of interval of target_chunk
        if (is_in_chunk) {
          // native_s has passed interval of target_chunk, so set target_chunk to next one
          is_in_chunk = false;
          if (target_chunk_it != user_defined_waypoint_chunks.end()) {
            target_chunk_it++;
          }
        }

        if (prev_native_s < path_interval.start && path_interval.start <= native_s) {
          // native_s has passed start of path_interval, so add previous waypoint for
          // interpolation
          if (native_point_it != centerline.begin()) {
            add_waypoints({{*std::prev(native_point_it), lanelet_id}});

          } else if (lanelet_it != lanelet_sequence_with_acc_dist.begin()) {
            add_waypoints(
              {{std::prev(lanelet_it)->element.centerline().back(),
                std::prev(lanelet_it)->element.id()}});
          }
        }

        // add native_point in centerline
        add_waypoints({{*native_point_it, lanelet_id}});

        if (native_s > path_interval.end) {
          break;
        }
        continue;
      }

      // native_s is within interval of target_chunk
      if (is_in_chunk) {
        if (native_s > path_interval.end) {
          break;
        }
        continue;
      }

      // native_s has just entered interval of target_chunk
      is_in_chunk = true;

      if (target_chunk_it->element.front().id != lanelet_id && !merged_waypoints.empty()) {
        // waypoints in target_chunk belong to next lanelet, so add border point
        const auto next_lanelet_it = std::find_if(
          std::next(lanelet_it), lanelet_sequence_with_acc_dist.end(),
          [&target_chunk_it](const LaneletWithDistance & lanelet) {
            return lanelet.element.id() == target_chunk_it->element.front().id;
          });
        if (next_lanelet_it != lanelet_sequence_with_acc_dist.end()) {
          const lanelet::BasicLineString3d segment_across_border{
            merged_waypoints.back().point.basicPoint(),
            target_chunk_it->element.front().point.basicPoint()};
          if (
            const auto border_point =
              get_border_point(segment_across_border, next_lanelet_it->element)) {
            if (!is_almost_same(merged_waypoints.back().point, *border_point)) {
              merged_waypoints.emplace_back(*border_point, lanelet_id);
            }
            merged_waypoints.back().next_id = next_lanelet_it->element.id();
          }
        }
      }

      // add waypoints in target_chunk
      add_waypoints(target_chunk_it->element);
    }

    if (is_in_chunk || lanelet_it == std::prev(lanelet_sequence_with_acc_dist.end())) {
      continue;
    }

    // add border point
    const auto & next_lanelet = std::next(lanelet_it)->element;
    const lanelet::BasicLineString3d segment_across_border{
      centerline.back().basicPoint(), next_lanelet.centerline().front().basicPoint()};
    if (const auto border_point = get_border_point(segment_across_border, next_lanelet)) {
      if (
        merged_waypoints.empty() || !is_almost_same(merged_waypoints.back().point, *border_point)) {
        merged_waypoints.emplace_back(*border_point, lanelet_id);
      }
      merged_waypoints.back().next_id = next_lanelet.id();
    }
  }

  return merged_waypoints;
}
}  // namespace

LaneletsWithDistance zip_accumulated_distance(const lanelet::ConstLanelets & lanelet_sequence)
{
  LaneletsWithDistance lanelet_sequence_with_acc_dist;
  auto acc_dist = 0.0;
  for (const auto & lanelet : lanelet_sequence) {
    lanelet_sequence_with_acc_dist.push_back({lanelet, acc_dist});
    acc_dist += lanelet::geometry::length3d(lanelet);
  }
  return lanelet_sequence_with_acc_dist;
}

std::optional<double> get_position_in_lanelet_sequence(
  const LaneletsWithDistance & lanelet_sequence_with_acc_dist, const Waypoint & waypoint)
{
  const auto waypoint_lanelet = std::find_if(
    lanelet_sequence_with_acc_dist.begin(), lanelet_sequence_with_acc_dist.end(),
    [&waypoint](const LaneletWithDistance & lanelet) {
      return lanelet.element.id() == waypoint.id;
    });
  if (waypoint_lanelet == lanelet_sequence_with_acc_dist.end()) {
    // waypoint is not on lanelet sequence
    return std::nullopt;
  }

  return waypoint_lanelet->distance +
         lanelet::geometry::toArcCoordinates(
           waypoint_lanelet->element.centerline2d(), waypoint.point.basicPoint2d())
           .length;
};

LaneletSequenceWithInterval extend_lanelet_sequence(
  const lanelet::ConstLanelets & lanelet_sequence,
  const lanelet::routing::RoutingGraphConstPtr routing_graph, const Interval & interval)
{
  auto new_lanelet_sequence = lanelet_sequence;
  auto new_length = lanelet::geometry::length3d(lanelet::LaneletSequence(new_lanelet_sequence));
  auto new_s_start = interval.start;
  auto new_s_end = interval.end;

  for (std::set<lanelet::Id> visited_lanelet_ids = {new_lanelet_sequence.front().id()};
       new_s_start < 0.0;) {
    const auto prev_lanelets = autoware::experimental::lanelet2_utils::previous_lanelets(
      new_lanelet_sequence.front(), routing_graph);
    if (prev_lanelets.empty()) {
      new_s_start = 0.0;
      break;
    }

    // take the longest previous lanelet for extension
    const auto & new_lanelet = *std::max_element(
      prev_lanelets.begin(), prev_lanelets.end(),
      [](const lanelet::ConstLanelet & a, const lanelet::ConstLanelet & b) {
        return lanelet::geometry::length3d(a) < lanelet::geometry::length3d(b);
      });
    if (visited_lanelet_ids.find(new_lanelet.id()) != visited_lanelet_ids.end()) {
      // loop detected
      break;
    }

    visited_lanelet_ids.insert(new_lanelet.id());
    new_lanelet_sequence.insert(new_lanelet_sequence.begin(), new_lanelet);
    const auto extended_length = lanelet::geometry::length3d(new_lanelet);
    new_length += extended_length;
    new_s_start += extended_length;
    new_s_end += extended_length;
  }

  for (std::set<lanelet::Id> visited_lanelet_ids = {new_lanelet_sequence.back().id()};
       new_s_end > new_length;) {
    const auto next_lanelets = autoware::experimental::lanelet2_utils::following_lanelets(
      new_lanelet_sequence.back(), routing_graph);
    if (next_lanelets.empty()) {
      new_s_end = new_length;
      break;
    }

    // take the longest next lanelet for extension
    const auto new_lanelet = *std::max_element(
      next_lanelets.begin(), next_lanelets.end(),
      [](const lanelet::ConstLanelet & a, const lanelet::ConstLanelet & b) {
        return lanelet::geometry::length3d(a) < lanelet::geometry::length3d(b);
      });
    if (visited_lanelet_ids.find(new_lanelet.id()) != visited_lanelet_ids.end()) {
      // loop detected
      break;
    }

    visited_lanelet_ids.insert(new_lanelet.id());
    new_lanelet_sequence.push_back(new_lanelet);
    new_length += lanelet::geometry::length3d(new_lanelet);
  }

  return {new_lanelet_sequence, {new_s_start, new_s_end}};
}

std::optional<Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>
build_reference_path(
  const LaneletSequenceWithInterval & lanelet_sequence_with_interval,
  const lanelet::LaneletMapConstPtr lanelet_map,
  const lanelet::traffic_rules::TrafficRulesPtr traffic_rules,
  const double waypoint_connection_gradient_from_centerline)
{
  const auto & lanelet_sequence = lanelet_sequence_with_interval.element;
  const auto & interval = lanelet_sequence_with_interval.interval;

  const auto lanelet_sequence_with_acc_dist = zip_accumulated_distance(lanelet_sequence);

  std::vector<WaypointsWithInterval> user_defined_waypoint_chunks;
  for (const auto & lanelet_with_acc_dist : lanelet_sequence_with_acc_dist) {
    if (
      const auto user_defined_waypoint =
        get_user_defined_waypoint(lanelet_with_acc_dist.element, lanelet_map)) {
      append_or_merge_waypoints_to_chunks(
        user_defined_waypoint_chunks, *user_defined_waypoint, lanelet_with_acc_dist, lanelet_map,
        waypoint_connection_gradient_from_centerline);
    }
  }

  auto path_points = merge_native_centerline_and_user_defined_waypoints(
    lanelet_sequence_with_acc_dist, user_defined_waypoint_chunks, interval);

  std::sort(path_points.begin(), path_points.end(), [&](const Waypoint & a, const Waypoint & b) {
    return get_position_in_lanelet_sequence(lanelet_sequence_with_acc_dist, a) <
           get_position_in_lanelet_sequence(lanelet_sequence_with_acc_dist, b);
  });

  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
    path_points_with_lane_ids =
      path_points | ranges::views::transform([&](const Waypoint & waypoint) {
        autoware_internal_planning_msgs::msg::PathPointWithLaneId point;
        // position
        point.point.pose.position = lanelet::utils::conversion::toGeomMsgPt(waypoint.point);
        // longitudinal_velocity
        point.point.longitudinal_velocity_mps =
          traffic_rules->speedLimit(lanelet_map->laneletLayer.get(waypoint.id)).speedLimit.value();
        // lane_ids
        point.lane_ids.push_back(waypoint.id);
        if (waypoint.next_id) {
          point.lane_ids.push_back(*waypoint.next_id);
        }
        return point;
      }) |
      ranges::to<std::vector>();

  if (auto trajectory = pretty_build(path_points_with_lane_ids)) {
    trajectory->align_orientation_with_trajectory_direction();
    return trajectory;
  }
  return std::nullopt;
}
}  // namespace autoware::experimental::trajectory
