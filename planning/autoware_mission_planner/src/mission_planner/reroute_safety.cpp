// Copyright 2025 The Autoware Contributors
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

#include "reroute_safety.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/lanelet2_utils/nn_search.hpp>
#include <rclcpp/logging.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>

#include <algorithm>
#include <functional>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::mission_planner
{

namespace
{
using autoware_planning_msgs::msg::LaneletPrimitive;
using autoware_planning_msgs::msg::LaneletRoute;

bool has_same_primitives(
  const std::vector<LaneletPrimitive> & original_primitives,
  const std::vector<LaneletPrimitive> & target_primitives)
{
  if (original_primitives.size() != target_primitives.size()) {
    return false;
  }

  for (const auto & primitive : original_primitives) {
    const auto has_same = [&](const auto & p) { return p.id == primitive.id; };
    const bool is_same =
      std::find_if(target_primitives.begin(), target_primitives.end(), has_same) !=
      target_primitives.end();
    if (!is_same) {
      return false;
    }
  }
  return true;
}

}  // namespace

bool check_reroute_safety(
  const LaneletRoute & original_route, const LaneletRoute & target_route,
  const lanelet::LaneletMapConstPtr & lanelet_map, const double current_velocity,
  const double reroute_time_threshold, const double minimum_reroute_length,
  const rclcpp::Logger & logger)
{
  if (original_route.segments.empty() || target_route.segments.empty() || !lanelet_map) {
    RCLCPP_ERROR(logger, "Check reroute safety failed. Route, map or odometry is not set.");
    return false;
  }

  // if vehicle is stopped, do not check safety
  if (current_velocity < 0.01) {
    return true;
  }

  // =============================================================================================
  // NOTE: the target route is calculated while ego is driving on the original route, so basically
  // the first lane of the target route should be in the original route lanelets. So the common
  // segment interval matches the beginning of the target route. The exception is that if ego is
  // on an intersection lanelet, getClosestLanelet() may not return the same lanelet which exists
  // in the original route. In that case the common segment interval does not match the beginning of
  // the target lanelet
  // =============================================================================================
  const auto start_idx_opt =
    std::invoke([&]() -> std::optional<std::pair<size_t /* original */, size_t /* target */>> {
      for (size_t i = 0; i < original_route.segments.size(); ++i) {
        const auto & original_segment = original_route.segments.at(i).primitives;
        for (size_t j = 0; j < target_route.segments.size(); ++j) {
          const auto & target_segment = target_route.segments.at(j).primitives;
          if (has_same_primitives(original_segment, target_segment)) {
            return std::make_pair(i, j);
          }
        }
      }
      return std::nullopt;
    });
  if (!start_idx_opt.has_value()) {
    RCLCPP_ERROR(logger, "Check reroute safety failed. Cannot find the start index of the route.");
    return false;
  }
  const auto [start_idx_original, start_idx_target] = start_idx_opt.value();

  // find last idx that matches the target primitives
  size_t end_idx_original = start_idx_original;
  size_t end_idx_target = start_idx_target;
  for (size_t i = 1; i < target_route.segments.size() - start_idx_target; ++i) {
    if (start_idx_original + i > original_route.segments.size() - 1) {
      break;
    }

    const auto & original_primitives =
      original_route.segments.at(start_idx_original + i).primitives;
    const auto & target_primitives = target_route.segments.at(start_idx_target + i).primitives;
    if (!has_same_primitives(original_primitives, target_primitives)) {
      break;
    }
    end_idx_original = start_idx_original + i;
    end_idx_target = start_idx_target + i;
  }

  // make sure that ego is within the reroute target
  const bool ego_is_on_first_target_section = std::any_of(
    target_route.segments.front().primitives.begin(),
    target_route.segments.front().primitives.end(), [&](const auto & primitive) {
      const auto lanelet = lanelet_map->laneletLayer.get(primitive.id);
      return autoware::experimental::lanelet2_utils::is_in_lanelet(
        target_route.start_pose, lanelet);
    });
  if (!ego_is_on_first_target_section) {
    RCLCPP_ERROR(
      logger, "Check reroute safety failed. Ego is not on the first section of target route.");
    return false;
  }

  // compute the remaining arc-length from the current pose to the end of the closest lanelet among
  // the primitives of the given original-route segment
  const auto current_pose = target_route.start_pose;
  const auto arc_length_to_lanelet_end =
    [&](const std::vector<LaneletPrimitive> & primitives) -> std::optional<double> {
    lanelet::ConstLanelets start_lanelets;
    for (const auto & primitive : primitives) {
      const auto lanelet = lanelet_map->laneletLayer.get(primitive.id);
      start_lanelets.push_back(lanelet);
    }
    // closest lanelet in start lanelets
    const auto closest_lanelet_opt =
      experimental::lanelet2_utils::get_closest_lanelet(start_lanelets, current_pose);
    if (!closest_lanelet_opt) {
      return std::nullopt;
    }
    const auto & closest_lanelet = closest_lanelet_opt.value();

    const auto & centerline_2d = lanelet::utils::to2D(closest_lanelet.centerline());
    const auto lanelet_point = experimental::lanelet2_utils::from_ros(current_pose.position);
    const auto arc_coordinates = lanelet::geometry::toArcCoordinates(
      centerline_2d, lanelet::utils::to2D(lanelet_point).basicPoint());
    const double dist_to_current_pose = arc_coordinates.length;
    const double lanelet_length = lanelet::geometry::length2d(closest_lanelet);
    return lanelet_length - dist_to_current_pose;
  };

  // if the front of target route is not the front of common segment, it is expected that the front
  // of the target route is conflicting with another lane which is equal to original
  // route[start_idx_original-1]. Otherwise compute the distance from the current pose to the end of
  // the current lanelet. Both cases reduce to the same arc-length computation; only the source
  // segment differs.
  const size_t start_segment_idx =
    (start_idx_target != 0 && start_idx_original > 1) ? start_idx_original - 1 : start_idx_original;
  const auto start_segment_length =
    arc_length_to_lanelet_end(original_route.segments.at(start_segment_idx).primitives);
  if (!start_segment_length.has_value()) {
    RCLCPP_ERROR(logger, "Check reroute safety failed. Cannot find the closest lanelet.");
    return false;
  }
  double accumulated_length = start_segment_length.value();

  // compute distance from the start_idx+1 to end_idx
  for (size_t i = start_idx_original + 1; i <= end_idx_original; ++i) {
    const auto & primitives = original_route.segments.at(i).primitives;
    if (primitives.empty()) {
      break;
    }

    std::vector<double> lanelets_length(primitives.size());
    for (size_t primitive_idx = 0; primitive_idx < primitives.size(); ++primitive_idx) {
      const auto & primitive = primitives.at(primitive_idx);
      const auto & lanelet = lanelet_map->laneletLayer.get(primitive.id);
      lanelets_length.at(primitive_idx) = (lanelet::geometry::length2d(lanelet));
    }
    accumulated_length += *std::min_element(lanelets_length.begin(), lanelets_length.end());
  }

  // check if the goal is inside of the target terminal lanelet
  const auto & target_end_primitives = target_route.segments.at(end_idx_target).primitives;
  const auto & target_goal = target_route.goal_pose;
  for (const auto & target_end_primitive : target_end_primitives) {
    const auto lanelet = lanelet_map->laneletLayer.get(target_end_primitive.id);
    if (autoware::experimental::lanelet2_utils::is_in_lanelet(target_goal, lanelet)) {
      const auto target_goal_position =
        experimental::lanelet2_utils::from_ros(target_goal.position);
      const double dist_to_goal = lanelet::geometry::toArcCoordinates(
                                    lanelet::utils::to2D(lanelet.centerline()),
                                    lanelet::utils::to2D(target_goal_position).basicPoint())
                                    .length;
      const double target_lanelet_length = lanelet::geometry::length2d(lanelet);
      // NOTE: `accumulated_length` here contains the length of the entire target_end_primitive, so
      // the remaining distance from the goal to the end of the target_end_primitive needs to be
      // subtracted.
      const double remaining_dist = target_lanelet_length - dist_to_goal;
      accumulated_length = std::max(accumulated_length - remaining_dist, 0.0);
      break;
    }
  }

  // check safety
  const double safety_length =
    std::max(current_velocity * reroute_time_threshold, minimum_reroute_length);
  if (accumulated_length > safety_length) {
    return true;
  }

  RCLCPP_WARN(
    logger,
    "Length of lane where original and B target (= %f) is less than safety length (= %f), so "
    "reroute is not safe.",
    accumulated_length, safety_length);
  return false;
}

}  // namespace autoware::mission_planner
