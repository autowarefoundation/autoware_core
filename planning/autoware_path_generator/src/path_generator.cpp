// Copyright 2024-2026 TIER IV, Inc.
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

#include "autoware/path_generator/path_generator.hpp"

#include "utils.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/trajectory/utils/reference_path.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::path_generator
{

PathGenerator::PathGenerator(const PathGeneratorConfig & config) : config_(config)
{
  if (config_.vehicle_length_m < 0.0) {
    throw std::invalid_argument("vehicle_length_m must not be negative");
  }
  if (config_.max_longitudinal_offset_m < 0.0) {
    throw std::invalid_argument("max_longitudinal_offset_m must not be negative");
  }
}

tl::expected<void, std::string> PathGenerator::set_map_and_route(
  const LaneletMapBin & map_bin, const LaneletRoute & route,
  const geometry_msgs::msg::Pose & initial_pose)
{
  route_ = route;
  route_manager_ = experimental::lanelet2_utils::RouteManager::create(map_bin, route, initial_pose);
  if (!route_manager_) {
    return tl::make_unexpected("Failed to create route manager");
  }
  return {};
}

void PathGenerator::reset_route_manager()
{
  route_manager_.reset();
  route_.reset();
}

bool PathGenerator::is_ready() const
{
  return route_manager_.has_value();
}

tl::expected<PathGeneratorResult, std::string> PathGenerator::generate(const Odometry & odometry)
{
  if (!route_manager_) {
    return tl::make_unexpected("Route manager is not initialized");
  }

  const auto path_result = generate_path(odometry.pose.pose, odometry.header.stamp);
  if (!path_result) {
    return tl::make_unexpected(path_result.error());
  }

  auto turn_signal = utils::get_turn_signal(
    *path_result, *route_manager_, odometry.pose.pose, odometry.twist.twist.linear.x,
    config_.params.turn_signal.search_distance, config_.params.turn_signal.search_time,
    config_.params.turn_signal.angle_threshold_deg, config_.max_longitudinal_offset_m);
  turn_signal.stamp = odometry.header.stamp;

  HazardLightsCommand hazard_signal;
  hazard_signal.command = HazardLightsCommand::NO_COMMAND;
  hazard_signal.stamp = odometry.header.stamp;

  PathGeneratorResult result;
  result.path = std::move(*path_result);
  result.turn_indicators_cmd = std::move(turn_signal);
  result.hazard_lights_cmd = std::move(hazard_signal);
  return result;
}

tl::expected<PathWithLaneId, std::string> PathGenerator::generate_path(
  const geometry_msgs::msg::Pose & current_pose,
  const std::optional<builtin_interfaces::msg::Time> & stamp)
{
  if (!route_manager_) {
    return tl::make_unexpected("Route manager is not initialized");
  }

  route_manager_ = std::move(*route_manager_)
                     .update_current_pose(
                       current_pose, config_.params.ego_nearest_dist_threshold,
                       config_.params.ego_nearest_yaw_threshold);
  if (!route_manager_) {
    return tl::make_unexpected("Failed to update current lanelet");
  }

  auto lanelets = route_manager_
                    ->get_lanelet_sequence_outward_route(
                      config_.params.path_length.forward + config_.max_longitudinal_offset_m,
                      config_.params.path_length.backward + config_.max_longitudinal_offset_m)
                    .as_lanelets();

  const auto & current_lanelet = route_manager_->current_lanelet();
  auto s_ego =
    experimental::lanelet2_utils::get_arc_coordinates({current_lanelet}, current_pose).length;
  auto s_start = s_ego - config_.params.path_length.backward;
  auto s_end = s_ego + config_.params.path_length.forward;
  auto current_lanelet_seen = false;

  const auto & goal_lanelet = route_manager_->goal_lanelet();
  auto connect_to_goal = false;

  for (auto [it, s] = std::make_tuple(lanelets.begin(), 0.); it != lanelets.end(); ++it) {
    const auto & lane_id = it->id();
    if (current_lanelet.id() == lane_id) {
      s_ego += s;
      s_start += s;
      s_end += s;
      current_lanelet_seen = true;
    }
    if (goal_lanelet.id() == lane_id) {
      const auto s_goal =
        s +
        experimental::lanelet2_utils::get_arc_coordinates({goal_lanelet}, route_->goal_pose).length;
      if (s_goal < s_end) {
        s_end = s_goal;
        connect_to_goal = true;
      }
    }

    s += lanelet::geometry::length2d(*it);
    if (current_lanelet_seen && s >= s_end + config_.max_longitudinal_offset_m) {
      lanelets.erase(std::next(it), lanelets.end());
      break;
    } else if (current_lanelet_seen && it == std::prev(lanelets.end())) {
      s_end = std::min(s_end, s - config_.max_longitudinal_offset_m);
    }
  }

  auto s_intersection = utils::get_first_intersection_arc_length(
    lanelets, std::max(0., s_start - config_.max_longitudinal_offset_m),
    s_end + config_.max_longitudinal_offset_m, config_.vehicle_length_m);
  if (s_intersection) {
    s_intersection = std::max(0., *s_intersection - config_.max_longitudinal_offset_m);
    if (s_intersection < s_end) {
      s_end = *s_intersection;
      connect_to_goal = false;
    }
  }

  return generate_path(lanelets, current_pose, s_ego, s_start, s_end, connect_to_goal, stamp);
}

tl::expected<PathWithLaneId, std::string> PathGenerator::generate_path(
  const lanelet::ConstLanelets & lanelet_sequence, const geometry_msgs::msg::Pose & current_pose,
  const double s_ego, const double s_start, const double s_end, const bool connect_to_goal,
  const std::optional<builtin_interfaces::msg::Time> & stamp) const
{
  if (lanelet_sequence.empty()) {
    return tl::make_unexpected("Lanelet sequence is empty");
  }

  if (!route_manager_) {
    return tl::make_unexpected("Route manager is not initialized");
  }

  auto path = experimental::trajectory::build_reference_path(
    lanelet_sequence, route_manager_->current_lanelet(), current_pose,
    route_manager_->lanelet_map_ptr(), route_manager_->routing_graph_ptr(),
    route_manager_->traffic_rules_ptr(), s_end - s_ego, s_ego - s_start);
  if (!path) {
    return tl::make_unexpected("Failed to build trajectory from path points");
  }

  const auto s_path_start = utils::get_arc_length_on_path(lanelet_sequence, *path, s_start);
  const auto s_path_end = utils::get_arc_length_on_path(lanelet_sequence, *path, s_end);

  if (path->length() - s_path_end > 0) {
    path->crop(0., s_path_end);
  }

  std::optional<Trajectory> path_to_goal = *path;
  if (connect_to_goal) {
    path_to_goal = utils::connect_path_to_goal_inside_lanelet_sequence(
      *path, lanelet_sequence, *route_manager_, route_->goal_pose, s_end,
      config_.params.goal_connection.connection_section_length,
      config_.params.goal_connection.pre_goal_offset);

    if (!path_to_goal) {
      return tl::make_unexpected("Failed to connect path to goal");
    }
  }

  if (path_to_goal->length() - s_path_start > 0) {
    path_to_goal->crop(s_path_start, path->length() - s_path_start);
  }

  // Compose the polished path
  PathWithLaneId finalized_path_with_lane_id{};
  finalized_path_with_lane_id.points = path_to_goal->restore();

  if (finalized_path_with_lane_id.points.empty()) {
    return tl::make_unexpected("Finalized path points are empty after cropping");
  }

  // Set header which is needed to engage
  if (route_.has_value()) {
    finalized_path_with_lane_id.header.frame_id = route_->header.frame_id;
  }
  if (stamp.has_value()) {
    finalized_path_with_lane_id.header.stamp = *stamp;
  }

  const auto [left_bound, right_bound] = utils::get_path_bounds(
    lanelet_sequence, std::max(0., s_start - config_.max_longitudinal_offset_m),
    s_end + config_.max_longitudinal_offset_m);
  finalized_path_with_lane_id.left_bound = left_bound;
  finalized_path_with_lane_id.right_bound = right_bound;

  return finalized_path_with_lane_id;
}

void PathGenerator::set_config(const PathGeneratorConfig & config)
{
  config_ = config;
}

const PathGeneratorConfig & PathGenerator::get_config() const
{
  return config_;
}

}  // namespace autoware::path_generator
