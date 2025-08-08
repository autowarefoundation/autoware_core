// Copyright 2024 TIER IV, Inc.
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

#include "autoware/path_generator/node.hpp"

#include "autoware/path_generator/utils.hpp"

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
template <typename LaneletT, typename PointT>
double get_arc_length_along_centerline(const LaneletT & lanelet, const PointT & point)
{
  return lanelet::geometry::toArcCoordinates(lanelet.centerline2d(), lanelet::utils::to2D(point))
    .length;
}
}  // namespace

namespace autoware::path_generator
{
PathGenerator::PathGenerator(const rclcpp::NodeOptions & node_options)
: Node("path_generator", node_options)
{
  param_listener_ =
    std::make_shared<::path_generator::ParamListener>(this->get_node_parameters_interface());

  path_publisher_ = create_publisher<PathWithLaneId>("~/output/path", 1);

  turn_signal_publisher_ =
    create_publisher<TurnIndicatorsCommand>("~/output/turn_indicators_cmd", 1);

  hazard_signal_publisher_ = create_publisher<HazardLightsCommand>("~/output/hazard_lights_cmd", 1);

  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();

  const auto debug_processing_time_detail =
    create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
      "~/debug/processing_time_detail_ms", 1);
  time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>(debug_processing_time_detail);

  debug_calculation_time_ = create_publisher<Float64Stamped>("~/debug/processing_time_ms", 1);

  const auto params = param_listener_->get_params();

  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(params.planning_hz).period(),
    std::bind(&PathGenerator::run, this));
}

void PathGenerator::run()
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  stop_watch_.tic();

  const auto input_data = take_data();
  set_planner_data(input_data);
  if (!is_data_ready(input_data)) {
    return;
  }

  const auto param = param_listener_->get_params();
  const auto path = plan_path(input_data, param);
  if (!path) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "output path is invalid");
    return;
  }

  auto turn_signal = utils::get_turn_signal(
    *path, planner_data_, input_data.odometry_ptr->pose.pose,
    input_data.odometry_ptr->twist.twist.linear.x, param.turn_signal.search_distance,
    param.turn_signal.search_time, param.turn_signal.angle_threshold_deg,
    vehicle_info_.max_longitudinal_offset_m);
  turn_signal.stamp = now();
  turn_signal_publisher_->publish(turn_signal);

  HazardLightsCommand hazard_signal;
  hazard_signal.command = HazardLightsCommand::NO_COMMAND;
  hazard_signal.stamp = now();
  hazard_signal_publisher_->publish(hazard_signal);

  path_publisher_->publish(*path);

  publishStopWatchTime();
}

PathGenerator::InputData PathGenerator::take_data()
{
  InputData input_data;

  // route
  if (const auto msg = route_subscriber_.take_data()) {
    if (msg->segments.empty()) {
      RCLCPP_ERROR(get_logger(), "input route is empty, ignoring...");
    } else {
      input_data.route_ptr = msg;
    }
  }

  // map
  if (const auto msg = vector_map_subscriber_.take_data()) {
    input_data.lanelet_map_bin_ptr = msg;
  }

  // velocity
  if (const auto msg = odometry_subscriber_.take_data()) {
    input_data.odometry_ptr = msg;
  }

  return input_data;
}

void PathGenerator::set_planner_data(const InputData & input_data)
{
  if (input_data.lanelet_map_bin_ptr) {
    planner_data_.lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(
      *input_data.lanelet_map_bin_ptr, planner_data_.lanelet_map_ptr,
      &planner_data_.traffic_rules_ptr, &planner_data_.routing_graph_ptr);
  }

  if (input_data.route_ptr) {
    set_route(input_data.route_ptr);
  }
}

void PathGenerator::set_route(const LaneletRoute::ConstSharedPtr & route_ptr)
{
  planner_data_.route_frame_id = route_ptr->header.frame_id;
  planner_data_.goal_pose = route_ptr->goal_pose;

  planner_data_.route_lanelets.clear();
  planner_data_.preferred_lanelets.clear();
  planner_data_.start_lanelets.clear();
  planner_data_.goal_lanelets.clear();

  size_t primitives_num = 0;
  for (const auto & route_section : route_ptr->segments) {
    primitives_num += route_section.primitives.size();
  }
  planner_data_.route_lanelets.reserve(primitives_num);

  for (const auto & route_section : route_ptr->segments) {
    for (const auto & primitive : route_section.primitives) {
      const auto id = primitive.id;
      const auto & lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(id);
      planner_data_.route_lanelets.push_back(lanelet);
      if (id == route_section.preferred_primitive.id) {
        planner_data_.preferred_lanelets.push_back(lanelet);
      }
    }
  }

  const auto set_lanelets_from_segment =
    [&](
      const autoware_planning_msgs::msg::LaneletSegment & segment,
      lanelet::ConstLanelets & lanelets) {
      lanelets.reserve(segment.primitives.size());
      for (const auto & primitive : segment.primitives) {
        const auto & lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(primitive.id);
        lanelets.push_back(lanelet);
      }
    };
  set_lanelets_from_segment(route_ptr->segments.front(), planner_data_.start_lanelets);
  set_lanelets_from_segment(route_ptr->segments.back(), planner_data_.goal_lanelets);
}

bool PathGenerator::is_data_ready(const InputData & input_data)
{
  const auto notify_waiting = [this](const std::string & name) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for %s", name.c_str());
  };

  if (!planner_data_.lanelet_map_ptr) {
    notify_waiting("map");
    return false;
  }

  if (planner_data_.route_lanelets.empty()) {
    notify_waiting("route");
    return false;
  }

  if (!input_data.odometry_ptr) {
    notify_waiting("odometry");
    return false;
  }

  return true;
}

std::optional<PathWithLaneId> PathGenerator::plan_path(
  const InputData & input_data, const Params & params)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto path = generate_path(input_data.odometry_ptr->pose.pose, params);

  if (!path) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "output path is invalid");
    return std::nullopt;
  }
  if (path->points.empty()) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "output path is empty");
    return std::nullopt;
  }

  return path;
}

std::optional<PathWithLaneId> PathGenerator::generate_path(
  const geometry_msgs::msg::Pose & current_pose, const Params & params)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  if (!update_current_lanelet(current_pose, params)) {
    RCLCPP_ERROR(get_logger(), "Failed to update current lanelet");
    return std::nullopt;
  }

  lanelet::ConstLanelets lanelets{*current_lanelet_};
  const auto s_on_current_lanelet =
    lanelet::utils::getArcCoordinates({*current_lanelet_}, current_pose).length;

  const auto backward_length = std::max(
    0.,
    params.path_length.backward + vehicle_info_.max_longitudinal_offset_m - s_on_current_lanelet);
  const auto backward_lanelets_within_route =
    utils::get_lanelets_within_route_up_to(*current_lanelet_, planner_data_, backward_length);
  if (!backward_lanelets_within_route) {
    RCLCPP_ERROR(
      get_logger(), "Failed to get backward lanelets within route for current lanelet (id: %ld)",
      current_lanelet_->id());
    return std::nullopt;
  }
  lanelets.insert(
    lanelets.begin(), backward_lanelets_within_route->begin(),
    backward_lanelets_within_route->end());

  //  Extend lanelets by backward_length even outside planned route to ensure
  //  ego footprint is inside lanelets if ego is at the beginning of start lane
  auto backward_lanelets_length =
    lanelet::utils::getLaneletLength2d(*backward_lanelets_within_route);
  while (backward_lanelets_length < backward_length) {
    const auto prev_lanelets = planner_data_.routing_graph_ptr->previous(lanelets.front());
    if (prev_lanelets.empty()) {
      break;
    }
    lanelets.insert(lanelets.begin(), prev_lanelets.front());
    backward_lanelets_length += lanelet::geometry::length2d(prev_lanelets.front());
  }

  const auto forward_length = std::max(
    0., params.path_length.forward + vehicle_info_.max_longitudinal_offset_m -
          (lanelet::geometry::length2d(*current_lanelet_) - s_on_current_lanelet));
  const auto forward_lanelets_within_route =
    utils::get_lanelets_within_route_after(*current_lanelet_, planner_data_, forward_length);
  if (!forward_lanelets_within_route) {
    RCLCPP_ERROR(
      get_logger(), "Failed to get forward lanelets within route for current lanelet (id: %ld)",
      current_lanelet_->id());
    return std::nullopt;
  }
  lanelets.insert(
    lanelets.end(), forward_lanelets_within_route->begin(), forward_lanelets_within_route->end());

  //  Extend lanelets by forward_length even outside planned route to ensure
  //  ego footprint is inside lanelets if ego is at the end of goal lane
  auto forward_lanelets_length = lanelet::utils::getLaneletLength2d(*forward_lanelets_within_route);
  while (forward_lanelets_length < forward_length) {
    const auto next_lanelets = planner_data_.routing_graph_ptr->following(lanelets.back());
    if (next_lanelets.empty()) {
      break;
    }
    lanelets.insert(lanelets.end(), next_lanelets.front());
    forward_lanelets_length += lanelet::geometry::length2d(next_lanelets.front());
  }

  const auto s = s_on_current_lanelet + backward_lanelets_length;
  const auto s_start = std::max(0., s - params.path_length.backward);

  std::optional<lanelet::ConstLanelet> goal_lanelet_for_path = std::nullopt;
  const auto s_end = [&]() {
    auto s_end = s + params.path_length.forward;

    if (!utils::get_next_lanelet_within_route(lanelets.back(), planner_data_)) {
      s_end = std::min(s_end, lanelet::utils::getLaneletLength2d(lanelets));
    }

    const auto s_intersection = utils::get_first_intersection_arc_length(
      lanelets, std::max(0., s_start - vehicle_info_.max_longitudinal_offset_m),
      s_end + vehicle_info_.max_longitudinal_offset_m, vehicle_info_.vehicle_length_m);
    if (s_intersection) {
      s_end =
        std::min(s_end, std::max(0., *s_intersection - vehicle_info_.max_longitudinal_offset_m));
    }

    for (auto [it, s_goal] = std::make_tuple(lanelets.begin(), 0.); it != lanelets.end(); ++it) {
      if (std::any_of(
            planner_data_.goal_lanelets.begin(), planner_data_.goal_lanelets.end(),
            [id = it->id()](const auto & goal_lanelet) { return id == goal_lanelet.id(); })) {
        s_goal += lanelet::utils::getArcCoordinates({*it}, planner_data_.goal_pose).length;
        if (s_goal < s_end) {
          goal_lanelet_for_path = *it;
          s_end = s_goal;
        }
        break;
      }
      s_goal += lanelet::geometry::length2d(*it);
    }

    return s_end;
  }();

  return generate_path(lanelets, s_start, s_end, goal_lanelet_for_path, params);
}

std::optional<PathWithLaneId> PathGenerator::generate_path(
  const lanelet::LaneletSequence & lanelet_sequence, const double s_start, const double s_end,
  const std::optional<lanelet::ConstLanelet> & goal_lanelet_for_path, const Params & params) const
{
  if (lanelet_sequence.empty()) {
    RCLCPP_ERROR(get_logger(), "Lanelet sequence is empty");
    return std::nullopt;
  }

  std::vector<PathPointWithLaneId> path_points_with_lane_id{};

  const auto waypoint_groups = utils::get_waypoint_groups(
    lanelet_sequence, *planner_data_.lanelet_map_ptr,
    params.waypoint.connection_gradient_from_centerline);

  auto extended_lanelets = lanelet_sequence.lanelets();
  auto extended_arc_length = 0.;
  for (const auto & [waypoints, interval] : waypoint_groups) {
    while (interval.start + extended_arc_length < 0.) {
      const auto prev_lanelets =
        planner_data_.routing_graph_ptr->previous(extended_lanelets.front());
      if (prev_lanelets.empty()) {
        break;
      }
      extended_lanelets.insert(extended_lanelets.begin(), prev_lanelets.front());
      extended_arc_length += lanelet::utils::getLaneletLength2d(prev_lanelets.front());
    }
  }

  const auto add_path_point =
    [&](const lanelet::ConstPoint3d & path_point, const lanelet::Id & lane_id) {
      PathPointWithLaneId path_point_with_lane_id{};
      path_point_with_lane_id.lane_ids.push_back(lane_id);
      path_point_with_lane_id.point.pose.position =
        lanelet::utils::conversion::toGeomMsgPt(path_point);
      path_point_with_lane_id.point.longitudinal_velocity_mps =
        planner_data_.traffic_rules_ptr
          ->speedLimit(planner_data_.lanelet_map_ptr->laneletLayer.get(lane_id))
          .speedLimit.value();
      path_points_with_lane_id.push_back(std::move(path_point_with_lane_id));
    };

  const lanelet::LaneletSequence extended_lanelet_sequence(extended_lanelets);
  std::optional<size_t> overlapping_waypoint_group_index = std::nullopt;

  for (auto [lanelet_it, s] = std::make_tuple(extended_lanelet_sequence.begin(), 0.);
       lanelet_it != extended_lanelet_sequence.end(); ++lanelet_it) {
    const auto & centerline = lanelet_it->centerline();

    for (auto point_it = centerline.begin(); point_it != centerline.end(); ++point_it) {
      if (point_it != centerline.begin()) {
        s += lanelet::geometry::distance2d(*std::prev(point_it), *point_it);
      } else if (lanelet_it != extended_lanelet_sequence.begin()) {
        continue;
      }

      if (overlapping_waypoint_group_index) {
        const auto & [waypoints, interval] = waypoint_groups[*overlapping_waypoint_group_index];
        if (s >= interval.start + extended_arc_length && s <= interval.end + extended_arc_length) {
          continue;
        }
        overlapping_waypoint_group_index = std::nullopt;
      }

      for (size_t i = 0; i < waypoint_groups.size(); ++i) {
        const auto & [waypoints, interval] = waypoint_groups[i];
        if (s < interval.start + extended_arc_length || s > interval.end + extended_arc_length) {
          continue;
        }
        for (const auto & waypoint : waypoints) {
          add_path_point(waypoint.point, waypoint.lane_id);
        }
        overlapping_waypoint_group_index = i;
        break;
      }
      if (overlapping_waypoint_group_index) {
        continue;
      }

      add_path_point(*point_it, lanelet_it->id());
      if (
        point_it == std::prev(centerline.end()) &&
        lanelet_it != std::prev(extended_lanelet_sequence.end())) {
        if (
          lanelet_it != extended_lanelet_sequence.begin() ||
          lanelet_it->id() == lanelet_sequence.begin()->id()) {
          path_points_with_lane_id.back().lane_ids.push_back(std::next(lanelet_it)->id());
        } else {
          path_points_with_lane_id.back().lane_ids = {std::next(lanelet_it)->id()};
        }
      }
    }
  }

  if (path_points_with_lane_id.empty()) {
    RCLCPP_ERROR(get_logger(), "No path points generated from lanelet sequence");
    return std::nullopt;
  }

  auto path = autoware::experimental::trajectory::pretty_build(path_points_with_lane_id);
  if (!path) {
    RCLCPP_ERROR(get_logger(), "Failed to build trajectory from path points");
    return std::nullopt;
  }

  // Attach orientation to path
  path->align_orientation_with_trajectory_direction();

  const auto s_path_start =
    utils::get_arc_length_on_path(extended_lanelet_sequence, *path, extended_arc_length + s_start);
  const auto s_path_end =
    utils::get_arc_length_on_path(extended_lanelet_sequence, *path, extended_arc_length + s_end);

  if (path->length() - s_path_end > 0) {
    path->crop(0., s_path_end);
  }

  if (goal_lanelet_for_path) {
    path = utils::connect_path_to_goal_inside_lanelet_sequence(
      *path, extended_lanelet_sequence, planner_data_.goal_pose, *goal_lanelet_for_path,
      extended_arc_length + s_end, planner_data_, params.goal_connection.connection_section_length,
      params.goal_connection.pre_goal_offset);

    if (!path) {
      RCLCPP_ERROR(get_logger(), "Failed to connect path to goal");
      return std::nullopt;
    }
  }

  if (path->length() - s_path_start > 0) {
    path->crop(s_path_start, path->length() - s_path_start);
  }

  // Compose the polished path
  PathWithLaneId finalized_path_with_lane_id{};
  finalized_path_with_lane_id.points = path->restore();

  if (finalized_path_with_lane_id.points.empty()) {
    RCLCPP_ERROR(get_logger(), "Finalized path points are empty after cropping");
    return std::nullopt;
  }

  // Set header which is needed to engage
  finalized_path_with_lane_id.header.frame_id = planner_data_.route_frame_id;
  finalized_path_with_lane_id.header.stamp = now();

  const auto [left_bound, right_bound] = utils::get_path_bounds(
    extended_lanelet_sequence,
    std::max(0., extended_arc_length + s_start - vehicle_info_.max_longitudinal_offset_m),
    extended_arc_length + s_end + vehicle_info_.max_longitudinal_offset_m);
  finalized_path_with_lane_id.left_bound = left_bound;
  finalized_path_with_lane_id.right_bound = right_bound;

  return finalized_path_with_lane_id;
}

bool PathGenerator::update_current_lanelet(
  const geometry_msgs::msg::Pose & current_pose, const Params & params)
{
  if (!current_lanelet_) {
    lanelet::ConstLanelet current_lanelet;
    if (lanelet::utils::query::getClosestLanelet(
          planner_data_.route_lanelets, current_pose, &current_lanelet)) {
      current_lanelet_ = current_lanelet;
      return true;
    }
    return false;
  }

  lanelet::ConstLanelets candidates;
  if (
    const auto previous_lanelet =
      utils::get_previous_lanelet_within_route(*current_lanelet_, planner_data_)) {
    candidates.push_back(*previous_lanelet);
  }
  candidates.push_back(*current_lanelet_);
  if (
    const auto next_lanelet =
      utils::get_next_lanelet_within_route(*current_lanelet_, planner_data_)) {
    candidates.push_back(*next_lanelet);
  }

  if (lanelet::utils::query::getClosestLaneletWithConstrains(
        candidates, current_pose, &*current_lanelet_, params.ego_nearest_dist_threshold,
        params.ego_nearest_yaw_threshold)) {
    return true;
  }

  if (lanelet::utils::query::getClosestLanelet(
        planner_data_.route_lanelets, current_pose, &*current_lanelet_)) {
    return true;
  }

  return false;
}

void PathGenerator::publishStopWatchTime()
{
  Float64Stamped calculation_time_data{};
  calculation_time_data.stamp = this->now();
  calculation_time_data.data = stop_watch_.toc();
  debug_calculation_time_->publish(calculation_time_data);
}
}  // namespace autoware::path_generator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::path_generator::PathGenerator)
