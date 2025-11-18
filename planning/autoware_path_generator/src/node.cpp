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

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/trajectory/utils/reference_path.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

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
  if (!is_data_ready(input_data)) {
    return;
  }

  if (!route_manager_) {
    initialize_route_manager(route_manager_data_, input_data.odometry_ptr->pose.pose);
    if (!route_manager_) {
      RCLCPP_ERROR(get_logger(), "Failed to create route manager");
      return;
    }
  }

  const auto param = param_listener_->get_params();
  const auto path = plan_path(input_data.odometry_ptr->pose.pose, param);
  if (!path) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "output path is invalid");
    return;
  }

  auto turn_signal = utils::get_turn_signal(
    *path, *route_manager_, input_data.odometry_ptr->pose.pose,
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

  // map
  if (const auto msg = vector_map_subscriber_.take_data()) {
    route_manager_data_.lanelet_map_bin_ptr = msg;
    route_manager_.reset();
  }

  // route
  if (const auto msg = route_subscriber_.take_data()) {
    if (msg->segments.empty()) {
      RCLCPP_ERROR(get_logger(), "input route is empty, ignoring...");
    } else {
      route_manager_data_.route_ptr = msg;
      planner_data_.route_frame_id = msg->header.frame_id;
      planner_data_.goal_pose = msg->goal_pose;
      route_manager_.reset();
    }
  }

  // odometry
  if (const auto msg = odometry_subscriber_.take_data()) {
    input_data.odometry_ptr = msg;
  }

  return input_data;
}

bool PathGenerator::is_data_ready(const InputData & input_data)
{
  const auto notify_waiting = [this](const std::string & name) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for %s", name.c_str());
  };

  if (!route_manager_data_.lanelet_map_bin_ptr) {
    notify_waiting("map");
    return false;
  }

  if (!route_manager_data_.route_ptr) {
    notify_waiting("route");
    return false;
  }

  if (!input_data.odometry_ptr) {
    notify_waiting("odometry");
    return false;
  }

  return true;
}

void PathGenerator::initialize_route_manager(
  const RouteManagerData & route_manager_data, const geometry_msgs::msg::Pose & initial_pose)
{
  route_manager_ = experimental::lanelet2_utils::RouteManager::create(
    *route_manager_data.lanelet_map_bin_ptr, *route_manager_data.route_ptr, initial_pose);
}

std::optional<PathWithLaneId> PathGenerator::plan_path(
  const geometry_msgs::msg::Pose & current_pose, const Params & params)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto path = generate_path(current_pose, params);
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
  if (!route_manager_) {
    RCLCPP_ERROR(get_logger(), "Route manager is not initialized");
    return std::nullopt;
  }

  route_manager_ =
    std::move(*route_manager_)
      .update_current_pose(
        current_pose, params.ego_nearest_dist_threshold, params.ego_nearest_yaw_threshold);
  if (!route_manager_) {
    RCLCPP_ERROR(get_logger(), "Failed to update current lanelet");
    return std::nullopt;
  }

  auto lanelets = route_manager_
                    ->get_lanelet_sequence_outward_route(
                      params.path_length.forward + vehicle_info_.max_longitudinal_offset_m,
                      params.path_length.backward + vehicle_info_.max_longitudinal_offset_m)
                    .as_lanelets();

  const auto & current_lanelet = route_manager_->current_lanelet();
  auto s_ego = lanelet::utils::getArcCoordinates({current_lanelet}, current_pose).length;
  auto s_start = s_ego - params.path_length.backward;
  auto s_end = s_ego + params.path_length.forward;

  const auto & goal_lanelet = route_manager_->goal_lanelet();
  auto connect_to_goal = false;

  for (auto [it, s] = std::make_tuple(lanelets.begin(), 0.); it != lanelets.end(); ++it) {
    const auto & lane_id = it->id();
    if (current_lanelet.id() == lane_id) {
      s_ego += s;
      s_start += s;
      s_end += s;
    }
    if (goal_lanelet.id() == lane_id) {
      const auto s_goal =
        s + lanelet::utils::getArcCoordinates({goal_lanelet}, planner_data_.goal_pose).length;
      if (s_goal < s_end) {
        s_end = s_goal;
        connect_to_goal = true;
      }
    }

    s += lanelet::geometry::length2d(*it);
    if (s >= s_end + vehicle_info_.max_longitudinal_offset_m) {
      lanelets.erase(std::next(it), lanelets.end());
      break;
    } else if (it == std::prev(lanelets.end())) {
      s_end = std::min(s_end, s - vehicle_info_.max_longitudinal_offset_m);
    }
  }

  auto s_intersection = utils::get_first_intersection_arc_length(
    lanelets, std::max(0., s_start - vehicle_info_.max_longitudinal_offset_m),
    s_end + vehicle_info_.max_longitudinal_offset_m, vehicle_info_.vehicle_length_m);
  if (s_intersection) {
    s_intersection = std::max(0., *s_intersection - vehicle_info_.max_longitudinal_offset_m);
    if (s_intersection < s_end) {
      s_end = *s_intersection;
      connect_to_goal = false;
    }
  }

  return generate_path(lanelets, current_pose, s_ego, s_start, s_end, connect_to_goal, params);
}

std::optional<PathWithLaneId> PathGenerator::generate_path(
  const lanelet::ConstLanelets & lanelet_sequence, const geometry_msgs::msg::Pose & current_pose,
  const double s_ego, const double s_start, const double s_end, const bool connect_to_goal,
  const Params & params) const
{
  if (lanelet_sequence.empty()) {
    RCLCPP_ERROR(get_logger(), "Lanelet sequence is empty");
    return std::nullopt;
  }

  if (!route_manager_) {
    RCLCPP_ERROR(get_logger(), "Route manager is not initialized");
    return std::nullopt;
  }

  std::vector<PathPointWithLaneId> path_points_with_lane_id{};

  auto path = experimental::trajectory::build_reference_path(
    lanelet_sequence, route_manager_->current_lanelet(), current_pose,
    route_manager_->lanelet_map_ptr(), route_manager_->routing_graph_ptr(),
    route_manager_->traffic_rules_ptr(), s_end - s_ego, s_ego - s_start);
  if (!path) {
    RCLCPP_ERROR(get_logger(), "Failed to build trajectory from path points");
    return std::nullopt;
  }

  const auto s_path_start = utils::get_arc_length_on_path(lanelet_sequence, *path, s_start);
  const auto s_path_end = utils::get_arc_length_on_path(lanelet_sequence, *path, s_end);

  if (path->length() - s_path_end > 0) {
    path->crop(0., s_path_end);
  }

  std::optional<Trajectory> path_to_goal = *path;
  if (connect_to_goal) {
    path_to_goal = utils::connect_path_to_goal_inside_lanelet_sequence(
      *path, lanelet_sequence, *route_manager_, planner_data_.goal_pose, s_end,
      params.goal_connection.connection_section_length, params.goal_connection.pre_goal_offset);

    if (!path_to_goal) {
      RCLCPP_ERROR(get_logger(), "Failed to connect path to goal");
      return std::nullopt;
    }
  }

  if (path_to_goal->length() - s_path_start > 0) {
    path_to_goal->crop(s_path_start, path->length() - s_path_start);
  }

  // Compose the polished path
  PathWithLaneId finalized_path_with_lane_id{};
  finalized_path_with_lane_id.points = path_to_goal->restore();

  if (finalized_path_with_lane_id.points.empty()) {
    RCLCPP_ERROR(get_logger(), "Finalized path points are empty after cropping");
    return std::nullopt;
  }

  // Set header which is needed to engage
  finalized_path_with_lane_id.header.frame_id = planner_data_.route_frame_id;
  finalized_path_with_lane_id.header.stamp = now();

  const auto [left_bound, right_bound] = utils::get_path_bounds(
    lanelet_sequence, std::max(0., s_start - vehicle_info_.max_longitudinal_offset_m),
    s_end + vehicle_info_.max_longitudinal_offset_m);
  finalized_path_with_lane_id.left_bound = left_bound;
  finalized_path_with_lane_id.right_bound = right_bound;

  return finalized_path_with_lane_id;
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
