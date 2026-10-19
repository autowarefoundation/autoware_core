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

#include "autoware/path_generator/path_generator_node.hpp"

#include <memory>
#include <string>
#include <utility>

namespace autoware::path_generator
{
PathGeneratorNode::PathGeneratorNode(const rclcpp::NodeOptions & node_options)
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

  PathGeneratorConfig config;
  config.params = params;
  config.vehicle_length_m = vehicle_info_.vehicle_length_m;
  config.max_longitudinal_offset_m = vehicle_info_.max_longitudinal_offset_m;
  path_generator_ = PathGeneratorCore(config);

  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(params.planning_hz).period(),
    std::bind(&PathGeneratorNode::run, this));
}

void PathGeneratorNode::run()
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  stop_watch_.tic();

  const auto input_data = take_data();
  if (!is_data_ready(input_data)) {
    return;
  }

  if (!path_generator_.is_ready()) {
    const auto res = path_generator_.set_map_and_route(
      *route_manager_data_.lanelet_map_bin_ptr, *route_manager_data_.route_ptr,
      input_data.odometry_ptr->pose.pose);
    if (!res) {
      RCLCPP_ERROR(get_logger(), "%s", res.error().c_str());
      return;
    }
  }

  // Update config from dynamic parameters
  PathGeneratorConfig config;
  config.params = param_listener_->get_params();
  config.vehicle_length_m = vehicle_info_.vehicle_length_m;
  config.max_longitudinal_offset_m = vehicle_info_.max_longitudinal_offset_m;
  path_generator_.set_config(config);

  const auto result = path_generator_.generate(*input_data.odometry_ptr);
  if (!result) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "%s", result.error().c_str());
    return;
  }

  turn_signal_publisher_->publish(result->turn_indicators_cmd);
  hazard_signal_publisher_->publish(result->hazard_lights_cmd);
  path_publisher_->publish(result->path);

  publishStopWatchTime();
}

PathGeneratorNode::InputData PathGeneratorNode::take_data()
{
  InputData input_data;

  // Map
  if (const auto msg = vector_map_subscriber_.take_data()) {
    route_manager_data_.lanelet_map_bin_ptr = msg;
    path_generator_.reset_route_manager();
  }

  // Route
  if (const auto msg = route_subscriber_.take_data()) {
    if (msg->segments.empty()) {
      RCLCPP_ERROR(get_logger(), "input route is empty, ignoring...");
    } else {
      route_manager_data_.route_ptr = msg;
      path_generator_.reset_route_manager();
    }
  }

  // Odometry
  if (const auto msg = odometry_subscriber_.take_data()) {
    input_data.odometry_ptr = msg;
  }

  return input_data;
}

bool PathGeneratorNode::is_data_ready(const InputData & input_data)
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

void PathGeneratorNode::initialize_route_manager(
  const RouteManagerData & route_manager_data, const geometry_msgs::msg::Pose & initial_pose)
{
  if (!route_manager_data.lanelet_map_bin_ptr) {
    RCLCPP_ERROR(get_logger(), "Lanelet map is not set");
    return;
  }
  if (!route_manager_data.route_ptr) {
    RCLCPP_ERROR(get_logger(), "Route is not set");
    return;
  }

  route_manager_data_ = route_manager_data;
  const auto res = path_generator_.set_map_and_route(
    *route_manager_data.lanelet_map_bin_ptr, *route_manager_data.route_ptr, initial_pose);
  if (!res) {
    RCLCPP_ERROR(get_logger(), "%s", res.error().c_str());
  }
}

std::optional<PathWithLaneId> PathGeneratorNode::generate_path(
  const geometry_msgs::msg::Pose & current_pose, const Params & params)
{
  auto config = path_generator_.get_config();
  config.params = params;
  path_generator_.set_config(config);

  const auto res = path_generator_.generate_path(current_pose, now());
  if (!res) {
    RCLCPP_ERROR(get_logger(), "%s", res.error().c_str());
    return std::nullopt;
  }
  return *res;
}

void PathGeneratorNode::publishStopWatchTime()
{
  Float64Stamped calculation_time_data{};
  calculation_time_data.stamp = this->now();
  calculation_time_data.data = stop_watch_.toc();
  debug_calculation_time_->publish(calculation_time_data);
}
}  // namespace autoware::path_generator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::path_generator::PathGeneratorNode)
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::path_generator::PathGenerator)
