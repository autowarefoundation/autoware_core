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

#ifndef PARAMETERS_HPP_
#define PARAMETERS_HPP_

#include "type_alias.hpp"
#include "types.hpp"

#include <autoware/motion_utils/marker/marker_helper.hpp>
#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/object_recognition_utils/predicted_path_utils.hpp>
#include <autoware/objects_of_interest_marker_interface/objects_of_interest_marker_interface.hpp>
#include <autoware_utils_rclcpp/parameter.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner
{
using autoware_utils_rclcpp::get_or_declare_parameter;

struct CommonParam
{
  double max_accel{};
  double min_accel{};
  double max_jerk{};
  double min_jerk{};
  double limit_max_accel{};
  double limit_min_accel{};
  double limit_max_jerk{};
  double limit_min_jerk{};

  CommonParam() = default;
  explicit CommonParam(rclcpp::Node & node)
  {
    max_accel = get_or_declare_parameter<double>(node, "normal.max_acc");
    min_accel = get_or_declare_parameter<double>(node, "normal.min_acc");
    max_jerk = get_or_declare_parameter<double>(node, "normal.max_jerk");
    min_jerk = get_or_declare_parameter<double>(node, "normal.min_jerk");
    limit_max_accel = get_or_declare_parameter<double>(node, "limit.max_acc");
    limit_min_accel = get_or_declare_parameter<double>(node, "limit.min_acc");
    limit_max_jerk = get_or_declare_parameter<double>(node, "limit.max_jerk");
    limit_min_jerk = get_or_declare_parameter<double>(node, "limit.min_jerk");
  }
};

struct ObstacleFilteringParam
{
  struct PointcloudObstacleFilteringParam
  {
    struct
    {
      bool enable_trim{};
      double min_trajectory_length{};
      double braking_distance_scale_factor{};
    } trim_trajectory;
    struct
    {
      double max_time_diff{};
      double min_velocity{};
      double max_velocity{};
      double position_diff{};
    } time_series_association;
    struct
    {
      bool use_estimated_velocity{};
      double min_clamp_velocity{};
      double max_clamp_velocity{};
      size_t required_velocity_count{};
      double lpf_gain{};
    } velocity_estimation;
    struct
    {
      double margin_from_bottom{};
      double margin_from_top{};
    } height_margin;

    PointcloudObstacleFilteringParam() = default;
    explicit PointcloudObstacleFilteringParam(rclcpp::Node & node)
    {
      const std::string ns = "obstacle_stop.obstacle_filtering.pointcloud.";
      trim_trajectory.enable_trim =
        get_or_declare_parameter<bool>(node, ns + "trim_trajectory.enable_trim");
      trim_trajectory.min_trajectory_length =
        get_or_declare_parameter<double>(node, ns + "trim_trajectory.min_trajectory_length");
      trim_trajectory.braking_distance_scale_factor = get_or_declare_parameter<double>(
        node, ns + "trim_trajectory.braking_distance_scale_factor");
      time_series_association.max_time_diff =
        get_or_declare_parameter<double>(node, ns + "time_series_association.max_time_diff");
      time_series_association.min_velocity =
        get_or_declare_parameter<double>(node, ns + "time_series_association.min_velocity");
      time_series_association.max_velocity =
        get_or_declare_parameter<double>(node, ns + "time_series_association.max_velocity");
      time_series_association.position_diff =
        get_or_declare_parameter<double>(node, ns + "time_series_association.position_diff");
      velocity_estimation.use_estimated_velocity =
        get_or_declare_parameter<bool>(node, ns + "velocity_estimation.use_estimated_velocity");
      velocity_estimation.min_clamp_velocity =
        get_or_declare_parameter<double>(node, ns + "velocity_estimation.min_clamp_velocity");
      velocity_estimation.max_clamp_velocity =
        get_or_declare_parameter<double>(node, ns + "velocity_estimation.max_clamp_velocity");
      velocity_estimation.required_velocity_count =
        get_or_declare_parameter<int>(node, ns + "velocity_estimation.required_velocity_count");
      velocity_estimation.lpf_gain =
        get_or_declare_parameter<double>(node, ns + "velocity_estimation.lpf_gain");
      height_margin.margin_from_bottom =
        get_or_declare_parameter<double>(node, ns + "height_margin.margin_from_bottom");
      height_margin.margin_from_top =
        get_or_declare_parameter<double>(node, ns + "height_margin.margin_from_top");
    }
  };
  PointcloudObstacleFilteringParam pointcloud_obstacle_filtering_param;

  bool use_pointcloud{false};
  std::vector<uint8_t> inside_stop_object_types{};
  std::vector<uint8_t> outside_stop_object_types{};

  double max_lat_margin{};
  double max_lat_margin_against_predicted_object_unknown{};
  double max_lat_margin_against_pointcloud{};

  double min_velocity_to_reach_collision_point{};
  double stop_obstacle_hold_time_threshold{};

  double outside_estimation_time_horizon{};
  double outside_max_lateral_velocity{};
  double outside_pedestrian_deceleration{};
  double outside_bicycle_deceleration{};

  double crossing_obstacle_collision_time_margin{};
  double crossing_obstacle_traj_angle_threshold{};

  ObstacleFilteringParam() = default;
  explicit ObstacleFilteringParam(rclcpp::Node & node)
  : pointcloud_obstacle_filtering_param(node),
    inside_stop_object_types(
      utils::get_target_object_type(node, "obstacle_stop.obstacle_filtering.object_type.inside.")),
    outside_stop_object_types(
      utils::get_target_object_type(node, "obstacle_stop.obstacle_filtering.object_type.outside."))
  {
    max_lat_margin =
      get_or_declare_parameter<double>(node, "obstacle_stop.obstacle_filtering.max_lat_margin");
    max_lat_margin_against_predicted_object_unknown = get_or_declare_parameter<double>(
      node, "obstacle_stop.obstacle_filtering.max_lat_margin_against_predicted_object_unknown");
    max_lat_margin_against_pointcloud = get_or_declare_parameter<double>(
      node, "obstacle_stop.obstacle_filtering.max_lat_margin_against_pointcloud");

    min_velocity_to_reach_collision_point = get_or_declare_parameter<double>(
      node, "obstacle_stop.obstacle_filtering.min_velocity_to_reach_collision_point");
    stop_obstacle_hold_time_threshold = get_or_declare_parameter<double>(
      node, "obstacle_stop.obstacle_filtering.stop_obstacle_hold_time_threshold");

    outside_estimation_time_horizon = get_or_declare_parameter<double>(
      node, "obstacle_stop.obstacle_filtering.outside_obstacle.estimation_time_horizon");
    outside_pedestrian_deceleration = get_or_declare_parameter<double>(
      node, "obstacle_stop.obstacle_filtering.outside_obstacle.pedestrian_deceleration");
    outside_bicycle_deceleration = get_or_declare_parameter<double>(
      node, "obstacle_stop.obstacle_filtering.outside_obstacle.bicycle_deceleration");
    outside_max_lateral_velocity = get_or_declare_parameter<double>(
      node, "obstacle_stop.obstacle_filtering.outside_obstacle.max_lateral_velocity");

    crossing_obstacle_collision_time_margin = get_or_declare_parameter<double>(
      node, "obstacle_stop.obstacle_filtering.crossing_obstacle.collision_time_margin");
    crossing_obstacle_traj_angle_threshold = get_or_declare_parameter<double>(
      node, "obstacle_stop.obstacle_filtering.crossing_obstacle.traj_angle_threshold");

    use_pointcloud = get_or_declare_parameter<bool>(
      node, "obstacle_stop.obstacle_filtering.object_type.pointcloud");
  }
};

struct RSSParam
{
  bool use_rss_stop{};
  double two_wheel_objects_deceleration{};
  double vehicle_objects_deceleration{};
  double no_wheel_objects_deceleration{};
  double pointcloud_deceleration{};
  double velocity_offset{};
};

struct StopPlanningParam
{
  double stop_margin{};
  double terminal_stop_margin{};
  double min_behavior_stop_margin{};
  double max_negative_velocity{};
  double stop_margin_opposing_traffic{};
  double effective_deceleration_opposing_traffic{};
  double hold_stop_velocity_threshold{};
  double hold_stop_distance_threshold{};
  double pointcloud_suppression_distance_margin{};
  bool enable_approaching_on_curve{};
  double additional_stop_margin_on_curve{};
  double min_stop_margin_on_curve{};
  RSSParam rss_params;
  double obstacle_velocity_threshold_enter_fixed_stop{};
  double obstacle_velocity_threshold_exit_fixed_stop{};

  struct ObjectTypeSpecificParams
  {
    double limit_min_acc{};
    double sudden_object_acc_threshold{};
    double sudden_object_dist_threshold{};
    bool abandon_to_stop{};
  };
  std::unordered_map<uint8_t, std::string> object_types_maps = {
    {ObjectClassification::UNKNOWN, "unknown"}, {ObjectClassification::CAR, "car"},
    {ObjectClassification::TRUCK, "truck"},     {ObjectClassification::BUS, "bus"},
    {ObjectClassification::TRAILER, "trailer"}, {ObjectClassification::MOTORCYCLE, "motorcycle"},
    {ObjectClassification::BICYCLE, "bicycle"}, {ObjectClassification::PEDESTRIAN, "pedestrian"}};
  std::unordered_map<std::string, ObjectTypeSpecificParams> object_type_specific_param_map;

  StopPlanningParam() = default;
  StopPlanningParam(rclcpp::Node & node, const CommonParam & common_param)
  {
    stop_margin = get_or_declare_parameter<double>(node, "obstacle_stop.stop_planning.stop_margin");
    terminal_stop_margin =
      get_or_declare_parameter<double>(node, "obstacle_stop.stop_planning.terminal_stop_margin");
    min_behavior_stop_margin = get_or_declare_parameter<double>(
      node, "obstacle_stop.stop_planning.min_behavior_stop_margin");
    max_negative_velocity =
      get_or_declare_parameter<double>(node, "obstacle_stop.stop_planning.max_negative_velocity");
    stop_margin_opposing_traffic = get_or_declare_parameter<double>(
      node, "obstacle_stop.stop_planning.stop_margin_opposing_traffic");
    effective_deceleration_opposing_traffic = get_or_declare_parameter<double>(
      node, "obstacle_stop.stop_planning.effective_deceleration_opposing_traffic");
    hold_stop_velocity_threshold = get_or_declare_parameter<double>(
      node, "obstacle_stop.stop_planning.hold_stop_velocity_threshold");
    hold_stop_distance_threshold = get_or_declare_parameter<double>(
      node, "obstacle_stop.stop_planning.hold_stop_distance_threshold");
    pointcloud_suppression_distance_margin = get_or_declare_parameter<double>(
      node, "obstacle_stop.stop_planning.pointcloud_suppression_distance_margin");
    enable_approaching_on_curve = get_or_declare_parameter<bool>(
      node, "obstacle_stop.stop_planning.stop_on_curve.enable_approaching");
    additional_stop_margin_on_curve = get_or_declare_parameter<double>(
      node, "obstacle_stop.stop_planning.stop_on_curve.additional_stop_margin");
    min_stop_margin_on_curve = get_or_declare_parameter<double>(
      node, "obstacle_stop.stop_planning.stop_on_curve.min_stop_margin");
    rss_params.use_rss_stop =
      get_or_declare_parameter<bool>(node, "obstacle_stop.stop_planning.rss_params.use_rss_stop");
    rss_params.two_wheel_objects_deceleration = get_or_declare_parameter<double>(
      node, "obstacle_stop.stop_planning.rss_params.two_wheel_objects_deceleration");
    rss_params.vehicle_objects_deceleration = get_or_declare_parameter<double>(
      node, "obstacle_stop.stop_planning.rss_params.vehicle_objects_deceleration");
    rss_params.no_wheel_objects_deceleration = get_or_declare_parameter<double>(
      node, "obstacle_stop.stop_planning.rss_params.no_wheel_objects_deceleration");
    rss_params.pointcloud_deceleration = get_or_declare_parameter<double>(
      node, "obstacle_stop.stop_planning.rss_params.pointcloud_deceleration");
    rss_params.velocity_offset = get_or_declare_parameter<double>(
      node, "obstacle_stop.stop_planning.rss_params.velocity_offset");
    obstacle_velocity_threshold_enter_fixed_stop = get_or_declare_parameter<double>(
      node, "obstacle_stop.stop_planning.obstacle_velocity_threshold_enter_fixed_stop");
    obstacle_velocity_threshold_exit_fixed_stop = get_or_declare_parameter<double>(
      node, "obstacle_stop.stop_planning.obstacle_velocity_threshold_exit_fixed_stop");

    const std::string param_prefix = "obstacle_stop.stop_planning.object_type_specified_params.";
    const auto object_types =
      get_or_declare_parameter<std::vector<std::string>>(node, param_prefix + "types");

    for (const auto & type_str : object_types) {
      if (type_str != "default") {
        ObjectTypeSpecificParams param{
          get_or_declare_parameter<double>(node, param_prefix + type_str + ".limit_min_acc"),
          get_or_declare_parameter<double>(
            node, param_prefix + type_str + ".sudden_object_acc_threshold"),
          get_or_declare_parameter<double>(
            node, param_prefix + type_str + ".sudden_object_dist_threshold"),
          get_or_declare_parameter<bool>(node, param_prefix + type_str + ".abandon_to_stop")};

        param.sudden_object_acc_threshold =
          std::min(param.sudden_object_acc_threshold, common_param.limit_min_accel);
        param.limit_min_acc = std::min(param.limit_min_acc, param.sudden_object_acc_threshold);

        object_type_specific_param_map.emplace(type_str, param);
      }
    }
  }

  std::string get_param_type(const ObjectClassWithPointCloud extended_label)
  {
    if (extended_label.is_point_cloud) {
      return "default";
    }
    const auto type_str = object_types_maps.at(extended_label.object_classification.label);
    if (object_type_specific_param_map.count(type_str) == 0) {
      return "default";
    }
    return type_str;
  }
  ObjectTypeSpecificParams get_param(const ObjectClassWithPointCloud extended_label)
  {
    return object_type_specific_param_map.at(get_param_type(extended_label));
  }
};
}  // namespace autoware::motion_velocity_planner

#endif  // PARAMETERS_HPP_
