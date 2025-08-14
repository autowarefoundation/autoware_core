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

#ifndef TYPES_HPP_
#define TYPES_HPP_

#include "type_alias.hpp"

#include <autoware/motion_utils/marker/marker_helper.hpp>
#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/motion_velocity_planner_common/utils.hpp>
#include <autoware/signal_processing/lowpass_filter_1d.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner
{

static const std::vector<std::string> object_label_list = {
  "pointcloud", "unknown", "car", "truck", "bus", "trailer", "motorcycle", "bicycle", "pedestrian"};
static const std::unordered_map<uint8_t, std::string> object_types_maps = {
  {ObjectClassification::UNKNOWN, "unknown"}, {ObjectClassification::CAR, "car"},
  {ObjectClassification::TRUCK, "truck"},     {ObjectClassification::BUS, "bus"},
  {ObjectClassification::TRAILER, "trailer"}, {ObjectClassification::MOTORCYCLE, "motorcycle"},
  {ObjectClassification::BICYCLE, "bicycle"}, {ObjectClassification::PEDESTRIAN, "pedestrian"}};

// TODO(takagi): std::pair<geometry_msgs::msg::Point, double> in mvp should be replaced with
// CollisionPointWithDist
struct CollisionPointWithDist
{
  geometry_msgs::msg::Point point{};
  double dist_to_collide{};
};

struct PointcloudStopCandidate
{
  std::vector<double> initial_velocities{};
  autoware::signal_processing::LowpassFilter1d vel_lpf{0.0};
  rclcpp::Time latest_collision_pointcloud_time;
  CollisionPointWithDist latest_collision_point;
};

struct StopObstacle
{
  StopObstacle(
    const std::string & arg_uuid, const rclcpp::Time & arg_stamp,
    const ObjectClassification & object_classification, const geometry_msgs::msg::Pose & arg_pose,
    const Shape & arg_shape, const double arg_lon_velocity,
    const geometry_msgs::msg::Point & arg_collision_point,
    const double arg_dist_to_collide_on_decimated_traj,
    const std::optional<double> arg_braking_dist = std::nullopt)
  : uuid(arg_uuid),
    stamp(arg_stamp),
    pose(arg_pose),
    velocity(arg_lon_velocity),
    shape(arg_shape),
    collision_point(arg_collision_point),
    dist_to_collide_on_decimated_traj(arg_dist_to_collide_on_decimated_traj),
    classification(object_types_maps.at(object_classification.label)),
    braking_dist(arg_braking_dist)
  {
  }
  StopObstacle(
    const rclcpp::Time & arg_stamp, const bool arg_is_point_cloud, const double arg_lon_velocity,
    const geometry_msgs::msg::Point & arg_collision_point,
    const double arg_dist_to_collide_on_decimated_traj,
    const std::optional<double> arg_braking_dist = std::nullopt)
  : uuid("point_cloud"),
    stamp(arg_stamp),
    velocity(arg_lon_velocity),
    collision_point(arg_collision_point),
    dist_to_collide_on_decimated_traj(arg_dist_to_collide_on_decimated_traj),
    classification("pointcloud"),
    braking_dist(arg_braking_dist)
  {
    shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  }
  std::string uuid;
  rclcpp::Time stamp;
  geometry_msgs::msg::Pose pose;  // interpolated with the current stamp
  double velocity;                // longitudinal velocity against ego's trajectory

  Shape shape;
  geometry_msgs::msg::Point
    collision_point;  // TODO(yuki_takagi): this member variable still used in
                      // calculateMarginFromObstacleOnCurve() and  should be removed as it can be
                      // replaced by ”dist_to_collide_on_decimated_traj”
  double dist_to_collide_on_decimated_traj;
  std::string classification;
  std::optional<double> braking_dist;
};

struct DebugData
{
  DebugData() = default;
  std::vector<std::shared_ptr<PlannerData::Object>> intentionally_ignored_obstacles;
  std::vector<StopObstacle> obstacles_to_stop;
  std::vector<Polygon2d> decimated_traj_polys;
  MarkerArray stop_wall_marker;
};

}  // namespace autoware::motion_velocity_planner

#endif  // TYPES_HPP_
