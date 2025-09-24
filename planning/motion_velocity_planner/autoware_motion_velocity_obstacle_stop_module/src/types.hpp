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
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner
{

struct StopObstacleClassification
{
  enum class Type {
    UNKNOWN,
    CAR,
    TRUCK,
    BUS,
    TRAILER,
    MOTORCYCLE,
    BICYCLE,
    PEDESTRIAN,
    POINTCLOUD
  };

  inline static const std::unordered_map<Type, std::string> to_string_map = {
    {Type::UNKNOWN, "unknown"},      {Type::CAR, "car"},
    {Type::TRUCK, "truck"},          {Type::BUS, "bus"},
    {Type::TRAILER, "trailer"},      {Type::MOTORCYCLE, "motorcycle"},
    {Type::BICYCLE, "bicycle"},      {Type::PEDESTRIAN, "pedestrian"},
    {Type::POINTCLOUD, "pointcloud"}};

  explicit StopObstacleClassification(const ObjectClassification object_classification)
  {
    switch (object_classification.label) {
      case ObjectClassification::UNKNOWN:
        label = Type::UNKNOWN;
        break;
      case ObjectClassification::CAR:
        label = Type::CAR;
        break;
      case ObjectClassification::TRUCK:
        label = Type::TRUCK;
        break;
      case ObjectClassification::BUS:
        label = Type::BUS;
        break;
      case ObjectClassification::TRAILER:
        label = Type::TRAILER;
        break;
      case ObjectClassification::MOTORCYCLE:
        label = Type::MOTORCYCLE;
        break;
      case ObjectClassification::BICYCLE:
        label = Type::BICYCLE;
        break;
      case ObjectClassification::PEDESTRIAN:
        label = Type::PEDESTRIAN;
        break;
      default:
        throw std::invalid_argument("Undefined ObjectClassification label");
    }
  }
  explicit StopObstacleClassification(
    const std::vector<ObjectClassification> & object_classifications)
  : StopObstacleClassification(object_classifications.at(0))
  {
  }
  explicit StopObstacleClassification(Type v) : label(v) {}
  StopObstacleClassification() = default;

  std::string to_string() const { return to_string_map.at(label); }

  Type label{};

  bool operator==(const StopObstacleClassification & other) const { return label == other.label; }
  bool operator!=(const StopObstacleClassification & other) const { return !(*this == other); }
};

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

struct PolygonParam
{
  std::optional<double> trimming_length{};
  double lateral_margin{};
  double off_track_scale{};

  bool operator<(const PolygonParam & other) const
  {
    return std::tie(trimming_length, lateral_margin, off_track_scale) <
           std::tie(other.trimming_length, other.lateral_margin, other.off_track_scale);
  }
};

struct StopObstacle
{
  StopObstacle(
    const UUID & arg_uuid, const rclcpp::Time & arg_stamp,
    const StopObstacleClassification & arg_object_classification,
    const geometry_msgs::msg::Pose & arg_pose, const Shape & arg_shape,
    const double arg_lon_velocity, const geometry_msgs::msg::Point & arg_collision_point,
    const double arg_dist_to_collide_on_decimated_traj, const PolygonParam & arg_polygon_param,
    const std::optional<double> arg_braking_dist = std::nullopt)
  : uuid(arg_uuid),
    stamp(arg_stamp),
    pose(arg_pose),
    velocity(arg_lon_velocity),
    shape(arg_shape),
    collision_point(arg_collision_point),
    dist_to_collide_on_decimated_traj(arg_dist_to_collide_on_decimated_traj),
    classification(arg_object_classification),
    polygon_param(arg_polygon_param),
    braking_dist(arg_braking_dist)
  {
  }
  StopObstacle(
    const rclcpp::Time & arg_stamp, const StopObstacleClassification & arg_object_classification,
    const double arg_lon_velocity, const geometry_msgs::msg::Point & arg_collision_point,
    const double arg_dist_to_collide_on_decimated_traj, const PolygonParam & arg_polygon_param,
    const std::optional<double> arg_braking_dist = std::nullopt)
  : stamp(arg_stamp),
    velocity(arg_lon_velocity),
    collision_point(arg_collision_point),
    dist_to_collide_on_decimated_traj(arg_dist_to_collide_on_decimated_traj),
    classification(arg_object_classification),
    polygon_param(arg_polygon_param),
    braking_dist(arg_braking_dist)
  {
    if (arg_object_classification.label != StopObstacleClassification::Type::POINTCLOUD) {
      throw std::invalid_argument(
        "Constructor for pointcloud StopObstacle must be called with POINTCLOUD label");
    }
    pose.position = arg_collision_point;
    shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  }
  UUID uuid{};
  rclcpp::Time stamp;
  geometry_msgs::msg::Pose pose;  // interpolated with the current stamp
  double velocity;                // longitudinal velocity against ego's trajectory

  Shape shape;
  geometry_msgs::msg::Point
    collision_point;  // TODO(yuki_takagi): this member variable still used in
                      // calculateMarginFromObstacleOnCurve() and  should be removed as it can be
                      // replaced by ”dist_to_collide_on_decimated_traj”
  double dist_to_collide_on_decimated_traj;
  StopObstacleClassification classification;
  PolygonParam polygon_param;
  std::optional<double> braking_dist;
};

struct DetectionPolygon
{
  const std::vector<TrajectoryPoint> traj_points;
  const std::vector<Polygon2d> polygons;
  DetectionPolygon(std::vector<TrajectoryPoint> && points, std::vector<Polygon2d> && polys)
  : traj_points(std::move(points)), polygons(std::move(polys))
  {
    if (traj_points.size() != polygons.size()) {
      throw std::invalid_argument("Vector sizes must be identical for DetectionPolygon.");
    }
  }
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
