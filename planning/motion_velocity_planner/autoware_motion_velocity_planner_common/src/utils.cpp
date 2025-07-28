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

#include "autoware/motion_velocity_planner_common/utils.hpp"

#include "autoware/motion_utils/resample/resample.hpp"
#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/motion_velocity_planner_common/planner_data.hpp"

#include <autoware_utils_visualization/marker_helper.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::utils
{
namespace
{
TrajectoryPoint extend_trajectory_point(
  const double extend_distance, const TrajectoryPoint & goal_point, const bool is_driving_forward)
{
  TrajectoryPoint extended_trajectory_point;
  extended_trajectory_point.pose = autoware_utils_geometry::calc_offset_pose(
    goal_point.pose, extend_distance * (is_driving_forward ? 1.0 : -1.0), 0.0, 0.0);
  extended_trajectory_point.longitudinal_velocity_mps = goal_point.longitudinal_velocity_mps;
  extended_trajectory_point.lateral_velocity_mps = goal_point.lateral_velocity_mps;
  extended_trajectory_point.acceleration_mps2 = goal_point.acceleration_mps2;
  return extended_trajectory_point;
}

}  // namespace

std::vector<TrajectoryPoint> get_extended_trajectory_points(
  const std::vector<TrajectoryPoint> & input_points, const double extend_distance,
  const double step_length)
{
  auto output_points = input_points;
  const auto is_driving_forward_opt =
    autoware::motion_utils::isDrivingForwardWithTwist(input_points);
  const bool is_driving_forward = is_driving_forward_opt ? *is_driving_forward_opt : true;

  if (extend_distance < std::numeric_limits<double>::epsilon()) {
    return output_points;
  }

  const auto goal_point = input_points.back();

  double extend_sum = 0.0;
  while (extend_sum <= (extend_distance - step_length)) {
    const auto extended_trajectory_point =
      extend_trajectory_point(extend_sum, goal_point, is_driving_forward);
    output_points.push_back(extended_trajectory_point);
    extend_sum += step_length;
  }
  const auto extended_trajectory_point =
    extend_trajectory_point(extend_distance, goal_point, is_driving_forward);
  output_points.push_back(extended_trajectory_point);

  return output_points;
}

std::vector<TrajectoryPoint> resample_trajectory_points(
  const std::vector<TrajectoryPoint> & traj_points, const double interval)
{
  const auto traj_msg = autoware::motion_utils::convertToTrajectory(traj_points);
  const auto resampled_traj_msg = autoware::motion_utils::resampleTrajectory(traj_msg, interval);
  auto resampled_traj = autoware::motion_utils::convertToTrajectoryPointArray(resampled_traj_msg);
  const bool is_driving_forward =
    autoware_utils_geometry::is_driving_forward(traj_points.at(0), traj_points.at(1));
  autoware::motion_utils::insertOrientationAsSpline(resampled_traj, is_driving_forward);
  return resampled_traj;
}

std::vector<TrajectoryPoint> decimate_trajectory_points_from_ego(
  const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & current_pose,
  const double ego_nearest_dist_threshold, const double ego_nearest_yaw_threshold,
  const double decimate_trajectory_step_length, const double goal_extended_trajectory_length)
{
  // trim trajectory points from ego pose
  const size_t traj_ego_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      traj_points, current_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
  const auto traj_points_from_ego =
    std::vector<TrajectoryPoint>(traj_points.begin() + traj_ego_seg_idx, traj_points.end());

  // decimate trajectory
  const auto decimated_traj_points_from_ego =
    resample_trajectory_points(traj_points_from_ego, decimate_trajectory_step_length);

  // extend trajectory
  const auto extended_traj_points_from_ego = get_extended_trajectory_points(
    decimated_traj_points_from_ego, goal_extended_trajectory_length,
    decimate_trajectory_step_length);
  if (extended_traj_points_from_ego.size() < 2) {
    return traj_points;
  }
  return extended_traj_points_from_ego;
}
geometry_msgs::msg::Point to_geometry_point(const pcl::PointXYZ & point)
{
  geometry_msgs::msg::Point geom_point;
  geom_point.x = point.x;
  geom_point.y = point.y;
  geom_point.z = point.z;
  return geom_point;
}

geometry_msgs::msg::Point to_geometry_point(const autoware_utils_geometry::Point2d & point)
{
  geometry_msgs::msg::Point geom_point;
  geom_point.x = point.x();
  geom_point.y = point.y();
  return geom_point;
}

std::optional<double> calc_distance_to_front_object(
  const std::vector<TrajectoryPoint> & traj_points, const size_t ego_idx,
  const geometry_msgs::msg::Point & obstacle_pos)
{
  const size_t obstacle_idx = autoware::motion_utils::findNearestIndex(traj_points, obstacle_pos);
  const auto ego_to_obstacle_distance =
    autoware::motion_utils::calcSignedArcLength(traj_points, ego_idx, obstacle_idx);
  if (ego_to_obstacle_distance < 0.0) return std::nullopt;
  return ego_to_obstacle_distance;
}

std::vector<uint8_t> get_target_object_type(rclcpp::Node & node, const std::string & param_prefix)
{
  std::unordered_map<std::string, uint8_t> types_map{
    {"unknown", ObjectClassification::UNKNOWN}, {"car", ObjectClassification::CAR},
    {"truck", ObjectClassification::TRUCK},     {"bus", ObjectClassification::BUS},
    {"trailer", ObjectClassification::TRAILER}, {"motorcycle", ObjectClassification::MOTORCYCLE},
    {"bicycle", ObjectClassification::BICYCLE}, {"pedestrian", ObjectClassification::PEDESTRIAN}};

  std::vector<uint8_t> types;
  for (const auto & type : types_map) {
    if (node.declare_parameter<bool>(param_prefix + type.first)) {
      types.push_back(type.second);
    }
  }
  return types;
}

double calc_object_possible_max_dist_from_center(const Shape & shape)
{
  if (shape.type == Shape::BOUNDING_BOX) {
    return std::hypot(shape.dimensions.x / 2.0, shape.dimensions.y / 2.0);
  } else if (shape.type == Shape::CYLINDER) {
    return shape.dimensions.x / 2.0;
  } else if (shape.type == Shape::POLYGON) {
    double max_length_to_point = 0.0;
    for (const auto rel_point : shape.footprint.points) {
      const double length_to_point = std::hypot(rel_point.x, rel_point.y);
      if (max_length_to_point < length_to_point) {
        max_length_to_point = length_to_point;
      }
    }
    return max_length_to_point;
  }

  throw std::logic_error("The shape type is not supported in motion_velocity_planner_common.");
}
visualization_msgs::msg::Marker get_object_marker(
  const geometry_msgs::msg::Pose & obj_pose, size_t idx, const std::string & ns, const double r,
  const double g, const double b)
{
  const auto current_time = rclcpp::Clock().now();

  auto marker = autoware_utils_visualization::create_default_marker(
    "map", current_time, ns, idx, visualization_msgs::msg::Marker::SPHERE,
    autoware_utils_visualization::create_marker_scale(2.0, 2.0, 2.0),
    autoware_utils_visualization::create_marker_color(r, g, b, 0.8));

  marker.pose = obj_pose;

  return marker;
}

double calc_possible_min_dist_from_obj_to_traj_poly(
  const std::shared_ptr<PlannerData::Object> object,
  const std::vector<TrajectoryPoint> & traj_points, const VehicleInfo & vehicle_info)
{
  const double object_possible_max_dist =
    calc_object_possible_max_dist_from_center(object->predicted_object.shape);
  // The minimum lateral distance to the trajectory polygon is estimated by assuming that the
  // ego-vehicle's front right or left corner is the furthest from the trajectory, in the very worst
  // case
  const double ego_possible_max_dist =
    std::hypot(vehicle_info.max_longitudinal_offset_m, vehicle_info.vehicle_width_m / 2.0);
  const double possible_min_dist_to_traj_poly =
    std::abs(object->get_dist_to_traj_lateral(traj_points)) - ego_possible_max_dist -
    object_possible_max_dist;
  return possible_min_dist_to_traj_poly;
}

double get_dist_to_traj_poly(
  const geometry_msgs::msg::Point & point,
  const std::vector<autoware_utils::Polygon2d> & decimated_traj_polys)
{
  const auto point_2d = autoware_utils_geometry::Point2d(point.x, point.y);

  double dist_to_traj_poly = std::numeric_limits<double>::infinity();

  for (const auto & decimated_traj_poly : decimated_traj_polys) {
    const double current_dist_to_traj_poly =
      boost::geometry::distance(decimated_traj_poly, point_2d);

    dist_to_traj_poly = std::min(dist_to_traj_poly, current_dist_to_traj_poly);
  }

  return dist_to_traj_poly;
}

}  // namespace autoware::motion_velocity_planner::utils
