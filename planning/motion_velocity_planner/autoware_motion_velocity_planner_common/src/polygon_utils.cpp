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

#include "autoware/motion_velocity_planner_common/polygon_utils.hpp"

#include "autoware/motion_utils/trajectory/trajectory.hpp"

#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::polygon_utils
{
namespace
{
inline Point2d msg_to_2d(const geometry_msgs::msg::Point & point)
{
  return Point2d{point.x, point.y};
}

PointWithStamp calc_nearest_collision_point(
  const size_t first_within_idx, const std::vector<PointWithStamp> & collision_points,
  const std::vector<TrajectoryPoint> & decimated_traj_points, const bool is_driving_forward)
{
  const size_t prev_idx = first_within_idx == 0 ? first_within_idx : first_within_idx - 1;
  const size_t next_idx = first_within_idx == 0 ? first_within_idx + 1 : first_within_idx;

  std::vector<geometry_msgs::msg::Pose> segment_points{
    decimated_traj_points.at(prev_idx).pose, decimated_traj_points.at(next_idx).pose};
  if (!is_driving_forward) {
    std::reverse(segment_points.begin(), segment_points.end());
  }

  std::vector<double> dist_vec;
  for (const auto & collision_point : collision_points) {
    const double dist = autoware::motion_utils::calcLongitudinalOffsetToSegment(
      segment_points, 0, collision_point.point);
    dist_vec.push_back(dist);
  }

  const size_t min_idx = std::min_element(dist_vec.begin(), dist_vec.end()) - dist_vec.begin();
  return collision_points.at(min_idx);
}

// NOTE: max_dist is used for efficient calculation to suppress boost::geometry's polygon
// calculation.
std::optional<std::pair<size_t, std::vector<PointWithStamp>>> get_collision_index(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const geometry_msgs::msg::Point & object_position, const rclcpp::Time & object_time,
  const Polygon2d & obj_polygon, const double max_dist = std::numeric_limits<double>::max())
{
  for (size_t i = 0; i < traj_polygons.size(); ++i) {
    const double approximated_dist =
      autoware_utils_geometry::calc_distance2d(traj_points.at(i).pose.position, object_position);
    if (approximated_dist > max_dist) {
      continue;
    }

    std::vector<Polygon2d> collision_polygons;
    boost::geometry::intersection(traj_polygons.at(i), obj_polygon, collision_polygons);

    std::vector<PointWithStamp> collision_geom_points;
    bool has_collision = false;
    for (const auto & collision_polygon : collision_polygons) {
      if (boost::geometry::area(collision_polygon) > 0.0) {
        has_collision = true;

        for (const auto & collision_point : collision_polygon.outer()) {
          PointWithStamp collision_geom_point;
          collision_geom_point.stamp = object_time;
          collision_geom_point.point.x = collision_point.x();
          collision_geom_point.point.y = collision_point.y();
          collision_geom_point.point.z = traj_points.at(i).pose.position.z;
          collision_geom_points.push_back(collision_geom_point);
        }
      }
    }

    if (has_collision) {
      const auto collision_info =
        std::pair<size_t, std::vector<PointWithStamp>>{i, collision_geom_points};
      return collision_info;
    }
  }

  return std::nullopt;
}

Polygon2d create_pose_footprint(
  const geometry_msgs::msg::Pose & pose, const VehicleInfo & vehicle_info, const double left_margin,
  const double right_margin)
{
  using autoware_utils_geometry::calc_offset_pose;
  const double half_width = vehicle_info.vehicle_width_m / 2.0;
  const auto point0 =
    calc_offset_pose(pose, vehicle_info.max_longitudinal_offset_m, half_width + left_margin, 0.0)
      .position;
  const auto point1 =
    calc_offset_pose(pose, vehicle_info.max_longitudinal_offset_m, -half_width - right_margin, 0.0)
      .position;
  const auto point2 =
    calc_offset_pose(pose, -vehicle_info.rear_overhang_m, -half_width - right_margin, 0.0).position;
  const auto point3 =
    calc_offset_pose(pose, -vehicle_info.rear_overhang_m, half_width + left_margin, 0.0).position;

  Polygon2d polygon;
  boost::geometry::append(polygon, msg_to_2d(point0));
  boost::geometry::append(polygon, msg_to_2d(point1));
  boost::geometry::append(polygon, msg_to_2d(point2));
  boost::geometry::append(polygon, msg_to_2d(point3));
  boost::geometry::append(polygon, msg_to_2d(point0));

  boost::geometry::correct(polygon);
  return polygon;
};

std::optional<std::pair<geometry_msgs::msg::Point, double>> find_max_penetration_point(
  const std::vector<Polygon2d> & collision_polygons, const geometry_msgs::msg::Pose & bumper_pose,
  const double trajectory_z)
{
  std::optional<std::pair<geometry_msgs::msg::Point, double>> result{std::nullopt};

  for (const auto & collision_polygon : collision_polygons) {
    for (const auto & collision_point_2d : collision_polygon.outer()) {
      geometry_msgs::msg::Point collision_point_3d;
      collision_point_3d.x = collision_point_2d.x();
      collision_point_3d.y = collision_point_2d.y();
      collision_point_3d.z = trajectory_z;

      const double dist_from_bumper = std::abs(
        autoware_utils_geometry::inverse_transform_point(collision_point_3d, bumper_pose).x);

      if (!result.has_value() || dist_from_bumper > result->second) {
        result = std::make_pair(collision_point_3d, dist_from_bumper);
      }
    }
  }

  return result;
}

}  // namespace

// FIXME(soblin): convergence should be applied from nearest_idx ?
std::vector<geometry_msgs::msg::Pose> calculate_error_poses(
  const std::vector<TrajectoryPoint> & traj_points,
  const geometry_msgs::msg::Pose & current_ego_pose, const double time_to_convergence)
{
  std::vector<geometry_msgs::msg::Pose> error_poses;
  error_poses.reserve(traj_points.size());

  const size_t nearest_idx =
    autoware::motion_utils::findNearestSegmentIndex(traj_points, current_ego_pose.position);
  const auto nearest_pose = traj_points.at(nearest_idx).pose;
  const auto current_ego_pose_error =
    autoware_utils_geometry::inverse_transform_pose(current_ego_pose, nearest_pose);
  const double current_ego_lat_error = current_ego_pose_error.position.y;
  const double current_ego_yaw_error = tf2::getYaw(current_ego_pose_error.orientation);
  double time_elapsed{0.0};

  for (size_t i = 0; i < traj_points.size(); ++i) {
    if (time_elapsed >= time_to_convergence) {
      break;
    }

    const double rem_ratio = (time_to_convergence - time_elapsed) / time_to_convergence;
    geometry_msgs::msg::Pose indexed_pose_err;
    indexed_pose_err.set__orientation(
      autoware_utils_geometry::create_quaternion_from_yaw(current_ego_yaw_error * rem_ratio));
    indexed_pose_err.set__position(
      autoware_utils_geometry::create_point(0.0, current_ego_lat_error * rem_ratio, 0.0));
    error_poses.push_back(
      autoware_utils_geometry::transform_pose(indexed_pose_err, traj_points.at(i).pose));

    if (traj_points.at(i).longitudinal_velocity_mps != 0.0 && i < traj_points.size() - 1) {
      time_elapsed += autoware_utils::calc_distance2d(
                        traj_points.at(i).pose.position, traj_points.at(i + 1).pose.position) /
                      std::abs(traj_points.at(i).longitudinal_velocity_mps);
    } else {
      time_elapsed = std::numeric_limits<double>::max();
    }
  }
  return error_poses;
}

std::optional<std::pair<geometry_msgs::msg::Point, double>> get_collision_point(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const geometry_msgs::msg::Point obj_position, [[maybe_unused]] const rclcpp::Time obj_stamp,
  const Polygon2d & obj_polygon, const double x_offset_to_bumper)
{
  assert(traj_points.size() == traj_polygons.size());
  const double obj_maximum_length = boost::geometry::perimeter(obj_polygon) * 0.5;

  std::optional<std::pair<geometry_msgs::msg::Point, double>> nearest_collision{std::nullopt};

  for (size_t i = 0; i < traj_polygons.size(); ++i) {
    const double ego_maximum_length = boost::geometry::perimeter(traj_polygons.at(i)) * 0.5;
    const double center_dist =
      autoware_utils_geometry::calc_distance2d(traj_points.at(i).pose.position, obj_position);
    if (center_dist > obj_maximum_length + ego_maximum_length) {
      continue;
    }

    const double current_arc_length = motion_utils::calcSignedArcLength(traj_points, 0, i);

    if (
      nearest_collision.has_value() &&
      std::abs(current_arc_length - x_offset_to_bumper) > std::abs(nearest_collision->second)) {
      break;
    }

    std::vector<Polygon2d> collision_polygons;
    boost::geometry::intersection(traj_polygons.at(i), obj_polygon, collision_polygons);
    if (collision_polygons.empty()) {
      continue;
    }
    const auto bumper_pose = autoware_utils_geometry::calc_offset_pose(
      traj_points.at(i).pose, x_offset_to_bumper, 0.0, 0.0);
    const auto max_penetration = find_max_penetration_point(
      collision_polygons, bumper_pose, traj_points.at(i).pose.position.z);

    if (max_penetration.has_value()) {
      const auto & [collision_point, max_penetration_dist] = *max_penetration;
      const double dist_to_collide = current_arc_length - max_penetration_dist;

      if (
        !nearest_collision.has_value() ||
        std::abs(dist_to_collide) < std::abs(nearest_collision->second)) {
        nearest_collision = std::make_pair(collision_point, dist_to_collide);
      }
    }
  }

  return nearest_collision;
}

// NOTE: max_lat_dist is used for efficient calculation to suppress boost::geometry's polygon
// calculation.
std::vector<PointWithStamp> get_collision_points(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const rclcpp::Time & obstacle_stamp, const PredictedPath & predicted_path, const Shape & shape,
  const rclcpp::Time & current_time, const bool is_driving_forward,
  std::vector<size_t> & collision_index, const double max_lat_dist,
  const double max_prediction_time_for_collision_check)
{
  std::vector<PointWithStamp> collision_points;
  for (size_t i = 0; i < predicted_path.path.size(); ++i) {
    if (
      max_prediction_time_for_collision_check <
      rclcpp::Duration(predicted_path.time_step).seconds() * static_cast<double>(i)) {
      break;
    }

    const auto object_time =
      rclcpp::Time(obstacle_stamp) + rclcpp::Duration(predicted_path.time_step) * i;
    // Ignore past position
    if ((object_time - current_time).seconds() < 0.0) {
      continue;
    }

    const auto collision_info = get_collision_index(
      traj_points, traj_polygons, predicted_path.path.at(i).position, object_time,
      autoware_utils_geometry::to_polygon2d(predicted_path.path.at(i), shape), max_lat_dist);
    if (collision_info) {
      const auto nearest_collision_point = calc_nearest_collision_point(
        collision_info->first, collision_info->second, traj_points, is_driving_forward);
      collision_points.push_back(nearest_collision_point);
      collision_index.push_back(collision_info->first);
    }
  }

  return collision_points;
}

// Calculate the off-tracking of the front outer wheel.
// This is defined as the difference in turning radii between the front and rear outer wheels.
std::vector<double> calc_front_outer_wheel_off_tracking(
  const std::vector<TrajectoryPoint> & traj_points, const VehicleInfo & vehicle_info)
{
  auto curvature_vec = motion_utils::calcCurvature(traj_points);
  if (motion_utils::isDrivingForward(traj_points) == false) {
    std::transform(curvature_vec.begin(), curvature_vec.end(), curvature_vec.begin(), [](double c) {
      return -c;
    });
  }

  std::vector<double> front_outer_wheel_off_track(curvature_vec.size(), 0.0);
  std::transform(
    curvature_vec.begin(), curvature_vec.end(), front_outer_wheel_off_track.begin(),
    [&vehicle_info](double base_link_curvature) {
      // Calculate the curvature of the outer rear wheel's path from the curvature at the rear axle
      // center.
      const double base_link_outer_wheel_curvature =
        base_link_curvature /
        (1.0 + std::abs(base_link_curvature) * vehicle_info.vehicle_width_m / 2.0);
      // Calculate the front outer wheel's off-tracking distance.
      // The absolute value of this formula is equivalent to:
      // std::hypot(radius_front_outer_wheel, wheel_base) - radius_front_outer_wheel;
      return -1.0 * vehicle_info.wheel_base_m *
             std::tan(0.5 * std::atan(base_link_outer_wheel_curvature * vehicle_info.wheel_base_m));
    });

  return front_outer_wheel_off_track;
}

std::vector<Polygon2d> create_one_step_polygons(
  const std::vector<TrajectoryPoint> & traj_points, const VehicleInfo & vehicle_info,
  const geometry_msgs::msg::Pose & current_ego_pose, const double lat_margin,
  const bool enable_to_consider_current_pose, const double time_to_convergence,
  [[maybe_unused]] const double decimate_trajectory_step_length,
  const double additional_front_outer_wheel_off_track_scale)
{
  using autoware_utils_geometry::calc_offset_pose;
  const double front_length = vehicle_info.max_longitudinal_offset_m;
  const double rear_length = vehicle_info.rear_overhang_m;
  const double half_width = vehicle_info.vehicle_width_m / 2.0;

  const auto error_poses =
    enable_to_consider_current_pose
      ? calculate_error_poses(traj_points, current_ego_pose, time_to_convergence)
      : std::vector<geometry_msgs::msg::Pose>{};
  const auto front_outer_wheel_off_tracks =
    additional_front_outer_wheel_off_track_scale > 0.0
      ? calc_front_outer_wheel_off_tracking(traj_points, vehicle_info)
      : std::vector<double>(traj_points.size(), 0.0);

  std::vector<Polygon2d> output_polygons;
  Polygon2d tmp_polys{};
  for (size_t i = 0; i < traj_points.size(); ++i) {
    std::vector<geometry_msgs::msg::Pose> current_poses = {traj_points.at(i).pose};
    if (i < error_poses.size()) {
      current_poses.push_back(error_poses.at(i));
    }

    const double left_margin = lat_margin + additional_front_outer_wheel_off_track_scale *
                                              std::max(front_outer_wheel_off_tracks.at(i), 0.0);
    const double right_margin = lat_margin + additional_front_outer_wheel_off_track_scale *
                                               std::max(-front_outer_wheel_off_tracks.at(i), 0.0);

    Polygon2d idx_poly{};
    for (const auto & pose : current_poses) {
      if (i == 0 && traj_points.at(i).longitudinal_velocity_mps > 1e-3) {
        const auto point0 =
          calc_offset_pose(pose, front_length, half_width + left_margin, 0.0).position;
        const auto point1 =
          calc_offset_pose(pose, front_length, -half_width - right_margin, 0.0).position;
        const auto point2 = calc_offset_pose(pose, -rear_length, -half_width, 0.0).position;
        const auto point3 = calc_offset_pose(pose, -rear_length, half_width, 0.0).position;

        boost::geometry::append(idx_poly, msg_to_2d(point0));
        boost::geometry::append(idx_poly, msg_to_2d(point1));
        boost::geometry::append(idx_poly, msg_to_2d(point2));
        boost::geometry::append(idx_poly, msg_to_2d(point3));
        boost::geometry::append(idx_poly, msg_to_2d(point0));
      } else {
        boost::geometry::append(
          idx_poly, create_pose_footprint(pose, vehicle_info, left_margin, right_margin).outer());
      }
    }

    boost::geometry::append(tmp_polys, idx_poly.outer());
    Polygon2d hull_polygon;
    boost::geometry::convex_hull(tmp_polys, hull_polygon);
    boost::geometry::correct(hull_polygon);

    output_polygons.push_back(hull_polygon);
    tmp_polys = std::move(idx_poly);
  }
  return output_polygons;
}
}  // namespace autoware::motion_velocity_planner::polygon_utils
