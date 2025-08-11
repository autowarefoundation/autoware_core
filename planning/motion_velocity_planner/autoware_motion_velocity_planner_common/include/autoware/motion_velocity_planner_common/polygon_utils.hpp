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

#ifndef AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__POLYGON_UTILS_HPP_
#define AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__POLYGON_UTILS_HPP_

#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <rclcpp/time.hpp>

#include "autoware_perception_msgs/msg/predicted_path.hpp"
#include "autoware_perception_msgs/msg/shape.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"

#include <boost/geometry.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner
{
namespace polygon_utils
{
namespace bg = boost::geometry;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;

using autoware_perception_msgs::msg::PredictedPath;
using autoware_perception_msgs::msg::Shape;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
namespace bg = boost::geometry;
using autoware::vehicle_info_utils::VehicleInfo;

struct PointWithStamp
{
  rclcpp::Time stamp;
  geometry_msgs::msg::Point point;
};

/**
 * @pre `traj_points` and `traj_polygons` have same size
 * @brief find the first `traj_polygons` that collide with `obj_polygon` and amount the collision
 * points, return the point whose distance from ego's bumper is farthest
 */
std::optional<std::pair<geometry_msgs::msg::Point, double>> get_collision_point(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const geometry_msgs::msg::Point obj_position, const rclcpp::Time obj_stamp,
  const Polygon2d & obj_polygon, const double dist_to_bumper);

/**
 * @brief for each prediction step of `predicted_path`, upto
 * `max_prediction_time_for_collision_check`, compute the representative collision point with
 * `traj_polygons`
 * @param collision_index contains the indices of `traj_polygons` that collide with the object for
 * some prediction step
 * @pre `traj_points` and `traj_polygons` have same size
 * @post the output and `collision_index` have same size
 * @note "bumper" denotes front line of ego if ego is driving forward, otherwise rear line of ego
 */
std::vector<PointWithStamp> get_collision_points(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const rclcpp::Time & obstacle_stamp, const PredictedPath & predicted_path, const Shape & shape,
  const rclcpp::Time & current_time, const bool is_driving_forward,
  std::vector<size_t> & collision_index, const double max_dist = std::numeric_limits<double>::max(),
  const double max_prediction_time_for_collision_check = std::numeric_limits<double>::max());

/**
 * @brief return MultiPolygon whose each element represents a convex polygon comprising footprint at
 * the corresponding index position + footprint at the previous index position
 * @param enable_to_consider_current_pose if true, `current_ego_pose`
 * @post the output has the same size as `traj_points`
 */
std::vector<Polygon2d> create_one_step_polygons(
  const std::vector<TrajectoryPoint> & traj_points, const VehicleInfo & vehicle_info,
  const geometry_msgs::msg::Pose & current_ego_pose, const double lat_margin,
  const bool enable_to_consider_current_pose, const double time_to_convergence,
  const double decimate_trajectory_step_length);
}  // namespace polygon_utils
}  // namespace autoware::motion_velocity_planner

#endif  // AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__POLYGON_UTILS_HPP_
