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

#include "autoware/motion_velocity_planner_common/planner_data.hpp"

#include "autoware/motion_velocity_planner_common/polygon_utils.hpp"
#include "autoware/motion_velocity_planner_common/utils.hpp"
#include "autoware/object_recognition_utils/predicted_path_utils.hpp"
#include "autoware_lanelet2_extension/utility/query.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_math/normalization.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/LineString.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner
{
using autoware_perception_msgs::msg::PredictedPath;
namespace bg = boost::geometry;

namespace
{
std::optional<geometry_msgs::msg::Pose> get_predicted_object_pose_from_predicted_path(
  const PredictedPath & predicted_path, const rclcpp::Time & obj_stamp,
  const rclcpp::Time & current_stamp)
{
  const double rel_time = (current_stamp - obj_stamp).seconds();
  if (rel_time < 0.0) {
    return std::nullopt;
  }

  const auto pose =
    autoware::object_recognition_utils::calcInterpolatedPose(predicted_path, rel_time);
  if (!pose) {
    return std::nullopt;
  }
  return pose.get();
}

std::optional<geometry_msgs::msg::Pose> get_predicted_object_pose_from_predicted_paths(
  const std::vector<PredictedPath> & predicted_paths, const rclcpp::Time & obj_stamp,
  const rclcpp::Time & current_stamp)
{
  if (predicted_paths.empty()) {
    return std::nullopt;
  }

  // Get the most reliable path
  const auto predicted_path = std::max_element(
    predicted_paths.begin(), predicted_paths.end(),
    [](const PredictedPath & a, const PredictedPath & b) { return a.confidence < b.confidence; });

  return get_predicted_object_pose_from_predicted_path(*predicted_path, obj_stamp, current_stamp);
}
}  // namespace

// @brief make a single enveloped polygon from the all trajectory polygons
pcl::PointCloud<pcl::PointXYZ>::Ptr crop_by_monolithic_trajectory_polygon(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_pointcloud_ptr,
  const PointcloudPreprocessParams::FilterByTrajectoryPolygon & filter_by_trajectory_param,
  const std::vector<autoware_utils_geometry::Polygon2d> & traj_polygons,
  const std::vector<TrajectoryPoint> & decimated_trajectory,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  pcl::CropBox<pcl::PointXYZ> crop_filter;
  crop_filter.setInputCloud(input_pointcloud_ptr);

  // make xy min, max
  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::lowest();
  for (const auto & poly : traj_polygons) {
    for (const auto & point : poly.outer()) {
      x_min = std::min(x_min, point[0]);
      x_max = std::max(x_max, point[0]);
      y_min = std::min(y_min, point[1]);
      y_max = std::max(y_max, point[1]);
    }
  }
  auto lowest_traj_height = std::numeric_limits<double>::max();
  auto highest_traj_height = std::numeric_limits<double>::lowest();
  for (const auto & trajectory_point : decimated_trajectory) {
    lowest_traj_height = std::min(lowest_traj_height, trajectory_point.pose.position.z);
    highest_traj_height = std::max(highest_traj_height, trajectory_point.pose.position.z);
  }
  crop_filter.setMin(Eigen::Vector4f(
    x_min, y_min, lowest_traj_height - filter_by_trajectory_param.height_margin, 1.0f));
  crop_filter.setMax(Eigen::Vector4f(
    x_max, y_max,
    highest_traj_height + vehicle_info.vehicle_height_m + filter_by_trajectory_param.height_margin,
    1.0f));

  auto ret_pointcloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  crop_filter.filter(*ret_pointcloud_ptr);

  return ret_pointcloud_ptr;
  // TODO(yuki takagi): monolithic crop_box filter may be cared on the ego_coordinate system.
}

pcl::PointCloud<pcl::PointXYZ>::Ptr filter_by_multi_trajectory_polygon(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_pointcloud_ptr,
  const std::vector<autoware_utils_geometry::Polygon2d> & traj_polygons)
{
  auto ret_pointcloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  ret_pointcloud_ptr->header = input_pointcloud_ptr->header;
  // Define types for Boost.Geometry
  namespace bg = boost::geometry;
  namespace bgi = boost::geometry::index;
  using BoostPoint2D = bg::model::point<double, 2, bg::cs::cartesian>;
  using BoostValue = std::pair<BoostPoint2D, size_t>;  // point + index

  // Build R-tree from input points
  std::vector<BoostValue> rtree_data;
  rtree_data.reserve(input_pointcloud_ptr->points.size());

  {
    std::transform(
      input_pointcloud_ptr->points.begin(), input_pointcloud_ptr->points.end(),
      std::back_inserter(rtree_data), [i = 0](const pcl::PointXYZ & pt) mutable {
        return std::make_pair(BoostPoint2D(pt.x, pt.y), i++);
      });
  }

  bgi::rtree<BoostValue, bgi::quadratic<16>> rtree(rtree_data.begin(), rtree_data.end());

  std::unordered_set<size_t> selected_indices;

  std::for_each(
    traj_polygons.begin(), traj_polygons.end(), [&](const Polygon2d & one_step_polygon) {
      bg::model::box<BoostPoint2D> bbox;
      bg::envelope(one_step_polygon, bbox);

      std::vector<BoostValue> result_s;
      rtree.query(bgi::intersects(bbox), std::back_inserter(result_s));

      for (const auto & val : result_s) {
        const BoostPoint2D & pt = val.first;
        if (bg::within(pt, one_step_polygon)) {
          selected_indices.insert(val.second);
        }
      }
    });

  ret_pointcloud_ptr->points.reserve(selected_indices.size());
  std::transform(
    selected_indices.begin(), selected_indices.end(),
    std::back_inserter(ret_pointcloud_ptr->points),
    [&](const size_t idx) { return input_pointcloud_ptr->points[idx]; });
  return ret_pointcloud_ptr;
}  // namespace autoware::motion_velocity_planner

pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_by_voxel_grid(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_pointcloud_ptr,
  const PointcloudPreprocessParams::DownsampleByVoxelGrid & downsample_params)
{
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(input_pointcloud_ptr);
  filter.setLeafSize(
    downsample_params.voxel_size_x, downsample_params.voxel_size_y, downsample_params.voxel_size_z);
  auto ret_pointcloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  filter.filter(*ret_pointcloud_ptr);

  return ret_pointcloud_ptr;
}

std::vector<pcl::PointIndices> make_cluster_indices(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_pointcloud_ptr,
  const PointcloudPreprocessParams::EuclideanClustering & clustering_params)
{
  std::vector<pcl::PointIndices> ret_clusters{};

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(input_pointcloud_ptr);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(clustering_params.cluster_tolerance);
  ec.setMinClusterSize(clustering_params.min_cluster_size);
  ec.setMaxClusterSize(clustering_params.max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(input_pointcloud_ptr);
  ec.extract(ret_clusters);

  return ret_clusters;
}

std::vector<pcl::PointIndices> make_individual_cluster_indices(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_pointcloud_ptr)
{
  std::vector<pcl::PointIndices> ret_clusters{};

  ret_clusters.resize(input_pointcloud_ptr->size());
  for (size_t i = 0; i < input_pointcloud_ptr->size(); ++i) {
    ret_clusters[i].indices.emplace_back(i);
  }

  return ret_clusters;
}  // namespace autoware::motion_velocity_planner

PlannerData::PlannerData(rclcpp::Node & node)
: no_ground_pointcloud(node),
  vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo())

{
  // nearest search
  ego_nearest_dist_threshold = get_or_declare_parameter<double>(node, "ego_nearest_dist_threshold");
  ego_nearest_yaw_threshold = get_or_declare_parameter<double>(node, "ego_nearest_yaw_threshold");

  trajectory_polygon_collision_check.decimate_trajectory_step_length =
    get_or_declare_parameter<double>(
      node, "trajectory_polygon_collision_check.decimate_trajectory_step_length");
  trajectory_polygon_collision_check.goal_extended_trajectory_length =
    get_or_declare_parameter<double>(
      node, "trajectory_polygon_collision_check.goal_extended_trajectory_length");
  trajectory_polygon_collision_check.enable_to_consider_current_pose =
    get_or_declare_parameter<bool>(
      node,
      "trajectory_polygon_collision_check.consider_current_pose.enable_to_consider_current_pose");
  trajectory_polygon_collision_check.time_to_convergence = get_or_declare_parameter<double>(
    node, "trajectory_polygon_collision_check.consider_current_pose.time_to_convergence");
}
std::optional<TrafficSignalStamped> PlannerData::get_traffic_signal(
  const lanelet::Id id, const bool keep_last_observation) const
{
  const auto & traffic_light_id_map =
    keep_last_observation ? traffic_light_id_map_last_observed_ : traffic_light_id_map_raw_;
  if (traffic_light_id_map.count(id) == 0) {
    return std::nullopt;
  }
  return std::make_optional<TrafficSignalStamped>(traffic_light_id_map.at(id));
}

std::optional<double> PlannerData::calculate_min_deceleration_distance(
  const double target_velocity) const
{
  return motion_utils::calcDecelDistWithJerkAndAccConstraints(
    std::abs(current_odometry.twist.twist.linear.x), target_velocity,
    current_acceleration.accel.accel.linear.x, velocity_smoother_->getMinDecel(),
    std::abs(velocity_smoother_->getMinJerk()), velocity_smoother_->getMinJerk());
}

double PlannerData::Object::get_dist_to_traj_poly(
  const std::vector<autoware_utils_geometry::Polygon2d> & decimated_traj_polys) const
{
  if (!dist_to_traj_poly) {
    const auto & obj_pose = predicted_object.kinematics.initial_pose_with_covariance.pose;
    const auto obj_poly = autoware_utils_geometry::to_polygon2d(obj_pose, predicted_object.shape);
    dist_to_traj_poly = std::numeric_limits<double>::max();
    for (const auto & traj_poly : decimated_traj_polys) {
      const double current_dist_to_traj_poly = bg::distance(traj_poly, obj_poly);
      dist_to_traj_poly = std::min(*dist_to_traj_poly, current_dist_to_traj_poly);
    }
  }
  return *dist_to_traj_poly;
}

double PlannerData::Object::get_dist_to_traj_lateral(
  const std::vector<TrajectoryPoint> & traj_points) const
{
  if (!dist_to_traj_lateral) {
    const auto & obj_pos = predicted_object.kinematics.initial_pose_with_covariance.pose.position;
    dist_to_traj_lateral = autoware::motion_utils::calcLateralOffset(traj_points, obj_pos);
  }
  return *dist_to_traj_lateral;
}

double PlannerData::Object::get_dist_from_ego_longitudinal(
  const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Point & ego_pos) const
{
  if (!dist_from_ego_longitudinal) {
    const auto & obj_pos = predicted_object.kinematics.initial_pose_with_covariance.pose.position;
    dist_from_ego_longitudinal =
      autoware::motion_utils::calcSignedArcLength(traj_points, ego_pos, obj_pos);
  }
  return *dist_from_ego_longitudinal;
}

double PlannerData::Object::get_lon_vel_relative_to_traj(
  const std::vector<TrajectoryPoint> & traj_points) const
{
  if (!lon_vel_relative_to_traj) {
    calc_vel_relative_to_traj(traj_points);
  }
  return *lon_vel_relative_to_traj;
}

double PlannerData::Object::get_lat_vel_relative_to_traj(
  const std::vector<TrajectoryPoint> & traj_points) const
{
  if (!lat_vel_relative_to_traj) {
    calc_vel_relative_to_traj(traj_points);
  }
  return *lat_vel_relative_to_traj;
}

void PlannerData::Object::calc_vel_relative_to_traj(
  const std::vector<TrajectoryPoint> & traj_points) const
{
  const auto & obj_pose = predicted_object.kinematics.initial_pose_with_covariance.pose;
  const auto & obj_twist = predicted_object.kinematics.initial_twist_with_covariance.twist;

  const size_t object_idx =
    autoware::motion_utils::findNearestIndex(traj_points, obj_pose.position);
  const auto & nearest_traj_point = traj_points.at(object_idx);

  const double traj_yaw = tf2::getYaw(nearest_traj_point.pose.orientation);
  const double obj_yaw = tf2::getYaw(obj_pose.orientation);
  const Eigen::Rotation2Dd R_ego_to_obstacle(
    autoware_utils_math::normalize_radian(obj_yaw - traj_yaw));

  // Calculate the trajectory direction and the vector from the trajectory to the obstacle
  const Eigen::Vector2d traj_direction(std::cos(traj_yaw), std::sin(traj_yaw));
  const Eigen::Vector2d traj_to_obstacle(
    obj_pose.position.x - nearest_traj_point.pose.position.x,
    obj_pose.position.y - nearest_traj_point.pose.position.y);

  // Determine if the obstacle is to the left or right of the trajectory using the cross product
  const double cross_product =
    traj_direction.x() * traj_to_obstacle.y() - traj_direction.y() * traj_to_obstacle.x();
  const int sign = (cross_product > 0) ? -1 : 1;

  const Eigen::Vector2d obstacle_velocity(obj_twist.linear.x, obj_twist.linear.y);
  const Eigen::Vector2d projected_velocity = R_ego_to_obstacle * obstacle_velocity;

  lon_vel_relative_to_traj = projected_velocity[0];
  lat_vel_relative_to_traj = sign * projected_velocity[1];
}

geometry_msgs::msg::Pose PlannerData::Object::get_predicted_current_pose(
  const rclcpp::Time & current_stamp, const rclcpp::Time & predicted_objects_stamp) const
{
  if (!predicted_pose) {
    predicted_pose = calc_predicted_pose(current_stamp, predicted_objects_stamp);
  }
  return *predicted_pose;
}

geometry_msgs::msg::Pose PlannerData::Object::calc_predicted_pose(
  const rclcpp::Time & time, const rclcpp::Time & predicted_objects_stamp) const
{
  const auto predicted_pose_opt = get_predicted_object_pose_from_predicted_paths(
    predicted_object.kinematics.predicted_paths, predicted_objects_stamp, time);
  if (!predicted_pose_opt) {
    RCLCPP_WARN(
      rclcpp::get_logger("motion_velocity_planner_common"),
      "Failed to calculate the predicted object pose.");
    return predicted_object.kinematics.initial_pose_with_covariance.pose;
  }
  return *predicted_pose_opt;
}

void PlannerData::process_predicted_objects(
  const autoware_perception_msgs::msg::PredictedObjects & predicted_objects)
{
  predicted_objects_header = predicted_objects.header;

  objects.clear();
  for (const auto & predicted_object : predicted_objects.objects) {
    objects.push_back(std::make_shared<Object>(predicted_object));
  }
}

std::vector<StopPoint> PlannerData::calculate_map_stop_points(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory) const
{
  std::vector<StopPoint> stop_points;
  if (!route_handler) {
    return stop_points;
  }
  autoware_utils_geometry::LineString2d trajectory_ls;
  for (const auto & p : trajectory) {
    trajectory_ls.emplace_back(p.pose.position.x, p.pose.position.y);
  }
  const auto candidates = route_handler->getLaneletMapPtr()->laneletLayer.search(
    boost::geometry::return_envelope<lanelet::BoundingBox2d>(trajectory_ls));
  for (const auto & candidate : candidates) {
    const auto stop_lines = lanelet::utils::query::stopLinesLanelet(candidate);
    for (const auto & stop_line : stop_lines) {
      const auto stop_line_2d = lanelet::utils::to2D(stop_line).basicLineString();
      autoware_utils_geometry::MultiPoint2d intersections;
      boost::geometry::intersection(trajectory_ls, stop_line_2d, intersections);
      for (const auto & intersection : intersections) {
        const auto p =
          geometry_msgs::msg::Point().set__x(intersection.x()).set__y(intersection.y());
        const auto stop_line_arc_length = motion_utils::calcSignedArcLength(trajectory, 0UL, p);
        StopPoint sp;
        sp.ego_trajectory_arc_length =
          stop_line_arc_length - vehicle_info_.max_longitudinal_offset_m;
        if (sp.ego_trajectory_arc_length < 0.0) {
          continue;
        }
        sp.stop_line = stop_line_2d;
        sp.ego_stop_pose =
          motion_utils::calcInterpolatedPose(trajectory, sp.ego_trajectory_arc_length);
        stop_points.push_back(sp);
      }
    }
  }
  return stop_points;
}

std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, std::vector<pcl::PointIndices>>
PlannerData::Pointcloud::filter_and_cluster_point_clouds(
  const std::vector<TrajectoryPoint> & raw_trajectory,
  const nav_msgs::msg::Odometry & current_odometry, double min_deceleration_distance,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check,
  const double ego_nearest_dist_threshold, const double ego_nearest_yaw_threshold)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr ret_pointcloud_ptr = pointcloud.makeShared();
  std::vector<pcl::PointIndices> ret_clusters{};

  const auto & filter_by_trajectory_param = preprocess_params_.filter_by_trajectory_polygon;
  const auto & traj_poly_param = trajectory_polygon_collision_check;
  if (
    !raw_trajectory.empty() && (filter_by_trajectory_param.enable_monolithic_crop_box ||
                                filter_by_trajectory_param.enable_multi_polygon_filtering)) {
    const auto decimated_trajectory =
      autoware::motion_velocity_planner::utils::decimate_trajectory_points_from_ego(
        raw_trajectory, current_odometry.pose.pose, ego_nearest_dist_threshold,
        ego_nearest_yaw_threshold, traj_poly_param.decimate_trajectory_step_length,
        traj_poly_param.goal_extended_trajectory_length);

    const double trajectory_trim_length =
      filter_by_trajectory_param.min_trajectory_length +
      min_deceleration_distance * filter_by_trajectory_param.braking_distance_scale_factor;
    const auto & trimmed_trajectory =
      motion_utils::isDrivingForward(raw_trajectory)
        ? motion_utils::cropForwardPoints(
            decimated_trajectory, decimated_trajectory.front().pose.position, 0,
            trajectory_trim_length)
        : decimated_trajectory;

    const auto traj_polygons =
      autoware::motion_velocity_planner::polygon_utils::create_one_step_polygons(
        trimmed_trajectory, vehicle_info, current_odometry.pose.pose,
        filter_by_trajectory_param.lateral_margin, traj_poly_param.enable_to_consider_current_pose,
        traj_poly_param.time_to_convergence, traj_poly_param.decimate_trajectory_step_length);

    if (filter_by_trajectory_param.enable_monolithic_crop_box && !ret_pointcloud_ptr->empty()) {
      const auto input_pointcloud_ptr = ret_pointcloud_ptr;
      ret_pointcloud_ptr = crop_by_monolithic_trajectory_polygon(
        input_pointcloud_ptr, filter_by_trajectory_param, traj_polygons, decimated_trajectory,
        vehicle_info);
    }
    if (filter_by_trajectory_param.enable_multi_polygon_filtering && !ret_pointcloud_ptr->empty()) {
      const auto input_pointcloud_ptr = ret_pointcloud_ptr;
      ret_pointcloud_ptr = autoware::motion_velocity_planner::filter_by_multi_trajectory_polygon(
        input_pointcloud_ptr, traj_polygons);
    }
  }

  const auto & downsample_params = preprocess_params_.downsample_by_voxel_grid;
  if (downsample_params.enable_downsample && !ret_pointcloud_ptr->empty()) {
    const auto input_pointcloud_ptr = ret_pointcloud_ptr;
    ret_pointcloud_ptr = downsample_by_voxel_grid(input_pointcloud_ptr, downsample_params);
  }

  const auto & clustering_param = preprocess_params_.euclidean_clustering;
  if (clustering_param.enable_clustering && !ret_pointcloud_ptr->empty()) {
    ret_clusters = make_cluster_indices(ret_pointcloud_ptr, clustering_param);
  } else {
    ret_clusters = make_individual_cluster_indices(ret_pointcloud_ptr);
  }
  return std::make_pair(ret_pointcloud_ptr, ret_clusters);
}

}  // namespace autoware::motion_velocity_planner
