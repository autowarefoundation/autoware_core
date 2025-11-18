// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#include "obstacle_stop_module.hpp"

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_rclcpp/parameter.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>

#include <autoware_perception_msgs/msg/detail/shape__struct.hpp>

#include <algorithm>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner
{
using autoware_utils_rclcpp::get_or_declare_parameter;

namespace
{

double calc_minimum_distance_to_stop(
  const double initial_vel, const double max_acc, const double min_acc)
{
  if (initial_vel < 0.0) {
    return -std::pow(initial_vel, 2) / 2.0 / max_acc;
  }

  return -std::pow(initial_vel, 2) / 2.0 / min_acc;
}

double calc_estimation_time(
  const PredictedObject & predicted_object, const ObstacleFilteringParam & obstacle_filtering_param)
{
  // convert constant deceleration to constant velocity
  // In this feature, we are assuming the pedestrians will decelerate by specified value,
  // hence the travel distance is derived as v^2/2a.
  // However, to maintain the compatibility with the other objects,
  // we have to capsulize this distance information as time with constant velocity assumption.
  // Therefore here we return a value (v^2/2a) / v = v/2a as the equivalent estimation time.
  const auto equivalent_estimation_time = [&]() {
    if (
      obstacle_filtering_param.outside_obstacle.deceleration <=
      std::numeric_limits<double>::epsilon()) {
      return std::numeric_limits<double>::infinity();
    }
    const auto & twist = predicted_object.kinematics.initial_twist_with_covariance.twist;
    return std::hypot(twist.linear.x, twist.linear.y) * 0.5 /
           obstacle_filtering_param.outside_obstacle.deceleration;
  };
  return std::clamp(
    equivalent_estimation_time(), 0.0,
    obstacle_filtering_param.outside_obstacle.estimation_time_horizon);
}

autoware_utils_geometry::Point2d convert_point(const geometry_msgs::msg::Point & p)
{
  return autoware_utils_geometry::Point2d{p.x, p.y};
}

std::vector<TrajectoryPoint> resample_trajectory_points(
  const std::vector<TrajectoryPoint> & traj_points, const double interval)
{
  const auto traj = autoware::motion_utils::convertToTrajectory(traj_points);
  const auto resampled_traj = autoware::motion_utils::resampleTrajectory(traj, interval);
  return autoware::motion_utils::convertToTrajectoryPointArray(resampled_traj);
}

std::vector<PredictedPath> resample_highest_confidence_predicted_paths(
  const std::vector<PredictedPath> & predicted_paths, const double time_interval,
  const double time_horizon, const size_t num_paths)
{
  std::vector<PredictedPath> sorted_paths = predicted_paths;

  // Sort paths by descending confidence
  std::sort(
    sorted_paths.begin(), sorted_paths.end(),
    [](const PredictedPath & a, const PredictedPath & b) { return a.confidence > b.confidence; });

  std::vector<PredictedPath> selected_paths;
  size_t path_count = 0;

  // Select paths that meet the confidence thresholds
  for (const auto & path : sorted_paths) {
    if (path_count < num_paths) {
      selected_paths.push_back(path);
      ++path_count;
    }
  }

  // Resample each selected path
  std::vector<PredictedPath> resampled_paths;
  for (const auto & path : selected_paths) {
    if (path.path.size() < 2) {
      continue;
    }
    resampled_paths.push_back(
      autoware::object_recognition_utils::resamplePredictedPath(path, time_interval, time_horizon));
  }

  return resampled_paths;
}

double calc_x_offset_to_bumper(const bool is_driving_forward, const VehicleInfo & vehicle_info)
{
  if (is_driving_forward) {
    return vehicle_info.max_longitudinal_offset_m;
  }
  return vehicle_info.min_longitudinal_offset_m;
}

Float64Stamped create_float64_stamped(const rclcpp::Time & now, const float & data)
{
  Float64Stamped msg;
  msg.stamp = now;
  msg.data = data;
  return msg;
}

double calc_time_to_reach_collision_point(
  const Odometry & odometry, const geometry_msgs::msg::Point & collision_point,
  const std::vector<TrajectoryPoint> & traj_points, const double x_offset_to_bumper,
  const double margin_distance, const double min_velocity_to_reach_collision_point)
{
  const double dist_from_ego_to_obstacle =
    std::abs(
      autoware::motion_utils::calcSignedArcLength(
        traj_points, odometry.pose.pose.position, collision_point) -
      x_offset_to_bumper) -
    margin_distance;
  return dist_from_ego_to_obstacle /
         std::max(min_velocity_to_reach_collision_point, std::abs(odometry.twist.twist.linear.x));
}

// TODO(takagi): refactor this function as same as obstacle_filtering_param
double calc_braking_dist_along_trajectory(
  const StopObstacleClassification::Type label, const double lon_vel, const RSSParam & rss_params)
{
  const double braking_acc = [&]() {
    if (label == StopObstacleClassification::Type::POINTCLOUD) {
      return rss_params.pointcloud_deceleration;
    }
    if (
      label == StopObstacleClassification::Type::UNKNOWN ||
      label == StopObstacleClassification::Type::PEDESTRIAN) {
      return rss_params.no_wheel_objects_deceleration;
    }
    if (
      label == StopObstacleClassification::Type::BICYCLE ||
      label == StopObstacleClassification::Type::MOTORCYCLE) {
      return rss_params.two_wheel_objects_deceleration;
    }
    return rss_params.vehicle_objects_deceleration;
  }();
  const double error_considered_vel = std::max(lon_vel + rss_params.velocity_offset, 0.0);
  return error_considered_vel * error_considered_vel * 0.5 / -braking_acc;
}

PolygonParam create_polygon_param(
  const ObstacleFilteringParam::TrimTrajectoryParam & trim_trajectory_param,
  const std::optional<double> ego_braking_distance,
  const ObstacleFilteringParam::LateralMarginParam & lateral_margin_param,
  const std::optional<double> object_velocity)
{
  PolygonParam p;
  if (!trim_trajectory_param.enable_trimming || !ego_braking_distance.has_value()) {
    p.trimming_length = std::nullopt;
  } else {
    p.trimming_length =
      trim_trajectory_param.min_trajectory_length +
      trim_trajectory_param.braking_distance_scale_factor * ego_braking_distance.value();
  }
  p.lateral_margin = lateral_margin_param.nominal_margin +
                     (object_velocity > lateral_margin_param.is_moving_threshold_velocity
                        ? lateral_margin_param.additional_is_moving_margin
                        : lateral_margin_param.additional_is_stop_margin);
  p.off_track_scale = lateral_margin_param.additional_wheel_off_track_scale;
  return p;
}

}  // namespace

void ObstacleStopModule::init(rclcpp::Node & node, const std::string & module_name)
{
  module_name_ = module_name;
  clock_ = node.get_clock();
  logger_ = node.get_logger();

  // ros parameters
  ignore_crossing_obstacle_ =
    get_or_declare_parameter<bool>(node, "obstacle_stop.option.ignore_crossing_obstacle");
  suppress_sudden_stop_ =
    get_or_declare_parameter<bool>(node, "obstacle_stop.option.suppress_sudden_stop");

  common_param_ = CommonParam(node);
  stop_planning_param_ = StopPlanningParam(node, common_param_);
  for (const auto & [type, str] : StopObstacleClassification::to_string_map) {
    obstacle_filtering_params_.emplace(type, ObstacleFilteringParam{node, str});
  }
  pointcloud_segmentation_param_ = PointcloudSegmentationParam(node);

  const double update_distance_th =
    get_or_declare_parameter<double>(node, "obstacle_stop.stop_planning.update_distance_th");
  const double min_off_duration =
    get_or_declare_parameter<double>(node, "obstacle_stop.stop_planning.min_off_duration");
  const double min_on_duration =
    get_or_declare_parameter<double>(node, "obstacle_stop.stop_planning.min_on_duration");

  path_length_buffer_ = autoware::motion_velocity_planner::obstacle_stop::PathLengthBuffer(
    update_distance_th, min_off_duration, min_on_duration);

  // common publisher
  processing_time_publisher_ =
    node.create_publisher<Float64Stamped>("~/debug/obstacle_stop/processing_time_ms", 1);
  virtual_wall_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacle_stop/virtual_walls", 1);
  debug_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacle_stop/debug_markers", 1);

  // module publisher
  debug_stop_planning_info_pub_ =
    node.create_publisher<Float32MultiArrayStamped>("~/debug/obstacle_stop/planning_info", 1);
  processing_time_detail_pub_ = node.create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms/obstacle_stop", 1);
  // interface publisher
  objects_of_interest_marker_interface_ = std::make_unique<
    autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface>(
    &node, "obstacle_stop");
  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      &node, "obstacle_stop");

  // time keeper
  time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>(processing_time_detail_pub_);
}

void ObstacleStopModule::update_parameters(const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils_rclcpp::update_param;

  update_param(
    parameters, "obstacle_stop.option.ignore_crossing_obstacle", ignore_crossing_obstacle_);
  update_param(parameters, "obstacle_stop.option.suppress_sudden_stop", suppress_sudden_stop_);
}

VelocityPlanningResult ObstacleStopModule::plan(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & raw_trajectory_points,
  [[maybe_unused]] const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> &
    smoothed_trajectory_points,
  const std::shared_ptr<const PlannerData> planner_data)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  // 1. init variables
  stop_watch_.tic();
  debug_data_ptr_ = std::make_shared<DebugData>();
  const double x_offset_to_bumper =
    calc_x_offset_to_bumper(planner_data->is_driving_forward, planner_data->vehicle_info_);
  stop_planning_debug_info_.reset();
  stop_planning_debug_info_.set(
    StopPlanningDebugInfo::TYPE::EGO_VELOCITY, planner_data->current_odometry.twist.twist.linear.x);
  stop_planning_debug_info_.set(
    StopPlanningDebugInfo::TYPE::EGO_ACCELERATION,
    planner_data->current_acceleration.accel.accel.linear.x);
  trajectory_polygon_for_inside_map_.clear();
  decimated_traj_polys_ = std::nullopt;

  // 2. pre-process
  const auto decimated_traj_points = utils::decimate_trajectory_points_from_ego(
    raw_trajectory_points, planner_data->current_odometry.pose.pose,
    planner_data->ego_nearest_dist_threshold, planner_data->ego_nearest_yaw_threshold,
    planner_data->trajectory_polygon_collision_check.decimate_trajectory_step_length,
    stop_planning_param_.stop_margin);

  // 3. filter obstacles of predicted objects
  auto stop_obstacles_for_predicted_object = filter_stop_obstacle_for_predicted_object(
    planner_data->current_odometry, planner_data->ego_nearest_dist_threshold,
    planner_data->ego_nearest_yaw_threshold,
    rclcpp::Time(planner_data->predicted_objects_header.stamp), raw_trajectory_points,
    decimated_traj_points, planner_data->objects, planner_data->vehicle_info_, x_offset_to_bumper,
    planner_data->trajectory_polygon_collision_check);

  // 4. filter obstacles of point cloud
  auto stop_obstacles_for_point_cloud = filter_stop_obstacle_for_point_cloud(
    planner_data->current_odometry, raw_trajectory_points, decimated_traj_points,
    planner_data->no_ground_pointcloud, planner_data->vehicle_info_, x_offset_to_bumper,
    planner_data->trajectory_polygon_collision_check);

  // 5. concat stop obstacles by predicted objects and point cloud
  const std::vector<StopObstacle> stop_obstacles =
    autoware::motion_velocity_planner::utils::concat_vectors(
      std::move(stop_obstacles_for_predicted_object), std::move(stop_obstacles_for_point_cloud));

  // 6. plan stop
  const auto stop_point =
    plan_stop(planner_data, raw_trajectory_points, stop_obstacles, x_offset_to_bumper);

  // 7. publish messages for debugging
  publish_debug_info();

  // 8. generate VelocityPlanningResult
  VelocityPlanningResult result;
  if (stop_point) {
    result.stop_points.push_back(*stop_point);
  }

  return result;
}

std::optional<double> ObstacleStopModule::calc_ego_forwarding_braking_distance(
  const std::vector<TrajectoryPoint> & traj_points, const Odometry & odometry) const
{
  if (traj_points.empty() || autoware::motion_utils::isDrivingForward(traj_points) != true) {
    return std::nullopt;
  }
  return autoware::motion_utils::calcDecelDistWithJerkAndAccConstraints(
    odometry.twist.twist.linear.x, 0.0, common_param_.max_accel, common_param_.min_accel,
    common_param_.max_jerk, common_param_.min_jerk);
}

std::optional<CollisionPointWithDist> ObstacleStopModule::get_nearest_collision_point(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const PlannerData::Pointcloud & point_cloud, const double x_offset_to_bumper,
  const VehicleInfo & vehicle_info) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  if (traj_points.size() != traj_polygons.size()) {
    RCLCPP_ERROR(
      logger_, "The size of trajectory points and polygons do not match: %zu vs %zu",
      traj_points.size(), traj_polygons.size());
    return std::nullopt;
  }

  if (point_cloud.pointcloud.empty()) {
    return std::nullopt;
  }

  const auto & clusters = point_cloud.get_cluster_indices();
  const auto & pointcloud_ptr = point_cloud.get_filtered_pointcloud_ptr();

  // height check will works even on the slope, but it is not accurate.
  const auto & height_margin = pointcloud_segmentation_param_.height_margin;
  std::vector<geometry_msgs::msg::Point> collision_geom_points{};
  for (size_t traj_index = 0; traj_index < traj_points.size(); ++traj_index) {
    const double rough_dist_th = boost::geometry::perimeter(traj_polygons.at(traj_index)) * 0.5;
    const double traj_height = traj_points.at(traj_index).pose.position.z;

    for (const auto & cluster : clusters) {
      for (const auto & point_index : cluster.indices) {
        const auto obstacle_point = autoware::motion_velocity_planner::utils::to_geometry_point(
          pointcloud_ptr->at(point_index));
        if (
          obstacle_point.z - traj_height < -height_margin.margin_from_bottom ||
          obstacle_point.z - traj_height >
            vehicle_info.max_height_offset_m + height_margin.margin_from_top) {
          continue;
        }
        const double dist_from_base_link =
          autoware_utils::calc_distance2d(traj_points.at(traj_index).pose, obstacle_point);
        if (dist_from_base_link > rough_dist_th) {
          continue;
        }
        Point2d obstacle_point_2d{obstacle_point.x, obstacle_point.y};
        if (boost::geometry::within(obstacle_point_2d, traj_polygons.at(traj_index))) {
          collision_geom_points.push_back(obstacle_point);
        }
      }
    }
    if (collision_geom_points.empty()) {
      continue;
    }

    const auto bumper_pose = autoware_utils::calc_offset_pose(
      traj_points.at(traj_index).pose, x_offset_to_bumper, 0.0, 0.0);
    std::optional<double> max_collision_length = std::nullopt;
    std::optional<geometry_msgs::msg::Point> max_collision_point = std::nullopt;
    for (const auto & point : collision_geom_points) {
      const double dist_from_bumper =
        std::abs(autoware_utils::inverse_transform_point(point, bumper_pose).x);

      if (!max_collision_length.has_value() || dist_from_bumper > *max_collision_length) {
        max_collision_length = dist_from_bumper;
        max_collision_point = point;
      }
    }
    return CollisionPointWithDist{
      *max_collision_point,
      autoware::motion_utils::calcSignedArcLength(traj_points, 0, traj_index) -
        *max_collision_length};
  }
  return std::nullopt;
}

std::vector<StopObstacle> ObstacleStopModule::filter_stop_obstacle_for_predicted_object(
  const Odometry & odometry, const double ego_nearest_dist_threshold,
  const double ego_nearest_yaw_threshold, const rclcpp::Time & predicted_objects_stamp,
  const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<TrajectoryPoint> & decimated_traj_points,
  const std::vector<std::shared_ptr<PlannerData::Object>> & objects,
  const VehicleInfo & vehicle_info, const double x_offset_to_bumper,
  const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & current_pose = odometry.pose.pose;

  std::vector<StopObstacle> stop_obstacles;
  for (const auto & object : objects) {
    autoware_utils_debug::ScopedTimeTrack st_for_each_object("for_each_object", *time_keeper_);

    const auto & filtering_params = obstacle_filtering_params_.at(
      StopObstacleClassification{object->predicted_object.classification}.label);

    // 1. rough filtering
    // 1.1. Check if the obstacle is in front of the ego.
    const double lon_dist_from_ego_to_obj =
      object->get_dist_from_ego_longitudinal(traj_points, current_pose.position);
    if (lon_dist_from_ego_to_obj < 0.0) {
      continue;
    }

    // 1.2. Check if the rough lateral distance is smaller than the threshold.
    const double min_lat_dist_to_traj_poly =
      utils::calc_possible_min_dist_from_obj_to_traj_poly(object, traj_points, vehicle_info);
    if (
      filtering_params.lateral_margin.max_margin(vehicle_info) <
      min_lat_dist_to_traj_poly - std::max(
                                    object->get_lat_vel_relative_to_traj(traj_points) *
                                      filtering_params.outside_obstacle.estimation_time_horizon,
                                    0.0)) {
      const auto obj_uuid_str =
        autoware_utils_uuid::to_hex_string(object->predicted_object.object_id);
      RCLCPP_DEBUG(
        logger_,
        "[Stop] Ignore obstacle (%s) since the rough lateral distance to the trajectory is too "
        "large.",
        obj_uuid_str.substr(0, 4).c_str());
      continue;
    }

    // 2. precise filtering
    const auto & decimated_traj_polys = [&]() {
      autoware_utils_debug::ScopedTimeTrack st_get_decimated_traj_polys(
        "get_decimated_traj_polys", *time_keeper_);
      return get_decimated_traj_polys(
        traj_points, current_pose, vehicle_info, ego_nearest_dist_threshold,
        ego_nearest_yaw_threshold, trajectory_polygon_collision_check);
    }();
    const double dist_from_obj_to_traj_poly = [&]() {
      autoware_utils_debug::ScopedTimeTrack st_get_dist_to_traj_poly(
        "get_dist_to_traj_poly", *time_keeper_);
      return object->get_dist_to_traj_poly(decimated_traj_polys);
    }();

    // 2.1. pick target object
    const auto current_step_stop_obstacle = pick_stop_obstacle_from_predicted_object(
      odometry, traj_points, decimated_traj_points, object, predicted_objects_stamp,
      dist_from_obj_to_traj_poly, vehicle_info, x_offset_to_bumper,
      trajectory_polygon_collision_check);
    if (current_step_stop_obstacle) {
      stop_obstacles.push_back(*current_step_stop_obstacle);
      continue;
    }
  }

  // Check target obstacles' consistency
  check_consistency(predicted_objects_stamp, objects, stop_obstacles);

  prev_stop_obstacles_ = stop_obstacles;

  RCLCPP_DEBUG(
    logger_, "The number of output obstacles of filter_stop_obstacles is %ld",
    stop_obstacles.size());
  return stop_obstacles;
}

void ObstacleStopModule::upsert_pointcloud_stop_candidates(
  const CollisionPointWithDist & nearest_collision_point,
  const std::vector<TrajectoryPoint> & traj_points, rclcpp::Time latest_point_cloud_time)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto & vel_params = pointcloud_segmentation_param_.velocity_estimation;
  const auto & assoc_params = pointcloud_segmentation_param_.time_series_association;

  // check association from the newest candidate to the oldest candidate
  for (auto stop_candidate = pointcloud_stop_candidates.rbegin();
       stop_candidate != pointcloud_stop_candidates.rend(); ++stop_candidate) {
    const double time_since_latest_collision =
      (latest_point_cloud_time - stop_candidate->latest_collision_pointcloud_time).seconds();
    if (time_since_latest_collision < 0.05) {
      return;  // latest_point_cloud_time is already checked.
    }
    const double longitudinal_displacement = autoware::motion_utils::calcSignedArcLength(
      traj_points, stop_candidate->latest_collision_point.point, nearest_collision_point.point);

    if (
      time_since_latest_collision < assoc_params.max_time_diff &&
      -assoc_params.position_diff + assoc_params.min_velocity * time_since_latest_collision <
        longitudinal_displacement &&
      longitudinal_displacement <
        assoc_params.position_diff + assoc_params.max_velocity * time_since_latest_collision) {
      const double clamped_vel = std::clamp(
        longitudinal_displacement / time_since_latest_collision, vel_params.min_clamp_velocity,
        vel_params.max_clamp_velocity);
      if (!stop_candidate->vel_lpf.getValue().has_value()) {
        auto & vel_vec = stop_candidate->initial_velocities;
        vel_vec.push_back(clamped_vel);
        if (vel_vec.size() >= vel_params.required_velocity_count) {
          stop_candidate->vel_lpf.reset(
            std::accumulate(vel_vec.begin(), vel_vec.end(), 0.0) / vel_vec.size());
        }
      } else {
        stop_candidate->vel_lpf.filter(clamped_vel);
      }
      stop_candidate->latest_collision_point = nearest_collision_point;
      stop_candidate->latest_collision_pointcloud_time = latest_point_cloud_time;

      std::sort(
        pointcloud_stop_candidates.begin(), pointcloud_stop_candidates.end(),
        [](const PointcloudStopCandidate & a, const PointcloudStopCandidate & b) {
          return a.latest_collision_pointcloud_time < b.latest_collision_pointcloud_time;
        });

      return;
    }
  }
  PointcloudStopCandidate new_stop_candidate;
  new_stop_candidate.latest_collision_point = nearest_collision_point;
  new_stop_candidate.latest_collision_pointcloud_time = latest_point_cloud_time;
  new_stop_candidate.vel_lpf.setGain(vel_params.lpf_gain);
  pointcloud_stop_candidates.push_back(new_stop_candidate);
}

std::vector<StopObstacle> ObstacleStopModule::filter_stop_obstacle_for_point_cloud(
  const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<TrajectoryPoint> & decimated_traj_points,
  const PlannerData::Pointcloud & point_cloud, const VehicleInfo & vehicle_info,
  const double x_offset_to_bumper,
  const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto & filtering_param =
    obstacle_filtering_params_.at(StopObstacleClassification::Type::POINTCLOUD);

  // outside stop for pointcloud is not implemented.
  if (!filtering_param.check_inside) {
    return std::vector<StopObstacle>{};
  }

  if (
    point_cloud.preprocess_params_.filter_by_trajectory_polygon.lateral_margin <
    filtering_param.lateral_margin.nominal_margin) {
    RCLCPP_WARN_ONCE(
      logger_,
      "pointcloud preprocessing lateral margin in motion_velocity_planner_node (%f) is smaller "
      "than obstacle_stop_module param (%f)",
      point_cloud.preprocess_params_.filter_by_trajectory_polygon.lateral_margin,
      filtering_param.lateral_margin.nominal_margin);
  }

  const auto & tp = trajectory_polygon_collision_check;
  const auto polygon_param = create_polygon_param(
    filtering_param.trim_trajectory, calc_ego_forwarding_braking_distance(traj_points, odometry),
    filtering_param.lateral_margin, std::nullopt);
  const auto detection_polygon_with_lat_margin = get_trajectory_polygon(
    decimated_traj_points, vehicle_info, odometry.pose.pose, polygon_param,
    tp.enable_to_consider_current_pose, tp.time_to_convergence, tp.decimate_trajectory_step_length);

  const auto nearest_collision_point = get_nearest_collision_point(
    detection_polygon_with_lat_margin.traj_points, detection_polygon_with_lat_margin.polygons,
    point_cloud, x_offset_to_bumper, vehicle_info);

  // update pointcloud_stop_candidates
  const auto latest_point_cloud_time =
    rclcpp::Time(point_cloud.pointcloud.header.stamp * static_cast<uint32_t>(1e3), RCL_ROS_TIME);
  if (nearest_collision_point) {
    upsert_pointcloud_stop_candidates(
      nearest_collision_point.value(), traj_points, latest_point_cloud_time);
  }

  // erase old data from the front of the deque
  // pointcloud_stop_candidates are sorted by latest_collision_pointcloud_time, so we can erase
  // old data from the front.
  while (
    !pointcloud_stop_candidates.empty() &&
    (latest_point_cloud_time - pointcloud_stop_candidates.front().latest_collision_pointcloud_time)
        .seconds() > filtering_param.stop_obstacle_hold_time_threshold) {
    pointcloud_stop_candidates.pop_front();
  }

  // pick stop_obstacle from candidates
  std::vector<StopObstacle> stop_obstacles;
  for (const auto & stop_candidate : pointcloud_stop_candidates) {
    if (!stop_candidate.vel_lpf.getValue().has_value()) {
      continue;
    }

    const double time_delay =
      (clock_->now() - stop_candidate.latest_collision_pointcloud_time).seconds();
    const double time_compensated_dist_to_collide =
      stop_candidate.latest_collision_point.dist_to_collide +
      *stop_candidate.vel_lpf.getValue() * time_delay;

    const bool use_estimated_velocity =
      pointcloud_segmentation_param_.velocity_estimation.use_estimated_velocity;
    if (
      !use_estimated_velocity ||
      *stop_candidate.vel_lpf.getValue() <
        stop_planning_param_.obstacle_velocity_threshold_enter_fixed_stop) {
      stop_obstacles.emplace_back(
        stop_candidate.latest_collision_pointcloud_time,
        StopObstacleClassification{StopObstacleClassification::Type::POINTCLOUD},
        stop_candidate.vel_lpf.getValue().value(), stop_candidate.latest_collision_point.point,
        time_compensated_dist_to_collide, polygon_param);
    } else if (stop_planning_param_.rss_params.use_rss_stop) {
      const auto braking_dist = calc_braking_dist_along_trajectory(
        StopObstacleClassification::Type::POINTCLOUD, *stop_candidate.vel_lpf.getValue(),
        stop_planning_param_.rss_params);
      stop_obstacles.emplace_back(
        stop_candidate.latest_collision_pointcloud_time,
        StopObstacleClassification{StopObstacleClassification::Type::POINTCLOUD},
        stop_candidate.vel_lpf.getValue().value(), stop_candidate.latest_collision_point.point,
        time_compensated_dist_to_collide, polygon_param, braking_dist);
      RCLCPP_DEBUG(
        logger_,
        "|_PC_| total_dist: %2.5f, raw_dist: %2.5f, time_compensated dist: %2.5f, "
        "braking_dist: %2.5f",
        (time_compensated_dist_to_collide + braking_dist),
        (stop_candidate.latest_collision_point.dist_to_collide), time_compensated_dist_to_collide,
        braking_dist);
    }
  }

  return stop_obstacles;
}

std::optional<StopObstacle> ObstacleStopModule::pick_stop_obstacle_from_predicted_object(
  const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<TrajectoryPoint> & decimated_traj_points,
  const std::shared_ptr<PlannerData::Object> object, const rclcpp::Time & predicted_objects_stamp,
  const double dist_from_obj_poly_to_traj_poly, const VehicleInfo & vehicle_info,
  const double x_offset_to_bumper,
  const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & filtering_params = obstacle_filtering_params_.at(
    StopObstacleClassification{object->predicted_object.classification}.label);

  const auto & predicted_object = object->predicted_object;
  const auto & obj_pose =
    object->get_predicted_current_pose(clock_->now(), predicted_objects_stamp);
  const double estimation_time = calc_estimation_time(predicted_object, filtering_params);
  const auto obj_uuid_str = autoware_utils_uuid::to_hex_string(predicted_object.object_id);

  // 1. filter by label
  if (!filtering_params.check_inside) {
    return std::nullopt;
  }

  // 2. filter by lateral distance
  // NOTE: max_lat_margin can be negative, so apply std::max with 1e-3.
  // dist_from_obj_poly_to_traj_poly: denotes the distance as is.
  if (
    std::max(filtering_params.lateral_margin.max_margin(vehicle_info), 1e-3) <=
    dist_from_obj_poly_to_traj_poly -
      std::max(object->get_lat_vel_relative_to_traj(traj_points) * estimation_time, 0.0)) {
    RCLCPP_DEBUG(
      logger_,
      "[Stop] Ignore obstacle (%s) since the lateral distance to the trajectory is too large.",
      obj_uuid_str.substr(0, 4).c_str());
    return std::nullopt;
  }

  // 4. check if the obstacle really collides with the trajectory
  // 4.1 generate polygon to be checked
  // calculate collision points with trajectory with lateral stop margin
  const auto & p = trajectory_polygon_collision_check;
  const auto & obj_vel = predicted_object.kinematics.initial_twist_with_covariance.twist.linear;
  const auto polygon_param = create_polygon_param(
    filtering_params.trim_trajectory, calc_ego_forwarding_braking_distance(traj_points, odometry),
    filtering_params.lateral_margin, std::hypot(obj_vel.x, obj_vel.y));
  const auto detection_polygon_with_lat_margin = get_trajectory_polygon(
    decimated_traj_points, vehicle_info, odometry.pose.pose, polygon_param,
    p.enable_to_consider_current_pose, p.time_to_convergence, p.decimate_trajectory_step_length);

  // 4.2. inside obstacle check
  auto collision_point = polygon_utils::get_collision_point(
    detection_polygon_with_lat_margin.traj_points, detection_polygon_with_lat_margin.polygons,
    obj_pose.position, clock_->now(),
    autoware_utils_geometry::to_polygon2d(obj_pose, predicted_object.shape), x_offset_to_bumper);

  // 4.3. outside obstacle check. Scope of this check is cut-in obstacles.
  if (!collision_point && filtering_params.check_outside) {
    collision_point = check_outside_cut_in_obstacle(
      object, traj_points, detection_polygon_with_lat_margin.traj_points,
      detection_polygon_with_lat_margin.polygons, x_offset_to_bumper, estimation_time,
      predicted_objects_stamp);
  }

  if (!collision_point) {
    RCLCPP_DEBUG(
      logger_, "[Stop] Ignore obstacle (%s) since there is no collision point.",
      obj_uuid_str.substr(0, 4).c_str());
    return std::nullopt;
  }

  // 5. filter if the obstacle will cross and go out of trajectory soon
  if (
    ignore_crossing_obstacle_ &&
    is_crossing_transient_obstacle(
      odometry, traj_points, detection_polygon_with_lat_margin.traj_points, object,
      x_offset_to_bumper, detection_polygon_with_lat_margin.polygons, collision_point)) {
    RCLCPP_DEBUG(
      logger_, "[Stop] Ignore obstacle (%s) since the obstacle will go out of the trajectory soon.",
      obj_uuid_str.substr(0, 4).c_str());
    return std::nullopt;
  }

  if (is_obstacle_velocity_requiring_fixed_stop(object, traj_points)) {
    return StopObstacle{
      predicted_object.object_id,
      predicted_objects_stamp,
      StopObstacleClassification{predicted_object.classification},
      obj_pose,
      predicted_object.shape,
      object->get_lon_vel_relative_to_traj(traj_points),
      collision_point->first,
      collision_point->second,
      polygon_param};
  }

  if (stop_planning_param_.rss_params.use_rss_stop) {
    const auto braking_dist = calc_braking_dist_along_trajectory(
      StopObstacleClassification{predicted_object.classification}.label,
      object->get_lon_vel_relative_to_traj(traj_points), stop_planning_param_.rss_params);

    RCLCPP_DEBUG(
      logger_, "|_OBJ_| total_dist: %2.5f, dist_to_collide: %2.5f, braking_dist: %2.5f",
      (collision_point->second + braking_dist), (collision_point->second), braking_dist);

    return StopObstacle{
      predicted_object.object_id,
      predicted_objects_stamp,
      StopObstacleClassification{predicted_object.classification},
      obj_pose,
      predicted_object.shape,
      object->get_lon_vel_relative_to_traj(traj_points),
      collision_point->first,
      collision_point->second,
      polygon_param,
      braking_dist};
  }

  return std::nullopt;
}

bool ObstacleStopModule::is_obstacle_velocity_requiring_fixed_stop(
  const std::shared_ptr<PlannerData::Object> object,
  const std::vector<TrajectoryPoint> & traj_points) const
{
  const auto stop_obstacle_opt =
    utils::get_obstacle_from_uuid(prev_stop_obstacles_, object->predicted_object.object_id);
  const bool is_prev_object_requires_fixed_stop =
    stop_obstacle_opt.has_value() && !stop_obstacle_opt->braking_dist.has_value();

  if (is_prev_object_requires_fixed_stop) {
    if (
      stop_planning_param_.obstacle_velocity_threshold_exit_fixed_stop <
      object->get_lon_vel_relative_to_traj(traj_points)) {
      return false;
    }
    return true;
  }
  if (
    object->get_lon_vel_relative_to_traj(traj_points) <
    stop_planning_param_.obstacle_velocity_threshold_enter_fixed_stop) {
    return true;
  }
  return false;
}

bool ObstacleStopModule::is_crossing_transient_obstacle(
  const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<TrajectoryPoint> & decimated_traj_points,
  const std::shared_ptr<PlannerData::Object> object, const double x_offset_to_bumper,
  const std::vector<Polygon2d> & decimated_traj_polys_with_lat_margin,
  const std::optional<std::pair<geometry_msgs::msg::Point, double>> & collision_point) const
{
  // Check if obstacle is moving in the same direction as the trajectory
  const double diff_angle = autoware::motion_utils::calc_diff_angle_against_trajectory(
    traj_points, object->predicted_object.kinematics.initial_pose_with_covariance.pose);

  const auto & filtering_params = obstacle_filtering_params_.at(
    StopObstacleClassification{object->predicted_object.classification}.label);
  bool near_zero =
    (-filtering_params.crossing_obstacle_traj_angle_threshold < diff_angle &&
     diff_angle < filtering_params.crossing_obstacle_traj_angle_threshold);
  bool near_pi =
    (M_PI - filtering_params.crossing_obstacle_traj_angle_threshold < std::abs(diff_angle) &&
     std::abs(diff_angle) < M_PI + filtering_params.crossing_obstacle_traj_angle_threshold);

  if (near_zero || near_pi) {
    return false;  // Not a crossing obstacle since it's moving in the same direction or opposite
                   // direction
  }

  //  calculate the time to reach the collision point
  const double time_to_reach_stop_point = calc_time_to_reach_collision_point(
    odometry, collision_point->first, traj_points, x_offset_to_bumper,
    stop_planning_param_.min_behavior_stop_margin,
    filtering_params.min_velocity_to_reach_collision_point);
  if (time_to_reach_stop_point <= filtering_params.crossing_obstacle_collision_time_margin) {
    return false;
  }

  // get the highest confident predicted paths
  std::vector<PredictedPath> predicted_paths;
  for (const auto & path : object->predicted_object.kinematics.predicted_paths) {
    predicted_paths.push_back(path);
  }
  constexpr double prediction_resampling_time_interval = 0.1;
  constexpr double prediction_resampling_time_horizon = 10.0;
  const auto resampled_predicted_paths = resample_highest_confidence_predicted_paths(
    predicted_paths, prediction_resampling_time_interval, prediction_resampling_time_horizon, 1);
  if (resampled_predicted_paths.empty() || resampled_predicted_paths.front().path.empty()) {
    return false;
  }

  // predict object pose when the ego reaches the collision point
  const auto future_obj_pose = [&]() {
    const auto opt_future_obj_pose = autoware::object_recognition_utils::calcInterpolatedPose(
      resampled_predicted_paths.front(),
      time_to_reach_stop_point - filtering_params.crossing_obstacle_collision_time_margin);
    if (opt_future_obj_pose) {
      return *opt_future_obj_pose;
    }
    return resampled_predicted_paths.front().path.back();
  }();

  // check if the ego will collide with the obstacle
  auto future_predicted_object = object->predicted_object;
  future_predicted_object.kinematics.initial_pose_with_covariance.pose = future_obj_pose;
  const auto future_collision_point = polygon_utils::get_collision_point(
    decimated_traj_points, decimated_traj_polys_with_lat_margin,
    future_predicted_object.kinematics.initial_pose_with_covariance.pose.position, clock_->now(),
    autoware_utils_geometry::to_polygon2d(
      future_predicted_object.kinematics.initial_pose_with_covariance.pose,
      future_predicted_object.shape),
    x_offset_to_bumper);
  const bool no_collision = !future_collision_point;

  return no_collision;
}

std::optional<geometry_msgs::msg::Point> ObstacleStopModule::plan_stop(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<StopObstacle> & stop_obstacles, const double x_offset_to_bumper)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  if (stop_obstacles.empty()) {
    const auto markers =
      autoware::motion_utils::createDeletedStopVirtualWallMarker(clock_->now(), 0);
    autoware_utils_visualization::append_marker_array(markers, &debug_data_ptr_->stop_wall_marker);

    prev_stop_distance_info_ = std::nullopt;
    return std::nullopt;
  }

  std::optional<StopObstacle> determined_stop_obstacle{};
  std::optional<double> determined_zero_vel_dist{};
  std::optional<double> determined_desired_stop_margin{};

  const auto closest_stop_obstacles = get_closest_stop_obstacles(stop_obstacles);
  for (const auto & stop_obstacle : closest_stop_obstacles) {
    const auto ego_segment_idx =
      planner_data->find_segment_index(traj_points, planner_data->current_odometry.pose.pose);

    // calculate dist to collide
    const double dist_to_collide_on_ref_traj =
      autoware::motion_utils::calcSignedArcLength(traj_points, 0, ego_segment_idx) +
      stop_obstacle.dist_to_collide_on_decimated_traj + stop_obstacle.braking_dist.value_or(0.0);

    // calculate desired stop margin
    const double desired_stop_margin = calc_desired_stop_margin(
      planner_data, traj_points, stop_obstacle, x_offset_to_bumper, ego_segment_idx,
      dist_to_collide_on_ref_traj);

    // calculate stop point against the obstacle
    const auto candidate_zero_vel_dist = calc_candidate_zero_vel_dist(
      planner_data, traj_points, stop_obstacle, dist_to_collide_on_ref_traj, desired_stop_margin);
    if (!candidate_zero_vel_dist) {
      continue;
    }

    if (determined_stop_obstacle) {
      const bool is_same_param_types =
        (stop_obstacle.classification == determined_stop_obstacle->classification);
      const auto point_cloud_suppression_margin = [&](const StopObstacle & obs) {
        return obs.classification.label == StopObstacleClassification::Type::POINTCLOUD
                 ? stop_planning_param_.pointcloud_suppression_distance_margin
                 : 0.0;
      };
      if (
        (is_same_param_types && stop_obstacle.dist_to_collide_on_decimated_traj +
                                    stop_obstacle.braking_dist.value_or(0.0) >
                                  determined_stop_obstacle->dist_to_collide_on_decimated_traj +
                                    determined_stop_obstacle->braking_dist.value_or(0.0)) ||
        (!is_same_param_types &&
         *candidate_zero_vel_dist + point_cloud_suppression_margin(stop_obstacle) >
           *determined_zero_vel_dist + point_cloud_suppression_margin(*determined_stop_obstacle))) {
        continue;
      }
    }
    determined_zero_vel_dist = *candidate_zero_vel_dist;
    determined_stop_obstacle = stop_obstacle;
    determined_desired_stop_margin = desired_stop_margin;
  }

  if (!(determined_zero_vel_dist && determined_stop_obstacle && determined_desired_stop_margin)) {
    // delete marker
    const auto markers =
      autoware::motion_utils::createDeletedStopVirtualWallMarker(clock_->now(), 0);
    autoware_utils_visualization::append_marker_array(markers, &debug_data_ptr_->stop_wall_marker);

    prev_stop_distance_info_ = std::nullopt;
    return std::nullopt;
  }

  // set debug polygon
  if (trajectory_polygon_for_inside_map_.count(determined_stop_obstacle->polygon_param) != 0) {
    debug_data_ptr_->decimated_traj_polys =
      trajectory_polygon_for_inside_map_.at(determined_stop_obstacle->polygon_param).polygons;
  }

  // Hold previous stop distance if necessary
  hold_previous_stop_if_necessary(planner_data, traj_points, determined_zero_vel_dist);

  // Insert stop point
  const auto stop_point = calc_stop_point(
    planner_data, traj_points, x_offset_to_bumper, determined_stop_obstacle,
    determined_zero_vel_dist);

  if (determined_stop_obstacle->velocity >= stop_planning_param_.max_negative_velocity) {
    // set stop_planning_debug_info
    set_stop_planning_debug_info(determined_stop_obstacle, determined_desired_stop_margin);

    return stop_point;
  }
  // Update path length buffer with current stop point
  path_length_buffer_.update_buffer(
    stop_point,
    [traj_points](const geometry_msgs::msg::Point & point) {
      return autoware::motion_utils::calcSignedArcLength(traj_points, 0, point);
    },
    clock_->now(), *determined_stop_obstacle, *determined_desired_stop_margin);

  // Get nearest active stop point from buffer
  const auto buffered_stop = path_length_buffer_.get_nearest_active_item();
  if (buffered_stop) {
    // Override with buffered stop point if available
    set_stop_planning_debug_info(
      buffered_stop->determined_stop_obstacle, buffered_stop->determined_desired_stop_margin);

    return std::make_optional(buffered_stop->stop_point);
  }

  return std::nullopt;
}

double ObstacleStopModule::calc_desired_stop_margin(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & traj_points, const StopObstacle & stop_obstacle,
  const double x_offset_to_bumper, const size_t ego_segment_idx,
  const double dist_to_collide_on_ref_traj)
{
  // calculate default stop margin
  const double default_stop_margin = [&]() {
    const double v_ego = planner_data->current_odometry.twist.twist.linear.x;
    const double v_obs = stop_obstacle.velocity;

    const auto ref_traj_length =
      autoware::motion_utils::calcSignedArcLength(traj_points, 0, traj_points.size() - 1);
    if (v_obs < stop_planning_param_.max_negative_velocity) {
      const double a_ego = stop_planning_param_.effective_deceleration_opposing_traffic;
      const double & bumper_to_bumper_distance = stop_obstacle.dist_to_collide_on_decimated_traj;

      const double braking_distance = v_ego * v_ego / (2 * a_ego);
      const double stopping_time = v_ego / a_ego;
      const double distance_obs_ego_braking = std::abs(v_obs * stopping_time);

      const double ego_stop_margin = stop_planning_param_.stop_margin_opposing_traffic;

      const double rel_vel = v_ego - v_obs;
      constexpr double epsilon = 1e-6;  // Small threshold for numerical stability
      if (std::abs(rel_vel) <= epsilon) {
        RCLCPP_WARN(
          logger_,
          "Relative velocity (%.3f) is too close to zero. Using minimum safe value for "
          "calculation.",
          rel_vel);
        return stop_planning_param_.stop_margin;  // Return default stop margin as fallback
      }

      const double T_coast = std::max(
        (bumper_to_bumper_distance - ego_stop_margin - braking_distance +
         distance_obs_ego_braking) /
          rel_vel,
        0.0);

      const double stopping_distance = v_ego * T_coast + braking_distance;

      const double stop_margin = bumper_to_bumper_distance - stopping_distance;

      return stop_margin;
    }

    if (dist_to_collide_on_ref_traj > ref_traj_length) {
      // Use terminal margin (terminal_stop_margin) for obstacle stop
      return stop_planning_param_.terminal_stop_margin;
    }

    return stop_planning_param_.stop_margin;
  }();

  // calculate stop margin on curve
  const double stop_margin_on_curve = calc_margin_from_obstacle_on_curve(
    planner_data, traj_points, stop_obstacle, x_offset_to_bumper, default_stop_margin);

  // calculate stop margin considering behavior's stop point
  // NOTE: If behavior stop point is ahead of the closest_obstacle_stop point within a certain
  //       margin we set closest_obstacle_stop_distance to closest_behavior_stop_distance
  const auto closest_behavior_stop_idx =
    autoware::motion_utils::searchZeroVelocityIndex(traj_points, ego_segment_idx + 1);
  const auto current_time = clock_->now();
  if (closest_behavior_stop_idx) {
    const double closest_behavior_stop_dist_on_ref_traj =
      autoware::motion_utils::calcSignedArcLength(traj_points, 0, *closest_behavior_stop_idx);
    const double stop_dist_diff =
      closest_behavior_stop_dist_on_ref_traj - (dist_to_collide_on_ref_traj - stop_margin_on_curve);
    if (0.0 < stop_dist_diff && stop_dist_diff < stop_margin_on_curve) {
      last_observed_behavior_stop_time_and_margin_ = std::make_pair(
        current_time, std::max(
                        stop_planning_param_.min_behavior_stop_margin,
                        dist_to_collide_on_ref_traj - closest_behavior_stop_dist_on_ref_traj));
    }
  }
  if (
    last_observed_behavior_stop_time_and_margin_.has_value() &&
    (current_time - last_observed_behavior_stop_time_and_margin_->first).seconds() <=
      stop_planning_param_.behavior_stop_margin_hold_time) {
    return last_observed_behavior_stop_time_and_margin_->second;
  }
  return stop_margin_on_curve;
}

std::optional<double> ObstacleStopModule::calc_candidate_zero_vel_dist(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & traj_points, const StopObstacle & stop_obstacle,
  const double dist_to_collide_on_ref_traj, const double desired_stop_margin)
{
  double candidate_zero_vel_dist = std::max(0.0, dist_to_collide_on_ref_traj - desired_stop_margin);
  if (suppress_sudden_stop_) {
    const auto acceptable_stop_acc = [&]() -> std::optional<double> {
      if (stop_planning_param_.get_param_type(stop_obstacle.classification) == "default") {
        return common_param_.limit_min_accel;
      }
      const double distance_to_judge_suddenness = std::min(
        calc_minimum_distance_to_stop(
          planner_data->current_odometry.twist.twist.linear.x, common_param_.limit_max_accel,
          stop_planning_param_.get_param(stop_obstacle.classification).sudden_object_acc_threshold),
        stop_planning_param_.get_param(stop_obstacle.classification).sudden_object_dist_threshold);
      if (candidate_zero_vel_dist > distance_to_judge_suddenness) {
        return common_param_.limit_min_accel;
      }
      if (stop_planning_param_.get_param(stop_obstacle.classification).abandon_to_stop) {
        RCLCPP_WARN(
          rclcpp::get_logger("ObstacleCruisePlanner::StopPlanner"),
          "[Cruise] abandon to stop against %s object",
          stop_obstacle.classification.to_string().c_str());
        return std::nullopt;
      } else {
        return stop_planning_param_.get_param(stop_obstacle.classification).limit_min_acc;
      }
    }();
    if (!acceptable_stop_acc) {
      return std::nullopt;
    }

    const double acceptable_stop_pos =
      autoware::motion_utils::calcSignedArcLength(
        traj_points, 0, planner_data->current_odometry.pose.pose.position) +
      calc_minimum_distance_to_stop(
        planner_data->current_odometry.twist.twist.linear.x, common_param_.limit_max_accel,
        acceptable_stop_acc.value());
    if (acceptable_stop_pos > candidate_zero_vel_dist) {
      candidate_zero_vel_dist = acceptable_stop_pos;
    }
  }
  return candidate_zero_vel_dist;
}

void ObstacleStopModule::hold_previous_stop_if_necessary(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & traj_points,
  std::optional<double> & determined_zero_vel_dist)
{
  if (
    std::abs(planner_data->current_odometry.twist.twist.linear.x) <
      stop_planning_param_.hold_stop_velocity_threshold &&
    prev_stop_distance_info_) {
    // NOTE: We assume that the current trajectory's front point is ahead of the previous
    // trajectory's front point.
    const size_t traj_front_point_prev_seg_idx =
      autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
        prev_stop_distance_info_->first, traj_points.front().pose);
    const double diff_dist_front_points = autoware::motion_utils::calcSignedArcLength(
      prev_stop_distance_info_->first, 0, traj_points.front().pose.position,
      traj_front_point_prev_seg_idx);

    const double prev_zero_vel_dist = prev_stop_distance_info_->second - diff_dist_front_points;
    if (
      std::abs(prev_zero_vel_dist - determined_zero_vel_dist.value()) <
      stop_planning_param_.hold_stop_distance_threshold) {
      determined_zero_vel_dist.value() = prev_zero_vel_dist;
    }
  }
}

std::optional<geometry_msgs::msg::Point> ObstacleStopModule::calc_stop_point(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & traj_points, const double x_offset_to_bumper,
  const std::optional<StopObstacle> & determined_stop_obstacle,
  const std::optional<double> & determined_zero_vel_dist)
{
  auto output_traj_points = traj_points;

  // insert stop point
  const auto zero_vel_idx = [&]() -> std::optional<size_t> {
    if (determined_zero_vel_dist <= 0.0) {
      return 0;
    }
    return autoware::motion_utils::insertStopPoint(
      0, *determined_zero_vel_dist, output_traj_points);
  }();
  if (!zero_vel_idx) {
    return std::nullopt;
  }

  // virtual wall marker for stop obstacle. This marker is not visualized in the default setting.
  const auto markers = autoware::motion_utils::createStopVirtualWallMarker(
    output_traj_points.at(*zero_vel_idx).pose, "obstacle stop", clock_->now(), 0,
    std::abs(x_offset_to_bumper), "", planner_data->is_driving_forward);
  autoware_utils_visualization::append_marker_array(markers, &debug_data_ptr_->stop_wall_marker);
  debug_data_ptr_->obstacles_to_stop.push_back(*determined_stop_obstacle);

  // update planning factor
  autoware_internal_planning_msgs::msg::SafetyFactor safety_factor;
  safety_factor.type =
    (determined_stop_obstacle->classification.label == StopObstacleClassification::Type::POINTCLOUD)
      ? autoware_internal_planning_msgs::msg::SafetyFactor::POINTCLOUD
      : autoware_internal_planning_msgs::msg::SafetyFactor::OBJECT;
  safety_factor.object_id = determined_stop_obstacle->uuid;
  safety_factor.points = {determined_stop_obstacle->pose.position};
  safety_factor.is_safe = false;

  autoware_internal_planning_msgs::msg::SafetyFactorArray safety_factor_array;
  safety_factor_array.factors = {safety_factor};
  safety_factor_array.is_safe = false;

  const auto stop_pose = output_traj_points.at(*zero_vel_idx).pose;
  planning_factor_interface_->add(
    output_traj_points, planner_data->current_odometry.pose.pose, stop_pose, PlanningFactor::STOP,
    safety_factor_array);

  prev_stop_distance_info_ = std::make_pair(output_traj_points, determined_zero_vel_dist.value());

  return stop_pose.position;
}

void ObstacleStopModule::set_stop_planning_debug_info(
  const std::optional<StopObstacle> & determined_stop_obstacle,
  const std::optional<double> & determined_desired_stop_margin) const
{
  stop_planning_debug_info_.set(
    StopPlanningDebugInfo::TYPE::STOP_CURRENT_OBSTACLE_DISTANCE,
    determined_stop_obstacle->dist_to_collide_on_decimated_traj);
  stop_planning_debug_info_.set(
    StopPlanningDebugInfo::TYPE::STOP_CURRENT_OBSTACLE_VELOCITY,
    determined_stop_obstacle->velocity);
  stop_planning_debug_info_.set(
    StopPlanningDebugInfo::TYPE::STOP_TARGET_OBSTACLE_DISTANCE,
    determined_desired_stop_margin.value());
  stop_planning_debug_info_.set(StopPlanningDebugInfo::TYPE::STOP_TARGET_VELOCITY, 0.0);
  stop_planning_debug_info_.set(StopPlanningDebugInfo::TYPE::STOP_TARGET_ACCELERATION, 0.0);
}

void ObstacleStopModule::publish_debug_info()
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  // 1. debug marker
  MarkerArray debug_marker;

  // 1.1. obstacles
  for (size_t i = 0; i < debug_data_ptr_->obstacles_to_stop.size(); ++i) {
    // obstacle
    const auto obstacle_marker = utils::get_object_marker(
      debug_data_ptr_->obstacles_to_stop.at(i).pose, i, "obstacles", 1.0, 0.0, 0.0);
    debug_marker.markers.push_back(obstacle_marker);

    // collision point
    auto collision_point_marker = autoware_utils_visualization::create_default_marker(
      "map", clock_->now(), "collision_points", 0, Marker::SPHERE,
      autoware_utils_visualization::create_marker_scale(0.25, 0.25, 0.25),
      autoware_utils_visualization::create_marker_color(1.0, 0.0, 0.0, 0.999));
    collision_point_marker.pose.position = debug_data_ptr_->obstacles_to_stop.at(i).collision_point;
    debug_marker.markers.push_back(collision_point_marker);
  }

  // 1.2. intentionally ignored obstacles
  for (size_t i = 0; i < debug_data_ptr_->intentionally_ignored_obstacles.size(); ++i) {
    const auto marker = utils::get_object_marker(
      debug_data_ptr_->intentionally_ignored_obstacles.at(i)
        ->predicted_object.kinematics.initial_pose_with_covariance.pose,
      i, "intentionally_ignored_obstacles", 0.0, 1.0, 0.0);
    debug_marker.markers.push_back(marker);
  }

  // 1.3. detection area
  auto decimated_traj_polys_marker = autoware_utils_visualization::create_default_marker(
    "map", clock_->now(), "detection_area", 0, Marker::LINE_LIST,
    autoware_utils_visualization::create_marker_scale(0.01, 0.0, 0.0),
    autoware_utils_visualization::create_marker_color(0.0, 1.0, 0.0, 0.999));
  for (const auto & decimated_traj_poly : debug_data_ptr_->decimated_traj_polys) {
    for (size_t dp_idx = 0; dp_idx < decimated_traj_poly.outer().size(); ++dp_idx) {
      const auto & current_point = decimated_traj_poly.outer().at(dp_idx);
      const auto & next_point =
        decimated_traj_poly.outer().at((dp_idx + 1) % decimated_traj_poly.outer().size());

      decimated_traj_polys_marker.points.push_back(
        autoware_utils_geometry::create_point(current_point.x(), current_point.y(), 0.0));
      decimated_traj_polys_marker.points.push_back(
        autoware_utils_geometry::create_point(next_point.x(), next_point.y(), 0.0));
    }
  }
  debug_marker.markers.push_back(decimated_traj_polys_marker);

  debug_publisher_->publish(debug_marker);

  // 2. virtual wall
  virtual_wall_publisher_->publish(debug_data_ptr_->stop_wall_marker);

  // 3. stop planning info
  const auto stop_debug_msg = stop_planning_debug_info_.convert_to_message(clock_->now());
  debug_stop_planning_info_pub_->publish(stop_debug_msg);

  // 4. objects of interest
  objects_of_interest_marker_interface_->publishMarkerArray();

  // 5. processing time
  processing_time_publisher_->publish(create_float64_stamped(clock_->now(), stop_watch_.toc()));
}

DetectionPolygon ObstacleStopModule::get_trajectory_polygon(
  const std::vector<TrajectoryPoint> & decimated_traj_points, const VehicleInfo & vehicle_info,
  const geometry_msgs::msg::Pose & current_ego_pose, const PolygonParam & polygon_param,
  const bool enable_to_consider_current_pose, const double time_to_convergence,
  const double decimate_trajectory_step_length) const
{
  if (trajectory_polygon_for_inside_map_.count(polygon_param) == 0) {
    auto cropped_traj_points =
      polygon_param.trimming_length.has_value()
        ? autoware::motion_utils::cropForwardPoints(
            decimated_traj_points, decimated_traj_points.front().pose.position, 0,
            polygon_param.trimming_length.value())
        : decimated_traj_points;

    auto traj_polys = polygon_utils::create_one_step_polygons(
      cropped_traj_points, vehicle_info, current_ego_pose, polygon_param.lateral_margin,
      enable_to_consider_current_pose, time_to_convergence, decimate_trajectory_step_length,
      polygon_param.off_track_scale);
    trajectory_polygon_for_inside_map_.emplace(
      polygon_param, DetectionPolygon{std::move(cropped_traj_points), std::move(traj_polys)});
  }
  return trajectory_polygon_for_inside_map_.at(polygon_param);
}

void ObstacleStopModule::check_consistency(
  const rclcpp::Time & current_time,
  const std::vector<std::shared_ptr<PlannerData::Object>> & objects,
  std::vector<StopObstacle> & stop_obstacles)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  for (const auto & prev_closest_stop_obstacle : prev_closest_stop_obstacles_) {
    const auto object_itr = std::find_if(
      objects.begin(), objects.end(),
      [&prev_closest_stop_obstacle](const std::shared_ptr<PlannerData::Object> & o) {
        return o->predicted_object.object_id == prev_closest_stop_obstacle.uuid;
      });
    // If previous closest obstacle disappear from the perception result, do nothing anymore.
    if (object_itr == objects.end()) {
      continue;
    }

    const auto is_disappeared_from_stop_obstacle = std::none_of(
      stop_obstacles.begin(), stop_obstacles.end(),
      [&prev_closest_stop_obstacle](const StopObstacle & so) {
        return so.uuid == prev_closest_stop_obstacle.uuid;
      });
    if (is_disappeared_from_stop_obstacle) {
      // re-evaluate as a stop candidate, and overwrite the current decision if "maintain stop"
      // condition is satisfied
      const double elapsed_time = (current_time - prev_closest_stop_obstacle.stamp).seconds();
      const auto & filtering_params = obstacle_filtering_params_.at(
        StopObstacleClassification{prev_closest_stop_obstacle.classification}.label);
      if (
        (*object_itr)->predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x <
          stop_planning_param_.obstacle_velocity_threshold_enter_fixed_stop &&
        elapsed_time < filtering_params.stop_obstacle_hold_time_threshold) {
        stop_obstacles.push_back(prev_closest_stop_obstacle);
      }
    }
  }

  prev_closest_stop_obstacles_ = get_closest_stop_obstacles(stop_obstacles);
}

double ObstacleStopModule::calc_margin_from_obstacle_on_curve(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & traj_points, const StopObstacle & stop_obstacle,
  const double x_offset_to_bumper, const double default_stop_margin) const
{
  if (
    !stop_planning_param_.enable_approaching_on_curve ||
    stop_obstacle.classification.label == StopObstacleClassification::Type::POINTCLOUD) {
    return default_stop_margin;
  }

  // calculate short trajectory points towards obstacle
  const size_t obj_segment_idx =
    autoware::motion_utils::findNearestSegmentIndex(traj_points, stop_obstacle.collision_point);
  std::vector<TrajectoryPoint> short_traj_points{traj_points.at(obj_segment_idx + 1)};
  double sum_short_traj_length{0.0};
  for (int i = obj_segment_idx; 0 <= i; --i) {
    short_traj_points.push_back(traj_points.at(i));

    if (
      1 < short_traj_points.size() &&
      stop_planning_param_.stop_margin + std::abs(x_offset_to_bumper) < sum_short_traj_length) {
      break;
    }
    sum_short_traj_length +=
      autoware_utils_geometry::calc_distance2d(traj_points.at(i), traj_points.at(i + 1));
  }
  std::reverse(short_traj_points.begin(), short_traj_points.end());
  if (short_traj_points.size() < 2) {
    return default_stop_margin;
  }

  // calculate collision index between straight line from ego pose and object
  const auto calculate_distance_from_straight_ego_path =
    [&](const auto & ego_pose, const auto & object_polygon) {
      const auto forward_ego_pose = autoware_utils_geometry::calc_offset_pose(
        ego_pose, stop_planning_param_.stop_margin + 3.0, 0.0, 0.0);
      const auto ego_straight_segment = autoware_utils_geometry::Segment2d{
        convert_point(ego_pose.position), convert_point(forward_ego_pose.position)};
      return boost::geometry::distance(ego_straight_segment, object_polygon);
    };
  const auto resampled_short_traj_points = resample_trajectory_points(short_traj_points, 0.5);
  const auto object_polygon =
    autoware_utils_geometry::to_polygon2d(stop_obstacle.pose, stop_obstacle.shape);
  const auto collision_idx = [&]() -> std::optional<size_t> {
    for (size_t i = 0; i < resampled_short_traj_points.size(); ++i) {
      const double dist_to_obj = calculate_distance_from_straight_ego_path(
        resampled_short_traj_points.at(i).pose, object_polygon);
      if (dist_to_obj < planner_data->vehicle_info_.vehicle_width_m / 2.0) {
        return i;
      }
    }
    return std::nullopt;
  }();
  if (!collision_idx) {
    return stop_planning_param_.min_stop_margin_on_curve;
  }
  if (*collision_idx == 0) {
    return default_stop_margin;
  }

  // calculate margin from obstacle
  const double partial_segment_length = [&]() {
    const double collision_segment_length = autoware_utils_geometry::calc_distance2d(
      resampled_short_traj_points.at(*collision_idx - 1),
      resampled_short_traj_points.at(*collision_idx));
    const double prev_dist = calculate_distance_from_straight_ego_path(
      resampled_short_traj_points.at(*collision_idx - 1).pose, object_polygon);
    const double next_dist = calculate_distance_from_straight_ego_path(
      resampled_short_traj_points.at(*collision_idx).pose, object_polygon);
    return (next_dist - planner_data->vehicle_info_.vehicle_width_m / 2.0) /
           (next_dist - prev_dist) * collision_segment_length;
  }();

  const double short_margin_from_obstacle =
    partial_segment_length +
    autoware::motion_utils::calcSignedArcLength(
      resampled_short_traj_points, *collision_idx, stop_obstacle.collision_point) -
    std::abs(x_offset_to_bumper) + stop_planning_param_.additional_stop_margin_on_curve;

  return std::min(
    default_stop_margin,
    std::max(stop_planning_param_.min_stop_margin_on_curve, short_margin_from_obstacle));
}

std::vector<StopObstacle> ObstacleStopModule::get_closest_stop_obstacles(
  const std::vector<StopObstacle> & stop_obstacles)
{
  std::vector<StopObstacle> candidates{};
  for (const auto & stop_obstacle : stop_obstacles) {
    const auto itr =
      std::find_if(candidates.begin(), candidates.end(), [&stop_obstacle](const StopObstacle & co) {
        return co.classification == stop_obstacle.classification;
      });
    if (itr == candidates.end()) {
      candidates.emplace_back(stop_obstacle);
    } else if (
      stop_obstacle.dist_to_collide_on_decimated_traj + stop_obstacle.braking_dist.value_or(0.0) <
      itr->dist_to_collide_on_decimated_traj + itr->braking_dist.value_or(0.0)) {
      *itr = stop_obstacle;
    }
  }
  return candidates;
}

std::vector<Polygon2d> ObstacleStopModule::get_decimated_traj_polys(
  const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & current_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const double ego_nearest_dist_threshold, const double ego_nearest_yaw_threshold,
  const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check) const
{
  if (!decimated_traj_polys_) {
    const auto & p = trajectory_polygon_collision_check;
    const auto decimated_traj_points = utils::decimate_trajectory_points_from_ego(
      traj_points, current_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold,
      p.decimate_trajectory_step_length, p.goal_extended_trajectory_length);
    decimated_traj_polys_ = polygon_utils::create_one_step_polygons(
      decimated_traj_points, vehicle_info, current_pose, 0.0, p.enable_to_consider_current_pose,
      p.time_to_convergence, p.decimate_trajectory_step_length);
  }
  return *decimated_traj_polys_;
}

std::optional<std::pair<geometry_msgs::msg::Point, double>>
ObstacleStopModule::check_outside_cut_in_obstacle(
  const std::shared_ptr<PlannerData::Object> object,
  const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<TrajectoryPoint> & decimated_traj_points,
  const std::vector<Polygon2d> & decimated_traj_polys_with_lat_margin,
  const double x_offset_to_bumper, const double estimation_time,
  const rclcpp::Time & predicted_objects_stamp) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & outside_params =
    obstacle_filtering_params_
      .at(StopObstacleClassification{object->predicted_object.classification}.label)
      .outside_obstacle;

  const double lat_vel = object->get_lat_vel_relative_to_traj(traj_points);
  const double long_vel = object->get_lon_vel_relative_to_traj(traj_points);
  if (
    std::abs(lat_vel) > outside_params.max_lateral_velocity ||
    long_vel < outside_params.min_longitudinal_velocity ||
    std::atan2(std::abs(lat_vel), long_vel) > outside_params.max_moving_direction_angle) {
    return std::nullopt;
  }

  const auto & current_obj_pose =
    object->get_predicted_current_pose(clock_->now(), predicted_objects_stamp);
  const auto future_obj_pose = object->calc_predicted_pose(
    clock_->now() + rclcpp::Duration::from_seconds(estimation_time), predicted_objects_stamp);

  using autoware_utils_geometry::to_polygon2d;
  autoware_utils_geometry::MultiPoint2d poly_points;
  autoware_utils_geometry::Polygon2d convex_poly;
  boost::geometry::append(
    poly_points, to_polygon2d(current_obj_pose, object->predicted_object.shape).outer());
  boost::geometry::append(
    poly_points, to_polygon2d(future_obj_pose, object->predicted_object.shape).outer());
  boost::geometry::convex_hull(poly_points, convex_poly);
  boost::geometry::correct(convex_poly);

  auto collision_point = polygon_utils::get_collision_point(
    decimated_traj_points, decimated_traj_polys_with_lat_margin, future_obj_pose.position,
    clock_->now(), convex_poly, x_offset_to_bumper);

  if (collision_point && collision_point->second < 0.0) {
    return std::nullopt;
  }

  return collision_point;
}

}  // namespace autoware::motion_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::motion_velocity_planner::ObstacleStopModule,
  autoware::motion_velocity_planner::PluginModuleInterface)
