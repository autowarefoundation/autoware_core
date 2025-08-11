// Copyright 2024 Autoware Foundation
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

#ifndef AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__PLANNER_DATA_HPP_
#define AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__PLANNER_DATA_HPP_

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/motion_velocity_planner_common/collision_checker.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware/velocity_smoother/smoother/smoother_base.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_rclcpp/parameter.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <lanelet2_core/Forward.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_utils_rclcpp::get_or_declare_parameter;
using TrajectoryPoints = std::vector<autoware_planning_msgs::msg::TrajectoryPoint>;
using Point2d = autoware_utils_geometry::Point2d;
using Polygon2d = boost::geometry::model::polygon<Point2d>;

struct TrafficSignalStamped
{
  builtin_interfaces::msg::Time stamp;
  autoware_perception_msgs::msg::TrafficLightGroup signal;
};

struct StopPoint
{
  double ego_trajectory_arc_length{};  // [m] arc length along the ego trajectory
  geometry_msgs::msg::Pose
    ego_stop_pose;                       // intersection between the trajectory and a map stop line
  lanelet::BasicLineString2d stop_line;  // stop line from the map
};
struct TrajectoryPolygonCollisionCheck
{
  double decimate_trajectory_step_length{};
  double goal_extended_trajectory_length{};
  bool enable_to_consider_current_pose{};
  double time_to_convergence{};
};

struct PointcloudObstacleFilteringParam  // TODO(takagi): delete this obsolete parameter type
{
  double pointcloud_voxel_grid_x{};
  double pointcloud_voxel_grid_y{};
  double pointcloud_voxel_grid_z{};
  double pointcloud_cluster_tolerance{};
  size_t pointcloud_min_cluster_size{};
  size_t pointcloud_max_cluster_size{};
};

struct PointcloudPreprocessParams
{
  explicit PointcloudPreprocessParams(rclcpp::Node & node)
  {
    std::string ns = "pointcloud_preprocessing.";
    {
      std::string ns_child = ns + "filter_by_trajectory_polygon.";
      filter_by_trajectory_polygon.enable_monolithic_crop_box =
        get_or_declare_parameter<bool>(node, ns_child + "enable_monolithic_crop_box");
      filter_by_trajectory_polygon.enable_multi_polygon_filtering =
        get_or_declare_parameter<bool>(node, ns_child + "enable_multi_polygon_filtering");
      filter_by_trajectory_polygon.min_trajectory_length =
        get_or_declare_parameter<double>(node, ns_child + "min_trajectory_length");
      filter_by_trajectory_polygon.braking_distance_scale_factor =
        get_or_declare_parameter<double>(node, ns_child + "braking_distance_scale_factor");
      filter_by_trajectory_polygon.lateral_margin =
        get_or_declare_parameter<double>(node, ns_child + "lateral_margin");
      filter_by_trajectory_polygon.height_margin =
        get_or_declare_parameter<double>(node, ns_child + "height_margin");
    }
    {
      std::string ns_child = ns + "downsample_by_voxel_grid.";
      downsample_by_voxel_grid.enable_downsample =
        get_or_declare_parameter<bool>(node, ns_child + "enable_downsample");
      downsample_by_voxel_grid.voxel_size_x =
        get_or_declare_parameter<double>(node, ns_child + "voxel_size_x");
      downsample_by_voxel_grid.voxel_size_y =
        get_or_declare_parameter<double>(node, ns_child + "voxel_size_y");
      downsample_by_voxel_grid.voxel_size_z =
        get_or_declare_parameter<double>(node, ns_child + "voxel_size_z");
    }
    {
      std::string ns_child = ns + "euclidean_clustering.";
      euclidean_clustering.enable_clustering =
        get_or_declare_parameter<bool>(node, ns_child + "enable_clustering");
      euclidean_clustering.cluster_tolerance =
        get_or_declare_parameter<double>(node, ns_child + "cluster_tolerance");
      euclidean_clustering.min_cluster_size =
        get_or_declare_parameter<int>(node, ns_child + "min_cluster_size");
      euclidean_clustering.max_cluster_size =
        get_or_declare_parameter<int>(node, ns_child + "max_cluster_size");
    }
  }
  struct FilterByTrajectoryPolygon
  {
    bool enable_monolithic_crop_box{false};
    bool enable_multi_polygon_filtering{false};
    double min_trajectory_length{};
    double braking_distance_scale_factor{};
    double lateral_margin{};
    double height_margin{};
  } filter_by_trajectory_polygon;
  struct DownsampleByVoxelGrid
  {
    bool enable_downsample{false};
    double voxel_size_x{};
    double voxel_size_y{};
    double voxel_size_z{};
  } downsample_by_voxel_grid;

  struct EuclideanClustering
  {
    bool enable_clustering{false};
    double cluster_tolerance{};
    int min_cluster_size{};
    int max_cluster_size{};
  } euclidean_clustering;
};

struct PlannerData
{
public:
  PlannerData(const PlannerData &) = delete;
  PlannerData & operator=(const PlannerData &) = delete;
  PlannerData(PlannerData &&) = default;
  PlannerData & operator=(PlannerData &&) = default;
  explicit PlannerData(rclcpp::Node & node);
  class Object
  {
  public:
    Object() = default;
    explicit Object(const autoware_perception_msgs::msg::PredictedObject & arg_predicted_object)
    : predicted_object(arg_predicted_object)
    {
    }
    autoware_perception_msgs::msg::PredictedObject predicted_object;

    /**
     * @brief compute and the minimal distance to `decimated_traj_polys` by bg::distance and cache
     * the result
     * @note is it really OK to cache the result if the object itself is also cached and used in
     * next iteration ?
     */
    double get_dist_to_traj_poly(
      const std::vector<autoware_utils_geometry::Polygon2d> & decimated_traj_polys) const;
    double get_dist_to_traj_lateral(const std::vector<TrajectoryPoint> & traj_points) const;
    double get_dist_from_ego_longitudinal(
      const std::vector<TrajectoryPoint> & traj_points,
      const geometry_msgs::msg::Point & ego_pos) const;
    double get_lon_vel_relative_to_traj(const std::vector<TrajectoryPoint> & traj_points) const;
    double get_lat_vel_relative_to_traj(const std::vector<TrajectoryPoint> & traj_points) const;
    geometry_msgs::msg::Pose get_predicted_current_pose(
      const rclcpp::Time & current_stamp, const rclcpp::Time & predicted_objects_stamp) const;
    geometry_msgs::msg::Pose calc_predicted_pose(
      const rclcpp::Time & specified_time, const rclcpp::Time & predicted_object_stamp) const;

  private:
    void calc_vel_relative_to_traj(const std::vector<TrajectoryPoint> & traj_points) const;

    mutable std::optional<double> dist_to_traj_poly{std::nullopt};
    mutable std::optional<double> dist_to_traj_lateral{std::nullopt};
    mutable std::optional<double> dist_from_ego_longitudinal{std::nullopt};
    mutable std::optional<double> lon_vel_relative_to_traj{std::nullopt};
    mutable std::optional<double> lat_vel_relative_to_traj{std::nullopt};
    mutable std::optional<geometry_msgs::msg::Pose> predicted_pose;
  };

  class Pointcloud
  {
  public:
    explicit Pointcloud(rclcpp::Node & node) : preprocess_params_(node) {}

    void preprocess_pointcloud(
      pcl::PointCloud<pcl::PointXYZ> && arg_pointcloud,
      const std::vector<TrajectoryPoint> & raw_trajectory, nav_msgs::msg::Odometry current_odometry,
      double min_deceleration_distance,
      const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
      const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check,
      const double ego_nearest_dist_threshold, const double ego_nearest_yaw_threshold)
    {
      pointcloud = arg_pointcloud;
      const auto preprocessed_result = filter_and_cluster_point_clouds(
        raw_trajectory, current_odometry, min_deceleration_distance, vehicle_info,
        trajectory_polygon_collision_check, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
      filtered_pointcloud_ptr = preprocessed_result.first;
      cluster_indices = preprocessed_result.second;
    }

    pcl::PointCloud<pcl::PointXYZ> pointcloud;

    const pcl::PointCloud<pcl::PointXYZ>::Ptr get_filtered_pointcloud_ptr() const
    {
      if (!filtered_pointcloud_ptr) {
        throw(std::runtime_error(
          "Filtered pointcloud pointer is not set. Please call preprocess_pointcloud() "
          "first."));
      }
      return filtered_pointcloud_ptr.value();
    };
    const std::vector<pcl::PointIndices> get_cluster_indices() const
    {
      if (!cluster_indices) {
        throw(std::runtime_error(
          "Cluster indices are not set. Please call preprocess_pointcloud() first."));
      }
      return cluster_indices.value();
    };
    // TODO(takagi): Remove these function after universe modules eliminates these functions.
    const pcl::PointCloud<pcl::PointXYZ>::Ptr get_filtered_pointcloud_ptr(
      [[maybe_unused]] const autoware::motion_velocity_planner::TrajectoryPoints &
        trajectory_points,
      [[maybe_unused]] const autoware::vehicle_info_utils::VehicleInfo & vehicle_info) const
    {
      return get_filtered_pointcloud_ptr();
    };
    const std::vector<pcl::PointIndices> get_cluster_indices(
      [[maybe_unused]] const autoware::motion_velocity_planner::TrajectoryPoints &
        trajectory_points,
      [[maybe_unused]] const autoware::vehicle_info_utils::VehicleInfo & vehicle_info) const
    {
      return get_cluster_indices();
    }

    PointcloudPreprocessParams preprocess_params_;

  private:
    std::optional<pcl::PointCloud<pcl::PointXYZ>::Ptr> filtered_pointcloud_ptr;
    std::optional<std::vector<pcl::PointIndices>> cluster_indices;

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, std::vector<pcl::PointIndices>>
    filter_and_cluster_point_clouds(
      const std::vector<TrajectoryPoint> & raw_trajectory,
      const nav_msgs::msg::Odometry & current_odometry, double min_deceleration_distance,
      const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
      const TrajectoryPolygonCollisionCheck & collision_check,
      const double ego_nearest_dist_threshold, const double ego_nearest_yaw_threshold);
  };

  void process_predicted_objects(
    const autoware_perception_msgs::msg::PredictedObjects & predicted_objects);

  // msgs from callbacks that are used for data-ready
  nav_msgs::msg::Odometry current_odometry;
  geometry_msgs::msg::AccelWithCovarianceStamped current_acceleration;
  std_msgs::msg::Header predicted_objects_header;
  std::vector<std::shared_ptr<Object>> objects;
  Pointcloud no_ground_pointcloud;
  nav_msgs::msg::OccupancyGrid occupancy_grid;
  std::shared_ptr<route_handler::RouteHandler> route_handler;

  // nearest search
  double ego_nearest_dist_threshold{};
  double ego_nearest_yaw_threshold{};

  // both of motion_velocity_planner own and motion_velocity_planner_modules use this parameter
  TrajectoryPolygonCollisionCheck trajectory_polygon_collision_check{};

  // other internal data
  // traffic_light_id_map_raw is the raw observation, while traffic_light_id_map_keep_last keeps the
  // last observed infomation for UNKNOWN
  std::map<lanelet::Id, TrafficSignalStamped> traffic_light_id_map_raw_;
  std::map<lanelet::Id, TrafficSignalStamped> traffic_light_id_map_last_observed_;

  // velocity smoother
  std::shared_ptr<autoware::velocity_smoother::SmootherBase> velocity_smoother_;
  // parameters
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  bool is_driving_forward{true};

  /**
   *@fn
   *@brief queries the traffic signal information of given Id. if keep_last_observation = true,
   *recent UNKNOWN observation is overwritten as the last non-UNKNOWN observation
   */
  [[nodiscard]] std::optional<TrafficSignalStamped> get_traffic_signal(
    const lanelet::Id id, const bool keep_last_observation = false) const;

  /// @brief calculate possible stop points along the current trajectory where it intersects with
  /// stop lines
  /// @param [in] trajectory ego trajectory
  /// @return stop points taken from the map
  [[nodiscard]] std::vector<StopPoint> calculate_map_stop_points(
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory) const;

  /// @brief calculate the minimum distance needed by ego to decelerate to the given velocity
  /// @param [in] target_velocity [m/s] target velocity
  /// @return [m] distance needed to reach the target velocity
  [[nodiscard]] std::optional<double> calculate_min_deceleration_distance(
    const double target_velocity) const;

  size_t find_index(
    const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & pose) const
  {
    return autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
      traj_points, pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
  }

  size_t find_segment_index(
    const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & pose) const
  {
    return autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      traj_points, pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
  }
};
struct RequiredSubscriptionInfo
{
  bool traffic_signals{false};
  bool predicted_objects{false};
  bool occupancy_grid_map{false};
  bool no_ground_pointcloud{false};

  void update(const RequiredSubscriptionInfo & required_subscriptions)
  {
    traffic_signals |= required_subscriptions.traffic_signals;
    predicted_objects |= required_subscriptions.predicted_objects;
    occupancy_grid_map |= required_subscriptions.occupancy_grid_map;
    no_ground_pointcloud |= required_subscriptions.no_ground_pointcloud;
  }
};
}  // namespace autoware::motion_velocity_planner

#endif  // AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__PLANNER_DATA_HPP_
