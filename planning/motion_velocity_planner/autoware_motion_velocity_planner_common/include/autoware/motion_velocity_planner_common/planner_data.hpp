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
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/ros/parameter.hpp>
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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <memory>
#include <optional>
#include <vector>

namespace autoware::motion_velocity_planner
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_utils::get_or_declare_parameter;

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
  double decimate_trajectory_step_length;
  double goal_extended_trajectory_length;
  bool enable_to_consider_current_pose;
  double time_to_convergence;
};

struct PointcloudObstacleFilteringParam
{
  double pointcloud_voxel_grid_x{};
  double pointcloud_voxel_grid_y{};
  double pointcloud_voxel_grid_z{};
  double pointcloud_cluster_tolerance{};
  size_t pointcloud_min_cluster_size{};
  size_t pointcloud_max_cluster_size{};
};

class PlannerData
{
  public:

  explicit PlannerData(rclcpp::Node & node)
  : vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo())
  {
    // nearest search
    ego_nearest_dist_threshold = get_or_declare_parameter<double>(
      node, "ego_nearest_dist_threshold");
    ego_nearest_yaw_threshold = get_or_declare_parameter<double>(
      node, "ego_nearest_yaw_threshold");
      
    use_pointcloud = get_or_declare_parameter<bool>(
      node, "object_type.pointcloud");
  
    trajectory_polygon_collision_check.decimate_trajectory_step_length = get_or_declare_parameter<double>(
      node, "trajectory_polygon_collision_check.decimate_trajectory_step_length");
    trajectory_polygon_collision_check.goal_extended_trajectory_length = get_or_declare_parameter<double>(
      node, "trajectory_polygon_collision_check.goal_extended_trajectory_length");
    trajectory_polygon_collision_check.enable_to_consider_current_pose = get_or_declare_parameter<bool>(
      node, "trajectory_polygon_collision_check.consider_current_pose.enable_to_consider_current_pose");
    trajectory_polygon_collision_check.time_to_convergence = get_or_declare_parameter<double>(
      node, "trajectory_polygon_collision_check.consider_current_pose.time_to_convergence");

    pointcloud_obstacle_filtering_param.pointcloud_voxel_grid_x = get_or_declare_parameter<double>(
      node, "pointcloud.pointcloud_voxel_grid_x");
    pointcloud_obstacle_filtering_param.pointcloud_voxel_grid_y = get_or_declare_parameter<double>(
      node, "pointcloud.pointcloud_voxel_grid_y");
    pointcloud_obstacle_filtering_param.pointcloud_voxel_grid_z = get_or_declare_parameter<double>(
      node, "pointcloud.pointcloud_voxel_grid_z");
    pointcloud_obstacle_filtering_param.pointcloud_cluster_tolerance = get_or_declare_parameter<double>(
      node, "pointcloud.pointcloud_cluster_tolerance");
    pointcloud_obstacle_filtering_param.pointcloud_min_cluster_size = get_or_declare_parameter<int>(
      node, "pointcloud.pointcloud_min_cluster_size");
    pointcloud_obstacle_filtering_param.pointcloud_max_cluster_size = get_or_declare_parameter<int>(
      node, "pointcloud.pointcloud_max_cluster_size");

    
  }
  class Object
  {
  public:
    Object() = default;
    explicit Object(const autoware_perception_msgs::msg::PredictedObject & arg_predicted_object)
    : predicted_object(arg_predicted_object)
    {
    }

    autoware_perception_msgs::msg::PredictedObject predicted_object;

    double get_dist_to_traj_poly(
      const std::vector<autoware_utils::Polygon2d> & decimated_traj_polys) const;
    double get_dist_to_traj_lateral(const std::vector<TrajectoryPoint> & traj_points) const;
    double get_dist_from_ego_longitudinal(
      const std::vector<TrajectoryPoint> & traj_points,
      const geometry_msgs::msg::Point & ego_pos) const;
    double get_lon_vel_relative_to_traj(const std::vector<TrajectoryPoint> & traj_points) const;
    double get_lat_vel_relative_to_traj(const std::vector<TrajectoryPoint> & traj_points) const;
    geometry_msgs::msg::Pose get_predicted_pose(
      const rclcpp::Time & current_stamp, const rclcpp::Time & predicted_object_stamp) const;

  private:
    void calc_vel_relative_to_traj(const std::vector<TrajectoryPoint> & traj_points) const;

    mutable std::optional<double> dist_to_traj_poly{std::nullopt};
    mutable std::optional<double> dist_to_traj_lateral{std::nullopt};
    mutable std::optional<double> dist_from_ego_longitudinal{std::nullopt};
    mutable std::optional<double> lon_vel_relative_to_traj{std::nullopt};
    mutable std::optional<double> lat_vel_relative_to_traj{std::nullopt};
    mutable std::optional<geometry_msgs::msg::Pose> predicted_pose;
  };

  struct Pointcloud
  {
    Pointcloud() = default;
    explicit Pointcloud(pcl::PointCloud<pcl::PointXYZ> && arg_pointcloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr && filtered_pointcloud_ptr,
                        std::vector<pcl::PointIndices> && cluster_indices)
    : pointcloud(arg_pointcloud),
      filtered_pointcloud_ptr(filtered_pointcloud_ptr),
      cluster_indices(cluster_indices)
    {
    }

    pcl::PointCloud<pcl::PointXYZ> pointcloud;

    const pcl::PointCloud<pcl::PointXYZ>::Ptr get_filtered_pointcloud_ptr() const { return filtered_pointcloud_ptr; };
    const std::vector<pcl::PointIndices> get_cluster_indices() const { return cluster_indices; };

  private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pointcloud_ptr;
    std::vector<pcl::PointIndices> cluster_indices;
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

  PointcloudObstacleFilteringParam pointcloud_obstacle_filtering_param{};
  TrajectoryPolygonCollisionCheck trajectory_polygon_collision_check{};

  double mask_lat_margin{};

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
  bool use_pointcloud{false};

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
}  // namespace autoware::motion_velocity_planner

#endif  // AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__PLANNER_DATA_HPP_
