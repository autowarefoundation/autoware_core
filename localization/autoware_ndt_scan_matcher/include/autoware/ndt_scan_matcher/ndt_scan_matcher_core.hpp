// Copyright 2015-2019 Autoware Foundation
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

#ifndef AUTOWARE__NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_CORE_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_CORE_HPP_

#define FMT_HEADER_ONLY

#include "guarded.hpp"
#include "hyper_parameters.hpp"
#include "map_update_module.hpp"
#include "ndt_omp/multigrid_ndt_omp.h"

#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/agnocast_wrapper/tf2.hpp>
#include <autoware/localization_util/smart_pose_buffer.hpp>
#include <autoware_utils_diagnostics/diagnostics_interface.hpp>
#include <autoware_utils_logging/logger_level_configure.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/int32_stamped.hpp>
#include <autoware_internal_localization_msgs/srv/pose_with_covariance_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <fmt/format.h>
#include <pcl/point_types.h>
#include <tf2/transform_datatypes.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

#include <array>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

namespace autoware::ndt_scan_matcher
{

class NDTScanMatcher : public autoware::agnocast_wrapper::Node
{
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;
  using NormalDistributionsTransform =
    pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>;

public:
  explicit NDTScanMatcher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // This function is only used in static tools to know when timer callbacks are triggered.
  std::chrono::nanoseconds time_until_trigger() const
  {
    return map_update_timer_->time_until_trigger();
  }

private:
  void callback_timer();

  void callback_initial_pose(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(geometry_msgs::msg::PoseWithCovarianceStamped) &
    initial_pose_msg_ptr);
  void callback_initial_pose_main(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(geometry_msgs::msg::PoseWithCovarianceStamped) &
    initial_pose_msg_ptr);

  void callback_regularization_pose(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(geometry_msgs::msg::PoseWithCovarianceStamped) &
    pose_conv_msg_ptr);

  void callback_sensor_points(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(sensor_msgs::msg::PointCloud2) &
    sensor_points_msg_in_sensor_frame);
  bool callback_sensor_points_main(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(sensor_msgs::msg::PointCloud2) &
    sensor_points_msg_in_sensor_frame);

  void service_trigger_node(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);

  void service_ndt_align(
    const autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr
      req,
    autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res);
  void service_ndt_align_main(
    const autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr
      req,
    autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res);

  std::tuple<geometry_msgs::msg::PoseWithCovarianceStamped, double> align_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_with_cov,
    NormalDistributionsTransform & ndt_ref);

  void transform_sensor_measurement(
    const std::string & source_frame, const std::string & target_frame,
    const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_input_ptr,
    pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_output_ptr);

  void publish_tf(
    const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg);
  void publish_pose(
    const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg,
    const std::array<double, 36> & ndt_covariance, const bool is_converged);
  void publish_point_cloud(
    const rclcpp::Time & sensor_ros_time, const std::string & frame_id,
    const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_in_map_ptr);
  void publish_marker(
    const rclcpp::Time & sensor_ros_time, const std::vector<geometry_msgs::msg::Pose> & pose_array,
    NormalDistributionsTransform & ndt_ref);
  void publish_initial_to_result(
    const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg,
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_cov_msg,
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_old_msg,
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_new_msg);

  static int count_oscillation(const std::vector<geometry_msgs::msg::Pose> & result_pose_msg_array);

  Eigen::Matrix2d estimate_covariance(
    const pclomp::NdtResult & ndt_result, const Eigen::Matrix4f & initial_pose_matrix,
    const rclcpp::Time & sensor_ros_time, NormalDistributionsTransform & ndt_ref);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr visualize_point_score(
    const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_in_map_ptr,
    const float & lower_nvs, const float & upper_nvs, NormalDistributionsTransform & ndt_ref);

  void add_regularization_pose(
    const rclcpp::Time & sensor_ros_time, NormalDistributionsTransform & ndt_ref);

  AUTOWARE_TIMER_PTR map_update_timer_;
  AUTOWARE_SUBSCRIPTION_PTR(geometry_msgs::msg::PoseWithCovarianceStamped) initial_pose_sub_;
  AUTOWARE_SUBSCRIPTION_PTR(sensor_msgs::msg::PointCloud2) sensor_points_sub_;
  AUTOWARE_SUBSCRIPTION_PTR(geometry_msgs::msg::PoseWithCovarianceStamped) regularization_pose_sub_;

  AUTOWARE_PUBLISHER_PTR(sensor_msgs::msg::PointCloud2) sensor_aligned_pose_pub_;
  AUTOWARE_PUBLISHER_PTR(sensor_msgs::msg::PointCloud2) no_ground_points_aligned_pose_pub_;
  AUTOWARE_PUBLISHER_PTR(geometry_msgs::msg::PoseStamped) ndt_pose_pub_;
  AUTOWARE_PUBLISHER_PTR(geometry_msgs::msg::PoseWithCovarianceStamped)
  ndt_pose_with_covariance_pub_;
  AUTOWARE_PUBLISHER_PTR(geometry_msgs::msg::PoseWithCovarianceStamped)
  initial_pose_with_covariance_pub_;
  AUTOWARE_PUBLISHER_PTR(geometry_msgs::msg::PoseArray) multi_ndt_pose_pub_;
  AUTOWARE_PUBLISHER_PTR(geometry_msgs::msg::PoseArray) multi_initial_pose_pub_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float32Stamped) exe_time_pub_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float32Stamped)
  transform_probability_pub_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float32Stamped)
  nearest_voxel_transformation_likelihood_pub_;
  AUTOWARE_PUBLISHER_PTR(sensor_msgs::msg::PointCloud2) voxel_score_points_pub_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float32Stamped)
  no_ground_transform_probability_pub_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float32Stamped)
  no_ground_nearest_voxel_transformation_likelihood_pub_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Int32Stamped) iteration_num_pub_;
  AUTOWARE_PUBLISHER_PTR(geometry_msgs::msg::PoseStamped) initial_to_result_relative_pose_pub_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float32Stamped)
  initial_to_result_distance_pub_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float32Stamped)
  initial_to_result_distance_old_pub_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float32Stamped)
  initial_to_result_distance_new_pub_;
  AUTOWARE_PUBLISHER_PTR(visualization_msgs::msg::MarkerArray) ndt_marker_pub_;
  AUTOWARE_PUBLISHER_PTR(visualization_msgs::msg::MarkerArray)
  ndt_monte_carlo_initial_pose_marker_pub_;

  AUTOWARE_SERVICE_PTR(autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped)
  service_;
  AUTOWARE_SERVICE_PTR(std_srvs::srv::SetBool) service_trigger_node_;

  autoware::agnocast_wrapper::TransformBroadcaster tf2_broadcaster_;
  autoware::agnocast_wrapper::Buffer tf2_buffer_;
  autoware::agnocast_wrapper::TransformListener tf2_listener_;

  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

  Guarded<std::shared_ptr<NormalDistributionsTransform>> ndt_ptr_{
    std::make_shared<NormalDistributionsTransform>()};

  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_baselink_frame_;

  std::unique_ptr<autoware::localization_util::SmartPoseBuffer> initial_pose_buffer_;

  // Keep latest position for dynamic map loading
  Guarded<std::optional<geometry_msgs::msg::Point>> latest_ekf_position_{std::nullopt};

  std::unique_ptr<autoware::localization_util::SmartPoseBuffer> regularization_pose_buffer_;

  std::atomic<bool> is_activated_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_scan_points_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_initial_pose_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_regularization_pose_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_map_update_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_ndt_align_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_trigger_node_;
  std::unique_ptr<MapUpdateModule> map_update_module_;
  std::unique_ptr<
    autoware_utils_logging::BasicLoggerLevelConfigure<autoware::agnocast_wrapper::Node>>
    logger_configure_;

  HyperParameters param_;
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_CORE_HPP_
