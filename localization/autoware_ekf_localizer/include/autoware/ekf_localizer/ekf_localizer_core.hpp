// Copyright 2018-2019 Autoware Foundation
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

#ifndef AUTOWARE__EKF_LOCALIZER__EKF_LOCALIZER_CORE_HPP_
#define AUTOWARE__EKF_LOCALIZER__EKF_LOCALIZER_CORE_HPP_

#include "autoware/ekf_localizer/aged_object_queue.hpp"
#include "autoware/ekf_localizer/ekf_module.hpp"
#include "autoware/ekf_localizer/hyper_parameters.hpp"
#include "autoware/ekf_localizer/warning.hpp"

#include <autoware_utils_logging/logger_level_configure.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/utils.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <autoware_internal_debug_msgs/msg/float64_multi_array_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace autoware::ekf_localizer
{

/// ROS-I/O-free EKF logic hostable on any rclcpp::Node (standalone or fusion host plugin).
class EKFLocalizerCore
{
public:
  using PostEstimateCallback = std::function<void(
    const nav_msgs::msg::Odometry &, const geometry_msgs::msg::TwistWithCovarianceStamped &)>;

  explicit EKFLocalizerCore(rclcpp::Node * node);

  std::chrono::nanoseconds time_until_trigger() const
  {
    return timer_control_->time_until_trigger();
  }

  void set_post_estimate_callback(PostEstimateCallback callback)
  {
    post_estimate_callback_ = std::move(callback);
  }

  void set_publish_ekf_odom_topic(bool publish) { publish_ekf_odom_topic_ = publish; }

private:
  rclcpp::Node * node_;
  const std::shared_ptr<Warning> warning_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_cov_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_cov_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr pub_yaw_bias_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_biased_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_biased_pose_cov_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    pub_processing_time_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_with_cov_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    sub_twist_with_cov_;

  rclcpp::TimerBase::SharedPtr timer_control_;
  std::shared_ptr<const rclcpp::Time> last_predict_time_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_trigger_node_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;
  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  std::unique_ptr<autoware_utils_logging::LoggerLevelConfigure> logger_configure_;
  std::unique_ptr<EKFModule> ekf_module_;

  const HyperParameters params_;
  std::string diagnostics_hardware_id_;

  double ekf_dt_;
  bool is_activated_;
  bool is_set_initialpose_;
  bool publish_ekf_odom_topic_{true};
  PostEstimateCallback post_estimate_callback_;

  EKFDiagnosticInfo pose_diag_info_;
  EKFDiagnosticInfo twist_diag_info_;

  AgedObjectQueue<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr> pose_queue_;
  AgedObjectQueue<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> twist_queue_;

  void timer_callback();
  void callback_pose_with_covariance(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void callback_twist_with_covariance(
    geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
  void callback_initial_pose(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void update_predict_frequency(const rclcpp::Time & current_time);
  bool get_transform_from_tf(
    std::string parent_frame, std::string child_frame,
    geometry_msgs::msg::TransformStamped & transform);
  void publish_estimate_result(
    const geometry_msgs::msg::PoseStamped & current_ekf_pose,
    const geometry_msgs::msg::PoseStamped & current_biased_ekf_pose,
    const geometry_msgs::msg::TwistStamped & current_ekf_twist);
  void publish_diagnostics(
    const geometry_msgs::msg::PoseStamped & current_ekf_pose, const rclcpp::Time & current_time);
  void publish_callback_return_diagnostics(
    const std::string & callback_name, const rclcpp::Time & current_time);
  void service_trigger_node(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);

  autoware_utils_system::StopWatch<std::chrono::milliseconds> stop_watch_;
  autoware_utils_system::StopWatch<std::chrono::milliseconds> stop_watch_timer_cb_;

  friend class EKFLocalizerTestSuite;
};

}  // namespace autoware::ekf_localizer

#endif  // AUTOWARE__EKF_LOCALIZER__EKF_LOCALIZER_CORE_HPP_
