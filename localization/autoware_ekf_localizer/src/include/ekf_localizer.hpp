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

#ifndef EKF_LOCALIZER_HPP_
#define EKF_LOCALIZER_HPP_

#include "aged_object_queue.hpp"
#include "ekf_module.hpp"
#include "hyper_parameters.hpp"
#include "warning.hpp"

#include <autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp>
#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/agnocast_wrapper/tf2.hpp>
#include <autoware_utils_logging/logger_level_configure.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/utils.hpp>

#include <autoware_internal_debug_msgs/msg/float64_multi_array_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace autoware::ekf_localizer
{

class EKFLocalizer : public autoware::agnocast_wrapper::Node
{
public:
  explicit EKFLocalizer(const rclcpp::NodeOptions & node_options);

  // This function is only used in static tools to know when timer callbacks are triggered.
  std::chrono::nanoseconds time_until_trigger() const
  {
    return timer_control_->time_until_trigger();
  }

private:
  const std::shared_ptr<Warning> warning_;

  //!< @brief ekf estimated pose publisher
  AUTOWARE_PUBLISHER_PTR(geometry_msgs::msg::PoseStamped) pub_pose_;
  //!< @brief estimated ekf pose with covariance publisher
  AUTOWARE_PUBLISHER_PTR(geometry_msgs::msg::PoseWithCovarianceStamped) pub_pose_cov_;
  //!< @brief estimated ekf odometry publisher
  AUTOWARE_PUBLISHER_PTR(nav_msgs::msg::Odometry) pub_odom_;
  //!< @brief ekf estimated twist publisher
  AUTOWARE_PUBLISHER_PTR(geometry_msgs::msg::TwistStamped) pub_twist_;
  //!< @brief ekf estimated twist with covariance publisher
  AUTOWARE_PUBLISHER_PTR(geometry_msgs::msg::TwistWithCovarianceStamped) pub_twist_cov_;
  //!< @brief ekf estimated yaw bias publisher
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float64Stamped) pub_yaw_bias_;
  //!< @brief ekf estimated yaw bias publisher
  AUTOWARE_PUBLISHER_PTR(geometry_msgs::msg::PoseStamped) pub_biased_pose_;
  //!< @brief ekf estimated yaw bias publisher
  AUTOWARE_PUBLISHER_PTR(geometry_msgs::msg::PoseWithCovarianceStamped) pub_biased_pose_cov_;
  //!< @brief processing_time publisher
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float64Stamped) pub_processing_time_;
  //!< @brief /diagnostics publisher (manual DiagnosticArray; same absolute topic as former
  //!< diagnostic_updater)
  AUTOWARE_PUBLISHER_PTR(diagnostic_msgs::msg::DiagnosticArray) pub_diagnostics_;
  //!< @brief initial pose subscriber
  AUTOWARE_SUBSCRIPTION_PTR(geometry_msgs::msg::PoseWithCovarianceStamped) sub_initialpose_;
  //!< @brief measurement pose with covariance subscriber
  AUTOWARE_SUBSCRIPTION_PTR(geometry_msgs::msg::PoseWithCovarianceStamped) sub_pose_with_cov_;
  //!< @brief measurement twist with covariance subscriber
  AUTOWARE_SUBSCRIPTION_PTR(geometry_msgs::msg::TwistWithCovarianceStamped) sub_twist_with_cov_;
  //!< @brief time for ekf calculation callback
  AUTOWARE_TIMER_PTR timer_control_;
  //!< @brief calls publish_diagnostics() at diagnostics_publish_period
  AUTOWARE_TIMER_PTR diagnostics_publish_timer_;
  //!< @brief last predict time
  std::shared_ptr<const rclcpp::Time> last_predict_time_;
  //!< @brief trigger_node service
  AUTOWARE_SERVICE_PTR(std_srvs::srv::SetBool) service_trigger_node_;

  //!< @brief tf broadcaster
  std::shared_ptr<autoware::agnocast_wrapper::TransformBroadcaster> tf_br_;
  //!< @brief tf buffer
  autoware::agnocast_wrapper::Buffer tf2_buffer_;
  //!< @brief tf listener
  autoware::agnocast_wrapper::TransformListener tf2_listener_;

  //!< @brief logger configure module
  std::unique_ptr<
    autoware_utils_logging::BasicLoggerLevelConfigure<autoware::agnocast_wrapper::Node>>
    logger_configure_;

  //!< @brief  extended kalman filter instance.
  std::unique_ptr<EKFModule> ekf_module_;

  const HyperParameters params_;

  double ekf_dt_;

  bool is_activated_;
  bool is_set_initialpose_;

  EKFDiagnosticInfo pose_diag_info_;
  EKFDiagnosticInfo twist_diag_info_;

  AgedObjectQueue<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr> pose_queue_;
  AgedObjectQueue<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> twist_queue_;

  //!< @brief Merged diagnostic status from the latest EKF cycle (message/values refresh every tick)
  diagnostic_msgs::msg::DiagnosticStatus merged_diagnostic_status_;
  //!< @brief Wall time of the latest merged diagnostic level change vs. the previous EKF tick
  //!< (includes recovery to OK); published as last_level_transition_timestamp once non-zero
  rclcpp::Time merged_diagnostic_last_transition_time_;
  //!< @brief last pose callback header stamp (for callback_pose diagnostic)
  rclcpp::Time last_pose_callback_time_;
  //!< @brief last twist callback header stamp (for callback_twist diagnostic)
  rclcpp::Time last_twist_callback_time_;

  /**
   * @brief computes update & prediction of EKF for each ekf_dt_[s] time
   */
  void timer_callback();

  /**
   * @brief set pose with covariance measurement
   */
  void callback_pose_with_covariance(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(geometry_msgs::msg::PoseWithCovarianceStamped) msg);

  /**
   * @brief set twist with covariance measurement
   */
  void callback_twist_with_covariance(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(geometry_msgs::msg::TwistWithCovarianceStamped) msg);

  /**
   * @brief set initial_pose to current EKF pose
   */
  void callback_initial_pose(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(geometry_msgs::msg::PoseWithCovarianceStamped) msg);

  /**
   * @brief update predict frequency
   */
  void update_predict_frequency(const rclcpp::Time & current_time);

  /**
   * @brief get transform from frame_id
   */
  bool get_transform_from_tf(
    std::string parent_frame, std::string child_frame,
    geometry_msgs::msg::TransformStamped & transform);

  /**
   * @brief publish current EKF estimation result
   */
  void publish_estimate_result(
    const geometry_msgs::msg::PoseStamped & current_ekf_pose,
    const geometry_msgs::msg::PoseStamped & current_biased_ekf_pose,
    const geometry_msgs::msg::TwistStamped & current_ekf_twist);

  /**
   * @brief Overwrite merged_diagnostic_status_ from merged diagnostics each tick;
   * publish_diagnostics() when merged severity increases vs. the previous EKF tick
   * (last transition time updates on any level change).
   */
  void update_diagnostics(
    const std::vector<diagnostic_msgs::msg::DiagnosticStatus> & diag_status_array,
    const rclcpp::Time & current_time);

  /** @brief Build and publish /diagnostics (main merged + callback_pose + callback_twist) */
  void publish_diagnostics();

  /**
   * @brief trigger node
   */
  void service_trigger_node(
    const AUTOWARE_SERVER_REQUEST_PTR(std_srvs::srv::SetBool) req,
    AUTOWARE_SERVER_RESPONSE_PTR(std_srvs::srv::SetBool) res);

  autoware_utils_system::StopWatch<std::chrono::milliseconds> stop_watch_;
  autoware_utils_system::StopWatch<std::chrono::milliseconds> stop_watch_timer_cb_;

  void initialize_diagnostic_info(
    EKFDiagnosticInfo & pose_diag_info, EKFDiagnosticInfo & twist_diag_info,
    const AgedObjectQueue<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr> & pose_queue,
    const AgedObjectQueue<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> & twist_queue);

  friend class EKFLocalizerDiagnosticsTest;  // for test code
};

}  // namespace autoware::ekf_localizer

#endif  // EKF_LOCALIZER_HPP_
