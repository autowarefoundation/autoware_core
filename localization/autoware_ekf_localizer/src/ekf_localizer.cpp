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

#include "include/ekf_localizer.hpp"

#include "autoware/localization_util/covariance_ellipse.hpp"
#include "include/diagnostics.hpp"
#include "include/string.hpp"
#include "include/warning_message.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_logging/logger_level_configure.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>

#include <fmt/core.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace autoware::ekf_localizer
{

// clang-format off
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl // NOLINT
#define DEBUG_INFO(...) {if (params_.show_debug_info) {RCLCPP_INFO(__VA_ARGS__);}} // NOLINT
// clang-format on

using std::placeholders::_1;

EKFLocalizer::EKFLocalizer(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("ekf_localizer", node_options),
  warning_(std::make_shared<Warning>(this)),
  tf2_buffer_(this->get_clock()),
  tf2_listener_(tf2_buffer_),
  params_(this),
  ekf_dt_(params_.ekf_dt),
  pose_queue_(params_.pose_smoothing_steps, params_.max_pose_queue_size),
  twist_queue_(params_.twist_smoothing_steps, params_.max_twist_queue_size),
  diagnostics_(this),
  merged_diagnostic_last_transition_time_(0, 0, RCL_ROS_TIME),
  last_pose_callback_time_(0, 0, RCL_ROS_TIME),
  last_twist_callback_time_(0, 0, RCL_ROS_TIME)
{
  is_activated_ = false;
  is_set_initialpose_ = false;
  merged_diagnostic_status_.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  merged_diagnostic_status_.message = "OK";

  // Configure diagnostic updater
  diagnostics_.setHardwareID(this->get_name());
  diagnostics_.setPeriod(rclcpp::Duration::from_seconds(params_.diagnostics_publish_period));
  diagnostics_.add("ekf_localizer", this, &EKFLocalizer::diagnose);
  diagnostics_.add("callback_pose", this, &EKFLocalizer::diagnose_callback_pose);
  diagnostics_.add("callback_twist", this, &EKFLocalizer::diagnose_callback_twist);

  /* initialize ros system */
  timer_control_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(ekf_dt_),
    std::bind(&EKFLocalizer::timer_callback, this));

  pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("ekf_pose", 1);
  pub_pose_cov_ =
    create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("ekf_pose_with_covariance", 1);
  pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("ekf_odom", 1);
  pub_twist_ = create_publisher<geometry_msgs::msg::TwistStamped>("ekf_twist", 1);
  pub_twist_cov_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "ekf_twist_with_covariance", 1);
  pub_yaw_bias_ =
    create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>("estimated_yaw_bias", 1);
  pub_biased_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("ekf_biased_pose", 1);
  pub_biased_pose_cov_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ekf_biased_pose_with_covariance", 1);
  pub_processing_time_ = create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/processing_time_ms", 1);
  sub_initialpose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 1, std::bind(&EKFLocalizer::callback_initial_pose, this, _1));
  sub_pose_with_cov_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "in_pose_with_covariance", 1,
    std::bind(&EKFLocalizer::callback_pose_with_covariance, this, _1));
  sub_twist_with_cov_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "in_twist_with_covariance", 1,
    std::bind(&EKFLocalizer::callback_twist_with_covariance, this, _1));
#if ROS_DISTRO_HUMBLE
  const auto service_trigger_qos = rclcpp::ServicesQoS().get_rmw_qos_profile();
#else
  const auto service_trigger_qos = rclcpp::ServicesQoS();
#endif
  service_trigger_node_ = create_service<std_srvs::srv::SetBool>(
    "trigger_node_srv",
    std::bind(
      &EKFLocalizer::service_trigger_node, this, std::placeholders::_1, std::placeholders::_2),
    service_trigger_qos);

  tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(
    std::shared_ptr<rclcpp::Node>(this, [](auto) {}));

  ekf_module_ = std::make_unique<EKFModule>(warning_, params_);
  logger_configure_ = std::make_unique<autoware_utils_logging::LoggerLevelConfigure>(this);
}

/*
 * update_predict_frequency
 */
void EKFLocalizer::update_predict_frequency(const rclcpp::Time & current_time)
{
  if (last_predict_time_) {
    if (current_time < *last_predict_time_) {
      warning_->warn("Detected jump back in time");
    } else {
      /* Measure dt */
      ekf_dt_ = (current_time - *last_predict_time_).seconds();
      DEBUG_INFO(
        get_logger(), "[EKF] update ekf_dt_ to %f seconds (= %f hz)", ekf_dt_, 1 / ekf_dt_);

      if (ekf_dt_ > 10.0) {
        ekf_dt_ = 10.0;
        warning_->warn(large_ekf_dt_waring_message(ekf_dt_));
      } else if (ekf_dt_ > static_cast<double>(params_.pose_smoothing_steps) / params_.ekf_rate) {
        warning_->warn_throttle(too_slow_ekf_dt_waring_message(ekf_dt_), 2000);
      }

      /* Register dt and accumulate time delay */
      ekf_module_->accumulate_delay_time(ekf_dt_);
    }
  }
  last_predict_time_ = std::make_shared<const rclcpp::Time>(current_time);
}

/*
 * timer_callback
 */
void EKFLocalizer::timer_callback()
{
  stop_watch_timer_cb_.tic();

  const rclcpp::Time current_time = this->now();

  // Initialize diagnostic status array to collect diagnostics during processing
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> diag_status_array;

  // Check process activation status
  diag_status_array.push_back(check_process_activated(is_activated_));

  if (!is_activated_) {
    warning_->warn_throttle(
      "The node is not activated. Provide initial pose to pose_initializer", 2000);
    // Update diagnostics before early return to ensure current status is latched
    update_diagnostics(diag_status_array, current_time);
    return;
  }

  // Check initial pose status
  diag_status_array.push_back(check_set_initialpose(is_set_initialpose_));

  if (!is_set_initialpose_) {
    warning_->warn_throttle(
      "Initial pose is not set. Provide initial pose to pose_initializer", 2000);
    // Update diagnostics before early return to ensure current status is latched
    update_diagnostics(diag_status_array, current_time);
    return;
  }

  DEBUG_INFO(get_logger(), "========================= timer called =========================");

  /* update predict frequency with measured timer rate */
  update_predict_frequency(current_time);

  /* predict model in EKF */
  stop_watch_.tic();
  DEBUG_INFO(get_logger(), "------------------------- start prediction -------------------------");
  ekf_module_->predict_with_delay(ekf_dt_);
  DEBUG_INFO(get_logger(), "[EKF] predictKinematicsModel calc time = %f [ms]", stop_watch_.toc());
  DEBUG_INFO(get_logger(), "------------------------- end prediction -------------------------\n");

  /* pose measurement update */
  pose_diag_info_.queue_size = pose_queue_.size();
  pose_diag_info_.is_passed_delay_gate = true;
  pose_diag_info_.delay_time = 0.0;
  pose_diag_info_.delay_time_threshold = 0.0;
  pose_diag_info_.is_passed_mahalanobis_gate = true;
  pose_diag_info_.mahalanobis_distance = 0.0;

  bool pose_is_updated = false;

  if (!pose_queue_.empty()) {
    DEBUG_INFO(get_logger(), "------------------------- start Pose -------------------------");
    stop_watch_.tic();

    // Sequential state update for all Pose observations in the queue
    const size_t n = pose_queue_.size();
    for (size_t i = 0; i < n; ++i) {
      const auto pose = pose_queue_.pop_increment_age();
      bool is_updated = ekf_module_->measurement_update_pose(*pose, current_time, pose_diag_info_);
      pose_is_updated = pose_is_updated || is_updated;
    }
    DEBUG_INFO(
      get_logger(), "[EKF] measurement_update_pose calc time = %f [ms]", stop_watch_.toc());
    DEBUG_INFO(get_logger(), "------------------------- end Pose -------------------------\n");
  }
  pose_diag_info_.no_update_count = pose_is_updated ? 0 : (pose_diag_info_.no_update_count + 1);

  // Add pose-related diagnostics after pose processing
  diag_status_array.push_back(check_measurement_updated(
    "pose", pose_diag_info_.no_update_count, params_.pose_no_update_count_threshold_warn,
    params_.pose_no_update_count_threshold_error));
  diag_status_array.push_back(check_measurement_queue_size("pose", pose_diag_info_.queue_size));
  diag_status_array.push_back(check_measurement_delay_gate(
    "pose", pose_diag_info_.is_passed_delay_gate, pose_diag_info_.delay_time,
    pose_diag_info_.delay_time_threshold));
  diag_status_array.push_back(check_measurement_mahalanobis_gate(
    "pose", pose_diag_info_.is_passed_mahalanobis_gate, pose_diag_info_.mahalanobis_distance,
    params_.pose_gate_dist));

  /* twist measurement update */
  twist_diag_info_.queue_size = twist_queue_.size();
  twist_diag_info_.is_passed_delay_gate = true;
  twist_diag_info_.delay_time = 0.0;
  twist_diag_info_.delay_time_threshold = 0.0;
  twist_diag_info_.is_passed_mahalanobis_gate = true;
  twist_diag_info_.mahalanobis_distance = 0.0;

  bool twist_is_updated = false;

  if (!twist_queue_.empty()) {
    DEBUG_INFO(get_logger(), "------------------------- start Twist -------------------------");
    stop_watch_.tic();

    // Sequential state update for all Twist observations in the queue
    const size_t n = twist_queue_.size();
    for (size_t i = 0; i < n; ++i) {
      const auto twist = twist_queue_.pop_increment_age();
      bool is_updated =
        ekf_module_->measurement_update_twist(*twist, current_time, twist_diag_info_);
      twist_is_updated = twist_is_updated || is_updated;
    }
    DEBUG_INFO(
      get_logger(), "[EKF] measurement_update_twist calc time = %f [ms]", stop_watch_.toc());
    DEBUG_INFO(get_logger(), "------------------------- end Twist -------------------------\n");
  }
  twist_diag_info_.no_update_count = twist_is_updated ? 0 : (twist_diag_info_.no_update_count + 1);

  // Add twist-related diagnostics after twist processing
  diag_status_array.push_back(check_measurement_updated(
    "twist", twist_diag_info_.no_update_count, params_.twist_no_update_count_threshold_warn,
    params_.twist_no_update_count_threshold_error));
  diag_status_array.push_back(check_measurement_queue_size("twist", twist_diag_info_.queue_size));
  diag_status_array.push_back(check_measurement_delay_gate(
    "twist", twist_diag_info_.is_passed_delay_gate, twist_diag_info_.delay_time,
    twist_diag_info_.delay_time_threshold));
  diag_status_array.push_back(check_measurement_mahalanobis_gate(
    "twist", twist_diag_info_.is_passed_mahalanobis_gate, twist_diag_info_.mahalanobis_distance,
    params_.twist_gate_dist));

  const geometry_msgs::msg::PoseStamped current_ekf_pose =
    ekf_module_->get_current_pose(current_time, false);
  const geometry_msgs::msg::PoseStamped current_biased_ekf_pose =
    ekf_module_->get_current_pose(current_time, true);
  const geometry_msgs::msg::TwistStamped current_ekf_twist =
    ekf_module_->get_current_twist(current_time);

  // Calculate covariance ellipse and add diagnostics
  geometry_msgs::msg::PoseWithCovariance pose_cov;
  pose_cov.pose = current_ekf_pose.pose;
  pose_cov.covariance = ekf_module_->get_current_pose_covariance();
  const autoware::localization_util::Ellipse ellipse =
    autoware::localization_util::calculate_xy_ellipse(pose_cov, params_.ellipse_scale);
  diag_status_array.push_back(check_covariance_ellipse(
    "cov_ellipse_long_axis", ellipse.long_radius, params_.warn_ellipse_size,
    params_.error_ellipse_size));
  diag_status_array.push_back(check_covariance_ellipse(
    "cov_ellipse_lateral_direction", ellipse.size_lateral_direction,
    params_.warn_ellipse_size_lateral_direction, params_.error_ellipse_size_lateral_direction));

  /* publish ekf result */
  publish_estimate_result(current_ekf_pose, current_biased_ekf_pose, current_ekf_twist);

  /* Latch merged diagnostics every EKF cycle; publishing is periodic via diagnostic_updater,
   * plus force_update() inside update_diagnostics when severity increases. */
  update_diagnostics(diag_status_array, current_time);

  /* publish processing time */
  const double elapsed_time = stop_watch_timer_cb_.toc();
  pub_processing_time_->publish(
    autoware_internal_debug_msgs::build<autoware_internal_debug_msgs::msg::Float64Stamped>()
      .stamp(current_time)
      .data(elapsed_time));
}

/*
 * get_transform_from_tf
 */
bool EKFLocalizer::get_transform_from_tf(
  std::string parent_frame, std::string child_frame,
  geometry_msgs::msg::TransformStamped & transform)
{
  parent_frame = erase_leading_slash(parent_frame);
  child_frame = erase_leading_slash(child_frame);

  try {
    transform = tf2_buffer_.lookupTransform(parent_frame, child_frame, tf2::TimePointZero);
    return true;
  } catch (tf2::TransformException & ex) {
    warning_->warn(ex.what());
  }
  return false;
}

/*
 * callback_initial_pose
 */
void EKFLocalizer::callback_initial_pose(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped transform;
  if (!get_transform_from_tf(params_.pose_frame_id, msg->header.frame_id, transform)) {
    RCLCPP_ERROR(
      get_logger(), "[EKF] TF transform failed. parent = %s, child = %s",
      params_.pose_frame_id.c_str(), msg->header.frame_id.c_str());
  }
  ekf_module_->initialize(*msg, transform);

  is_set_initialpose_ = true;
}

/*
 * callback_pose_with_covariance
 */
void EKFLocalizer::callback_pose_with_covariance(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (!is_activated_ && !is_set_initialpose_) {
    return;
  }

  pose_queue_.push(msg);

  // Warn if queue is exceeded
  if (pose_queue_.exceeded()) {
    warning_->warn_throttle(
      fmt::format(
        "[EKF] Pose queue size ({}) is exceeding max_queue_size ({}). Consider increasing "
        "max_queue_size or reducing input frequency.",
        pose_queue_.size(), pose_queue_.max_queue_size()),
      2000);
    pose_queue_.pop();
  }

  last_pose_callback_time_ = msg->header.stamp;
}

/*
 * callback_twist_with_covariance
 */
void EKFLocalizer::callback_twist_with_covariance(
  geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  // Ignore twist if velocity is too small.
  // Note that this inequality must not include "equal".
  if (std::abs(msg->twist.twist.linear.x) < params_.threshold_observable_velocity_mps) {
    msg->twist.covariance[0 * 6 + 0] = 10000.0;
  }

  twist_queue_.push(msg);

  // Warn if queue is exceeded
  if (twist_queue_.exceeded()) {
    warning_->warn_throttle(
      fmt::format(
        "[EKF] Twist queue size ({}) is exceeding max_queue_size ({}). Consider increasing "
        "max_queue_size or reducing input frequency.",
        twist_queue_.size(), twist_queue_.max_queue_size()),
      2000);
    twist_queue_.pop();
  }

  last_twist_callback_time_ = msg->header.stamp;
}

/*
 * publish_estimate_result
 */
void EKFLocalizer::publish_estimate_result(
  const geometry_msgs::msg::PoseStamped & current_ekf_pose,
  const geometry_msgs::msg::PoseStamped & current_biased_ekf_pose,
  const geometry_msgs::msg::TwistStamped & current_ekf_twist)
{
  /* publish latest pose */
  pub_pose_->publish(current_ekf_pose);
  pub_biased_pose_->publish(current_biased_ekf_pose);

  /* publish latest pose with covariance */
  geometry_msgs::msg::PoseWithCovarianceStamped pose_cov;
  pose_cov.header.stamp = current_ekf_pose.header.stamp;
  pose_cov.header.frame_id = current_ekf_pose.header.frame_id;
  pose_cov.pose.pose = current_ekf_pose.pose;
  pose_cov.pose.covariance = ekf_module_->get_current_pose_covariance();
  pub_pose_cov_->publish(pose_cov);

  geometry_msgs::msg::PoseWithCovarianceStamped biased_pose_cov = pose_cov;
  biased_pose_cov.pose.pose = current_biased_ekf_pose.pose;
  pub_biased_pose_cov_->publish(biased_pose_cov);

  /* publish latest twist */
  pub_twist_->publish(current_ekf_twist);

  /* publish latest twist with covariance */
  geometry_msgs::msg::TwistWithCovarianceStamped twist_cov;
  twist_cov.header.stamp = current_ekf_twist.header.stamp;
  twist_cov.header.frame_id = current_ekf_twist.header.frame_id;
  twist_cov.twist.twist = current_ekf_twist.twist;
  twist_cov.twist.covariance = ekf_module_->get_current_twist_covariance();
  pub_twist_cov_->publish(twist_cov);

  /* publish yaw bias */
  autoware_internal_debug_msgs::msg::Float64Stamped yawb;
  yawb.stamp = current_ekf_twist.header.stamp;
  yawb.data = ekf_module_->get_yaw_bias();
  pub_yaw_bias_->publish(yawb);

  /* publish latest odometry */
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = current_ekf_pose.header.stamp;
  odometry.header.frame_id = current_ekf_pose.header.frame_id;
  odometry.child_frame_id = "base_link";
  odometry.pose = pose_cov.pose;
  odometry.twist = twist_cov.twist;
  pub_odom_->publish(odometry);

  /* publish tf */
  const geometry_msgs::msg::TransformStamped transform_stamped =
    autoware_utils_geometry::pose2transform(current_ekf_pose, "base_link");
  tf_br_->sendTransform(transform_stamped);
}

void EKFLocalizer::diagnose(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // merged_diagnostic_status_ is updated in update_diagnostics() each EKF tick. Publishing is
  // driven by diagnostic_updater at diagnostics_publish_period, or force_update() on severity
  // increase vs. the previous tick.
  //
  // Thread safety: copy first for a consistent snapshot if a multi-threaded executor is used.
  const diagnostic_msgs::msg::DiagnosticStatus snapshot = merged_diagnostic_status_;

  stat.name = "localization: " + std::string(this->get_name());
  stat.hardware_id = this->get_name();
  stat.summary(snapshot);

  for (const auto & value : snapshot.values) {
    stat.add(value.key, value.value);
  }
}

void EKFLocalizer::diagnose_callback_pose(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.name = "localization: " + std::string(this->get_name()) + ": callback_pose";
  stat.hardware_id = this->get_name();
  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
  stat.add("topic_time_stamp", std::to_string(last_pose_callback_time_.nanoseconds()));
}

void EKFLocalizer::diagnose_callback_twist(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.name = "localization: " + std::string(this->get_name()) + ": callback_twist";
  stat.hardware_id = this->get_name();
  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
  stat.add("topic_time_stamp", std::to_string(last_twist_callback_time_.nanoseconds()));
}

void EKFLocalizer::update_diagnostics(
  const std::vector<diagnostic_msgs::msg::DiagnosticStatus> & diag_status_array,
  const rclcpp::Time & current_time)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  const uint8_t level_before = merged_diagnostic_status_.level;

  diagnostic_msgs::msg::DiagnosticStatus diag_merged_status;
  diag_merged_status = merge_diagnostic_status(diag_status_array);
  const uint8_t level_merged = diag_merged_status.level;

  // merged_diagnostic_status_ always tracks merge: ERROR→WARN when merge worst is WARN,
  // ERROR/WARN→OK when merge is all OK, same level refreshes message/values.
  merged_diagnostic_status_ = diag_merged_status;
  // last_transition_time: any level change; force_update below: severity increase only
  if (level_merged != level_before) {
    merged_diagnostic_last_transition_time_ = current_time;
  }

  // Remove transition timestamp keys if present (re-added below when we have a stored time)
  merged_diagnostic_status_.values.erase(
    std::remove_if(
      merged_diagnostic_status_.values.begin(), merged_diagnostic_status_.values.end(),
      [](const diagnostic_msgs::msg::KeyValue & kv) {
        return kv.key == "last_level_transition_timestamp";
      }),
    merged_diagnostic_status_.values.end());

  if (merged_diagnostic_last_transition_time_.nanoseconds() != 0) {
    diagnostic_msgs::msg::KeyValue transition_ts;
    transition_ts.key = "last_level_transition_timestamp";
    transition_ts.value = std::to_string(merged_diagnostic_last_transition_time_.nanoseconds());
    merged_diagnostic_status_.values.push_back(transition_ts);
  }

  if (level_merged > level_before) {
    diagnostics_.force_update();
  }
}

/**
 * @brief trigger node
 */
void EKFLocalizer::service_trigger_node(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  if (req->data) {
    pose_queue_.clear();
    twist_queue_.clear();
    is_activated_ = true;
  } else {
    is_activated_ = false;
    is_set_initialpose_ = false;
  }
  res->success = true;
}

}  // namespace autoware::ekf_localizer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::ekf_localizer::EKFLocalizer)
