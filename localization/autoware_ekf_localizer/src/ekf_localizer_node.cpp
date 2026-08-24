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

#include "ekf_localizer_node.hpp"

#include "autoware/localization_util/covariance_ellipse.hpp"
#include "utils/diagnostics.hpp"
#include "utils/string.hpp"
#include "utils/warning_message.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_logging/logger_level_configure.hpp>
#include <ekf_localizer.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>

#include <fmt/core.h>
#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::ekf_localizer
{

// clang-format off
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl // NOLINT
#define DEBUG_INFO(...) {if (params_.show_debug_info) {RCLCPP_INFO(__VA_ARGS__);}} // NOLINT
// clang-format on

using std::placeholders::_1;

EKFLocalizerNode::EKFLocalizerNode(const rclcpp::NodeOptions & node_options)
: autoware::agnocast_wrapper::Node("ekf_localizer", node_options),
  tf2_buffer_(this->get_clock()),
  tf2_listener_(tf2_buffer_, *this),
  params_(load_hyper_parameters(this)),
  cb_group_pose_(create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
  cb_group_twist_(create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
  merged_diagnostic_last_transition_time_(0, 0, RCL_ROS_TIME)
{
  merged_diagnostic_status_.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  merged_diagnostic_status_.message = "OK";

  /* initialize ros system */
  timer_control_ = autoware::agnocast_wrapper::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(params_.ekf_dt),
    std::bind(&EKFLocalizerNode::timer_callback, this));

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
  pub_diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 1);
  diagnostics_publish_timer_ = autoware::agnocast_wrapper::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(params_.diagnostics_publish_period),
    [this]() { publish_diagnostics(); });
  sub_initialpose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 1, std::bind(&EKFLocalizerNode::callback_initial_pose, this, _1));
  AUTOWARE_SUBSCRIPTION_OPTIONS pose_sub_opt;
  pose_sub_opt.callback_group = cb_group_pose_;
  sub_pose_with_cov_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "in_pose_with_covariance", 1,
    std::bind(&EKFLocalizerNode::callback_pose_with_covariance, this, _1), pose_sub_opt);

  AUTOWARE_SUBSCRIPTION_OPTIONS twist_sub_opt;
  twist_sub_opt.callback_group = cb_group_twist_;
  sub_twist_with_cov_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "in_twist_with_covariance", 1,
    std::bind(&EKFLocalizerNode::callback_twist_with_covariance, this, _1), twist_sub_opt);
  service_trigger_node_ = this->create_service<std_srvs::srv::SetBool>(
    "trigger_node_srv",
    std::bind(
      &EKFLocalizerNode::service_trigger_node, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS());

  tf_br_ = std::make_shared<autoware::agnocast_wrapper::TransformBroadcaster>(*this);

  ekf_localizer_ = std::make_unique<EKFLocalizer>(params_);
  logger_configure_ = std::make_unique<
    autoware_utils_logging::BasicLoggerLevelConfigure<autoware::agnocast_wrapper::Node>>(this);
}

/*
 * timer_callback
 */
void EKFLocalizerNode::timer_callback()
{
  stop_watch_timer_cb_.tic();
  const rclcpp::Time current_time = this->now();

  stop_watch_.tic();
  auto update_result = ekf_localizer_->update_step(current_time.seconds());

  // Log all warnings & debug logs
  for (const auto & warning : update_result.warnings) {
    if (warning.throttle_ms > 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), warning.throttle_ms, "%s", warning.text.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "%s", warning.text.c_str());
    }
  }

  // Initialize diagnostic status array to collect diagnostics during processing
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> diag_status_array;

  // Check process activation status
  diag_status_array.push_back(check_process_activated(update_result.is_activated));

  if (!update_result.is_activated) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "The node is not activated. Provide initial pose to pose_initializer");
    // Update diagnostics before early return to ensure current status is latched
    update_diagnostics(diag_status_array, current_time);
    return;
  }

  // Check initial pose status
  diag_status_array.push_back(check_set_initialpose(update_result.is_set_initialpose));

  if (!update_result.is_set_initialpose) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Initial pose is not set. Provide initial pose to pose_initializer");
    // Update diagnostics before early return to ensure current status is latched
    update_diagnostics(diag_status_array, current_time);
    return;
  }

  DEBUG_INFO(get_logger(), "========================= timer called =========================");

  for (const auto & debug_log : update_result.debug_logs) {
    DEBUG_INFO(get_logger(), "%s", debug_log.c_str());
  }

  DEBUG_INFO(get_logger(), "[EKF] update_step calc time = %f [ms]", stop_watch_.toc());

  // Add pose-related diagnostics after pose processing
  diag_status_array.push_back(check_measurement_updated(
    "pose", update_result.pose_diag_info.no_update_count,
    params_.pose_no_update_count_threshold_warn, params_.pose_no_update_count_threshold_error));
  diag_status_array.push_back(
    check_measurement_queue_size("pose", update_result.pose_diag_info.queue_size));
  diag_status_array.push_back(check_measurement_delay_gate(
    "pose", update_result.pose_diag_info.is_passed_delay_gate,
    update_result.pose_diag_info.delay_time, update_result.pose_diag_info.delay_time_threshold));
  diag_status_array.push_back(check_measurement_mahalanobis_gate(
    "pose", update_result.pose_diag_info.is_passed_mahalanobis_gate,
    update_result.pose_diag_info.mahalanobis_distance, params_.pose_gate_dist));

  // Add twist-related diagnostics after twist processing
  diag_status_array.push_back(check_measurement_updated(
    "twist", update_result.twist_diag_info.no_update_count,
    params_.twist_no_update_count_threshold_warn, params_.twist_no_update_count_threshold_error));
  diag_status_array.push_back(
    check_measurement_queue_size("twist", update_result.twist_diag_info.queue_size));
  diag_status_array.push_back(check_measurement_delay_gate(
    "twist", update_result.twist_diag_info.is_passed_delay_gate,
    update_result.twist_diag_info.delay_time, update_result.twist_diag_info.delay_time_threshold));
  diag_status_array.push_back(check_measurement_mahalanobis_gate(
    "twist", update_result.twist_diag_info.is_passed_mahalanobis_gate,
    update_result.twist_diag_info.mahalanobis_distance, params_.twist_gate_dist));

  diag_status_array.push_back(check_covariance_ellipse(
    "cov_ellipse_long_axis", update_result.ellipse_long_radius, params_.warn_ellipse_size,
    params_.error_ellipse_size));
  diag_status_array.push_back(check_covariance_ellipse(
    "cov_ellipse_lateral_direction", update_result.ellipse_size_lateral_direction,
    params_.warn_ellipse_size_lateral_direction, params_.error_ellipse_size_lateral_direction));

  /* publish ekf result */
  publish_estimate_result(update_result);

  /* Latch merged diagnostics every EKF cycle; publishing is periodic via
   * diagnostics_publish_timer_, plus publish_diagnostics() inside update_diagnostics when severity
   * increases. */
  update_diagnostics(diag_status_array, current_time);

  /* publish processing time */
  const double elapsed_time = stop_watch_timer_cb_.toc();
  {
    auto msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_processing_time_);
    msg->stamp = current_time;
    msg->data = elapsed_time;
    pub_processing_time_->publish(std::move(msg));
  }
}

/*
 * get_transform_from_tf
 */
bool EKFLocalizerNode::get_transform_from_tf(
  std::string parent_frame, std::string child_frame,
  geometry_msgs::msg::TransformStamped & transform)
{
  parent_frame = erase_leading_slash(parent_frame);
  child_frame = erase_leading_slash(child_frame);

  try {
    transform = tf2_buffer_.lookupTransform(parent_frame, child_frame, tf2::TimePointZero);
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
  }
  return false;
}

/*
 * callback_initial_pose
 */
void EKFLocalizerNode::callback_initial_pose(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(geometry_msgs::msg::PoseWithCovarianceStamped) msg)
{
  geometry_msgs::msg::TransformStamped transform;
  if (!get_transform_from_tf(params_.pose_frame_id, msg->header.frame_id, transform)) {
    RCLCPP_ERROR(
      get_logger(), "[EKF] TF transform failed. parent = %s, child = %s",
      params_.pose_frame_id.c_str(), msg->header.frame_id.c_str());
  }
  ekf_localizer_->initialize(*msg, transform);
}

/*
 * callback_pose_with_covariance
 */
void EKFLocalizerNode::callback_pose_with_covariance(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(geometry_msgs::msg::PoseWithCovarianceStamped) msg)
{
  ekf_localizer_->push_pose(msg);
  last_pose_callback_time_ns_.store(rclcpp::Time(msg->header.stamp).nanoseconds());
}

/*
 * callback_twist_with_covariance
 */
void EKFLocalizerNode::callback_twist_with_covariance(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(geometry_msgs::msg::TwistWithCovarianceStamped) msg)
{
  ekf_localizer_->push_twist(msg);
  last_twist_callback_time_ns_.store(rclcpp::Time(msg->header.stamp).nanoseconds());
}

/*
 * publish_estimate_result
 */
void EKFLocalizerNode::publish_estimate_result(const EKFUpdateResult & result)
{
  /* publish latest pose */
  {
    auto msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_pose_);
    *msg = result.pose;
    pub_pose_->publish(std::move(msg));
  }
  {
    auto msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_biased_pose_);
    *msg = result.biased_pose;
    pub_biased_pose_->publish(std::move(msg));
  }

  /* publish latest pose with covariance */
  {
    auto msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_pose_cov_);
    *msg = result.pose_cov;
    pub_pose_cov_->publish(std::move(msg));
  }
  {
    auto msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_biased_pose_cov_);
    *msg = result.biased_pose_cov;
    pub_biased_pose_cov_->publish(std::move(msg));
  }

  /* publish latest twist */
  {
    auto msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_twist_);
    *msg = result.twist;
    pub_twist_->publish(std::move(msg));
  }

  /* publish latest twist with covariance */
  {
    auto msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_twist_cov_);
    *msg = result.twist_cov;
    pub_twist_cov_->publish(std::move(msg));
  }

  /* publish yaw bias */
  {
    auto msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_yaw_bias_);
    msg->stamp = result.twist.header.stamp;
    msg->data = result.yaw_bias;
    pub_yaw_bias_->publish(std::move(msg));
  }

  /* publish latest odometry */
  {
    auto msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_odom_);
    msg->header = result.pose.header;
    msg->child_frame_id = "base_link";
    msg->pose = result.pose_cov.pose;
    msg->twist = result.twist_cov.twist;
    pub_odom_->publish(std::move(msg));
  }

  /* publish tf */
  const geometry_msgs::msg::TransformStamped transform_stamped =
    autoware_utils_geometry::pose2transform(result.pose, "base_link");
  tf_br_->sendTransform(transform_stamped);
}

void EKFLocalizerNode::publish_diagnostics()
{
  const std::string node_name = this->get_name();
  const std::string main_name = "localization: " + node_name;
  const std::string pose_name = main_name + ": callback_pose";
  const std::string twist_name = main_name + ": callback_twist";

  // Thread safety: snapshot merged status for a consistent array if a multi-threaded executor is
  // used.
  diagnostic_msgs::msg::DiagnosticStatus main_st = merged_diagnostic_status_;
  main_st.name = main_name;
  main_st.hardware_id = node_name;

  diagnostic_msgs::msg::DiagnosticStatus pose_st;
  pose_st.name = pose_name;
  pose_st.hardware_id = node_name;
  pose_st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  pose_st.message = "OK";
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "topic_time_stamp";
    kv.value = std::to_string(last_pose_callback_time_ns_.load());
    pose_st.values.push_back(kv);
  }

  diagnostic_msgs::msg::DiagnosticStatus twist_st;
  twist_st.name = twist_name;
  twist_st.hardware_id = node_name;
  twist_st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  twist_st.message = "OK";
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "topic_time_stamp";
    kv.value = std::to_string(last_twist_callback_time_ns_.load());
    twist_st.values.push_back(kv);
  }

  auto msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_diagnostics_);
  msg->header.stamp = this->now();
  msg->status.push_back(main_st);
  msg->status.push_back(pose_st);
  msg->status.push_back(twist_st);
  pub_diagnostics_->publish(std::move(msg));
}

void EKFLocalizerNode::update_diagnostics(
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
  // last_transition_time: any level change; immediate publish below: severity increase only
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
    publish_diagnostics();
  }
}

/**
 * @brief trigger node
 */
void EKFLocalizerNode::service_trigger_node(
  const AUTOWARE_SERVER_REQUEST_PTR(std_srvs::srv::SetBool) req,
  AUTOWARE_SERVER_RESPONSE_PTR(std_srvs::srv::SetBool) res)
{
  ekf_localizer_->activate(req->data);
  res->success = true;
}

}  // namespace autoware::ekf_localizer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::ekf_localizer::EKFLocalizerNode)
