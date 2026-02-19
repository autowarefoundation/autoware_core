// Copyright 2025 Autoware Foundation
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

#include "autoware/unified_localization_core/fusion_pipeline.hpp"
#include "autoware/unified_localization_core/types.hpp"

#include <autoware/component_interface_specs/localization.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/exceptions.hpp>

#include <autoware_internal_localization_msgs/srv/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace autoware::unified_localization
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::TwistWithCovarianceStamped;
using nav_msgs::msg::Odometry;

using NdtAlignSrv = autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped;
using SetBool = std_srvs::srv::SetBool;

static void msg_to_pose_core(
  const PoseWithCovarianceStamped & msg,
  autoware::unified_localization_core::PoseWithCovariance & out)
{
  out.timestamp_sec = rclcpp::Time(msg.header.stamp).seconds();
  out.position_x = msg.pose.pose.position.x;
  out.position_y = msg.pose.pose.position.y;
  out.position_z = msg.pose.pose.position.z;
  out.orientation_x = msg.pose.pose.orientation.x;
  out.orientation_y = msg.pose.pose.orientation.y;
  out.orientation_z = msg.pose.pose.orientation.z;
  out.orientation_w = msg.pose.pose.orientation.w;
  for (size_t i = 0; i < 36u; ++i) {
    out.covariance[i] = msg.pose.covariance[i];
  }
}

static void msg_to_twist_core(
  const TwistWithCovarianceStamped & msg,
  autoware::unified_localization_core::TwistWithCovariance & out)
{
  out.timestamp_sec = rclcpp::Time(msg.header.stamp).seconds();
  out.linear_x = msg.twist.twist.linear.x;
  out.linear_y = msg.twist.twist.linear.y;
  out.linear_z = msg.twist.twist.linear.z;
  out.angular_x = msg.twist.twist.angular.x;
  out.angular_y = msg.twist.twist.angular.y;
  out.angular_z = msg.twist.twist.angular.z;
  for (size_t i = 0; i < 36u; ++i) {
    out.covariance[i] = msg.twist.covariance[i];
  }
}

class LocalizationNode : public rclcpp::Node
{
public:
  explicit LocalizationNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("localization_node", options)
  {
    load_params();
    pipeline_ = std::make_unique<unified_localization_core::FusionPipeline>(params_);

    const double rate = params_.ekf.predict_frequency;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&LocalizationNode::timer_callback, this));

    pub_kinematic_state_ = create_publisher<Odometry>("kinematic_state", rclcpp::QoS(1));
    pub_acceleration_ =
      create_publisher<AccelWithCovarianceStamped>("acceleration", rclcpp::QoS(1));
    {
      using State = autoware::component_interface_specs::localization::InitializationState;
      rclcpp::QoS qos_state(State::depth);
      qos_state.reliability(State::reliability);
      qos_state.durability(State::durability);
      pub_initialization_state_ = create_publisher<State::Message>(State::name, qos_state);
    }
    pub_diagnostics_ = create_publisher<DiagnosticArray>("/diagnostics", rclcpp::QoS(1));
    diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&LocalizationNode::diagnostics_timer_callback, this));
    tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    sub_initial_pose_ = create_subscription<PoseWithCovarianceStamped>(
      "initialpose", rclcpp::QoS(1),
      std::bind(&LocalizationNode::on_initial_pose, this, std::placeholders::_1));
    sub_pose_ = create_subscription<PoseWithCovarianceStamped>(
      "in_pose_with_covariance", rclcpp::QoS(1),
      std::bind(&LocalizationNode::on_pose, this, std::placeholders::_1));
    sub_twist_ = create_subscription<TwistWithCovarianceStamped>(
      "in_twist_with_covariance", rclcpp::QoS(1),
      std::bind(&LocalizationNode::on_twist, this, std::placeholders::_1));

    if (!auto_gnss_pose_topic_.empty()) {
      sub_gnss_pose_ = create_subscription<PoseWithCovarianceStamped>(
        auto_gnss_pose_topic_, rclcpp::QoS(1),
        std::bind(&LocalizationNode::on_gnss_pose, this, std::placeholders::_1));
    }

    using Initialize = autoware::component_interface_specs::localization::Initialize;
    srv_initialize_ = create_service<Initialize::Service>(
      Initialize::name, std::bind(
                          &LocalizationNode::on_initialize_service, this, std::placeholders::_1,
                          std::placeholders::_2));

    if (auto_ndt_align_enabled_) {
      cli_ndt_align_ = create_client<NdtAlignSrv>(auto_ndt_align_service_name_);
    }
    if (auto_yabloc_align_enabled_) {
      cli_yabloc_align_ = create_client<NdtAlignSrv>(auto_yabloc_align_service_name_);
    }
    for (const std::string & name : trigger_service_names_) {
      trigger_clients_.push_back(create_client<SetBool>(name));
    }

    publish_initialization_state(
      autoware::component_interface_specs::localization::InitializationState::Message::
        UNINITIALIZED);
  }

private:
  void load_params()
  {
    params_.ekf.predict_frequency = declare_parameter<double>("node.predict_frequency", 50.0);
    params_.ekf.ekf_dt = 1.0 / std::max(params_.ekf.predict_frequency, 0.1);
    params_.ekf.enable_yaw_bias_estimation =
      declare_parameter<bool>("node.enable_yaw_bias_estimation", true);
    params_.ekf.extend_state_step =
      static_cast<size_t>(declare_parameter<int>("node.extend_state_step", 50));

    params_.ekf.pose_additional_delay =
      declare_parameter<double>("pose_measurement.pose_additional_delay", 0.0);
    params_.ekf.pose_gate_dist = declare_parameter<double>("pose_measurement.pose_gate_dist", 49.5);
    params_.ekf.pose_smoothing_steps =
      static_cast<size_t>(declare_parameter<int>("pose_measurement.pose_smoothing_steps", 5));
    params_.ekf.max_pose_queue_size =
      static_cast<size_t>(declare_parameter<int>("pose_measurement.max_pose_queue_size", 5));

    params_.ekf.twist_additional_delay =
      declare_parameter<double>("twist_measurement.twist_additional_delay", 0.0);
    params_.ekf.twist_gate_dist =
      declare_parameter<double>("twist_measurement.twist_gate_dist", 46.1);
    params_.ekf.twist_smoothing_steps =
      static_cast<size_t>(declare_parameter<int>("twist_measurement.twist_smoothing_steps", 2));
    params_.ekf.max_twist_queue_size =
      static_cast<size_t>(declare_parameter<int>("twist_measurement.max_twist_queue_size", 2));

    params_.ekf.proc_stddev_vx_c =
      declare_parameter<double>("process_noise.proc_stddev_vx_c", 10.0);
    params_.ekf.proc_stddev_wz_c = declare_parameter<double>("process_noise.proc_stddev_wz_c", 5.0);
    params_.ekf.proc_stddev_yaw_c =
      declare_parameter<double>("process_noise.proc_stddev_yaw_c", 0.005);

    params_.ekf.z_filter_proc_dev =
      declare_parameter<double>("simple_1d_filter_parameters.z_filter_proc_dev", 5.0);
    params_.ekf.roll_filter_proc_dev =
      declare_parameter<double>("simple_1d_filter_parameters.roll_filter_proc_dev", 0.1);
    params_.ekf.pitch_filter_proc_dev =
      declare_parameter<double>("simple_1d_filter_parameters.pitch_filter_proc_dev", 0.1);

    params_.stop_filter.linear_x_threshold =
      declare_parameter<double>("stop_filter.linear_x_threshold", 0.1);
    params_.stop_filter.angular_z_threshold =
      declare_parameter<double>("stop_filter.angular_z_threshold", 0.02);

    params_.twist2accel.accel_lowpass_gain =
      declare_parameter<double>("twist2accel.accel_lowpass_gain", 0.2);

    pose_frame_id_ = declare_parameter<std::string>("pose_frame_id", "map");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base_link");

    auto_stop_check_enabled_ = declare_parameter<bool>("initialize.auto_stop_check_enabled", false);
    auto_stop_check_duration_sec_ =
      declare_parameter<double>("initialize.auto_stop_check_duration_sec", 3.0);
    auto_stop_velocity_threshold_ =
      declare_parameter<double>("initialize.auto_stop_velocity_threshold", 0.1);

    auto_ndt_align_enabled_ = declare_parameter<bool>("initialize.auto_ndt_align_enabled", false);
    auto_ndt_align_service_name_ = declare_parameter<std::string>(
      "initialize.auto_ndt_align_service_name", "/localization/pose_estimator/ndt_align_srv");
    auto_ndt_align_timeout_sec_ =
      declare_parameter<double>("initialize.auto_ndt_align_timeout_sec", 10.0);

    auto_yabloc_align_enabled_ =
      declare_parameter<bool>("initialize.auto_yabloc_align_enabled", false);
    auto_yabloc_align_service_name_ = declare_parameter<std::string>(
      "initialize.auto_yabloc_align_service_name",
      "/localization/pose_estimator/yabloc_align_srv");
    auto_yabloc_align_timeout_sec_ =
      declare_parameter<double>("initialize.auto_yabloc_align_timeout_sec", 10.0);

    try {
      trigger_service_names_ = declare_parameter<std::vector<std::string>>(
        "initialize.trigger_services", std::vector<std::string>{});
    } catch (const rclcpp::exceptions::InvalidParameterValueException &) {
      // Empty list [] in YAML can fail type conversion to std::vector<std::string> in some setups
      trigger_service_names_ = std::vector<std::string>{};
    }
    trigger_timeout_sec_ = declare_parameter<double>("initialize.trigger_timeout_sec", 5.0);

    auto_gnss_pose_topic_ = declare_parameter<std::string>("initialize.auto_gnss_pose_topic", "");

    initial_pose_covariance_override_enabled_ =
      declare_parameter<bool>("initialize.initial_pose_covariance_override_enabled", false);
    const std::vector<double> cov_param = declare_parameter<std::vector<double>>(
      "initialize.initial_pose_covariance", std::vector<double>{});
    if (initial_pose_covariance_override_enabled_ && cov_param.size() == 36u) {
      for (size_t i = 0; i < 36u; ++i) {
        initial_pose_covariance_[i] = cov_param[i];
      }
    } else if (initial_pose_covariance_override_enabled_ && cov_param.size() == 6u) {
      for (size_t i = 0; i < 36u; ++i) {
        initial_pose_covariance_[i] = (i % 7 == 0) ? cov_param[i / 7] : 0.0;
      }
    } else if (initial_pose_covariance_override_enabled_) {
      RCLCPP_WARN(
        get_logger(),
        "initialize.initial_pose_covariance has size %zu (need 36 or 6 for diagonal); override disabled.",
        cov_param.size());
      initial_pose_covariance_override_enabled_ = false;
    }
  }

  void apply_initial_pose_covariance_override(
    unified_localization_core::PoseWithCovariance & pose_core)
  {
    if (!initial_pose_covariance_override_enabled_) {
      return;
    }
    for (size_t i = 0; i < 36u; ++i) {
      pose_core.covariance[i] = initial_pose_covariance_[i];
    }
  }

  void publish_initialization_state(
    autoware::component_interface_specs::localization::InitializationState::Message::_state_type
      state)
  {
    using State = autoware::component_interface_specs::localization::InitializationState;
    State::Message msg;
    msg.stamp = this->now();
    msg.state = state;
    pub_initialization_state_->publish(msg);
  }

  void diagnostics_timer_callback()
  {
    const bool initialized = pipeline_->is_initialized();
    if (!initialized) {
      publish_initialization_state(
        autoware::component_interface_specs::localization::InitializationState::Message::
          UNINITIALIZED);
    }

    DiagnosticArray array;
    array.header.stamp = this->now();
    array.header.frame_id = "";

    DiagnosticStatus status;
    status.name = "localization: localization_node";
    status.hardware_id = "localization_node";
    if (initialized) {
      status.level = DiagnosticStatus::OK;
      status.message = "Initialized";
    } else {
      status.level = DiagnosticStatus::WARN;
      status.message = "Not initialized";
    }

    KeyValue kv_init;
    kv_init.key = "initialized";
    kv_init.value = initialized ? "true" : "false";
    status.values.push_back(kv_init);

    double pose_age_sec = -1.0;
    double twist_age_sec = -1.0;
    {
      std::lock_guard<std::mutex> lock(meas_mutex_);
      const double now_sec = this->now().seconds();
      if (last_pose_) {
        pose_age_sec = now_sec - last_pose_->timestamp_sec;
      }
      if (last_twist_) {
        twist_age_sec = now_sec - last_twist_->timestamp_sec;
      }
    }
    KeyValue kv_pose;
    kv_pose.key = "pose_age_sec";
    kv_pose.value = pose_age_sec >= 0 ? std::to_string(pose_age_sec) : "none";
    status.values.push_back(kv_pose);
    KeyValue kv_twist;
    kv_twist.key = "twist_age_sec";
    kv_twist.value = twist_age_sec >= 0 ? std::to_string(twist_age_sec) : "none";
    status.values.push_back(kv_twist);

    array.status.push_back(status);
    pub_diagnostics_->publish(array);
  }

  void on_initial_pose(const PoseWithCovarianceStamped::SharedPtr msg)
  {
    unified_localization_core::PoseWithCovariance pose_core;
    msg_to_pose_core(*msg, pose_core);
    apply_initial_pose_covariance_override(pose_core);
    pipeline_->initialize(pose_core);
    publish_initialization_state(
      autoware::component_interface_specs::localization::InitializationState::Message::INITIALIZED);
    RCLCPP_INFO(get_logger(), "Initial pose set (Adapter).");
  }

  void on_pose(const PoseWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(meas_mutex_);
    last_pose_ = std::make_shared<unified_localization_core::PoseWithCovariance>();
    msg_to_pose_core(*msg, *last_pose_);
  }

  void on_gnss_pose(const PoseWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(gnss_pose_mutex_);
    last_gnss_pose_ = msg;
  }

  void on_twist(const TwistWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(meas_mutex_);
    last_twist_ = std::make_shared<unified_localization_core::TwistWithCovariance>();
    msg_to_twist_core(*msg, *last_twist_);
    if (auto_stop_check_enabled_) {
      const double vx = msg->twist.twist.linear.x;
      const double vy = msg->twist.twist.linear.y;
      const double vz = msg->twist.twist.linear.z;
      const double linear_speed = std::sqrt(vx * vx + vy * vy + vz * vz);
      twist_history_.emplace_back(this->now(), linear_speed);
      const rclcpp::Duration max_age(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(auto_stop_check_duration_sec_)));
      while (twist_history_.size() > 1u && (this->now() - twist_history_.front().first) > max_age) {
        twist_history_.pop_front();
      }
    }
  }

  bool try_align(
    rclcpp::Client<NdtAlignSrv>::SharedPtr client, double timeout_sec,
    const PoseWithCovarianceStamped & seed, PoseWithCovarianceStamped & out_aligned)
  {
    auto req = std::make_shared<NdtAlignSrv::Request>();
    req->pose_with_covariance = seed;
    {
      std::lock_guard<std::mutex> lock(align_mutex_);
      align_done_ = false;
      align_success_ = false;
    }
    client->async_send_request(
      req, [this](rclcpp::Client<NdtAlignSrv>::SharedFutureWithRequest future_with_req) {
        auto result = future_with_req.get();
        std::lock_guard<std::mutex> lock(align_mutex_);
        align_success_ = result.second->success;
        if (align_success_) {
          align_pose_ = result.second->pose_with_covariance;
        }
        align_done_ = true;
        align_cv_.notify_one();
      });
    const auto timeout = std::chrono::duration<double>(timeout_sec);
    std::unique_lock<std::mutex> lock(align_mutex_);
    const bool completed = align_cv_.wait_for(lock, timeout, [this] { return align_done_; });
    if (!completed || !align_success_) {
      return false;
    }
    out_aligned = align_pose_;
    return true;
  }

  bool call_trigger_services(bool activate, std::string * out_error_message = nullptr)
  {
    if (trigger_clients_.empty()) {
      return true;
    }
    const std::chrono::duration<double> timeout(trigger_timeout_sec_);
    for (size_t i = 0; i < trigger_clients_.size(); ++i) {
      const auto & client = trigger_clients_[i];
      if (!client->service_is_ready()) {
        if (out_error_message) {
          *out_error_message = "Trigger service [" + std::to_string(i) + "] is not ready.";
        }
        return false;
      }
      auto req = std::make_shared<SetBool::Request>();
      req->data = activate;
      {
        std::lock_guard<std::mutex> lock(trigger_mutex_);
        trigger_done_ = false;
        trigger_success_ = false;
      }
      client->async_send_request(
        req, [this](rclcpp::Client<SetBool>::SharedFutureWithRequest future_with_req) {
          auto result = future_with_req.get();
          std::lock_guard<std::mutex> lock(trigger_mutex_);
          trigger_success_ = result.second->success;
          trigger_done_ = true;
          trigger_cv_.notify_one();
        });
      std::unique_lock<std::mutex> lock(trigger_mutex_);
      const bool completed = trigger_cv_.wait_for(lock, timeout, [this] { return trigger_done_; });
      if (!completed || !trigger_success_) {
        if (out_error_message) {
          *out_error_message =
            completed ? "Trigger service returned failure." : "Trigger service timed out.";
        }
        return false;
      }
    }
    return true;
  }

  bool is_vehicle_stopped()
  {
    std::lock_guard<std::mutex> lock(meas_mutex_);
    if (twist_history_.empty()) {
      return false;
    }
    const rclcpp::Duration duration(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(auto_stop_check_duration_sec_)));
    if ((this->now() - twist_history_.front().first) < duration) {
      return false;
    }
    for (const auto & entry : twist_history_) {
      if (entry.second > auto_stop_velocity_threshold_) {
        return false;
      }
    }
    return true;
  }

  void on_initialize_service(
    const autoware::component_interface_specs::localization::Initialize::Service::Request::SharedPtr
      req,
    const autoware::component_interface_specs::localization::Initialize::Service::Response::
      SharedPtr res)
  {
    using Initialize = autoware::component_interface_specs::localization::Initialize;
    using InitState =
      autoware::component_interface_specs::localization::InitializationState::Message;
    const auto method = req->method;

    if (method == Initialize::Service::Request::AUTO) {
      PoseWithCovarianceStamped pose;
      if (req->pose_with_covariance.empty()) {
        if (auto_gnss_pose_topic_.empty()) {
          res->status.success = false;
          res->status.code = Initialize::Service::Response::ERROR_GNSS_SUPPORT;
          res->status.message =
            "AUTO with empty pose requires initialize.auto_gnss_pose_topic to be set (GNSS-only "
            "mode).";
          RCLCPP_WARN(get_logger(), "%s", res->status.message.c_str());
          return;
        }
        PoseWithCovarianceStamped::SharedPtr gnss_pose;
        {
          std::lock_guard<std::mutex> lock(gnss_pose_mutex_);
          gnss_pose = last_gnss_pose_;
        }
        if (!gnss_pose) {
          res->status.success = false;
          res->status.code = Initialize::Service::Response::ERROR_GNSS_SUPPORT;
          res->status.message = "No GNSS pose received yet. Ensure the topic '" +
                                auto_gnss_pose_topic_ + "' is published.";
          RCLCPP_WARN(get_logger(), "%s", res->status.message.c_str());
          return;
        }
        pose = *gnss_pose;
      } else {
        pose = req->pose_with_covariance.front();
      }
      if (auto_stop_check_enabled_ && !is_vehicle_stopped()) {
        res->status.success = false;
        res->status.code = Initialize::Service::Response::ERROR_UNSAFE;
        res->status.message = "The vehicle is not stopped.";
        RCLCPP_WARN(get_logger(), "%s", res->status.message.c_str());
        return;
      }
      publish_initialization_state(InitState::INITIALIZING);
      std::string trigger_error;
      if (!call_trigger_services(false, &trigger_error)) {
        res->status.success = false;
        res->status.code = Initialize::Service::Response::ERROR_ESTIMATION;
        res->status.message = "Trigger deactivation failed: " + trigger_error;
        RCLCPP_WARN(get_logger(), "%s", res->status.message.c_str());
        publish_initialization_state(InitState::UNINITIALIZED);
        return;
      }
      if (auto_ndt_align_enabled_) {
        if (!cli_ndt_align_ || !cli_ndt_align_->service_is_ready()) {
          res->status.success = false;
          res->status.code = Initialize::Service::Response::ERROR_ESTIMATION;
          res->status.message = "NDT align is enabled but the align service is not ready.";
          RCLCPP_WARN(get_logger(), "%s", res->status.message.c_str());
          publish_initialization_state(InitState::UNINITIALIZED);
          return;
        }
        PoseWithCovarianceStamped aligned;
        if (!try_align(cli_ndt_align_, auto_ndt_align_timeout_sec_, pose, aligned)) {
          res->status.success = false;
          res->status.code = Initialize::Service::Response::ERROR_ESTIMATION;
          res->status.message = "NDT align failed or timed out.";
          RCLCPP_WARN(get_logger(), "%s", res->status.message.c_str());
          publish_initialization_state(InitState::UNINITIALIZED);
          return;
        }
        pose = aligned;
        RCLCPP_INFO(
          get_logger(), "Initial pose set via /localization/initialize (AUTO) with NDT align.");
      } else if (auto_yabloc_align_enabled_) {
        if (!cli_yabloc_align_ || !cli_yabloc_align_->service_is_ready()) {
          res->status.success = false;
          res->status.code = Initialize::Service::Response::ERROR_ESTIMATION;
          res->status.message = "YabLoc align is enabled but the align service is not ready.";
          RCLCPP_WARN(get_logger(), "%s", res->status.message.c_str());
          publish_initialization_state(InitState::UNINITIALIZED);
          return;
        }
        PoseWithCovarianceStamped aligned;
        if (!try_align(
              cli_yabloc_align_, auto_yabloc_align_timeout_sec_, pose, aligned)) {
          res->status.success = false;
          res->status.code = Initialize::Service::Response::ERROR_ESTIMATION;
          res->status.message = "YabLoc align failed or timed out.";
          RCLCPP_WARN(get_logger(), "%s", res->status.message.c_str());
          publish_initialization_state(InitState::UNINITIALIZED);
          return;
        }
        pose = aligned;
        RCLCPP_INFO(
          get_logger(), "Initial pose set via /localization/initialize (AUTO) with YabLoc align.");
      } else {
        RCLCPP_INFO(get_logger(), "Initial pose set via /localization/initialize (AUTO).");
      }
      unified_localization_core::PoseWithCovariance pose_core;
      msg_to_pose_core(pose, pose_core);
      apply_initial_pose_covariance_override(pose_core);
      pipeline_->initialize(pose_core);
      if (!call_trigger_services(true, &trigger_error)) {
        res->status.success = false;
        res->status.message = "Trigger activation failed: " + trigger_error;
        RCLCPP_WARN(get_logger(), "%s", res->status.message.c_str());
        publish_initialization_state(InitState::UNINITIALIZED);
        return;
      }
      publish_initialization_state(InitState::INITIALIZED);
      res->status.success = true;
      res->status.message = "";
      return;
    }

    if (method == Initialize::Service::Request::DIRECT) {
      if (req->pose_with_covariance.empty()) {
        res->status.success = false;
        res->status.code = Initialize::Service::Response::ERROR_GNSS_SUPPORT;
        res->status.message = "DIRECT method requires at least one pose_with_covariance.";
        RCLCPP_WARN(get_logger(), "%s", res->status.message.c_str());
        return;
      }
      publish_initialization_state(InitState::INITIALIZING);
      std::string trigger_error;
      if (!call_trigger_services(false, &trigger_error)) {
        res->status.success = false;
        res->status.code = Initialize::Service::Response::ERROR_ESTIMATION;
        res->status.message = "Trigger deactivation failed: " + trigger_error;
        RCLCPP_WARN(get_logger(), "%s", res->status.message.c_str());
        publish_initialization_state(InitState::UNINITIALIZED);
        return;
      }
      const PoseWithCovarianceStamped & msg = req->pose_with_covariance.front();
      unified_localization_core::PoseWithCovariance pose_core;
      msg_to_pose_core(msg, pose_core);
      apply_initial_pose_covariance_override(pose_core);
      pipeline_->initialize(pose_core);
      if (!call_trigger_services(true, &trigger_error)) {
        res->status.success = false;
        res->status.message = "Trigger activation failed: " + trigger_error;
        RCLCPP_WARN(get_logger(), "%s", res->status.message.c_str());
        publish_initialization_state(InitState::UNINITIALIZED);
        return;
      }
      publish_initialization_state(InitState::INITIALIZED);
      res->status.success = true;
      res->status.message = "";
      RCLCPP_INFO(get_logger(), "Initial pose set via /localization/initialize (DIRECT).");
      return;
    }

    res->status.success = false;
    res->status.code = Initialize::Service::Response::ERROR_ESTIMATION;
    res->status.message = "Unknown method type (use AUTO or DIRECT).";
    RCLCPP_WARN(get_logger(), "%s", res->status.message.c_str());
  }

  void timer_callback()
  {
    const rclcpp::Time now = this->now();
    const double t_sec = now.seconds();

    if (!pipeline_->is_initialized()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Pipeline not initialized. Send initialpose (e.g. from pose_initializer).");
      return;
    }

    double dt_sec = params_.ekf.ekf_dt;
    if (last_timer_time_) {
      dt_sec = (now - *last_timer_time_).seconds();
      if (dt_sec <= 0.0 || dt_sec > 10.0) {
        dt_sec = params_.ekf.ekf_dt;
      }
    }
    last_timer_time_ = now;

    const unified_localization_core::PoseWithCovariance * pose_ptr = nullptr;
    const unified_localization_core::TwistWithCovariance * twist_ptr = nullptr;
    {
      std::lock_guard<std::mutex> lock(meas_mutex_);
      if (last_pose_) pose_ptr = last_pose_.get();
      if (last_twist_) twist_ptr = last_twist_.get();
    }

    pipeline_->step(t_sec, dt_sec, pose_ptr, twist_ptr);

    unified_localization_core::OdometryOutput odom;
    pipeline_->get_odometry(t_sec, odom);

    // Publish kinematic_state (Odometry)
    Odometry odom_msg;
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = pose_frame_id_;
    odom_msg.child_frame_id = child_frame_id_;
    odom_msg.pose.pose.position.x = odom.position_x;
    odom_msg.pose.pose.position.y = odom.position_y;
    odom_msg.pose.pose.position.z = odom.position_z;
    odom_msg.pose.pose.orientation.x = odom.orientation_x;
    odom_msg.pose.pose.orientation.y = odom.orientation_y;
    odom_msg.pose.pose.orientation.z = odom.orientation_z;
    odom_msg.pose.pose.orientation.w = odom.orientation_w;
    for (size_t i = 0; i < 36u; ++i) {
      odom_msg.pose.covariance[i] = odom.pose_covariance[i];
    }
    odom_msg.twist.twist.linear.x = odom.linear_x;
    odom_msg.twist.twist.linear.y = odom.linear_y;
    odom_msg.twist.twist.linear.z = odom.linear_z;
    odom_msg.twist.twist.angular.x = odom.angular_x;
    odom_msg.twist.twist.angular.y = odom.angular_y;
    odom_msg.twist.twist.angular.z = odom.angular_z;
    for (size_t i = 0; i < 36u; ++i) {
      odom_msg.twist.covariance[i] = odom.twist_covariance[i];
    }
    pub_kinematic_state_->publish(odom_msg);

    // Publish acceleration (from previous and current twist)
    unified_localization_core::AccelerationOutput acc_core;
    if (prev_odom_) {
      unified_localization_core::Vector3 prev_lin{
        prev_odom_->linear_x, prev_odom_->linear_y, prev_odom_->linear_z};
      unified_localization_core::Vector3 prev_ang{
        prev_odom_->angular_x, prev_odom_->angular_y, prev_odom_->angular_z};
      unified_localization_core::Vector3 curr_lin{odom.linear_x, odom.linear_y, odom.linear_z};
      unified_localization_core::Vector3 curr_ang{odom.angular_x, odom.angular_y, odom.angular_z};
      pipeline_->get_acceleration(
        prev_odom_->timestamp_sec, odom.timestamp_sec, prev_lin, prev_ang, curr_lin, curr_ang,
        acc_core);
      AccelWithCovarianceStamped acc_msg;
      acc_msg.header.stamp = now;
      acc_msg.header.frame_id = child_frame_id_;
      acc_msg.accel.accel.linear.x = acc_core.linear_x;
      acc_msg.accel.accel.linear.y = acc_core.linear_y;
      acc_msg.accel.accel.linear.z = acc_core.linear_z;
      acc_msg.accel.accel.angular.x = acc_core.angular_x;
      acc_msg.accel.accel.angular.y = acc_core.angular_y;
      acc_msg.accel.accel.angular.z = acc_core.angular_z;
      acc_msg.accel.covariance.fill(0.0);
      pub_acceleration_->publish(acc_msg);
    }
    prev_odom_ = odom;

    // Publish TF (map -> base_link)
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = pose_frame_id_;
    tf.child_frame_id = child_frame_id_;
    tf.transform.translation.x = odom.position_x;
    tf.transform.translation.y = odom.position_y;
    tf.transform.translation.z = odom.position_z;
    tf.transform.rotation.x = odom.orientation_x;
    tf.transform.rotation.y = odom.orientation_y;
    tf.transform.rotation.z = odom.orientation_z;
    tf.transform.rotation.w = odom.orientation_w;
    tf_br_->sendTransform(tf);
  }

  unified_localization_core::FusionPipelineParams params_;
  std::unique_ptr<unified_localization_core::FusionPipeline> pipeline_;
  std::string pose_frame_id_;
  std::string child_frame_id_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<rclcpp::Time> last_timer_time_;

  rclcpp::Publisher<Odometry>::SharedPtr pub_kinematic_state_;
  rclcpp::Publisher<AccelWithCovarianceStamped>::SharedPtr pub_acceleration_;
  rclcpp::Publisher<
    autoware::component_interface_specs::localization::InitializationState::Message>::SharedPtr
    pub_initialization_state_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr pub_diagnostics_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;

  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_initial_pose_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<TwistWithCovarianceStamped>::SharedPtr sub_twist_;
  std::string auto_gnss_pose_topic_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_gnss_pose_;
  PoseWithCovarianceStamped::SharedPtr last_gnss_pose_;
  std::mutex gnss_pose_mutex_;

  rclcpp::Service<autoware::component_interface_specs::localization::Initialize::Service>::SharedPtr
    srv_initialize_;

  std::mutex meas_mutex_;
  std::shared_ptr<unified_localization_core::PoseWithCovariance> last_pose_;
  std::shared_ptr<unified_localization_core::TwistWithCovariance> last_twist_;
  std::optional<unified_localization_core::OdometryOutput> prev_odom_;

  bool auto_stop_check_enabled_{false};
  double auto_stop_check_duration_sec_{3.0};
  double auto_stop_velocity_threshold_{0.1};
  std::deque<std::pair<rclcpp::Time, double>> twist_history_;

  bool auto_ndt_align_enabled_{false};
  std::string auto_ndt_align_service_name_;
  double auto_ndt_align_timeout_sec_{10.0};
  rclcpp::Client<NdtAlignSrv>::SharedPtr cli_ndt_align_;
  bool auto_yabloc_align_enabled_{false};
  std::string auto_yabloc_align_service_name_;
  double auto_yabloc_align_timeout_sec_{10.0};
  rclcpp::Client<NdtAlignSrv>::SharedPtr cli_yabloc_align_;
  std::mutex align_mutex_;
  std::condition_variable align_cv_;
  bool align_done_{false};
  bool align_success_{false};
  PoseWithCovarianceStamped align_pose_;

  std::vector<std::string> trigger_service_names_;
  double trigger_timeout_sec_{5.0};
  std::vector<rclcpp::Client<SetBool>::SharedPtr> trigger_clients_;
  std::mutex trigger_mutex_;
  std::condition_variable trigger_cv_;
  bool trigger_done_{false};
  bool trigger_success_{false};

  bool initial_pose_covariance_override_enabled_{false};
  std::array<double, 36> initial_pose_covariance_{};
};

}  // namespace autoware::unified_localization

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<autoware::unified_localization::LocalizationNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
