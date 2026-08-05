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

#include "gyro_odometer_node.hpp"

#include "gyro_odometer.hpp"
#include "gyro_odometer_diagnostics.hpp"

#include <rclcpp/rclcpp.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

namespace autoware::gyro_odometer
{

GyroOdometerNode::GyroOdometerNode(const rclcpp::NodeOptions & node_options)
: autoware::agnocast_wrapper::Node("gyro_odometer", node_options),
  output_frame_(declare_parameter<std::string>("output_frame")),
  cached_imu_frame_id_(std::nullopt),
  message_timeout_sec_(declare_parameter<double>("message_timeout_sec"))
{
  transform_listener_ = std::make_shared<TransformListener>(this);
  logger_configure_ = std::make_unique<
    autoware_utils_logging::BasicLoggerLevelConfigure<autoware::agnocast_wrapper::Node>>(this);

  vehicle_twist_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "vehicle/twist_with_covariance", rclcpp::QoS{10},
    std::bind(&GyroOdometerNode::callback_vehicle_twist, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu", rclcpp::QoS{10},
    std::bind(&GyroOdometerNode::callback_imu, this, std::placeholders::_1));

  twist_raw_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("twist_raw", rclcpp::QoS{10});
  twist_with_covariance_raw_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "twist_with_covariance_raw", rclcpp::QoS{10});

  twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("twist", rclcpp::QoS{10});
  twist_with_covariance_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "twist_with_covariance", rclcpp::QoS{10});

  diagnostics_ = std::make_unique<
    autoware_utils_diagnostics::BasicDiagnosticsInterface<autoware::agnocast_wrapper::Node>>(
    this, "gyro_odometer_status");

  timer_ = autoware::agnocast_wrapper::create_timer(
    this, this->get_clock(), std::chrono::milliseconds(100),
    std::bind(&GyroOdometerNode::publish_diagnostics, this));
}

void GyroOdometerNode::update_cached_transform()
{
  if (cached_transform_) {
    return;
  }
  if (!cached_imu_frame_id_) {
    return;
  }

  // get transformation
  geometry_msgs::msg::TransformStamped::ConstSharedPtr tf_imu2base_ptr =
    transform_listener_->get_latest_transform(*cached_imu_frame_id_, output_frame_);

  if (tf_imu2base_ptr == nullptr) {  // TODO(kazkomiya): remove this (possibly dead) path after
                                     // confirming get_latest_transform never fails
    return;
  }
  cached_transform_ = *tf_imu2base_ptr;
}

void GyroOdometerNode::callback_vehicle_twist(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(geometry_msgs::msg::TwistWithCovarianceStamped)
    vehicle_twist_msg_ptr)
{
  update_cached_transform();
  auto twist_with_cov = internal_gyro_odometer_.callback_vehicle_twist_internal(
    *vehicle_twist_msg_ptr, this->now(), message_timeout_sec_, cached_transform_, output_frame_);

  if (twist_with_cov) {
    auto output = internal_gyro_odometer_.publish_data_internal(*twist_with_cov);
    publish_data(output);
  }
}

void GyroOdometerNode::callback_imu(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(sensor_msgs::msg::Imu) imu_msg_ptr)
{
  if (!cached_imu_frame_id_) {  // TODO(kazkomiya): duplicated check with imu_arrived_
    cached_imu_frame_id_ = std::make_optional<std::string>(imu_msg_ptr->header.frame_id);
  }
  update_cached_transform();

  auto twist_with_cov = internal_gyro_odometer_.callback_imu_internal(
    *imu_msg_ptr, this->now(), message_timeout_sec_, cached_transform_, output_frame_);

  if (twist_with_cov) {
    auto output = internal_gyro_odometer_.publish_data_internal(*twist_with_cov);
    publish_data(output);
  }
}

void GyroOdometerNode::publish_data(const GyroOdometer::OutputData & output_data)
{
  auto [twist_raw, twist_with_covariance_raw, twist, twist_with_covariance] = output_data;
  twist_raw_pub_->publish(twist_raw);
  twist_with_covariance_raw_pub_->publish(twist_with_covariance_raw);
  twist_pub_->publish(twist);
  twist_with_covariance_pub_->publish(twist_with_covariance);
}

void GyroOdometerNode::publish_diagnostics()
{
#if 0   // FIXME(kazkomiya)
  diagnostics_->clear();

  const auto vehicle_twist_time =
    vehicle_twist_arrived_
      ? static_cast<double>(static_cast<rclcpp::Time>(latest_vehicle_twist_ros_time_).nanoseconds())
      : std::nan("");
  const auto imu_time =
    imu_arrived_
      ? static_cast<double>(static_cast<rclcpp::Time>(latest_imu_ros_time_).nanoseconds())
      : std::nan("");
  diagnostics_->add_key_value("latest_vehicle_twist_time_stamp", vehicle_twist_time);
  diagnostics_->add_key_value("latest_imu_time_stamp", imu_time);
  diagnostics_->add_key_value("is_arrived_first_vehicle_twist", vehicle_twist_arrived_);
  diagnostics_->add_key_value("is_arrived_first_imu", imu_arrived_);
  diagnostics_->add_key_value("vehicle_twist_time_stamp_dt", latest_vehicle_twist_dt_);
  diagnostics_->add_key_value("imu_time_stamp_dt", latest_imu_dt_);
  diagnostics_->add_key_value("vehicle_twist_queue_size", latest_vehicle_twist_queue_size_);
  diagnostics_->add_key_value("imu_queue_size", latest_imu_queue_size_);
  diagnostics_->add_key_value("is_succeed_transform_imu", is_succeed_transform_imu_);

  DiagnosticsState state;
  state.vehicle_twist_arrived = vehicle_twist_arrived_;
  state.imu_arrived = imu_arrived_;
  state.is_succeed_transform_imu = is_succeed_transform_imu_;
  state.latest_vehicle_twist_dt = latest_vehicle_twist_dt_;
  state.latest_imu_dt = latest_imu_dt_;
  state.message_timeout_sec = message_timeout_sec_;
  state.output_frame = output_frame_;

  const DiagnosticsResult diagnostics_result = determine_diagnostics(state);

  for (const auto & entry : diagnostics_result.entries) {
    diagnostics_->update_level_and_message(entry.level, entry.message);
  }

  if (diagnostics_result.level == diagnostic_msgs::msg::DiagnosticStatus::WARN)
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, diagnostics_result.log_message);

  if (diagnostics_result.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR)
    RCLCPP_ERROR_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, diagnostics_result.log_message);

  diagnostics_->publish(this->now());
#endif  // FIXME(kazkomiya)
}

}  // namespace autoware::gyro_odometer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::gyro_odometer::GyroOdometerNode)
