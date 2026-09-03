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

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

namespace autoware::gyro_odometer
{

std::optional<sensor_msgs::msg::Imu> transform_imu(
  const sensor_msgs::msg::Imu & imu_msg, TransformListener & transform_listener,
  const std::string & output_frame)
{
  geometry_msgs::msg::TransformStamped::ConstSharedPtr tf_imu2base_ptr =
    transform_listener.get_latest_transform(imu_msg.header.frame_id, output_frame);
  if (tf_imu2base_ptr == nullptr) {
    return std::nullopt;
  }

  geometry_msgs::msg::Vector3Stamped angular_velocity;
  angular_velocity.header = imu_msg.header;
  angular_velocity.vector = imu_msg.angular_velocity;

  geometry_msgs::msg::Vector3Stamped transformed_angular_velocity;
  transformed_angular_velocity.header = tf_imu2base_ptr->header;
  tf2::doTransform(angular_velocity, transformed_angular_velocity, *tf_imu2base_ptr);

  sensor_msgs::msg::Imu transformed_imu_msg = imu_msg;
  transformed_imu_msg.header.frame_id = output_frame;
  transformed_imu_msg.angular_velocity = transformed_angular_velocity.vector;
  transformed_imu_msg.angular_velocity_covariance =
    transform_covariance(imu_msg.angular_velocity_covariance);
  return transformed_imu_msg;
}

GyroOdometerNode::GyroOdometerNode(const rclcpp::NodeOptions & node_options)
: autoware::agnocast_wrapper::Node("gyro_odometer", node_options),
  transform_listener_(std::make_shared<TransformListener>(this)),
  output_frame_(declare_parameter<std::string>("output_frame")),
  message_timeout_sec_(declare_parameter<double>("message_timeout_sec")),
  gyro_odometer_(message_timeout_sec_)
{
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

void GyroOdometerNode::callback_vehicle_twist(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(geometry_msgs::msg::TwistWithCovarianceStamped)
    vehicle_twist_msg_ptr)
{
  const auto output = gyro_odometer_.input_vehicle_twist(*vehicle_twist_msg_ptr);

  if (output) {
    publish_data(*output);
  }
}

void GyroOdometerNode::callback_imu(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(sensor_msgs::msg::Imu) imu_msg_ptr)
{
  const std::optional<sensor_msgs::msg::Imu> transformed_imu_msg =
    transform_imu(*imu_msg_ptr, *transform_listener_, output_frame_);
  is_succeed_transform_imu_ = transformed_imu_msg.has_value();

  // A sample that cannot be brought into the output frame must never take part in a fusion.
  if (!transformed_imu_msg) {
    return;
  }

  const auto output = gyro_odometer_.input_imu(*transformed_imu_msg);

  if (output) {
    publish_data(*output);
  }
}

void GyroOdometerNode::publish_data(const GyroOdometer::OutputData & output_data)
{
  const auto & [twist_raw, twist_with_covariance_raw, twist, twist_with_covariance] = output_data;

  twist_raw_pub_->publish(twist_raw);
  twist_with_covariance_raw_pub_->publish(twist_with_covariance_raw);

  twist_pub_->publish(twist);
  twist_with_covariance_pub_->publish(twist_with_covariance);
}

void GyroOdometerNode::publish_diagnostics()
{
  const GyroOdometer::Status status = gyro_odometer_.take_status();

  diagnostics_->clear();

  const auto vehicle_twist_time =
    status.vehicle_twist_arrived
      ? static_cast<double>(status.latest_vehicle_twist_ros_time.nanoseconds())
      : std::nan("");
  const auto imu_time = status.imu_arrived
                          ? static_cast<double>(status.latest_imu_ros_time.nanoseconds())
                          : std::nan("");
  diagnostics_->add_key_value("latest_vehicle_twist_time_stamp", vehicle_twist_time);
  diagnostics_->add_key_value("latest_imu_time_stamp", imu_time);
  diagnostics_->add_key_value("is_arrived_first_vehicle_twist", status.vehicle_twist_arrived);
  diagnostics_->add_key_value("is_arrived_first_imu", status.imu_arrived);
  diagnostics_->add_key_value("vehicle_twist_time_stamp_dt", status.latest_vehicle_twist_dt);
  diagnostics_->add_key_value("imu_time_stamp_dt", status.latest_imu_dt);
  diagnostics_->add_key_value("vehicle_twist_queue_size", status.vehicle_twist_queue_size);
  diagnostics_->add_key_value("imu_queue_size", status.imu_queue_size);
  diagnostics_->add_key_value("is_succeed_transform_imu", is_succeed_transform_imu_);

  DiagnosticsState state;
  state.vehicle_twist_arrived = status.vehicle_twist_arrived;
  state.imu_arrived = status.imu_arrived;
  state.is_succeed_transform_imu = is_succeed_transform_imu_;
  state.latest_vehicle_twist_dt = status.latest_vehicle_twist_dt;
  state.latest_imu_dt = status.latest_imu_dt;
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
}

}  // namespace autoware::gyro_odometer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::gyro_odometer::GyroOdometerNode)
