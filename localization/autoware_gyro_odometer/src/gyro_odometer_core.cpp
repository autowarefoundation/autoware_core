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

#include "gyro_odometer_core.hpp"

#include "gyro_odometer_fusion.hpp"

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
  message_timeout_sec_(declare_parameter<double>("message_timeout_sec")),
  vehicle_twist_arrived_(false),
  imu_arrived_(false)
{
  transform_listener_ = std::make_shared<
    autoware_utils_tf::TransformListenerImpl<autoware::agnocast_wrapper::Node>>(this);
  logger_configure_ = std::make_unique<
    autoware_utils_logging::LoggerLevelConfigureImpl<autoware::agnocast_wrapper::Node>>(this);

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
  vehicle_twist_arrived_ = true;
  latest_vehicle_twist_ros_time_ = vehicle_twist_msg_ptr->header.stamp;
  vehicle_twist_queue_.push_back(*vehicle_twist_msg_ptr);
  concat_gyro_and_odometer();
}

void GyroOdometerNode::callback_imu(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(sensor_msgs::msg::Imu) imu_msg_ptr)
{
  imu_arrived_ = true;
  latest_imu_ros_time_ = imu_msg_ptr->header.stamp;
  gyro_queue_.push_back(*imu_msg_ptr);
  concat_gyro_and_odometer();
}

void GyroOdometerNode::concat_gyro_and_odometer()
{
  // check arrive first topic
  if (!vehicle_twist_arrived_) {
    vehicle_twist_queue_.clear();
    gyro_queue_.clear();
    return;
  }
  if (!imu_arrived_) {
    vehicle_twist_queue_.clear();
    gyro_queue_.clear();
    return;
  }

  // check timeout
  latest_vehicle_twist_dt_ = std::abs((this->now() - latest_vehicle_twist_ros_time_).seconds());
  latest_imu_dt_ = std::abs((this->now() - latest_imu_ros_time_).seconds());
  if (latest_vehicle_twist_dt_ > message_timeout_sec_) {
    vehicle_twist_queue_.clear();
    gyro_queue_.clear();
    return;
  }
  if (latest_imu_dt_ > message_timeout_sec_) {
    vehicle_twist_queue_.clear();
    gyro_queue_.clear();
    return;
  }

  // check queue size
  latest_vehicle_twist_queue_size_ = static_cast<int32_t>(vehicle_twist_queue_.size());
  latest_imu_queue_size_ = static_cast<int32_t>(gyro_queue_.size());
  if (vehicle_twist_queue_.empty()) {
    // not output error and clear queue
    return;
  }
  if (gyro_queue_.empty()) {
    // not output error and clear queue
    return;
  }

  // get transformation
  geometry_msgs::msg::TransformStamped::ConstSharedPtr tf_imu2base_ptr =
    transform_listener_->get_latest_transform(gyro_queue_.front().header.frame_id, output_frame_);
  is_succeed_transform_imu_ = (tf_imu2base_ptr != nullptr);
  if (!is_succeed_transform_imu_) {
    vehicle_twist_queue_.clear();
    gyro_queue_.clear();
    return;
  }

  // transform gyro frame
  for (auto & gyro : gyro_queue_) {
    geometry_msgs::msg::Vector3Stamped angular_velocity;
    angular_velocity.header = gyro.header;
    angular_velocity.vector = gyro.angular_velocity;

    geometry_msgs::msg::Vector3Stamped transformed_angular_velocity;
    transformed_angular_velocity.header = tf_imu2base_ptr->header;
    tf2::doTransform(angular_velocity, transformed_angular_velocity, *tf_imu2base_ptr);

    gyro.header.frame_id = output_frame_;
    gyro.angular_velocity = transformed_angular_velocity.vector;
    gyro.angular_velocity_covariance = transform_covariance(gyro.angular_velocity_covariance);
  }

  // fuse the vehicle twist and the (already transformed) gyro queue
  const geometry_msgs::msg::TwistWithCovarianceStamped twist_with_cov =
    fuse_twist(vehicle_twist_queue_, gyro_queue_);

  publish_data(twist_with_cov);

  vehicle_twist_queue_.clear();
  gyro_queue_.clear();
}

void GyroOdometerNode::publish_data(
  const geometry_msgs::msg::TwistWithCovarianceStamped & twist_with_cov_raw)
{
  geometry_msgs::msg::TwistStamped twist_raw;
  twist_raw.header = twist_with_cov_raw.header;
  twist_raw.twist = twist_with_cov_raw.twist.twist;

  {
    auto msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(twist_raw_pub_);
    *msg = twist_raw;
    twist_raw_pub_->publish(std::move(msg));
  }
  {
    auto msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(twist_with_covariance_raw_pub_);
    *msg = twist_with_cov_raw;
    twist_with_covariance_raw_pub_->publish(std::move(msg));
  }

  // clear imu yaw bias if vehicle is stopped
  const geometry_msgs::msg::TwistWithCovarianceStamped twist_with_covariance =
    apply_stop_compensation(twist_with_cov_raw);

  geometry_msgs::msg::TwistStamped twist;
  twist.header = twist_with_covariance.header;
  twist.twist = twist_with_covariance.twist.twist;

  {
    auto msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(twist_pub_);
    *msg = twist;
    twist_pub_->publish(std::move(msg));
  }
  {
    auto msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(twist_with_covariance_pub_);
    *msg = twist_with_covariance;
    twist_with_covariance_pub_->publish(std::move(msg));
  }
}

void GyroOdometerNode::publish_diagnostics()
{
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
}

}  // namespace autoware::gyro_odometer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::gyro_odometer::GyroOdometerNode)
