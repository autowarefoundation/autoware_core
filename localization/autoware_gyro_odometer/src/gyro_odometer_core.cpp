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

#include <rclcpp/rclcpp.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <fmt/core.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

namespace autoware::gyro_odometer
{

std::array<double, 9> transform_covariance(const std::array<double, 9> & cov)
{
  using COV_IDX = autoware_utils_geometry::xyz_covariance_index::XYZ_COV_IDX;

  double max_cov = std::max({cov[COV_IDX::X_X], cov[COV_IDX::Y_Y], cov[COV_IDX::Z_Z]});

  std::array<double, 9> cov_transformed = {};
  cov_transformed.fill(0.);
  cov_transformed[COV_IDX::X_X] = max_cov;
  cov_transformed[COV_IDX::Y_Y] = max_cov;
  cov_transformed[COV_IDX::Z_Z] = max_cov;
  return cov_transformed;
}

GyroOdometerNode::GyroOdometerNode(const rclcpp::NodeOptions & node_options)
: Node("gyro_odometer", node_options),
  output_frame_(declare_parameter<std::string>("output_frame")),
  message_timeout_sec_(declare_parameter<double>("message_timeout_sec")),
  vehicle_twist_arrived_(false),
  imu_arrived_(false)
{
  transform_listener_ = std::make_shared<autoware_utils_tf::TransformListener>(this);
  logger_configure_ = std::make_unique<autoware_utils_logging::LoggerLevelConfigure>(this);

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

  diagnostics_ = std::make_unique<autoware_utils_diagnostics::DiagnosticsInterface>(
    this, "gyro_odometer_status");

  timer_ = rclcpp::create_timer(
    this, this->get_clock(), std::chrono::milliseconds(100),
    std::bind(&GyroOdometerNode::publish_diagnostics, this));
}

void GyroOdometerNode::callback_vehicle_twist(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr vehicle_twist_msg_ptr)
{
  vehicle_twist_arrived_ = true;
  latest_vehicle_twist_ros_time_ = vehicle_twist_msg_ptr->header.stamp;
  vehicle_twist_queue_.push_back(*vehicle_twist_msg_ptr);
  concat_gyro_and_odometer();
}

void GyroOdometerNode::callback_imu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr)
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
  latest_vehicle_twist_queue_size_ = vehicle_twist_queue_.size();
  latest_imu_queue_size_ = gyro_queue_.size();
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

  using COV_IDX_XYZ = autoware_utils_geometry::xyz_covariance_index::XYZ_COV_IDX;
  using COV_IDX_XYZRPY = autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;

  // calc mean, covariance
  double vx_mean = 0;
  geometry_msgs::msg::Vector3 gyro_mean{};
  double vx_covariance_original = 0;
  geometry_msgs::msg::Vector3 gyro_covariance_original{};
  for (const auto & vehicle_twist : vehicle_twist_queue_) {
    vx_mean += vehicle_twist.twist.twist.linear.x;
    vx_covariance_original += vehicle_twist.twist.covariance[0 * 6 + 0];
  }
  vx_mean /= static_cast<double>(vehicle_twist_queue_.size());
  vx_covariance_original /= static_cast<double>(vehicle_twist_queue_.size());

  for (const auto & gyro : gyro_queue_) {
    gyro_mean.x += gyro.angular_velocity.x;
    gyro_mean.y += gyro.angular_velocity.y;
    gyro_mean.z += gyro.angular_velocity.z;
    gyro_covariance_original.x += gyro.angular_velocity_covariance[COV_IDX_XYZ::X_X];
    gyro_covariance_original.y += gyro.angular_velocity_covariance[COV_IDX_XYZ::Y_Y];
    gyro_covariance_original.z += gyro.angular_velocity_covariance[COV_IDX_XYZ::Z_Z];
  }
  gyro_mean.x /= static_cast<double>(gyro_queue_.size());
  gyro_mean.y /= static_cast<double>(gyro_queue_.size());
  gyro_mean.z /= static_cast<double>(gyro_queue_.size());
  gyro_covariance_original.x /= static_cast<double>(gyro_queue_.size());
  gyro_covariance_original.y /= static_cast<double>(gyro_queue_.size());
  gyro_covariance_original.z /= static_cast<double>(gyro_queue_.size());

  // concat
  geometry_msgs::msg::TwistWithCovarianceStamped twist_with_cov;
  const auto latest_vehicle_twist_stamp = rclcpp::Time(vehicle_twist_queue_.back().header.stamp);
  const auto latest_imu_stamp = rclcpp::Time(gyro_queue_.back().header.stamp);
  if (latest_vehicle_twist_stamp < latest_imu_stamp) {
    twist_with_cov.header.stamp = latest_imu_stamp;
  } else {
    twist_with_cov.header.stamp = latest_vehicle_twist_stamp;
  }
  twist_with_cov.header.frame_id = gyro_queue_.front().header.frame_id;
  twist_with_cov.twist.twist.linear.x = vx_mean;
  twist_with_cov.twist.twist.angular = gyro_mean;

  // From a statistical point of view, here we reduce the covariances according to the number of
  // observed data
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::X_X] =
    vx_covariance_original / static_cast<double>(vehicle_twist_queue_.size());
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::Y_Y] = 100000.0;
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::Z_Z] = 100000.0;
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::ROLL_ROLL] =
    gyro_covariance_original.x / static_cast<double>(gyro_queue_.size());
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::PITCH_PITCH] =
    gyro_covariance_original.y / static_cast<double>(gyro_queue_.size());
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::YAW_YAW] =
    gyro_covariance_original.z / static_cast<double>(gyro_queue_.size());

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

  twist_raw_pub_->publish(twist_raw);
  twist_with_covariance_raw_pub_->publish(twist_with_cov_raw);

  geometry_msgs::msg::TwistWithCovarianceStamped twist_with_covariance = twist_with_cov_raw;
  geometry_msgs::msg::TwistStamped twist = twist_raw;

  // clear imu yaw bias if vehicle is stopped
  if (
    std::fabs(twist_with_cov_raw.twist.twist.angular.z) < 0.01 &&
    std::fabs(twist_with_cov_raw.twist.twist.linear.x) < 0.01) {
    twist.twist.angular.x = 0.0;
    twist.twist.angular.y = 0.0;
    twist.twist.angular.z = 0.0;
    twist_with_covariance.twist.twist.angular.x = 0.0;
    twist_with_covariance.twist.twist.angular.y = 0.0;
    twist_with_covariance.twist.twist.angular.z = 0.0;
  }

  twist_pub_->publish(twist);
  twist_with_covariance_pub_->publish(twist_with_covariance);
}

void GyroOdometerNode::publish_diagnostics()
{
  diagnostics_->clear();

  const auto vehicle_twist_time =
    vehicle_twist_arrived_ ? static_cast<rclcpp::Time>(latest_vehicle_twist_ros_time_).nanoseconds()
                           : std::nan("");
  const auto imu_time =
    imu_arrived_ ? static_cast<rclcpp::Time>(latest_imu_ros_time_).nanoseconds() : std::nan("");
  diagnostics_->add_key_value("latest_vehicle_twist_time_stamp", vehicle_twist_time);
  diagnostics_->add_key_value("latest_imu_time_stamp", imu_time);
  diagnostics_->add_key_value("is_arrived_first_vehicle_twist", vehicle_twist_arrived_);
  diagnostics_->add_key_value("is_arrived_first_imu", imu_arrived_);
  diagnostics_->add_key_value("vehicle_twist_time_stamp_dt", latest_vehicle_twist_dt_);
  diagnostics_->add_key_value("imu_time_stamp_dt", latest_imu_dt_);
  diagnostics_->add_key_value("vehicle_twist_queue_size", latest_vehicle_twist_queue_size_);
  diagnostics_->add_key_value("imu_queue_size", latest_imu_queue_size_);
  diagnostics_->add_key_value("is_succeed_transform_imu", is_succeed_transform_imu_);

  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string log_message = "";

  if (!vehicle_twist_arrived_) {
    const std::string message = "Twist msg has not been arrived yet.";
    diagnostics_->update_level_and_message(diagnostic_msgs::msg::DiagnosticStatus::WARN, message);
    log_message += message;
    log_message += "; ";
  }
  if (!imu_arrived_) {
    const std::string message = "IMU msg has not been arrived yet.";
    diagnostics_->update_level_and_message(diagnostic_msgs::msg::DiagnosticStatus::WARN, message);
    log_message += message;
    log_message += "; ";
  }
  if (latest_vehicle_twist_dt_ > message_timeout_sec_) {
    const std::string message = fmt::format(
      "Vehicle twist msg is timeout. vehicle_twist_dt: {}[sec], tolerance {}[sec]",
      latest_vehicle_twist_dt_, message_timeout_sec_);
    diagnostics_->update_level_and_message(diagnostic_msgs::msg::DiagnosticStatus::ERROR, message);
    log_message += message;
    log_message += "; ";
  }
  if (latest_imu_dt_ > message_timeout_sec_) {
    const std::string message = fmt::format(
      "IMU msg is timeout. imu_dt: {}[sec], tolerance {}[sec]", latest_imu_dt_,
      message_timeout_sec_);
    diagnostics_->update_level_and_message(diagnostic_msgs::msg::DiagnosticStatus::ERROR, message);
    log_message += message;
    log_message += "; ";
  }
  if (!is_succeed_transform_imu_) {
    const std::string message = "Please publish TF from " + output_frame_ + " to frame of IMU.";
    diagnostics_->update_level_and_message(diagnostic_msgs::msg::DiagnosticStatus::ERROR, message);
    log_message += message;
    log_message += "; ";
  }

  if (level == diagnostic_msgs::msg::DiagnosticStatus::WARN)
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, log_message);

  if (level == diagnostic_msgs::msg::DiagnosticStatus::ERROR)
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, log_message);

  diagnostics_->publish(this->now());
}

}  // namespace autoware::gyro_odometer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::gyro_odometer::GyroOdometerNode)
