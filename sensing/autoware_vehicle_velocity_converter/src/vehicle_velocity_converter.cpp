// Copyright 2021 TierIV
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

#include "vehicle_velocity_converter.hpp"

#include <autoware_utils_geometry/msg/covariance.hpp>

#include <string>

namespace autoware::vehicle_velocity_converter
{
geometry_msgs::msg::TwistWithCovarianceStamped convert(
  const autoware_vehicle_msgs::msg::VelocityReport & msg, const double speed_scale_factor,
  const double stddev_vx, const double stddev_wz)
{
  using COV_IDX = autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;

  geometry_msgs::msg::TwistWithCovarianceStamped twist_with_covariance_msg;
  twist_with_covariance_msg.header = msg.header;
  twist_with_covariance_msg.twist.twist.linear.x = msg.longitudinal_velocity * speed_scale_factor;
  twist_with_covariance_msg.twist.twist.linear.y = msg.lateral_velocity;
  twist_with_covariance_msg.twist.twist.angular.z = msg.heading_rate;
  twist_with_covariance_msg.twist.covariance[COV_IDX::X_X] = stddev_vx * stddev_vx;
  twist_with_covariance_msg.twist.covariance[COV_IDX::Y_Y] = 10000.0;
  twist_with_covariance_msg.twist.covariance[COV_IDX::Z_Z] = 10000.0;
  twist_with_covariance_msg.twist.covariance[COV_IDX::ROLL_ROLL] = 10000.0;
  twist_with_covariance_msg.twist.covariance[COV_IDX::PITCH_PITCH] = 10000.0;
  twist_with_covariance_msg.twist.covariance[COV_IDX::YAW_YAW] = stddev_wz * stddev_wz;

  return twist_with_covariance_msg;
}

VehicleVelocityConverter::VehicleVelocityConverter(const rclcpp::NodeOptions & options)
: rclcpp::Node("vehicle_velocity_converter", options),
  frame_id_(declare_parameter<std::string>("frame_id")),
  stddev_vx_(declare_parameter<double>("velocity_stddev_xx")),
  stddev_wz_(declare_parameter<double>("angular_velocity_stddev_zz")),
  speed_scale_factor_(declare_parameter<double>("speed_scale_factor"))
{
  vehicle_report_sub_ = create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
    "velocity_status", rclcpp::QoS{10},
    std::bind(&VehicleVelocityConverter::callback_velocity_report, this, std::placeholders::_1));

  twist_with_covariance_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "twist_with_covariance", rclcpp::QoS{10});
}

void VehicleVelocityConverter::callback_velocity_report(
  const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg)
{
  if (msg->header.frame_id != frame_id_) {
    RCLCPP_WARN(get_logger(), "frame_id is not base_link.");
  }

  // set twist with covariance msg from vehicle report msg
  twist_with_covariance_pub_->publish(convert(*msg, speed_scale_factor_, stddev_vx_, stddev_wz_));
}
}  // namespace autoware::vehicle_velocity_converter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::vehicle_velocity_converter::VehicleVelocityConverter)
