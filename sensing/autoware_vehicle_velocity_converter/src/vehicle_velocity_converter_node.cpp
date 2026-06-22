// Copyright 2026 TIER IV
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

#include "vehicle_velocity_converter_node.hpp"

#include <string>

namespace autoware::vehicle_velocity_converter
{
VehicleVelocityConverterNode::VehicleVelocityConverterNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("vehicle_velocity_converter", options),
  frame_id_(declare_parameter<std::string>("frame_id")),
  converter_(
    declare_parameter<double>("speed_scale_factor"),
    declare_parameter<double>("velocity_stddev_xx"),
    declare_parameter<double>("angular_velocity_stddev_zz"))
{
  vehicle_report_sub_ = create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
    "velocity_status", rclcpp::QoS{10},
    std::bind(
      &VehicleVelocityConverterNode::callback_velocity_report, this, std::placeholders::_1));

  twist_with_covariance_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "twist_with_covariance", rclcpp::QoS{10});
}

void VehicleVelocityConverterNode::callback_velocity_report(
  const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg)
{
  if (msg->header.frame_id != frame_id_) {
    RCLCPP_WARN(get_logger(), "frame_id is not base_link.");
  }

  // set twist with covariance msg from vehicle report msg
  twist_with_covariance_pub_->publish(converter_.convert(*msg));
}
}  // namespace autoware::vehicle_velocity_converter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::vehicle_velocity_converter::VehicleVelocityConverterNode)
