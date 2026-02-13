// Copyright 2021 TierIV
// Copyright 2025 The Autoware Contributors
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

namespace autoware::vehicle_velocity_converter
{

VehicleVelocityConverterNode::VehicleVelocityConverterNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("vehicle_velocity_converter", options)
{
  std::string frame_id = declare_parameter<std::string>("frame_id");
  double velocity_stddev_xx = declare_parameter<double>("velocity_stddev_xx");
  double angular_velocity_stddev_zz = declare_parameter<double>("angular_velocity_stddev_zz");
  double speed_scale_factor = declare_parameter<double>("speed_scale_factor");

  // Initialize the converter library
  converter_ = std::make_unique<VehicleVelocityConverter>(
    frame_id, velocity_stddev_xx, angular_velocity_stddev_zz, speed_scale_factor);

  // Create subscription and publisher
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
  // Convert using the library
  const auto result = converter_->convert(*msg);

  // Log warning if frame ID doesn't match
  if (!result.frame_id_matched) {
    RCLCPP_WARN(get_logger(), "frame_id is not base_link.");
  }

  // Publish the converted message
  twist_with_covariance_pub_->publish(result.twist_with_covariance);
}

}  // namespace autoware::vehicle_velocity_converter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::vehicle_velocity_converter::VehicleVelocityConverterNode)
