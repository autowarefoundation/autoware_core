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

#ifndef VEHICLE_VELOCITY_CONVERTER_NODE_HPP_
#define VEHICLE_VELOCITY_CONVERTER_NODE_HPP_

#include "autoware/vehicle_velocity_converter/vehicle_velocity_converter.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <memory>
#include <string>

namespace autoware::vehicle_velocity_converter
{

class VehicleVelocityConverterNode : public rclcpp::Node
{
public:
  explicit VehicleVelocityConverterNode(const rclcpp::NodeOptions & options);

private:
  void callback_velocity_report(const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg);

  rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr vehicle_report_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    twist_with_covariance_pub_;

  std::unique_ptr<VehicleVelocityConverter> converter_;
};

}  // namespace autoware::vehicle_velocity_converter

#endif  // VEHICLE_VELOCITY_CONVERTER_NODE_HPP_
