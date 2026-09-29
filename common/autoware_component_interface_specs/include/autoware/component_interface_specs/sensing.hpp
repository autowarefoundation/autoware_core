// Copyright 2026 The Autoware Contributors
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

#ifndef AUTOWARE__COMPONENT_INTERFACE_SPECS__SENSING_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_SPECS__SENSING_HPP_

#include <autoware/component_interface_specs/utils.hpp>
#include <autoware/component_interface_specs/version.hpp>
#include <rclcpp/qos.hpp>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

namespace autoware::component_interface_specs::sensing
{

struct VehicleVelocityConverterTwist
{
  using Message = geometry_msgs::msg::TwistWithCovarianceStamped;
  static constexpr char name[] = "/sensing/vehicle_velocity_converter/twist_with_covariance";
  static constexpr size_t depth = 10;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

AUTOWARE_COMPONENT_INTERFACE_SPECS_DEFINE_DOMAIN(0, 1, 0, VehicleVelocityConverterTwist)

}  // namespace autoware::component_interface_specs::sensing

#endif  // AUTOWARE__COMPONENT_INTERFACE_SPECS__SENSING_HPP_
