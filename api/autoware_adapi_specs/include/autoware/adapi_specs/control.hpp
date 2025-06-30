// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__ADAPI_SPECS__CONTROL_HPP_
#define AUTOWARE__ADAPI_SPECS__CONTROL_HPP_

#include <rclcpp/qos.hpp>

#include <autoware_adapi_v1_msgs/msg/acceleration_command.hpp>
#include <autoware_adapi_v1_msgs/msg/pedals_command.hpp>
#include <autoware_adapi_v1_msgs/msg/steering_command.hpp>
#include <autoware_adapi_v1_msgs/msg/velocity_command.hpp>

namespace autoware::adapi_specs::control
{

struct PedalsCommand
{
  using Message = autoware_adapi_v1_msgs::msg::PedalsCommand;
  static constexpr char name[] = "/api/control/command/pedals";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

struct AccelerationCommand
{
  using Message = autoware_adapi_v1_msgs::msg::AccelerationCommand;
  static constexpr char name[] = "/api/control/command/acceleration";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

struct VelocityCommand
{
  using Message = autoware_adapi_v1_msgs::msg::VelocityCommand;
  static constexpr char name[] = "/api/control/command/velocity";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

struct SteeringCommand
{
  using Message = autoware_adapi_v1_msgs::msg::SteeringCommand;
  static constexpr char name[] = "/api/control/command/steering";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

}  // namespace autoware::adapi_specs::control

#endif  // AUTOWARE__ADAPI_SPECS__CONTROL_HPP_
