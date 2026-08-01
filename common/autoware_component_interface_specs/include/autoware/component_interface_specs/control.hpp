// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__COMPONENT_INTERFACE_SPECS__CONTROL_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_SPECS__CONTROL_HPP_

#include <autoware/component_interface_specs/utils.hpp>
#include <autoware/component_interface_specs/version.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_vehicle_msgs/srv/control_mode_command.hpp>

#include <rmw/qos_profiles.h>

namespace autoware::component_interface_specs::control
{

struct ControlCommand
{
  using Message = autoware_control_msgs::msg::Control;
  static constexpr char name[] = "/control/command/control_cmd";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

struct GearCommand
{
  using Message = autoware_vehicle_msgs::msg::GearCommand;
  static constexpr char name[] = "/control/command/gear_cmd";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

struct TurnIndicatorsCommand
{
  using Message = autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
  static constexpr char name[] = "/control/command/turn_indicators_cmd";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

struct HazardLightsCommand
{
  using Message = autoware_vehicle_msgs::msg::HazardLightsCommand;
  static constexpr char name[] = "/control/command/hazard_lights_cmd";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

struct PredictedTrajectory  // new - control->planning predicted-path feedback
{
  using Message = autoware_planning_msgs::msg::Trajectory;
  static constexpr char name[] = "/control/trajectory_follower/lateral/predicted_trajectory";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

struct ControlModeRequest  // new - control/system->vehicle control-mode request
{
  using Service = autoware_vehicle_msgs::srv::ControlModeCommand;
  static constexpr char name[] = "/control/control_mode_request";
};

AUTOWARE_COMPONENT_INTERFACE_SPECS_DEFINE_DOMAIN(
  0, 1, 0, ControlCommand, GearCommand, TurnIndicatorsCommand, HazardLightsCommand,
  PredictedTrajectory, ControlModeRequest)

}  // namespace autoware::component_interface_specs::control

#endif  // AUTOWARE__COMPONENT_INTERFACE_SPECS__CONTROL_HPP_
