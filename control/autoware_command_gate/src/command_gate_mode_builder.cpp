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

#include "command_gate_mode_builder.hpp"

#include <string>

namespace autoware::control::command_gate
{

ModeOutputs CommandGateModeBuilder::make_stop(const builtin_interfaces::msg::Time & stamp) const
{
  ModeOutputs outputs;
  fill_state(outputs.state, autoware_adapi_v1_msgs::msg::OperationModeState::STOP, stamp);
  fill_gear(outputs.gear, autoware_vehicle_msgs::msg::GearCommand::PARK, stamp);
  outputs.status = make_status("Switched to STOP");
  return outputs;
}

ModeOutputs CommandGateModeBuilder::make_autonomous(
  const builtin_interfaces::msg::Time & stamp) const
{
  ModeOutputs outputs;
  fill_state(outputs.state, autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS, stamp);
  fill_gear(outputs.gear, autoware_vehicle_msgs::msg::GearCommand::DRIVE, stamp);
  outputs.status = make_status("Switched to AUTONOMOUS");
  return outputs;
}

void CommandGateModeBuilder::fill_state(
  autoware_adapi_v1_msgs::msg::OperationModeState & msg, uint8_t mode,
  const builtin_interfaces::msg::Time & stamp) const
{
  msg.stamp = stamp;
  msg.mode = mode;
  msg.is_autoware_control_enabled =
    (mode == autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS);
  msg.is_in_transition = false;
  msg.is_stop_mode_available = true;
  msg.is_autonomous_mode_available = true;
  msg.is_local_mode_available = true;
  msg.is_remote_mode_available = true;
}

void CommandGateModeBuilder::fill_gear(
  autoware_vehicle_msgs::msg::GearCommand & msg, uint8_t command,
  const builtin_interfaces::msg::Time & stamp) const
{
  msg.stamp = stamp;
  msg.command = command;
}

autoware_adapi_v1_msgs::msg::ResponseStatus CommandGateModeBuilder::make_status(
  const std::string & message) const
{
  autoware_adapi_v1_msgs::msg::ResponseStatus status;
  status.success = true;
  status.code = 0;  // 0 represents success (no specific success code defined)
  status.message = message;
  return status;
}

}  // namespace autoware::control::command_gate
