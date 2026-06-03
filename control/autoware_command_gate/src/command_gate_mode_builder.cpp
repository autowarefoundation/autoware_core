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

#include <autoware_common_msgs/msg/response_status.hpp>

#include <optional>
#include <string>

namespace autoware::control::command_gate
{

std::optional<ModeOutputs> CommandGateModeBuilder::create_mode_output(
  const Request & request, const builtin_interfaces::msg::Time & stamp)
{
  return dispatch_mode(request.mode, stamp);
}

CommandGateModeBuilder::Response CommandGateModeBuilder::create_response(
  const Request & request, const builtin_interfaces::msg::Time & /*stamp*/)
{
  Response response;
  const auto message = success_message(request.mode);
  if (!message) {
    response.status.success = false;
    response.status.code = autoware_common_msgs::msg::ResponseStatus::PARAMETER_ERROR;
    response.status.message = "Unknown operation mode requested.";
    return response;
  }

  response.status.success = true;
  response.status.code = 0;  // 0 represents success (no specific success code defined)
  response.status.message = *message;
  return response;
}

std::optional<std::string> CommandGateModeBuilder::success_message(uint16_t mode)
{
  switch (mode) {
    case Request::STOP:
      return "Switched to STOP";
    case Request::AUTONOMOUS:
      return "Switched to AUTONOMOUS";
    case Request::LOCAL:
      return "Switched to LOCAL";
    case Request::REMOTE:
      return "Switched to REMOTE";
    default:
      return std::nullopt;
  }
}

std::optional<ModeOutputs> CommandGateModeBuilder::dispatch_mode(
  uint16_t mode, const builtin_interfaces::msg::Time & stamp)
{
  switch (mode) {
    case Request::STOP:
      return make_stop(stamp);
    case Request::AUTONOMOUS:
      return make_autonomous(stamp);
    case Request::LOCAL:
      return make_local(stamp);
    case Request::REMOTE:
      return make_remote(stamp);
    default:
      return std::nullopt;
  }
}

ModeOutputs CommandGateModeBuilder::make_stop(const builtin_interfaces::msg::Time & stamp)
{
  ModeOutputs outputs;
  fill_state(outputs.state, autoware_adapi_v1_msgs::msg::OperationModeState::STOP, stamp);
  fill_gear(outputs.gear, autoware_vehicle_msgs::msg::GearCommand::PARK, stamp);
  return outputs;
}

ModeOutputs CommandGateModeBuilder::make_autonomous(const builtin_interfaces::msg::Time & stamp)
{
  ModeOutputs outputs;
  fill_state(outputs.state, autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS, stamp);
  fill_gear(outputs.gear, autoware_vehicle_msgs::msg::GearCommand::DRIVE, stamp);
  return outputs;
}

ModeOutputs CommandGateModeBuilder::make_local(const builtin_interfaces::msg::Time & stamp)
{
  ModeOutputs outputs;
  fill_state(outputs.state, autoware_adapi_v1_msgs::msg::OperationModeState::LOCAL, stamp);
  fill_gear(outputs.gear, autoware_vehicle_msgs::msg::GearCommand::NONE, stamp);
  return outputs;
}

ModeOutputs CommandGateModeBuilder::make_remote(const builtin_interfaces::msg::Time & stamp)
{
  ModeOutputs outputs;
  fill_state(outputs.state, autoware_adapi_v1_msgs::msg::OperationModeState::REMOTE, stamp);
  fill_gear(outputs.gear, autoware_vehicle_msgs::msg::GearCommand::NONE, stamp);
  return outputs;
}

void CommandGateModeBuilder::fill_state(
  autoware_adapi_v1_msgs::msg::OperationModeState & msg, uint8_t mode,
  const builtin_interfaces::msg::Time & stamp)
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
  const builtin_interfaces::msg::Time & stamp)
{
  msg.stamp = stamp;
  msg.command = command;
}

}  // namespace autoware::control::command_gate
