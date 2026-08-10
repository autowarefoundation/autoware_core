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

#ifndef COMMAND_GATE_MODE_BUILDER_HPP_
#define COMMAND_GATE_MODE_BUILDER_HPP_

#include <builtin_interfaces/msg/time.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_system_msgs/srv/change_operation_mode.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>

#include <cstdint>
#include <optional>
#include <string>

namespace autoware::control::command_gate
{

struct ModeOutputs
{
  autoware_adapi_v1_msgs::msg::OperationModeState state;
  autoware_vehicle_msgs::msg::GearCommand gear;
};

/// Pure (ROS-node-free) builders that convert a ChangeOperationMode request message into the
/// messages the command gate emits. Every entry point is a plain message-in -> message-out
/// function so it can be unit-tested without spinning up a ROS node, publisher or service.
class CommandGateModeBuilder
{
public:
  using Request = autoware_system_msgs::srv::ChangeOperationMode::Request;
  using Response = autoware_system_msgs::srv::ChangeOperationMode::Response;

  /// Build the messages to publish for the requested operation mode.
  /// Returns std::nullopt for unknown/unsupported modes so the caller publishes nothing.
  /// The accepted values mirror the constants of
  /// autoware_system_msgs::srv::ChangeOperationMode::Request (STOP/AUTONOMOUS/LOCAL/REMOTE).
  static std::optional<ModeOutputs> create_mode_output(
    const Request & request, const builtin_interfaces::msg::Time & stamp);

  /// Build the service response for the requested operation mode.
  /// Reports success for known modes and a PARAMETER_ERROR for unknown ones, mirroring the
  /// publish decision made by create_mode_output().
  static Response create_response(
    const Request & request, const builtin_interfaces::msg::Time & stamp);

  static ModeOutputs make_stop(const builtin_interfaces::msg::Time & stamp);
  static ModeOutputs make_autonomous(const builtin_interfaces::msg::Time & stamp);
  static ModeOutputs make_local(const builtin_interfaces::msg::Time & stamp);
  static ModeOutputs make_remote(const builtin_interfaces::msg::Time & stamp);

private:
  /// Map an operation-mode request value to its outputs, or std::nullopt for unknown modes.
  static std::optional<ModeOutputs> dispatch_mode(
    uint16_t mode, const builtin_interfaces::msg::Time & stamp);

  /// Human-readable success message for a known mode, or std::nullopt for unknown modes.
  static std::optional<std::string> success_message(uint16_t mode);

  static void fill_state(
    autoware_adapi_v1_msgs::msg::OperationModeState & msg, uint8_t mode,
    const builtin_interfaces::msg::Time & stamp);
  static void fill_gear(
    autoware_vehicle_msgs::msg::GearCommand & msg, uint8_t command,
    const builtin_interfaces::msg::Time & stamp);
};

}  // namespace autoware::control::command_gate

#endif  // COMMAND_GATE_MODE_BUILDER_HPP_
