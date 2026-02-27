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
#include <autoware_adapi_v1_msgs/msg/response_status.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>

#include <string>

namespace autoware::control::command_gate
{

struct ModeOutputs
{
  autoware_adapi_v1_msgs::msg::OperationModeState state;
  autoware_vehicle_msgs::msg::GearCommand gear;
  autoware_adapi_v1_msgs::msg::ResponseStatus status;
};

class CommandGateModeBuilder
{
public:
  static ModeOutputs make_stop(const builtin_interfaces::msg::Time & stamp);
  static ModeOutputs make_autonomous(const builtin_interfaces::msg::Time & stamp);

private:
  static void fill_state(
    autoware_adapi_v1_msgs::msg::OperationModeState & msg, uint8_t mode,
    const builtin_interfaces::msg::Time & stamp);
  static void fill_gear(
    autoware_vehicle_msgs::msg::GearCommand & msg, uint8_t command,
    const builtin_interfaces::msg::Time & stamp);
  static autoware_adapi_v1_msgs::msg::ResponseStatus make_status(const std::string & message);
};

}  // namespace autoware::control::command_gate

#endif  // COMMAND_GATE_MODE_BUILDER_HPP_
