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

#include <autoware/adapi_specs/operation_mode.hpp>
#include <autoware/component_interface_specs/control.hpp>
#include <autoware/component_interface_specs/system.hpp>
#include <autoware/component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <rmw/types.h>

#include <cstdint>

namespace autoware::control::command_gate
{

namespace spec
{
using ChangeToStop = autoware::adapi_specs::operation_mode::ChangeToStop;
using ChangeToAutonomous = autoware::adapi_specs::operation_mode::ChangeToAutonomous;
using OperationModeState = autoware::adapi_specs::operation_mode::OperationModeState;
using GearCommand = autoware::component_interface_specs::control::GearCommand;
}  // namespace spec

namespace system
{
using OperationModeState = autoware::component_interface_specs::system::OperationModeState;
}  // namespace system

class AutowareCommandGateNode : public rclcpp::Node
{
  using SystemChangeOperationMode =
    autoware::component_interface_specs::system::ChangeOperationMode;

public:
  explicit AutowareCommandGateNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("autoware_command_gate", options),
    state_pub_(adaptor_.create_publisher<spec::OperationModeState>()),
    system_state_pub_(adaptor_.create_publisher<system::OperationModeState>()),
    gear_pub_(adaptor_.create_publisher<spec::GearCommand>())
  {
    /*
      System layer where the final decision to trigger the mode change is made.
      The state is published to the same topic for simplicity, but it can be separated if needed.
    */
    srv_system_mode_ = adaptor_.create_service<SystemChangeOperationMode>(
      [this](
        const SystemChangeOperationMode::Service::Request::SharedPtr req,
        const SystemChangeOperationMode::Service::Response::SharedPtr res) {
        const builtin_interfaces::msg::Time stamp = now();
        const auto outputs = CommandGateModeBuilder::create_mode_output(*req, stamp);
        if (outputs) {
          publish(*outputs);
        }
        *res = CommandGateModeBuilder::create_response(*req, stamp);
      });
  }

private:
  void publish(const ModeOutputs & outputs)
  {
    state_pub_->publish(outputs.state);
    gear_pub_->publish(outputs.gear);
    system_state_pub_->publish(outputs.state);
  }

  autoware::component_interface_utils::NodeAdaptor<rclcpp::Node> adaptor_{this};
  autoware::component_interface_utils::Publisher<spec::OperationModeState>::SharedPtr state_pub_;
  autoware::component_interface_utils::Publisher<system::OperationModeState>::SharedPtr
    system_state_pub_;
  autoware::component_interface_utils::Publisher<spec::GearCommand>::SharedPtr gear_pub_;
  autoware::component_interface_utils::Service<SystemChangeOperationMode>::SharedPtr
    srv_system_mode_;
};

}  // namespace autoware::control::command_gate

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::control::command_gate::AutowareCommandGateNode)
