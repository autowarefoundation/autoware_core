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
#include <autoware/component_interface_specs/system.hpp>
#include <autoware/component_interface_specs/utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <autoware_vehicle_msgs/msg/gear_command.hpp>

#include <rmw/types.h>

#include <cstdint>

namespace autoware::control::command_gate
{

namespace spec
{
using ChangeToStop = autoware::adapi_specs::operation_mode::ChangeToStop;
using ChangeToAutonomous = autoware::adapi_specs::operation_mode::ChangeToAutonomous;
using OperationModeState = autoware::adapi_specs::operation_mode::OperationModeState;

struct GearCommand
{
  using Message = autoware_vehicle_msgs::msg::GearCommand;
  static constexpr char name[] = "/control/command/gear_cmd";
};
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
    state_pub_(
      create_publisher<spec::OperationModeState::Message>(
        spec::OperationModeState::name,
        autoware::component_interface_specs::get_qos<spec::OperationModeState>())),
    system_state_pub_(
      create_publisher<system::OperationModeState::Message>(
        system::OperationModeState::name,
        autoware::component_interface_specs::get_qos<system::OperationModeState>())),
    gear_pub_(create_publisher<spec::GearCommand::Message>(spec::GearCommand::name, rclcpp::QoS{1}))
  {
    /*
      System layer where the final decision to trigger the mode change is made.
      The state is published to the same topic for simplicity, but it can be separated if needed.
    */
    srv_system_mode_ = create_service<SystemChangeOperationMode::Service>(
      SystemChangeOperationMode::name,
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
  using OperationModeStateMsg = spec::OperationModeState::Message;
  using OperationModeSystemStateMsg = system::OperationModeState::Message;
  using GearCommand = autoware_vehicle_msgs::msg::GearCommand;
  void publish(const ModeOutputs & outputs)
  {
    state_pub_->publish(outputs.state);
    gear_pub_->publish(outputs.gear);
    system_state_pub_->publish(outputs.state);
  }

  rclcpp::Publisher<OperationModeStateMsg>::SharedPtr state_pub_;
  rclcpp::Publisher<OperationModeSystemStateMsg>::SharedPtr system_state_pub_;
  rclcpp::Publisher<GearCommand>::SharedPtr gear_pub_;
  rclcpp::Service<SystemChangeOperationMode::Service>::SharedPtr srv_system_mode_;
};

}  // namespace autoware::control::command_gate

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::control::command_gate::AutowareCommandGateNode)
