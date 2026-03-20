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

#include <autoware/component_interface_specs/system.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/response_status.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_common_msgs/msg/response_status.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>

#include <rmw/types.h>

#include <cstdint>

namespace autoware::control::command_gate
{

namespace spec
{
struct ChangeToStop
{
  using Service = autoware_adapi_v1_msgs::srv::ChangeOperationMode;
  static constexpr char name[] = "/api/operation_mode/change_to_stop";
};

struct ChangeToAutonomous
{
  using Service = autoware_adapi_v1_msgs::srv::ChangeOperationMode;
  static constexpr char name[] = "/api/operation_mode/change_to_autonomous";
};

struct OperationModeState
{
  using Message = autoware_adapi_v1_msgs::msg::OperationModeState;
  static constexpr char name[] = "/api/operation_mode/state";
};

struct GearCommand
{
  using Message = autoware_vehicle_msgs::msg::GearCommand;
  static constexpr char name[] = "/control/command/gear_cmd";
};
}  // namespace spec

namespace system
{
struct OperationModeState
{
  using Message = autoware_adapi_v1_msgs::msg::OperationModeState;
  static constexpr char name[] = "/system/operation_mode/state";
};
}  // namespace system

class AutowareCommandGateNode : public rclcpp::Node
{
  using SystemChangeOperationMode =
    autoware::component_interface_specs::system::ChangeOperationMode;

public:
  explicit AutowareCommandGateNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("autoware_command_gate", options)
  {
    // The depth of the state topic is set to 1 to ensure that the latest state is always delivered
    // to subscribers.
    static constexpr size_t depth = 1;

    // Publishers
    rclcpp::QoS state_qos(depth);
    state_qos.reliable();
    state_qos.transient_local();

    state_pub_ = create_publisher<spec::OperationModeState::Message>(
      spec::OperationModeState::name, state_qos);
    gear_pub_ =
      create_publisher<spec::GearCommand::Message>(spec::GearCommand::name, rclcpp::QoS{depth});

    srv_stop_ = create_service<spec::ChangeToStop::Service>(
      spec::ChangeToStop::name, [this](
                                  const spec::ChangeToStop::Service::Request::SharedPtr,
                                  const spec::ChangeToStop::Service::Response::SharedPtr res) {
        const builtin_interfaces::msg::Time stamp = now();
        const auto outputs = mode_builder_.make_stop(stamp);
        publish(outputs);
        res->status = outputs.status;
      });

    srv_auto_ = create_service<spec::ChangeToAutonomous::Service>(
      spec::ChangeToAutonomous::name,
      [this](
        const spec::ChangeToAutonomous::Service::Request::SharedPtr,
        const spec::ChangeToAutonomous::Service::Response::SharedPtr res) {
        const builtin_interfaces::msg::Time stamp = now();
        const auto outputs = mode_builder_.make_autonomous(stamp);
        publish(outputs);
        res->status = outputs.status;
      });

    /*
      System layer where the final decision to trigger the mode change is made.
      The state is published to the same topic for simplicity, but it can be separated if needed.
    */
    system_state_pub_ = create_publisher<system::OperationModeState::Message>(
      system::OperationModeState::name, state_qos);
    srv_system_mode_ = create_service<SystemChangeOperationMode::Service>(
      SystemChangeOperationMode::name,
      [this](
        const SystemChangeOperationMode::Service::Request::SharedPtr req,
        const SystemChangeOperationMode::Service::Response::SharedPtr res) {
        const builtin_interfaces::msg::Time stamp = now();
        ModeOutputs outputs;

        switch (req->mode) {
          case SystemChangeOperationMode::Service::Request::STOP:
            outputs = mode_builder_.make_stop(stamp);
            break;
          case SystemChangeOperationMode::Service::Request::AUTONOMOUS:
            outputs = mode_builder_.make_autonomous(stamp);
            break;
          case SystemChangeOperationMode::Service::Request::LOCAL:
            outputs = mode_builder_.make_local(stamp);
            break;
          case SystemChangeOperationMode::Service::Request::REMOTE:
            outputs = mode_builder_.make_remote(stamp);
            break;
          default:
            res->status.success = false;
            res->status.code = autoware_common_msgs::msg::ResponseStatus::PARAMETER_ERROR;
            res->status.message = "Unknown operation mode requested.";
            return;
        }

        publish(outputs);
        res->status.success = true;
        res->status.code = 0;
        res->status.message = outputs.status.message;
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
  rclcpp::Service<spec::ChangeToStop::Service>::SharedPtr srv_stop_;
  rclcpp::Service<spec::ChangeToAutonomous::Service>::SharedPtr srv_auto_;
  rclcpp::Service<SystemChangeOperationMode::Service>::SharedPtr srv_system_mode_;
  CommandGateModeBuilder mode_builder_;
};

}  // namespace autoware::control::command_gate

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::control::command_gate::AutowareCommandGateNode)
