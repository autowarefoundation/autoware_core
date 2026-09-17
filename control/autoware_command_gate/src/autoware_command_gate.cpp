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

#include <autoware/component_interface_specs/control.hpp>
#include <autoware/component_interface_specs/system.hpp>
#include <autoware/component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_common_msgs/msg/response_status.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>

#include <string>

namespace autoware::control::command_gate
{

namespace spec
{
using GearCommand = autoware::component_interface_specs::control::GearCommand;
}  // namespace spec

namespace system
{
using OperationModeState = autoware::component_interface_specs::system::OperationModeState;
using ChangeOperationMode = autoware::component_interface_specs::system::ChangeOperationMode;
using ChangeAutowareControl = autoware::component_interface_specs::system::ChangeAutowareControl;
}  // namespace system

class AutowareCommandGateNode : public rclcpp::Node
{
public:
  explicit AutowareCommandGateNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("autoware_command_gate", options),
    system_state_pub_(adaptor_.create_publisher<system::OperationModeState>()),
    gear_pub_(adaptor_.create_publisher<spec::GearCommand>())
  {
    // Initial latch
    current_state_.mode = autoware_adapi_v1_msgs::msg::OperationModeState::STOP;
    current_state_.is_autoware_control_enabled = false;
    current_state_.is_in_transition = false;
    current_state_.is_stop_mode_available = true;
    current_state_.is_autonomous_mode_available = true;
    current_state_.is_local_mode_available = true;
    current_state_.is_remote_mode_available = true;

    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(100), [this]() {
      if (!initial_published_) {
        publish_state();
        initial_published_ = true;
      }
    });

    srv_system_mode_ = adaptor_.create_service<system::ChangeOperationMode>(
      [this](
        const system::ChangeOperationMode::Service::Request::SharedPtr req,
        const system::ChangeOperationMode::Service::Response::SharedPtr res) {
        bool valid = true;
        std::string message;
        switch (req->mode) {
          case system::ChangeOperationMode::Service::Request::STOP:
            current_state_.mode = autoware_adapi_v1_msgs::msg::OperationModeState::STOP;
            message = "Switched to STOP";
            break;
          case system::ChangeOperationMode::Service::Request::AUTONOMOUS:
            current_state_.mode = autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS;
            message = "Switched to AUTONOMOUS";
            break;
          case system::ChangeOperationMode::Service::Request::LOCAL:
            current_state_.mode = autoware_adapi_v1_msgs::msg::OperationModeState::LOCAL;
            message = "Switched to LOCAL";
            break;
          case system::ChangeOperationMode::Service::Request::REMOTE:
            current_state_.mode = autoware_adapi_v1_msgs::msg::OperationModeState::REMOTE;
            message = "Switched to REMOTE";
            break;
          default:
            valid = false;
            break;
        }

        if (valid) {
          res->status.success = true;
          res->status.code = 0;
          res->status.message = message;
          publish_state();
        } else {
          res->status.success = false;
          res->status.code = autoware_common_msgs::msg::ResponseStatus::PARAMETER_ERROR;
          res->status.message = "Unknown operation mode requested.";
        }
      });

    srv_system_control_ = adaptor_.create_service<system::ChangeAutowareControl>(
      [this](
        const system::ChangeAutowareControl::Service::Request::SharedPtr req,
        const system::ChangeAutowareControl::Service::Response::SharedPtr res) {
        current_state_.is_autoware_control_enabled = req->autoware_control;
        res->status.success = true;
        res->status.code = 0;
        res->status.message =
          req->autoware_control ? "Autoware control enabled" : "Autoware control disabled";
        publish_state();
      });
  }

private:
  void publish_state()
  {
    current_state_.stamp = now();
    system_state_pub_->publish(current_state_);

    autoware_vehicle_msgs::msg::GearCommand gear_cmd;
    gear_cmd.stamp = current_state_.stamp;
    switch (current_state_.mode) {
      case autoware_adapi_v1_msgs::msg::OperationModeState::STOP:
        gear_cmd.command = autoware_vehicle_msgs::msg::GearCommand::PARK;
        break;
      case autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS:
        gear_cmd.command = autoware_vehicle_msgs::msg::GearCommand::DRIVE;
        break;
      case autoware_adapi_v1_msgs::msg::OperationModeState::LOCAL:
      case autoware_adapi_v1_msgs::msg::OperationModeState::REMOTE:
      default:
        gear_cmd.command = autoware_vehicle_msgs::msg::GearCommand::NONE;
        break;
    }
    gear_pub_->publish(gear_cmd);
  }

  autoware::component_interface_utils::NodeAdaptor<rclcpp::Node> adaptor_{this};
  autoware::component_interface_utils::Publisher<system::OperationModeState>::SharedPtr
    system_state_pub_;
  autoware::component_interface_utils::Publisher<spec::GearCommand>::SharedPtr gear_pub_;

  autoware::component_interface_utils::Service<system::ChangeOperationMode>::SharedPtr
    srv_system_mode_;
  autoware::component_interface_utils::Service<system::ChangeAutowareControl>::SharedPtr
    srv_system_control_;

  rclcpp::TimerBase::SharedPtr timer_;
  autoware_adapi_v1_msgs::msg::OperationModeState current_state_;
  bool initial_published_ = false;
};

}  // namespace autoware::control::command_gate

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::control::command_gate::AutowareCommandGateNode)
