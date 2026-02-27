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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/response_status.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
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
  static constexpr size_t depth = 1;
  static constexpr rmw_qos_reliability_policy_t reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr rmw_qos_durability_policy_t durability =
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
};
}  // namespace spec

class AutowareCommandGateNode : public rclcpp::Node
{
public:
  explicit AutowareCommandGateNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("autoware_command_gate", options)
  {
    rclcpp::QoS state_qos(spec::OperationModeState::depth);
    state_qos.reliable();
    state_qos.transient_local();

    state_pub_ = create_publisher<spec::OperationModeState::Message>(
      spec::OperationModeState::name, state_qos);
    gear_pub_ = create_publisher<autoware_vehicle_msgs::msg::GearCommand>(
      "/control/command/gear_cmd", rclcpp::QoS{1});

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
  }

private:
  using OperationModeStateMsg = spec::OperationModeState::Message;
  using GearCommand = autoware_vehicle_msgs::msg::GearCommand;
  void publish(const ModeOutputs & outputs)
  {
    state_pub_->publish(outputs.state);
    gear_pub_->publish(outputs.gear);
  }

  rclcpp::Publisher<OperationModeStateMsg>::SharedPtr state_pub_;
  rclcpp::Publisher<GearCommand>::SharedPtr gear_pub_;
  rclcpp::Service<spec::ChangeToStop::Service>::SharedPtr srv_stop_;
  rclcpp::Service<spec::ChangeToAutonomous::Service>::SharedPtr srv_auto_;
  CommandGateModeBuilder mode_builder_;
};

}  // namespace autoware::control::command_gate

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::control::command_gate::AutowareCommandGateNode)
