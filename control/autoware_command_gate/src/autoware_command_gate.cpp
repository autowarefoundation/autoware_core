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
#include <autoware/component_interface_specs/vehicle.hpp>
#include <autoware/component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_common_msgs/msg/response_status.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/srv/control_mode_command.hpp>

#include <chrono>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>

namespace autoware::control::command_gate
{

namespace spec
{
using GearCommand = autoware::component_interface_specs::control::GearCommand;
using ControlModeRequest = autoware::component_interface_specs::control::ControlModeRequest;
}  // namespace spec

namespace system
{
using OperationModeState = autoware::component_interface_specs::system::OperationModeState;
using ChangeOperationMode = autoware::component_interface_specs::system::ChangeOperationMode;
using ChangeAutowareControl = autoware::component_interface_specs::system::ChangeAutowareControl;
}  // namespace system

namespace vehicle
{
using ControlModeStatus = autoware::component_interface_specs::vehicle::ControlModeStatus;
}  // namespace vehicle

using SystemControlService = system::ChangeAutowareControl::Service;
using VehicleControlService = spec::ControlModeRequest::Service;
using ControlModeReport = vehicle::ControlModeStatus::Message;

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

    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      if (!initial_published_) {
        publish_state();
      }
    });

    sub_control_mode_ = adaptor_.create_subscription<vehicle::ControlModeStatus>(
      [this](const ControlModeReport::ConstSharedPtr msg) {
        const bool enabled = msg->mode == ControlModeReport::AUTONOMOUS ||
                             msg->mode == ControlModeReport::AUTONOMOUS_STEER_ONLY ||
                             msg->mode == ControlModeReport::AUTONOMOUS_VELOCITY_ONLY;
        if (current_state_.is_autoware_control_enabled != enabled) {
          current_state_.is_autoware_control_enabled = enabled;
          publish_state();
        }
      });
    cli_vehicle_control_ = create_client<VehicleControlService>(spec::ControlModeRequest::name);

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
          publish_gear();
        } else {
          res->status.success = false;
          res->status.code = autoware_common_msgs::msg::ResponseStatus::PARAMETER_ERROR;
          res->status.message = "Unknown operation mode requested.";
        }
      });

    // Defer the reply so a single-threaded executor can process the vehicle response.
    srv_system_control_ = create_service<SystemControlService>(
      system::ChangeAutowareControl::name,
      [this](
        std::shared_ptr<rclcpp::Service<SystemControlService>>,
        std::shared_ptr<rmw_request_id_t> header, SystemControlService::Request::SharedPtr req) {
        if (pending_control_header_) {
          send_control_response(
            header, false, autoware_common_msgs::msg::ResponseStatus::UNKNOWN,
            "A control mode request is already in progress.");
          return;
        }
        if (!cli_vehicle_control_->service_is_ready()) {
          send_control_response(
            header, false, autoware_common_msgs::msg::ResponseStatus::SERVICE_UNREADY,
            "Vehicle control mode service is not ready.");
          return;
        }

        pending_control_header_ = header;
        const auto token = ++control_request_token_;
        auto vehicle_request = std::make_shared<VehicleControlService::Request>();
        vehicle_request->mode = req->autoware_control ? VehicleControlService::Request::AUTONOMOUS
                                                      : VehicleControlService::Request::MANUAL;
        const auto future = cli_vehicle_control_->async_send_request(
          vehicle_request,
          [this, token](rclcpp::Client<VehicleControlService>::SharedFuture result) {
            if (!pending_control_header_ || token != control_request_token_) {
              return;
            }
            VehicleControlService::Response::SharedPtr vehicle_response;
            try {
              vehicle_response = result.get();
            } catch (const std::exception & error) {
              complete_control_request(
                token, false, autoware_common_msgs::msg::ResponseStatus::UNKNOWN, error.what());
              return;
            }
            if (vehicle_response->success) {
              complete_control_request(
                token, true, 0, "Vehicle accepted the control mode request.");
            } else {
              complete_control_request(
                token, false, autoware_common_msgs::msg::ResponseStatus::UNKNOWN,
                "Vehicle rejected the control mode request.");
            }
          });
        pending_vehicle_request_id_ = future.request_id;
        control_timeout_timer_ = create_wall_timer(std::chrono::seconds(2), [this, token]() {
          if (pending_control_header_ && token == control_request_token_) {
            // The vehicle can still act after this local request times out.
            cli_vehicle_control_->remove_pending_request(pending_vehicle_request_id_);
            complete_control_request(
              token, false, autoware_common_msgs::msg::ResponseStatus::SERVICE_TIMEOUT,
              "Vehicle control mode request timed out.");
          }
        });
      });
  }

private:
  void send_control_response(
    const std::shared_ptr<rmw_request_id_t> & header, bool success, uint16_t code,
    const std::string & message)
  {
    SystemControlService::Response response;
    response.status.success = success;
    response.status.code = code;
    response.status.message = message;
    srv_system_control_->send_response(*header, response);
  }

  void complete_control_request(
    uint64_t token, bool success, uint16_t code, const std::string & message)
  {
    if (!pending_control_header_ || token != control_request_token_) {
      return;
    }
    auto header = pending_control_header_;
    pending_control_header_.reset();
    control_timeout_timer_->cancel();
    control_timeout_timer_.reset();
    send_control_response(header, success, code, message);
  }

  void publish_state()
  {
    current_state_.stamp = now();
    system_state_pub_->publish(current_state_);
    initial_published_ = true;
  }

  void publish_gear()
  {
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
  rclcpp::Service<SystemControlService>::SharedPtr srv_system_control_;
  rclcpp::Client<VehicleControlService>::SharedPtr cli_vehicle_control_;
  autoware::component_interface_utils::Subscription<vehicle::ControlModeStatus>::SharedPtr
    sub_control_mode_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr control_timeout_timer_;
  autoware_adapi_v1_msgs::msg::OperationModeState current_state_;
  bool initial_published_ = false;
  std::shared_ptr<rmw_request_id_t> pending_control_header_;
  int64_t pending_vehicle_request_id_ = 0;
  uint64_t control_request_token_ = 0;
};

}  // namespace autoware::control::command_gate

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::control::command_gate::AutowareCommandGateNode)
