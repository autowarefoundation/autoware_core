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

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <rclcpp_components/node_instance_wrapper.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_common_msgs/msg/response_status.hpp>
#include <autoware_system_msgs/srv/change_autoware_control.hpp>
#include <autoware_system_msgs/srv/change_operation_mode.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/srv/control_mode_command.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace
{
using autoware_adapi_v1_msgs::msg::OperationModeState;
using SystemChangeOperationMode = autoware_system_msgs::srv::ChangeOperationMode;
using SystemChangeAutowareControl = autoware_system_msgs::srv::ChangeAutowareControl;
using autoware_vehicle_msgs::msg::ControlModeReport;
using autoware_vehicle_msgs::msg::GearCommand;
using autoware_vehicle_msgs::srv::ControlModeCommand;

bool spin_until(
  rclcpp::executors::SingleThreadedExecutor & executor, const std::function<bool()> & predicate,
  const std::chrono::milliseconds timeout)
{
  const auto start = std::chrono::steady_clock::now();
  while (rclcpp::ok() && (std::chrono::steady_clock::now() - start) < timeout) {
    executor.spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  executor.spin_some();
  return predicate();
}

}  // namespace

class CommandGateRosIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    test_node_ = std::make_shared<rclcpp::Node>("command_gate_test_node");
    vehicle_mode_pub_ = test_node_->create_publisher<ControlModeReport>(
      "/vehicle/status/control_mode", rclcpp::QoS{1});
    if (provide_vehicle_service()) {
      if (defer_vehicle_response()) {
        vehicle_control_srv_ = test_node_->create_service<ControlModeCommand>(
          "/control/control_mode_request",
          [this](
            std::shared_ptr<rclcpp::Service<ControlModeCommand>>,
            std::shared_ptr<rmw_request_id_t> header, ControlModeCommand::Request::SharedPtr req) {
            requested_vehicle_modes_.push_back(req->mode);
            pending_vehicle_header_ = header;
          });
      } else {
        vehicle_control_srv_ = test_node_->create_service<ControlModeCommand>(
          "/control/control_mode_request", [this](
                                             const ControlModeCommand::Request::SharedPtr req,
                                             const ControlModeCommand::Response::SharedPtr res) {
            requested_vehicle_modes_.push_back(req->mode);
            res->success = vehicle_accepts_requests_;
            if (res->success) {
              ControlModeReport report;
              report.mode = req->mode;
              vehicle_mode_pub_->publish(report);
            }
          });
      }
    }
    component_manager_ = std::make_shared<rclcpp_components::ComponentManager>();
    const auto resources = component_manager_->get_component_resources("autoware_command_gate");
    auto resource_it = std::find_if(resources.begin(), resources.end(), [](const auto & resource) {
      return resource.first == "autoware::control::command_gate::AutowareCommandGateNode";
    });
    ASSERT_TRUE(resource_it != resources.end());
    factory_ = component_manager_->create_component_factory(*resource_it);
    wrapper_ = std::make_unique<rclcpp_components::NodeInstanceWrapper>(
      factory_->create_node_instance(gate_options()));
    component_node_base_ = wrapper_->get_node_base_interface();

    executor_.add_node(test_node_);
    executor_.add_node(component_node_base_);
  }

  virtual rclcpp::NodeOptions gate_options() const { return rclcpp::NodeOptions{}; }
  virtual bool provide_vehicle_service() const { return true; }
  virtual bool defer_vehicle_response() const { return false; }

  void TearDown() override
  {
    if (component_node_base_) {
      executor_.remove_node(component_node_base_);
    }
    if (test_node_) {
      executor_.remove_node(test_node_);
    }
    component_node_base_.reset();
    wrapper_.reset();
    factory_.reset();
    component_manager_.reset();
    test_node_.reset();
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<rclcpp::Node> test_node_;
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr component_node_base_;
  std::shared_ptr<rclcpp_components::ComponentManager> component_manager_;
  std::shared_ptr<rclcpp_components::NodeFactory> factory_;
  std::unique_ptr<rclcpp_components::NodeInstanceWrapper> wrapper_;
  rclcpp::Publisher<ControlModeReport>::SharedPtr vehicle_mode_pub_;
  rclcpp::Service<ControlModeCommand>::SharedPtr vehicle_control_srv_;
  std::shared_ptr<rmw_request_id_t> pending_vehicle_header_;
  std::vector<uint8_t> requested_vehicle_modes_;
  bool vehicle_accepts_requests_ = true;
};

TEST_F(CommandGateRosIntegrationTest, ChangeToStopPublishesStateAndGear)
{
  std::optional<OperationModeState> state_msg;
  std::optional<GearCommand> gear_msg;

  rclcpp::QoS state_qos(1);
  state_qos.reliable();
  state_qos.transient_local();

  auto state_sub = test_node_->create_subscription<OperationModeState>(
    "/system/operation_mode/state", state_qos,
    [&state_msg](const OperationModeState::SharedPtr msg) { state_msg = *msg; });
  auto gear_sub = test_node_->create_subscription<GearCommand>(
    "/control/command/gear_cmd", rclcpp::QoS{1},
    [&gear_msg](const GearCommand::SharedPtr msg) { gear_msg = *msg; });

  auto client = test_node_->create_client<SystemChangeOperationMode>(
    "/system/operation_mode/change_operation_mode");
  ASSERT_TRUE(spin_until(
    executor_, [&client]() { return client->wait_for_service(std::chrono::seconds(0)); },
    std::chrono::seconds(2)));

  auto request = std::make_shared<SystemChangeOperationMode::Request>();
  request->mode = SystemChangeOperationMode::Request::STOP;
  auto future = client->async_send_request(request);
  ASSERT_TRUE(spin_until(
    executor_,
    [&future]() { return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready; },
    std::chrono::seconds(2)));

  const auto response = future.get();
  EXPECT_TRUE(response->status.success);
  EXPECT_EQ(response->status.code, 0);
  EXPECT_EQ(response->status.message, "Switched to STOP");

  ASSERT_TRUE(spin_until(
    executor_, [&state_msg, &gear_msg]() { return state_msg.has_value() && gear_msg.has_value(); },
    std::chrono::seconds(2)));

  EXPECT_EQ(state_msg->mode, OperationModeState::STOP);
  EXPECT_FALSE(state_msg->is_autoware_control_enabled);
  EXPECT_FALSE(state_msg->is_in_transition);
  EXPECT_TRUE(state_msg->is_stop_mode_available);
  EXPECT_TRUE(state_msg->is_autonomous_mode_available);
  EXPECT_TRUE(state_msg->is_local_mode_available);
  EXPECT_TRUE(state_msg->is_remote_mode_available);

  EXPECT_EQ(gear_msg->command, GearCommand::PARK);
}

TEST_F(CommandGateRosIntegrationTest, ChangeToAutonomousPublishesStateAndGear)
{
  std::optional<OperationModeState> state_msg;
  std::optional<GearCommand> gear_msg;

  rclcpp::QoS state_qos(1);
  state_qos.reliable();
  state_qos.transient_local();

  auto state_sub = test_node_->create_subscription<OperationModeState>(
    "/system/operation_mode/state", state_qos,
    [&state_msg](const OperationModeState::SharedPtr msg) { state_msg = *msg; });
  auto gear_sub = test_node_->create_subscription<GearCommand>(
    "/control/command/gear_cmd", rclcpp::QoS{1},
    [&gear_msg](const GearCommand::SharedPtr msg) { gear_msg = *msg; });

  auto client = test_node_->create_client<SystemChangeOperationMode>(
    "/system/operation_mode/change_operation_mode");
  ASSERT_TRUE(spin_until(
    executor_, [&client]() { return client->wait_for_service(std::chrono::seconds(0)); },
    std::chrono::seconds(2)));

  auto request = std::make_shared<SystemChangeOperationMode::Request>();
  request->mode = SystemChangeOperationMode::Request::AUTONOMOUS;
  auto future = client->async_send_request(request);
  ASSERT_TRUE(spin_until(
    executor_,
    [&future]() { return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready; },
    std::chrono::seconds(2)));

  const auto response = future.get();
  EXPECT_TRUE(response->status.success);
  EXPECT_EQ(response->status.code, 0);
  EXPECT_EQ(response->status.message, "Switched to AUTONOMOUS");

  ASSERT_TRUE(spin_until(
    executor_, [&state_msg, &gear_msg]() { return state_msg.has_value() && gear_msg.has_value(); },
    std::chrono::seconds(2)));

  EXPECT_EQ(state_msg->mode, OperationModeState::AUTONOMOUS);
  EXPECT_FALSE(state_msg->is_autoware_control_enabled);  // Two-axis: control flag independent
  EXPECT_FALSE(state_msg->is_in_transition);
  EXPECT_TRUE(state_msg->is_stop_mode_available);
  EXPECT_TRUE(state_msg->is_autonomous_mode_available);
  EXPECT_TRUE(state_msg->is_local_mode_available);
  EXPECT_TRUE(state_msg->is_remote_mode_available);

  EXPECT_EQ(gear_msg->command, GearCommand::DRIVE);
}

TEST_F(CommandGateRosIntegrationTest, SystemChangeToLocalPublishesStateAndGear)
{
  std::optional<OperationModeState> state_msg;
  std::optional<GearCommand> gear_msg;

  rclcpp::QoS state_qos(1);
  state_qos.reliable();
  state_qos.transient_local();

  auto state_sub = test_node_->create_subscription<OperationModeState>(
    "/system/operation_mode/state", state_qos,
    [&state_msg](const OperationModeState::SharedPtr msg) { state_msg = *msg; });
  auto gear_sub = test_node_->create_subscription<GearCommand>(
    "/control/command/gear_cmd", rclcpp::QoS{1},
    [&gear_msg](const GearCommand::SharedPtr msg) { gear_msg = *msg; });

  auto client = test_node_->create_client<SystemChangeOperationMode>(
    "/system/operation_mode/change_operation_mode");
  ASSERT_TRUE(spin_until(
    executor_, [&client]() { return client->wait_for_service(std::chrono::seconds(0)); },
    std::chrono::seconds(2)));

  auto request = std::make_shared<SystemChangeOperationMode::Request>();
  request->mode = SystemChangeOperationMode::Request::LOCAL;
  auto future = client->async_send_request(request);
  ASSERT_TRUE(spin_until(
    executor_,
    [&future]() { return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready; },
    std::chrono::seconds(2)));

  const auto response = future.get();
  EXPECT_TRUE(response->status.success);
  EXPECT_EQ(response->status.code, 0);
  EXPECT_EQ(response->status.message, "Switched to LOCAL");

  ASSERT_TRUE(spin_until(
    executor_, [&state_msg, &gear_msg]() { return state_msg.has_value() && gear_msg.has_value(); },
    std::chrono::seconds(2)));

  EXPECT_EQ(state_msg->mode, OperationModeState::LOCAL);
  EXPECT_FALSE(state_msg->is_autoware_control_enabled);
  EXPECT_FALSE(state_msg->is_in_transition);
  EXPECT_TRUE(state_msg->is_stop_mode_available);
  EXPECT_TRUE(state_msg->is_autonomous_mode_available);
  EXPECT_TRUE(state_msg->is_local_mode_available);
  EXPECT_TRUE(state_msg->is_remote_mode_available);

  EXPECT_EQ(gear_msg->command, GearCommand::NONE);
}

TEST_F(CommandGateRosIntegrationTest, SystemChangeToRemotePublishesStateAndGear)
{
  std::optional<OperationModeState> state_msg;
  std::optional<GearCommand> gear_msg;

  rclcpp::QoS state_qos(1);
  state_qos.reliable();
  state_qos.transient_local();

  auto state_sub = test_node_->create_subscription<OperationModeState>(
    "/system/operation_mode/state", state_qos,
    [&state_msg](const OperationModeState::SharedPtr msg) { state_msg = *msg; });
  auto gear_sub = test_node_->create_subscription<GearCommand>(
    "/control/command/gear_cmd", rclcpp::QoS{1},
    [&gear_msg](const GearCommand::SharedPtr msg) { gear_msg = *msg; });

  auto client = test_node_->create_client<SystemChangeOperationMode>(
    "/system/operation_mode/change_operation_mode");
  ASSERT_TRUE(spin_until(
    executor_, [&client]() { return client->wait_for_service(std::chrono::seconds(0)); },
    std::chrono::seconds(2)));

  auto request = std::make_shared<SystemChangeOperationMode::Request>();
  request->mode = SystemChangeOperationMode::Request::REMOTE;
  auto future = client->async_send_request(request);
  ASSERT_TRUE(spin_until(
    executor_,
    [&future]() { return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready; },
    std::chrono::seconds(2)));

  const auto response = future.get();
  EXPECT_TRUE(response->status.success);
  EXPECT_EQ(response->status.code, 0);
  EXPECT_EQ(response->status.message, "Switched to REMOTE");

  ASSERT_TRUE(spin_until(
    executor_, [&state_msg, &gear_msg]() { return state_msg.has_value() && gear_msg.has_value(); },
    std::chrono::seconds(2)));

  EXPECT_EQ(state_msg->mode, OperationModeState::REMOTE);
  EXPECT_FALSE(state_msg->is_autoware_control_enabled);
  EXPECT_FALSE(state_msg->is_in_transition);
  EXPECT_TRUE(state_msg->is_stop_mode_available);
  EXPECT_TRUE(state_msg->is_autonomous_mode_available);
  EXPECT_TRUE(state_msg->is_local_mode_available);
  EXPECT_TRUE(state_msg->is_remote_mode_available);

  EXPECT_EQ(gear_msg->command, GearCommand::NONE);
}

TEST_F(CommandGateRosIntegrationTest, ChangeAutowareControlTogglesControlFlag)
{
  std::optional<OperationModeState> state_msg;

  rclcpp::QoS state_qos(1);
  state_qos.reliable();
  state_qos.transient_local();

  auto state_sub = test_node_->create_subscription<OperationModeState>(
    "/system/operation_mode/state", state_qos,
    [&state_msg](const OperationModeState::SharedPtr msg) { state_msg = *msg; });

  auto client = test_node_->create_client<SystemChangeAutowareControl>(
    "/system/operation_mode/change_autoware_control");
  ASSERT_TRUE(spin_until(
    executor_, [&client]() { return client->wait_for_service(std::chrono::seconds(0)); },
    std::chrono::seconds(2)));

  // 1. Enable control
  auto request_enable = std::make_shared<SystemChangeAutowareControl::Request>();
  request_enable->autoware_control = true;
  auto future_enable = client->async_send_request(request_enable);
  ASSERT_TRUE(spin_until(
    executor_,
    [&future_enable]() {
      return future_enable.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    },
    std::chrono::seconds(2)));

  const auto response_enable = future_enable.get();
  EXPECT_TRUE(response_enable->status.success);
  EXPECT_EQ(response_enable->status.code, 0);

  ASSERT_TRUE(spin_until(
    executor_,
    [&state_msg]() { return state_msg.has_value() && state_msg->is_autoware_control_enabled; },
    std::chrono::seconds(2)));

  EXPECT_TRUE(state_msg->is_autoware_control_enabled);
  ASSERT_EQ(requested_vehicle_modes_.size(), 1U);
  EXPECT_EQ(requested_vehicle_modes_[0], ControlModeCommand::Request::AUTONOMOUS);

  // 2. Disable control
  auto request_disable = std::make_shared<SystemChangeAutowareControl::Request>();
  request_disable->autoware_control = false;
  auto future_disable = client->async_send_request(request_disable);
  ASSERT_TRUE(spin_until(
    executor_,
    [&future_disable]() {
      return future_disable.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    },
    std::chrono::seconds(2)));

  const auto response_disable = future_disable.get();
  EXPECT_TRUE(response_disable->status.success);
  EXPECT_EQ(response_disable->status.code, 0);

  ASSERT_TRUE(spin_until(
    executor_,
    [&state_msg]() { return state_msg.has_value() && !state_msg->is_autoware_control_enabled; },
    std::chrono::seconds(2)));

  EXPECT_FALSE(state_msg->is_autoware_control_enabled);
  ASSERT_EQ(requested_vehicle_modes_.size(), 2U);
  EXPECT_EQ(requested_vehicle_modes_[1], ControlModeCommand::Request::MANUAL);
}

TEST_F(CommandGateRosIntegrationTest, RejectedVehicleControlRequestDoesNotEnableControl)
{
  vehicle_accepts_requests_ = false;
  std::optional<OperationModeState> state_msg;
  auto state_qos = rclcpp::QoS(1).reliable().transient_local();
  auto state_sub = test_node_->create_subscription<OperationModeState>(
    "/system/operation_mode/state", state_qos,
    [&state_msg](const OperationModeState::SharedPtr msg) { state_msg = *msg; });
  auto client = test_node_->create_client<SystemChangeAutowareControl>(
    "/system/operation_mode/change_autoware_control");
  ASSERT_TRUE(spin_until(
    executor_, [&client]() { return client->service_is_ready(); }, std::chrono::seconds(2)));

  auto request = std::make_shared<SystemChangeAutowareControl::Request>();
  request->autoware_control = true;
  auto future = client->async_send_request(request);
  ASSERT_TRUE(spin_until(
    executor_,
    [&future]() { return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready; },
    std::chrono::seconds(2)));
  EXPECT_FALSE(future.get()->status.success);
  ASSERT_EQ(requested_vehicle_modes_.size(), 1U);
  EXPECT_EQ(requested_vehicle_modes_[0], ControlModeCommand::Request::AUTONOMOUS);
  ASSERT_TRUE(spin_until(
    executor_, [&state_msg]() { return state_msg.has_value(); }, std::chrono::seconds(2)));
  EXPECT_FALSE(state_msg->is_autoware_control_enabled);
}

TEST_F(CommandGateRosIntegrationTest, VehicleReportUpdatesControlFlag)
{
  std::optional<OperationModeState> state_msg;
  auto state_qos = rclcpp::QoS(1).reliable().transient_local();
  auto state_sub = test_node_->create_subscription<OperationModeState>(
    "/system/operation_mode/state", state_qos,
    [&state_msg](const OperationModeState::SharedPtr msg) { state_msg = *msg; });
  ASSERT_TRUE(spin_until(
    executor_, [&state_msg]() { return state_msg.has_value(); }, std::chrono::seconds(2)));
  ASSERT_TRUE(spin_until(
    executor_, [this]() { return vehicle_mode_pub_->get_subscription_count() > 0; },
    std::chrono::seconds(2)));

  ControlModeReport report;
  report.mode = ControlModeReport::AUTONOMOUS;
  vehicle_mode_pub_->publish(report);
  ASSERT_TRUE(spin_until(
    executor_, [&state_msg]() { return state_msg->is_autoware_control_enabled; },
    std::chrono::seconds(2)));
  report.mode = ControlModeReport::MANUAL;
  vehicle_mode_pub_->publish(report);
  ASSERT_TRUE(spin_until(
    executor_, [&state_msg]() { return !state_msg->is_autoware_control_enabled; },
    std::chrono::seconds(2)));
  EXPECT_TRUE(requested_vehicle_modes_.empty());
}

TEST_F(CommandGateRosIntegrationTest, ControlFlagChangeDoesNotPublishGear)
{
  std::optional<OperationModeState> state_msg;
  std::optional<GearCommand> gear_msg;
  std::size_t gear_count = 0;

  rclcpp::QoS state_qos(1);
  state_qos.reliable();
  state_qos.transient_local();
  auto state_sub = test_node_->create_subscription<OperationModeState>(
    "/system/operation_mode/state", state_qos,
    [&state_msg](const OperationModeState::SharedPtr msg) { state_msg = *msg; });
  auto gear_sub = test_node_->create_subscription<GearCommand>(
    "/control/command/gear_cmd", rclcpp::QoS{1},
    [&gear_msg, &gear_count](const GearCommand::SharedPtr msg) {
      gear_msg = *msg;
      ++gear_count;
    });

  auto mode_client = test_node_->create_client<SystemChangeOperationMode>(
    "/system/operation_mode/change_operation_mode");
  auto control_client = test_node_->create_client<SystemChangeAutowareControl>(
    "/system/operation_mode/change_autoware_control");
  ASSERT_TRUE(spin_until(
    executor_,
    [&mode_client, &control_client]() {
      return mode_client->service_is_ready() && control_client->service_is_ready();
    },
    std::chrono::seconds(2)));

  auto mode_request = std::make_shared<SystemChangeOperationMode::Request>();
  mode_request->mode = SystemChangeOperationMode::Request::AUTONOMOUS;
  auto mode_future = mode_client->async_send_request(mode_request);
  ASSERT_TRUE(spin_until(
    executor_,
    [&mode_future]() {
      return mode_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    },
    std::chrono::seconds(2)));
  ASSERT_TRUE(mode_future.get()->status.success);
  ASSERT_TRUE(spin_until(
    executor_,
    [&state_msg, &gear_msg]() {
      return state_msg && state_msg->mode == OperationModeState::AUTONOMOUS && gear_msg &&
             gear_msg->command == GearCommand::DRIVE;
    },
    std::chrono::seconds(2)));
  const auto gear_count_after_mode = gear_count;

  auto control_request = std::make_shared<SystemChangeAutowareControl::Request>();
  control_request->autoware_control = true;
  auto control_future = control_client->async_send_request(control_request);
  ASSERT_TRUE(spin_until(
    executor_,
    [&control_future]() {
      return control_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    },
    std::chrono::seconds(2)));
  ASSERT_TRUE(control_future.get()->status.success);
  ASSERT_TRUE(spin_until(
    executor_, [&state_msg]() { return state_msg && state_msg->is_autoware_control_enabled; },
    std::chrono::seconds(2)));
  spin_until(executor_, []() { return false; }, std::chrono::milliseconds(100));
  EXPECT_EQ(gear_count, gear_count_after_mode);
  ASSERT_EQ(requested_vehicle_modes_.size(), 1U);
  EXPECT_EQ(requested_vehicle_modes_[0], ControlModeCommand::Request::AUTONOMOUS);
}

class PausedClockGateTest : public CommandGateRosIntegrationTest
{
protected:
  rclcpp::NodeOptions gate_options() const override
  {
    return rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  }
};

TEST_F(PausedClockGateTest, PublishesInitialStateWithoutClock)
{
  std::optional<OperationModeState> state_msg;
  auto state_qos = rclcpp::QoS(1).reliable().transient_local();
  auto state_sub = test_node_->create_subscription<OperationModeState>(
    "/system/operation_mode/state", state_qos,
    [&state_msg](const OperationModeState::SharedPtr msg) { state_msg = *msg; });

  ASSERT_TRUE(spin_until(
    executor_, [&state_msg]() { return state_msg.has_value(); }, std::chrono::seconds(2)));
  EXPECT_EQ(state_msg->mode, OperationModeState::STOP);
  EXPECT_FALSE(state_msg->is_autoware_control_enabled);
}

class NoVehicleServiceGateTest : public CommandGateRosIntegrationTest
{
protected:
  bool provide_vehicle_service() const override { return false; }
};

TEST_F(NoVehicleServiceGateTest, RejectsControlRequestWithoutVehicle)
{
  auto client = test_node_->create_client<SystemChangeAutowareControl>(
    "/system/operation_mode/change_autoware_control");
  ASSERT_TRUE(spin_until(
    executor_, [&client]() { return client->service_is_ready(); }, std::chrono::seconds(2)));

  auto request = std::make_shared<SystemChangeAutowareControl::Request>();
  request->autoware_control = true;
  auto future = client->async_send_request(request);
  ASSERT_TRUE(spin_until(
    executor_,
    [&future]() { return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready; },
    std::chrono::seconds(2)));
  const auto response = future.get();
  EXPECT_FALSE(response->status.success);
  EXPECT_EQ(response->status.code, autoware_common_msgs::msg::ResponseStatus::SERVICE_UNREADY);
}

class DeferredVehicleServiceGateTest : public CommandGateRosIntegrationTest
{
protected:
  bool defer_vehicle_response() const override { return true; }
};

TEST_F(DeferredVehicleServiceGateTest, TimesOutControlRequest)
{
  auto client = test_node_->create_client<SystemChangeAutowareControl>(
    "/system/operation_mode/change_autoware_control");
  ASSERT_TRUE(spin_until(
    executor_, [&client]() { return client->service_is_ready(); }, std::chrono::seconds(2)));

  auto request = std::make_shared<SystemChangeAutowareControl::Request>();
  request->autoware_control = true;
  auto future = client->async_send_request(request);
  ASSERT_TRUE(spin_until(
    executor_,
    [&future]() { return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready; },
    std::chrono::seconds(4)));
  const auto response = future.get();
  EXPECT_FALSE(response->status.success);
  EXPECT_EQ(response->status.code, autoware_common_msgs::msg::ResponseStatus::SERVICE_TIMEOUT);
  ASSERT_EQ(requested_vehicle_modes_.size(), 1U);
  EXPECT_EQ(requested_vehicle_modes_[0], ControlModeCommand::Request::AUTONOMOUS);
}

TEST_F(DeferredVehicleServiceGateTest, SuccessfulReplyDoesNotOverrideVehicleReport)
{
  std::optional<OperationModeState> state_msg;
  auto state_qos = rclcpp::QoS(1).reliable().transient_local();
  auto state_sub = test_node_->create_subscription<OperationModeState>(
    "/system/operation_mode/state", state_qos,
    [&state_msg](const OperationModeState::SharedPtr msg) { state_msg = *msg; });
  ASSERT_TRUE(spin_until(
    executor_, [&state_msg]() { return state_msg.has_value(); }, std::chrono::seconds(2)));
  ASSERT_TRUE(spin_until(
    executor_, [this]() { return vehicle_mode_pub_->get_subscription_count() > 0; },
    std::chrono::seconds(2)));

  ControlModeReport report;
  report.mode = ControlModeReport::AUTONOMOUS;
  vehicle_mode_pub_->publish(report);
  ASSERT_TRUE(spin_until(
    executor_, [&state_msg]() { return state_msg->is_autoware_control_enabled; },
    std::chrono::seconds(2)));

  auto client = test_node_->create_client<SystemChangeAutowareControl>(
    "/system/operation_mode/change_autoware_control");
  ASSERT_TRUE(spin_until(
    executor_, [&client]() { return client->service_is_ready(); }, std::chrono::seconds(2)));
  auto request = std::make_shared<SystemChangeAutowareControl::Request>();
  request->autoware_control = true;
  auto future = client->async_send_request(request);
  ASSERT_TRUE(spin_until(
    executor_, [this]() { return pending_vehicle_header_ != nullptr; }, std::chrono::seconds(2)));

  report.mode = ControlModeReport::MANUAL;
  vehicle_mode_pub_->publish(report);
  ASSERT_TRUE(spin_until(
    executor_, [&state_msg]() { return !state_msg->is_autoware_control_enabled; },
    std::chrono::seconds(2)));

  ControlModeCommand::Response vehicle_response;
  vehicle_response.success = true;
  vehicle_control_srv_->send_response(*pending_vehicle_header_, vehicle_response);
  ASSERT_TRUE(spin_until(
    executor_,
    [&future]() { return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready; },
    std::chrono::seconds(2)));
  EXPECT_TRUE(future.get()->status.success);
  spin_until(executor_, []() { return false; }, std::chrono::milliseconds(100));
  EXPECT_FALSE(state_msg->is_autoware_control_enabled);
  ASSERT_EQ(requested_vehicle_modes_.size(), 1U);
  EXPECT_EQ(requested_vehicle_modes_[0], ControlModeCommand::Request::AUTONOMOUS);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
