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
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <thread>

namespace
{
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_adapi_v1_msgs::srv::ChangeOperationMode;
using autoware_vehicle_msgs::msg::GearCommand;

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
    component_manager_ = std::make_shared<rclcpp_components::ComponentManager>();
    const auto resources = component_manager_->get_component_resources("autoware_command_gate");
    auto resource_it = std::find_if(resources.begin(), resources.end(), [](const auto & resource) {
      return resource.first == "autoware::control::command_gate::AutowareCommandGateNode";
    });
    ASSERT_TRUE(resource_it != resources.end());
    factory_ = component_manager_->create_component_factory(*resource_it);
    wrapper_ = std::make_unique<rclcpp_components::NodeInstanceWrapper>(
      factory_->create_node_instance(rclcpp::NodeOptions{}));
    component_node_base_ = wrapper_->get_node_base_interface();

    executor_.add_node(test_node_);
    executor_.add_node(component_node_base_);
  }

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
};

TEST_F(CommandGateRosIntegrationTest, ChangeToStopPublishesStateAndGear)
{
  std::optional<OperationModeState> state_msg;
  std::optional<GearCommand> gear_msg;

  rclcpp::QoS state_qos(1);
  state_qos.reliable();
  state_qos.transient_local();

  auto state_sub = test_node_->create_subscription<OperationModeState>(
    "/api/operation_mode/state", state_qos,
    [&state_msg](const OperationModeState::SharedPtr msg) { state_msg = *msg; });
  auto gear_sub = test_node_->create_subscription<GearCommand>(
    "/control/command/gear_cmd", rclcpp::QoS{1},
    [&gear_msg](const GearCommand::SharedPtr msg) { gear_msg = *msg; });

  auto client =
    test_node_->create_client<ChangeOperationMode>("/api/operation_mode/change_to_stop");
  ASSERT_TRUE(spin_until(
    executor_, [&client]() { return client->wait_for_service(std::chrono::seconds(0)); },
    std::chrono::seconds(2)));

  auto request = std::make_shared<ChangeOperationMode::Request>();
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
    "/api/operation_mode/state", state_qos,
    [&state_msg](const OperationModeState::SharedPtr msg) { state_msg = *msg; });
  auto gear_sub = test_node_->create_subscription<GearCommand>(
    "/control/command/gear_cmd", rclcpp::QoS{1},
    [&gear_msg](const GearCommand::SharedPtr msg) { gear_msg = *msg; });

  auto client =
    test_node_->create_client<ChangeOperationMode>("/api/operation_mode/change_to_autonomous");
  ASSERT_TRUE(spin_until(
    executor_, [&client]() { return client->wait_for_service(std::chrono::seconds(0)); },
    std::chrono::seconds(2)));

  auto request = std::make_shared<ChangeOperationMode::Request>();
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
  EXPECT_TRUE(state_msg->is_autoware_control_enabled);
  EXPECT_FALSE(state_msg->is_in_transition);
  EXPECT_TRUE(state_msg->is_stop_mode_available);
  EXPECT_TRUE(state_msg->is_autonomous_mode_available);
  EXPECT_TRUE(state_msg->is_local_mode_available);
  EXPECT_TRUE(state_msg->is_remote_mode_available);

  EXPECT_EQ(gear_msg->command, GearCommand::DRIVE);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
