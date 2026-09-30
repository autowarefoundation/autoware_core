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

#include "operation_mode.hpp"

#include <autoware/adapi_specs/operation_mode.hpp>
#include <autoware/component_interface_specs/system.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_system_msgs/srv/change_autoware_control.hpp>
#include <autoware_system_msgs/srv/change_operation_mode.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace autoware::default_adapi
{

using AdapiOperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
using AdapiChangeOperationMode = autoware_adapi_v1_msgs::srv::ChangeOperationMode;
using SystemChangeOperationMode = autoware_system_msgs::srv::ChangeOperationMode;
using SystemChangeAutowareControl = autoware_system_msgs::srv::ChangeAutowareControl;

class OperationModeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    harness_ = std::make_shared<rclcpp::Node>("operation_mode_test_harness");

    // Mock system services
    srv_mock_mode_ = harness_->create_service<SystemChangeOperationMode>(
      "/system/operation_mode/change_operation_mode",
      [this](
        const SystemChangeOperationMode::Request::SharedPtr req,
        SystemChangeOperationMode::Response::SharedPtr res) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_requested_mode_ = req->mode;
        res->status.success = mock_mode_success_;
        res->status.code = mock_mode_code_;
        res->status.message = mock_mode_message_;
      });

    srv_mock_control_ = harness_->create_service<SystemChangeAutowareControl>(
      "/system/operation_mode/change_autoware_control",
      [this](
        const SystemChangeAutowareControl::Request::SharedPtr req,
        SystemChangeAutowareControl::Response::SharedPtr res) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_requested_control_ = req->autoware_control;
        res->status.success = mock_control_success_;
        res->status.code = mock_control_code_;
        res->status.message = mock_control_message_;
      });

    // Publisher for /system/operation_mode/state
    rclcpp::QoS system_state_qos(1);
    system_state_qos.reliable();
    system_state_qos.transient_local();
    pub_system_state_ = harness_->create_publisher<AdapiOperationModeState>(
      "/system/operation_mode/state", system_state_qos);

    // Subscriber for /api/operation_mode/state
    sub_api_state_ = harness_->create_subscription<AdapiOperationModeState>(
      "/api/operation_mode/state", system_state_qos,
      [this](const AdapiOperationModeState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_api_state_ = *msg;
        api_state_received_ = true;
      });

    // Service clients for AD API endpoints exposed by OperationModeNode
    cli_stop_ =
      harness_->create_client<AdapiChangeOperationMode>("/api/operation_mode/change_to_stop");
    cli_auto_ =
      harness_->create_client<AdapiChangeOperationMode>("/api/operation_mode/change_to_autonomous");
    cli_local_ =
      harness_->create_client<AdapiChangeOperationMode>("/api/operation_mode/change_to_local");
    cli_remote_ =
      harness_->create_client<AdapiChangeOperationMode>("/api/operation_mode/change_to_remote");
    cli_enable_ctrl_ = harness_->create_client<AdapiChangeOperationMode>(
      "/api/operation_mode/enable_autoware_control");
    cli_disable_ctrl_ = harness_->create_client<AdapiChangeOperationMode>(
      "/api/operation_mode/disable_autoware_control");

    // Create node under test
    node_ = std::make_shared<OperationModeNode>(rclcpp::NodeOptions{});

    // MultiThreadedExecutor to allow synchronous service calls from node to harness
    exec_ =
      std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions{}, 4);
    exec_->add_node(node_->get_node_base_interface());
    exec_->add_node(harness_);
    exec_thread_ = std::thread([this]() { exec_->spin(); });

    // Wait for all AD API service servers to be ready
    ASSERT_TRUE(cli_stop_->wait_for_service(std::chrono::seconds(3)));
    ASSERT_TRUE(cli_auto_->wait_for_service(std::chrono::seconds(3)));
    ASSERT_TRUE(cli_local_->wait_for_service(std::chrono::seconds(3)));
    ASSERT_TRUE(cli_remote_->wait_for_service(std::chrono::seconds(3)));
    ASSERT_TRUE(cli_enable_ctrl_->wait_for_service(std::chrono::seconds(3)));
    ASSERT_TRUE(cli_disable_ctrl_->wait_for_service(std::chrono::seconds(3)));
  }

  void TearDown() override
  {
    if (exec_) {
      exec_->cancel();
    }
    if (exec_thread_.joinable()) {
      exec_thread_.join();
    }
    node_.reset();
    harness_.reset();
    exec_.reset();
  }

  AdapiChangeOperationMode::Response::SharedPtr call_service(
    const rclcpp::Client<AdapiChangeOperationMode>::SharedPtr & client,
    const std::chrono::seconds timeout = std::chrono::seconds(2))
  {
    auto req = std::make_shared<AdapiChangeOperationMode::Request>();
    auto future = client->async_send_request(req);
    if (future.wait_for(timeout) == std::future_status::ready) {
      return future.get();
    }
    return nullptr;
  }

  std::shared_ptr<OperationModeNode> node_;
  std::shared_ptr<rclcpp::Node> harness_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;
  std::thread exec_thread_;

  rclcpp::Service<SystemChangeOperationMode>::SharedPtr srv_mock_mode_;
  rclcpp::Service<SystemChangeAutowareControl>::SharedPtr srv_mock_control_;
  rclcpp::Publisher<AdapiOperationModeState>::SharedPtr pub_system_state_;
  rclcpp::Subscription<AdapiOperationModeState>::SharedPtr sub_api_state_;

  rclcpp::Client<AdapiChangeOperationMode>::SharedPtr cli_stop_;
  rclcpp::Client<AdapiChangeOperationMode>::SharedPtr cli_auto_;
  rclcpp::Client<AdapiChangeOperationMode>::SharedPtr cli_local_;
  rclcpp::Client<AdapiChangeOperationMode>::SharedPtr cli_remote_;
  rclcpp::Client<AdapiChangeOperationMode>::SharedPtr cli_enable_ctrl_;
  rclcpp::Client<AdapiChangeOperationMode>::SharedPtr cli_disable_ctrl_;

  std::mutex mutex_;
  uint16_t last_requested_mode_{0};
  bool last_requested_control_{false};
  bool mock_mode_success_{true};
  uint32_t mock_mode_code_{0};
  std::string mock_mode_message_{"OK"};
  bool mock_control_success_{true};
  uint32_t mock_control_code_{0};
  std::string mock_control_message_{"OK"};

  AdapiOperationModeState last_api_state_;
  bool api_state_received_{false};
};

// Verifies that calling /api/operation_mode/change_to_stop forwards a STOP request to
// /system/operation_mode/change_operation_mode.
TEST_F(OperationModeTest, ChangeToStopSuccess)
{
  auto res = call_service(cli_stop_);
  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->status.success);
  EXPECT_EQ(res->status.code, 0u);

  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_EQ(last_requested_mode_, SystemChangeOperationMode::Request::STOP);
}

// Verifies that calling /api/operation_mode/change_to_autonomous forwards an AUTONOMOUS request to
// /system/operation_mode/change_operation_mode.
TEST_F(OperationModeTest, ChangeToAutonomousSuccess)
{
  auto res = call_service(cli_auto_);
  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->status.success);
  EXPECT_EQ(res->status.code, 0u);

  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_EQ(last_requested_mode_, SystemChangeOperationMode::Request::AUTONOMOUS);
}

// Verifies that calling /api/operation_mode/change_to_local forwards a LOCAL request to
// /system/operation_mode/change_operation_mode.
TEST_F(OperationModeTest, ChangeToLocalSuccess)
{
  auto res = call_service(cli_local_);
  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->status.success);
  EXPECT_EQ(res->status.code, 0u);

  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_EQ(last_requested_mode_, SystemChangeOperationMode::Request::LOCAL);
}

// Verifies that calling /api/operation_mode/change_to_remote forwards a REMOTE request to
// /system/operation_mode/change_operation_mode.
TEST_F(OperationModeTest, ChangeToRemoteSuccess)
{
  auto res = call_service(cli_remote_);
  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->status.success);
  EXPECT_EQ(res->status.code, 0u);

  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_EQ(last_requested_mode_, SystemChangeOperationMode::Request::REMOTE);
}

// Verifies that /api/operation_mode/enable_autoware_control is rejected with ERROR_NOT_AVAILABLE
// when the current system mode is UNKNOWN/unready.
TEST_F(OperationModeTest, EnableAutowareControlBlockedWhenModeUnknown)
{
  // Initially, before any system state update, curr_state_.mode is UNKNOWN (not available)
  auto res = call_service(cli_enable_ctrl_);
  ASSERT_NE(res, nullptr);
  EXPECT_FALSE(res->status.success);
  EXPECT_EQ(
    res->status.code,
    autoware_adapi_v1_msgs::srv::ChangeOperationMode::Response::ERROR_NOT_AVAILABLE);
}

// Verifies that /api/operation_mode/enable_autoware_control forwards autoware_control=true to
// /system/operation_mode/change_autoware_control once a valid mode is active.
TEST_F(OperationModeTest, EnableAutowareControlSuccessAfterStateSet)
{
  // Publish state with mode = STOP (available)
  AdapiOperationModeState state;
  state.stamp = harness_->now();
  state.mode = AdapiOperationModeState::STOP;
  state.is_stop_mode_available = true;
  pub_system_state_->publish(state);

  // Allow node to process state update
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto res = call_service(cli_enable_ctrl_);
  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->status.success);
  EXPECT_EQ(res->status.code, 0u);

  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_TRUE(last_requested_control_);
}

// Verifies that /api/operation_mode/disable_autoware_control forwards autoware_control=false to
// /system/operation_mode/change_autoware_control.
TEST_F(OperationModeTest, DisableAutowareControlSuccess)
{
  auto res = call_service(cli_disable_ctrl_);
  ASSERT_NE(res, nullptr);
  EXPECT_TRUE(res->status.success);
  EXPECT_EQ(res->status.code, 0u);

  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_FALSE(last_requested_control_);
}

// Verifies that /system/operation_mode/state updates are correctly ingested, synthesized with mode
// availability, and published to /api/operation_mode/state.
TEST_F(OperationModeTest, StateRelayAndSynthesis)
{
  AdapiOperationModeState sys_state;
  sys_state.stamp = harness_->now();
  sys_state.mode = AdapiOperationModeState::AUTONOMOUS;
  sys_state.is_autoware_control_enabled = true;
  sys_state.is_in_transition = false;
  sys_state.is_stop_mode_available = true;
  sys_state.is_autonomous_mode_available = true;
  sys_state.is_local_mode_available = false;
  sys_state.is_remote_mode_available = true;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    api_state_received_ = false;
  }

  pub_system_state_->publish(sys_state);

  // Wait for state to be published on /api/operation_mode/state
  for (int i = 0; i < 50; ++i) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (api_state_received_) {
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  std::lock_guard<std::mutex> lock(mutex_);
  ASSERT_TRUE(api_state_received_);
  EXPECT_EQ(last_api_state_.mode, AdapiOperationModeState::AUTONOMOUS);
  EXPECT_TRUE(last_api_state_.is_autoware_control_enabled);
  EXPECT_FALSE(last_api_state_.is_in_transition);
  EXPECT_TRUE(last_api_state_.is_stop_mode_available);
  EXPECT_TRUE(last_api_state_.is_autonomous_mode_available);
  EXPECT_FALSE(last_api_state_.is_local_mode_available);
  EXPECT_TRUE(last_api_state_.is_remote_mode_available);
}

// Verifies that error codes and messages returned by the system mode service are propagated back
// to the AD API caller.
TEST_F(OperationModeTest, ModeChangeErrorPropagation)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    mock_mode_success_ = false;
    mock_mode_code_ = 42;
    mock_mode_message_ = "System mode change error";
  }

  auto res = call_service(cli_auto_);
  ASSERT_NE(res, nullptr);
  EXPECT_FALSE(res->status.success);
  EXPECT_EQ(res->status.code, 42u);
  EXPECT_EQ(res->status.message, "System mode change error");
}

// Verifies that error codes and messages returned by the system control service are propagated
// back to the AD API caller.
TEST_F(OperationModeTest, ControlChangeErrorPropagation)
{
  // First ensure mode is valid
  AdapiOperationModeState state;
  state.stamp = harness_->now();
  state.mode = AdapiOperationModeState::STOP;
  pub_system_state_->publish(state);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  {
    std::lock_guard<std::mutex> lock(mutex_);
    mock_control_success_ = false;
    mock_control_code_ = 99;
    mock_control_message_ = "System control rejected";
  }

  auto res = call_service(cli_enable_ctrl_);
  ASSERT_NE(res, nullptr);
  EXPECT_FALSE(res->status.success);
  EXPECT_EQ(res->status.code, 99u);
  EXPECT_EQ(res->status.message, "System control rejected");
}

}  // namespace autoware::default_adapi

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  const int result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
