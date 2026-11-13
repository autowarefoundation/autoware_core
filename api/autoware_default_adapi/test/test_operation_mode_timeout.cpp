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

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/response_status.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_system_msgs/srv/change_autoware_control.hpp>
#include <autoware_system_msgs/srv/change_operation_mode.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <thread>

namespace
{
using ApiService = autoware_adapi_v1_msgs::srv::ChangeOperationMode;
using ModeService = autoware_system_msgs::srv::ChangeOperationMode;
using ControlService = autoware_system_msgs::srv::ChangeAutowareControl;
using State = autoware_adapi_v1_msgs::msg::OperationModeState;

bool wait_until(const std::function<bool()> & predicate)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return predicate();
}

class OperationModeTimeoutTest : public ::testing::TestWithParam<const char *>
{
protected:
  void SetUp() override
  {
    const auto options =
      rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    api_ = std::make_shared<autoware::default_adapi::CoreOperationModeNode>(options);
    node_ = std::make_shared<rclcpp::Node>("operation_mode_timeout_test");
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    state_pub_ = node_->create_publisher<State>("/system/operation_mode/state", qos);
    state_sub_ = node_->create_subscription<State>(
      "/api/operation_mode/state", qos,
      [this](const State::ConstSharedPtr msg) { observed_mode_ = msg->mode; });
    client_ = node_->create_client<ApiService>(std::string("/api/operation_mode/") + GetParam());
    mode_server_ = node_->create_service<ModeService>(
      "/system/operation_mode/change_operation_mode",
      [this](
        std::shared_ptr<rclcpp::Service<ModeService>>, std::shared_ptr<rmw_request_id_t>,
        ModeService::Request::SharedPtr) { request_received_ = true; });
    control_server_ = node_->create_service<ControlService>(
      "/system/operation_mode/change_autoware_control",
      [this](
        std::shared_ptr<rclcpp::Service<ControlService>>, std::shared_ptr<rmw_request_id_t>,
        ControlService::Request::SharedPtr) { request_received_ = true; });
    executor_.add_node(node_);
    executor_.add_node(api_->get_node_base_interface());
    spin_thread_ = std::thread([this]() { executor_.spin(); });

    ASSERT_TRUE(client_->wait_for_service(std::chrono::seconds(5)));
    State state;
    state.mode = State::STOP;
    state.is_stop_mode_available = true;
    state.is_autonomous_mode_available = true;
    state_pub_->publish(state);
    ASSERT_TRUE(wait_until([this]() { return observed_mode_ == State::STOP; }));
  }

  void TearDown() override
  {
    executor_.cancel();
    spin_thread_.join();
    executor_.remove_node(api_->get_node_base_interface());
    executor_.remove_node(node_);
  }

  rclcpp::executors::MultiThreadedExecutor executor_{rclcpp::ExecutorOptions{}, 2};
  std::thread spin_thread_;
  std::shared_ptr<autoware::default_adapi::CoreOperationModeNode> api_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<State>::SharedPtr state_pub_;
  rclcpp::Subscription<State>::SharedPtr state_sub_;
  rclcpp::Client<ApiService>::SharedPtr client_;
  rclcpp::Service<ModeService>::SharedPtr mode_server_;
  rclcpp::Service<ControlService>::SharedPtr control_server_;
  std::atomic<bool> request_received_{false};
  std::atomic<State::_mode_type> observed_mode_{State::UNKNOWN};
};

TEST_P(OperationModeTimeoutTest, BackendLossReturnsTimeoutAndAllowsRecovery)
{
  auto future = client_->async_send_request(std::make_shared<ApiService::Request>());
  ASSERT_TRUE(wait_until([this]() { return request_received_.load(); }));
  // Remove the backend after it accepts the request, without sending a response.
  mode_server_.reset();
  control_server_.reset();
  ASSERT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  const auto response = future.get();
  EXPECT_FALSE(response->status.success);
  EXPECT_EQ(response->status.code, autoware_adapi_v1_msgs::msg::ResponseStatus::SERVICE_TIMEOUT);

  mode_server_ = node_->create_service<ModeService>(
    "/system/operation_mode/change_operation_mode",
    [](ModeService::Request::SharedPtr, ModeService::Response::SharedPtr res) {
      res->status.success = true;
    });
  control_server_ = node_->create_service<ControlService>(
    "/system/operation_mode/change_autoware_control",
    [](ControlService::Request::SharedPtr, ControlService::Response::SharedPtr res) {
      res->status.success = true;
    });
  // State relay shares the public services' callback group and must also recover.
  State state;
  state.mode = State::AUTONOMOUS;
  state_pub_->publish(state);
  ASSERT_TRUE(wait_until([this]() { return observed_mode_ == State::AUTONOMOUS; }));
  ASSERT_TRUE(wait_until([this]() {
    auto retry = client_->async_send_request(std::make_shared<ApiService::Request>());
    return retry.wait_for(std::chrono::seconds(4)) == std::future_status::ready &&
           retry.get()->status.success;
  }));
}

INSTANTIATE_TEST_SUITE_P(
  PublicServices, OperationModeTimeoutTest,
  ::testing::Values("change_to_autonomous", "enable_autoware_control", "disable_autoware_control"));
}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
