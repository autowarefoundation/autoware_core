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

#include "../src/pose_initializer_core.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <autoware_internal_localization_msgs/srv/initialize_localization.hpp>
#include <autoware_internal_localization_msgs/srv/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <thread>
#include <vector>

using autoware::pose_initializer::PoseInitializer;
using InitializeLocalization =
  autoware_internal_localization_msgs::srv::InitializeLocalization;  // Legacy type handling
using RequestPoseAlignment = autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using TwistWithCovarianceStamped = geometry_msgs::msg::TwistWithCovarianceStamped;
using InitializationState = autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
using SetBool = std_srvs::srv::SetBool;

class PoseInitializerNodeIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    rclcpp::NodeOptions options;
    options.append_parameter_override("ekf_enabled", true);
    options.append_parameter_override("gnss_enabled", true);
    options.append_parameter_override("ndt_enabled", true);
    options.append_parameter_override("yabloc_enabled", false);
    options.append_parameter_override("stop_check_enabled", true);
    options.append_parameter_override("stop_check_duration", 0.5);
    options.append_parameter_override("gnss_pose_timeout", 3.0);
    options.append_parameter_override("pose_error_check_enabled", true);
    options.append_parameter_override("pose_error_threshold", 5.0);
    options.append_parameter_override("user_defined_initial_pose.enable", false);
    options.append_parameter_override(
      "user_defined_initial_pose.pose", std::vector<double>{0, 0, 0, 0, 0, 0, 1});

    const std::vector<double> cov(36, 0.01);  // Satisfy the 36-element array requirement
    options.append_parameter_override("output_pose_covariance", cov);
    options.append_parameter_override("gnss_particle_covariance", cov);

    node_ = std::make_shared<PoseInitializer>(options);
    harness_ = std::make_shared<rclcpp::Node>("test_harness");

    // Harness pub/sub
    pub_gnss_ = harness_->create_publisher<PoseWithCovarianceStamped>("gnss_pose_cov", 1);
    pub_twist_ = harness_->create_publisher<TwistWithCovarianceStamped>("stop_check_twist", 1);

    sub_reset_ = harness_->create_subscription<PoseWithCovarianceStamped>(
      "pose_reset", 1,
      [this](PoseWithCovarianceStamped::ConstSharedPtr msg) { last_reset_pose_ = msg; });
    sub_state_ = harness_->create_subscription<InitializationState>(
      "/localization/initialization_state", 10,
      [this](InitializationState::ConstSharedPtr msg) { last_state_ = msg; });

    // Harness service mocks
    srv_ndt_align_ = harness_->create_service<RequestPoseAlignment>(
      "ndt_align", [this](
                     const std::shared_ptr<RequestPoseAlignment::Request> req,
                     std::shared_ptr<RequestPoseAlignment::Response> res) {
        res->success = mock_align_success_;
        res->pose_with_covariance = req->pose_with_covariance;
        res->pose_with_covariance.pose.pose.position.x += 1.0;  // Simulate an alignment shift
      });

    auto trigger_callback =
      [this](const std::shared_ptr<SetBool::Request> req, std::shared_ptr<SetBool::Response> res) {
        trigger_calls_++;
        res->success = mock_trigger_success_;
      };
    srv_ekf_trigger_ = harness_->create_service<SetBool>("ekf_trigger_node", trigger_callback);
    srv_ndt_trigger_ = harness_->create_service<SetBool>("ndt_trigger_node", trigger_callback);

    // Background executor to spin both node and harness
    // Prevents deadlocks
    exec_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    exec_->add_node(node_);
    exec_->add_node(harness_);
    exec_thread_ = std::thread([this]() { exec_->spin(); });
  }

  void TearDown() override
  {
    exec_->cancel();
    if (exec_thread_.joinable()) {
      exec_thread_.join();
    }
    rclcpp::shutdown();
  }

  // Helper to publish stopped twist data for a while
  // Simulates stationary ego
  void simulate_vehicle_stopped(double duration_sec)
  {
    const rclcpp::Time start_time = harness_->now();
    rclcpp::Rate rate(10);  // 10 Hz
    while ((harness_->now() - start_time).seconds() < duration_sec) {
      TwistWithCovarianceStamped twist;
      twist.header.stamp = harness_->now();
      twist.header.frame_id = "base_link";
      twist.twist.twist.linear.x = 0.0;  // Stationary
      pub_twist_->publish(twist);
      rate.sleep();
    }
  }

  std::shared_ptr<PoseInitializer> node_;
  std::shared_ptr<rclcpp::Node> harness_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;
  std::thread exec_thread_;

  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pub_gnss_;
  rclcpp::Publisher<TwistWithCovarianceStamped>::SharedPtr pub_twist_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_reset_;
  rclcpp::Subscription<InitializationState>::SharedPtr sub_state_;

  rclcpp::Service<RequestPoseAlignment>::SharedPtr srv_ndt_align_;
  rclcpp::Service<SetBool>::SharedPtr srv_ekf_trigger_;
  rclcpp::Service<SetBool>::SharedPtr srv_ndt_trigger_;

  PoseWithCovarianceStamped::ConstSharedPtr last_reset_pose_ = nullptr;
  InitializationState::ConstSharedPtr last_state_ = nullptr;

  std::atomic<bool> mock_align_success_{true};
  std::atomic<bool> mock_trigger_success_{true};
  std::atomic<int> trigger_calls_{0};
};

TEST_F(PoseInitializerNodeIntegrationTest, DirectInitBypassAligners)
{
  auto cli_init = harness_->create_client<InitializeLocalization>("/localization/initialize");
  ASSERT_TRUE(cli_init->wait_for_service(std::chrono::seconds(2)));

  auto req = std::make_shared<InitializeLocalization::Request>();
  req->method = InitializeLocalization::Request::DIRECT;

  PoseWithCovarianceStamped initial_pose;
  initial_pose.header.frame_id = "map";
  initial_pose.pose.pose.position.x = 100.0;
  initial_pose.pose.pose.orientation.w = 1.0;
  req->pose_with_covariance.push_back(initial_pose);

  auto future = cli_init->async_send_request(req);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto res = future.get();

  EXPECT_TRUE(res->status.success);

  // Wait for pub to register output
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  ASSERT_NE(last_reset_pose_, nullptr) << "Failed to publish reset pose!";
  EXPECT_DOUBLE_EQ(last_reset_pose_->pose.pose.position.x, 100.0);

  // Expect trigger calls
  // (deactivate + activate) x 2 for both EKF and NDT = 4 calls
  EXPECT_EQ(trigger_calls_.load(), 4);
}
