// Copyright 2025 TIER IV
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

#include "../src/stop_filter_node.hpp"

#include <gtest/gtest.h>

#include <memory>

nav_msgs::msg::Odometry::SharedPtr createOdometryMessage(
  double linear_x, double linear_y, double linear_z, double angular_x, double angular_y,
  double angular_z)
{
  auto msg = std::make_shared<nav_msgs::msg::Odometry>();
  msg->header.frame_id = "base_link";
  msg->header.stamp = rclcpp::Clock().now();
  msg->twist.twist.linear.x = linear_x;
  msg->twist.twist.linear.y = linear_y;
  msg->twist.twist.linear.z = linear_z;
  msg->twist.twist.angular.x = angular_x;
  msg->twist.twist.angular.y = angular_y;
  msg->twist.twist.angular.z = angular_z;
  return msg;
}

TEST(StopFilterProcessorTest, TestCreateStopFlagMsgMoving)
{
  // Create message with velocities above threshold (moving)
  auto message_filter_ = std::make_unique<autoware::stop_filter::StopFilterProcessor>(0.1, 0.1);
  auto input_msg = createOdometryMessage(0.2, 0.0, 0.0, 0.0, 0.0, 0.2);

  // Test stop flag creation
  auto stop_flag_msg = message_filter_->create_stop_flag_msg(input_msg);

  // Verify stop flag is false (vehicle is moving)
  ASSERT_FALSE(stop_flag_msg.data);
  ASSERT_EQ(stop_flag_msg.stamp, input_msg->header.stamp);
}

TEST(StopFilterProcessorTest, TestCreateFilteredMsgStopped)
{
  // Create message with velocities below threshold (stopped)
  auto message_filter_ = std::make_unique<autoware::stop_filter::StopFilterProcessor>(0.1, 0.1);
  auto input_msg = createOdometryMessage(0.05, 0.02, 0.01, 0.03, 0.04, 0.05);

  // Test filtered message creation
  auto filtered_msg = message_filter_->create_filtered_msg(input_msg);

  // Verify velocities are set to zero when stopped
  ASSERT_EQ(filtered_msg.twist.twist.linear.x, 0.0);
  ASSERT_EQ(filtered_msg.twist.twist.linear.y, 0.0);
  ASSERT_EQ(filtered_msg.twist.twist.linear.z, 0.0);
  ASSERT_EQ(filtered_msg.twist.twist.angular.x, 0.0);
  ASSERT_EQ(filtered_msg.twist.twist.angular.y, 0.0);
  ASSERT_EQ(filtered_msg.twist.twist.angular.z, 0.0);

  // Verify header is preserved
  ASSERT_EQ(filtered_msg.header.frame_id, input_msg->header.frame_id);
  ASSERT_EQ(filtered_msg.header.stamp, input_msg->header.stamp);
}

// Test for stop detection in StopFilterNode
// This test is disabled by default due to its reliance on real-time execution
// To run this test, you need to enable it manually by removing the DISABLED_ prefix
TEST(StopFilterNodeTest, DISABLED_TestStopDetection)
{
  // Initialize ROS 2 context
  rclcpp::init(0, nullptr);

  // Variable to hold received messages
  std::shared_ptr<nav_msgs::msg::Odometry> received_odom;
  std::shared_ptr<autoware_internal_debug_msgs::msg::BoolStamped> received_stop_flag;
  bool odom_received = false;
  bool stop_flag_received = false;
  std::mutex msg_mutex;

  // Subscription to receive output messages
  std::shared_ptr<rclcpp::Node> test_control_node =
    std::make_shared<rclcpp::Node>("test_control_node");
  auto odom_subscription = test_control_node->create_subscription<nav_msgs::msg::Odometry>(
    "output/odom", 10, [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(msg_mutex);
      received_odom = msg;
      odom_received = true;
    });

  auto stop_flag_subscription =
    test_control_node->create_subscription<autoware_internal_debug_msgs::msg::BoolStamped>(
      "debug/stop_flag", 10,
      [&](const autoware_internal_debug_msgs::msg::BoolStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(msg_mutex);
        received_stop_flag = msg;
        stop_flag_received = true;
      });

  // Create stop filter node and register subscriptions
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"vx_threshold", 1.0}, {"wz_threshold", 1.0}});
  std::shared_ptr<autoware::stop_filter::StopFilterNode> stop_filter_node =
    std::make_shared<autoware::stop_filter::StopFilterNode>(options);

  // Create executor and register nodes
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(stop_filter_node);
  executor.add_node(test_control_node);

  // Run executor in separate thread
  std::thread executor_thread([&]() { executor.spin(); });

  // Create publisher for input messages
  auto publisher = test_control_node->create_publisher<nav_msgs::msg::Odometry>("input/odom", 10);

  // Wait for connection to be established
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Create and publish stop state test message
  auto stop_msg = nav_msgs::msg::Odometry();
  stop_msg.header.frame_id = "base_link";
  stop_msg.header.stamp = rclcpp::Clock().now();
  stop_msg.twist.twist.linear.x = 0.2;   // below threshold
  stop_msg.twist.twist.angular.z = 0.2;  // below threshold
  publisher->publish(stop_msg);

  // Wait for messages to be received
  auto start_time = std::chrono::steady_clock::now();
  while ((!odom_received || !stop_flag_received) &&
         std::chrono::steady_clock::now() - start_time < std::chrono::seconds(4)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Stop executor
  executor.cancel();
  if (executor_thread.joinable()) {
    executor_thread.join();
  }
  rclcpp::shutdown();

  // Verify results
  ASSERT_TRUE(odom_received) << "Odometry message was not received within timeout";
  ASSERT_TRUE(stop_flag_received) << "Stop flag message was not received within timeout";
  ASSERT_NE(received_odom, nullptr);
  ASSERT_NE(received_stop_flag, nullptr);
  ASSERT_EQ(received_odom->twist.twist.linear.x, 0.0);
  ASSERT_EQ(received_odom->twist.twist.angular.z, 0.0);
  ASSERT_TRUE(received_stop_flag->data);
}
