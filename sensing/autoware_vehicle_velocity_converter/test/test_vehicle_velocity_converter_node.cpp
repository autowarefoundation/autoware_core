// Copyright 2026 TIER IV
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

#include "../src/vehicle_velocity_converter_node.hpp"

#include <autoware_utils_geometry/msg/covariance.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

using COV_IDX = autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;

TEST(VehicleVelocityConverterNodeTest, TestConversion)
{
  // Initialize ROS 2 context
  rclcpp::init(0, nullptr);

  // Variable to hold received messages
  std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped> received_twist;
  bool twist_received = false;
  std::mutex msg_mutex;

  // Subscription to receive output messages
  std::shared_ptr<rclcpp::Node> test_control_node =
    std::make_shared<rclcpp::Node>("test_control_node");
  auto twist_subscription =
    test_control_node->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "twist_with_covariance", 10,
      [&](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(msg_mutex);
        received_twist = msg;
        twist_received = true;
      });

  // Create vehicle velocity converter node and register subscriptions
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {{"frame_id", "base_link"},
     {"velocity_stddev_xx", 0.2},
     {"angular_velocity_stddev_zz", 0.1},
     {"speed_scale_factor", 1.5}});
  std::shared_ptr<autoware::vehicle_velocity_converter::VehicleVelocityConverterNode>
    vehicle_velocity_converter_node =
      std::make_shared<autoware::vehicle_velocity_converter::VehicleVelocityConverterNode>(options);

  // Create executor and register nodes
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(vehicle_velocity_converter_node);
  executor.add_node(test_control_node);

  // Run executor in separate thread
  std::thread executor_thread([&]() { executor.spin(); });

  // Create publisher for input messages
  auto publisher = test_control_node->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
    "velocity_status", 10);

  // Wait for connection to be established
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Create and publish a velocity report test message
  auto report_msg = autoware_vehicle_msgs::msg::VelocityReport();
  report_msg.header.frame_id = "base_link";
  report_msg.header.stamp = rclcpp::Clock().now();
  report_msg.longitudinal_velocity = 2.0F;
  report_msg.lateral_velocity = 0.1F;
  report_msg.heading_rate = 0.3F;
  publisher->publish(report_msg);

  // Wait for messages to be received
  auto start_time = std::chrono::steady_clock::now();
  while (!twist_received &&
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
  ASSERT_TRUE(twist_received) << "Twist message was not received within timeout";
  ASSERT_NE(received_twist, nullptr);

  // Longitudinal velocity scaled by speed_scale_factor; lateral and yaw mapped directly.
  EXPECT_DOUBLE_EQ(received_twist->twist.twist.linear.x, 2.0F * 1.5);
  EXPECT_DOUBLE_EQ(received_twist->twist.twist.linear.y, static_cast<double>(0.1F));
  EXPECT_DOUBLE_EQ(received_twist->twist.twist.angular.z, static_cast<double>(0.3F));

  // Diagonal covariance entries come from the configured standard deviations.
  EXPECT_DOUBLE_EQ(received_twist->twist.covariance[COV_IDX::X_X], 0.2 * 0.2);
  EXPECT_DOUBLE_EQ(received_twist->twist.covariance[COV_IDX::YAW_YAW], 0.1 * 0.1);

  // Header is copied verbatim from the input report.
  EXPECT_EQ(received_twist->header.frame_id, "base_link");
}
