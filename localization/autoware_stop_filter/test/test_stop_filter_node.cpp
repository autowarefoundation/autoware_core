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

#include <chrono>
#include <memory>
#include <utility>

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

// End-to-end node test fixture.
//
// This replaces the previously DISABLED_ real-time test (background spin thread +
// wall-clock sleeps) with a deterministic SingleThreadedExecutor / spin_some loop:
// one Odometry message is published, the executor is pumped with spin_some() until
// both outputs arrive (bounded by a generous wall-clock safety net rather than a
// fixed sleep), and the published values are asserted.
class StopFilterNodeTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }

  // Publishes a single Odometry message into the StopFilterNode, deterministically
  // pumps the executor, and returns the (filtered odometry, stop flag) outputs.
  std::pair<nav_msgs::msg::Odometry, autoware_internal_debug_msgs::msg::BoolStamped> run_once(
    double vx_threshold, double wz_threshold, const nav_msgs::msg::Odometry::SharedPtr & input)
  {
    nav_msgs::msg::Odometry received_odom;
    autoware_internal_debug_msgs::msg::BoolStamped received_stop_flag;
    bool odom_received = false;
    bool stop_flag_received = false;

    auto test_node = std::make_shared<rclcpp::Node>("test_stop_filter_helper_node");
    auto odom_sub = test_node->create_subscription<nav_msgs::msg::Odometry>(
      "output/odom", 10, [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
        received_odom = *msg;
        odom_received = true;
      });
    auto stop_flag_sub =
      test_node->create_subscription<autoware_internal_debug_msgs::msg::BoolStamped>(
        "debug/stop_flag", 10,
        [&](const autoware_internal_debug_msgs::msg::BoolStamped::SharedPtr msg) {
          received_stop_flag = *msg;
          stop_flag_received = true;
        });
    auto publisher = test_node->create_publisher<nav_msgs::msg::Odometry>("input/odom", 10);

    rclcpp::NodeOptions options;
    options.parameter_overrides({{"vx_threshold", vx_threshold}, {"wz_threshold", wz_threshold}});
    auto stop_filter_node = std::make_shared<autoware::stop_filter::StopFilterNode>(options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(stop_filter_node);
    executor.add_node(test_node);

    // Pump until the pub/sub graph is fully matched in BOTH directions before
    // publishing, otherwise the node may emit its outputs in the input callback
    // before the test's subscriptions are connected and the messages get dropped:
    //   - the test's input/odom publisher must see the node's subscription, and
    //   - the test's output/odom and debug/stop_flag subscriptions must each see
    //     the node's publisher.
    const auto discovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while ((publisher->get_subscription_count() == 0 || odom_sub->get_publisher_count() == 0 ||
            stop_flag_sub->get_publisher_count() == 0) &&
           std::chrono::steady_clock::now() < discovery_deadline) {
      executor.spin_some();
    }
    EXPECT_GT(publisher->get_subscription_count(), 0u)
      << "StopFilterNode did not subscribe to input/odom";
    EXPECT_GT(odom_sub->get_publisher_count(), 0u)
      << "StopFilterNode did not advertise output/odom";
    EXPECT_GT(stop_flag_sub->get_publisher_count(), 0u)
      << "StopFilterNode did not advertise debug/stop_flag";

    publisher->publish(*input);

    // Use a fresh deadline for the receive phase so discovery time does not eat
    // into the receive timeout.
    const auto receive_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while ((!odom_received || !stop_flag_received) &&
           std::chrono::steady_clock::now() < receive_deadline) {
      executor.spin_some();
    }

    EXPECT_TRUE(odom_received) << "output/odom message was not received";
    EXPECT_TRUE(stop_flag_received) << "debug/stop_flag message was not received";

    return {received_odom, received_stop_flag};
  }
};

TEST_F(StopFilterNodeTest, ZeroesTwistAndRaisesFlagWhenStopped)
{
  // Velocities below the configured thresholds -> treated as stopped.
  auto input = createOdometryMessage(0.05, 0.0, 0.0, 0.0, 0.0, 0.05);
  auto [odom, stop_flag] = run_once(0.1, 0.1, input);

  // Longitudinal/yaw twist is zeroed and the stop flag is raised.
  EXPECT_EQ(odom.twist.twist.linear.x, 0.0);
  EXPECT_EQ(odom.twist.twist.linear.y, 0.0);
  EXPECT_EQ(odom.twist.twist.linear.z, 0.0);
  EXPECT_EQ(odom.twist.twist.angular.x, 0.0);
  EXPECT_EQ(odom.twist.twist.angular.y, 0.0);
  EXPECT_EQ(odom.twist.twist.angular.z, 0.0);
  EXPECT_TRUE(stop_flag.data);

  // Header is preserved end-to-end.
  EXPECT_EQ(odom.header.frame_id, input->header.frame_id);
  EXPECT_EQ(odom.header.stamp, input->header.stamp);
  EXPECT_EQ(stop_flag.stamp, input->header.stamp);
}

TEST_F(StopFilterNodeTest, PassesTwistThroughAndClearsFlagWhenMoving)
{
  // Velocities above the configured thresholds -> treated as moving.
  auto input = createOdometryMessage(0.2, 0.0, 0.0, 0.0, 0.0, 0.2);
  auto [odom, stop_flag] = run_once(0.1, 0.1, input);

  // Twist passes through unchanged and the stop flag stays low.
  EXPECT_EQ(odom.twist.twist.linear.x, 0.2);
  EXPECT_EQ(odom.twist.twist.angular.z, 0.2);
  EXPECT_FALSE(stop_flag.data);
  EXPECT_EQ(stop_flag.stamp, input->header.stamp);
}
