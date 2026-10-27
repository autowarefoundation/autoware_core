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
#include <functional>
#include <memory>

namespace
{
struct PublishedMessages
{
  nav_msgs::msg::Odometry::SharedPtr filtered_odom;
  autoware_internal_debug_msgs::msg::BoolStamped::SharedPtr stop_flag;
};

nav_msgs::msg::Odometry create_odometry_message(const double linear_x, const double angular_z)
{
  nav_msgs::msg::Odometry msg;
  msg.twist.twist.linear.x = linear_x;
  msg.twist.twist.angular.z = angular_z;
  return msg;
}

// Spin the executor until the condition becomes true or the timeout of 2 seconds expires.
void spin_until(rclcpp::Executor & executor, const std::function<bool()> & condition)
{
  const auto timeout = std::chrono::seconds(2);
  const auto start_time = std::chrono::steady_clock::now();
  while (!condition() && std::chrono::steady_clock::now() - start_time < timeout) {
    executor.spin_once(std::chrono::milliseconds(10));
  }
}
}  // namespace

class StopFilterNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    test_node_ = std::make_shared<rclcpp::Node>("test_stop_filter_node");
    odom_subscription_ = test_node_->create_subscription<nav_msgs::msg::Odometry>(
      "output/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        received_messages_.filtered_odom = msg;
      });
    stop_flag_subscription_ =
      test_node_->create_subscription<autoware_internal_debug_msgs::msg::BoolStamped>(
        "debug/stop_flag", 10,
        [this](const autoware_internal_debug_msgs::msg::BoolStamped::SharedPtr msg) {
          received_messages_.stop_flag = msg;
        });
    odom_publisher_ = test_node_->create_publisher<nav_msgs::msg::Odometry>("input/odom", 10);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(test_node_);
  }

  void TearDown() override { rclcpp::shutdown(); }

  void start_stop_filter_node(const double vx_threshold, const double wz_threshold)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"vx_threshold", vx_threshold}, {"wz_threshold", wz_threshold}});
    stop_filter_node_ = std::make_shared<autoware::stop_filter::StopFilterNode>(options);
    executor_->add_node(stop_filter_node_->get_node_base_interface());

    spin_until(*executor_, [this]() {
      return odom_publisher_->get_subscription_count() > 0 &&
             odom_subscription_->get_publisher_count() > 0 &&
             stop_flag_subscription_->get_publisher_count() > 0;
    });
  }

  PublishedMessages receive_published_messages()
  {
    spin_until(*executor_, [this]() {
      return received_messages_.filtered_odom && received_messages_.stop_flag;
    });
    return received_messages_;
  }

  std::shared_ptr<rclcpp::Node> test_node_;
  std::shared_ptr<autoware::stop_filter::StopFilterNode> stop_filter_node_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<autoware_internal_debug_msgs::msg::BoolStamped>::SharedPtr
    stop_flag_subscription_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  PublishedMessages received_messages_;
};

TEST_F(StopFilterNodeTest, PublishesFilteredOdomAndTrueStopFlagWhenVelocityIsUnderThreshold)
{
  // Arrange
  start_stop_filter_node(1.0, 1.0);
  const auto input_odom = create_odometry_message(0.2, 0.2);

  // Act
  odom_publisher_->publish(input_odom);
  const auto result = receive_published_messages();

  // Assert
  ASSERT_NE(result.filtered_odom, nullptr);
  ASSERT_NE(result.stop_flag, nullptr);
  EXPECT_TRUE(result.stop_flag->data);
}
