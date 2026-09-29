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

#include "../src/twist2accel.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

namespace
{
using autoware::twist2accel::Twist2Accel;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using geometry_msgs::msg::TwistWithCovarianceStamped;
using nav_msgs::msg::Odometry;

class Twist2AccelNodeTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }

  void create_nodes(bool use_odom)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"use_odom", use_odom}, {"accel_lowpass_gain", 0.9}});
    node_ = std::make_shared<Twist2Accel>(options);
    helper_ = std::make_shared<rclcpp::Node>("twist2accel_test_helper");

    sub_ = helper_->create_subscription<AccelWithCovarianceStamped>(
      "output/accel", 10,
      [this](const AccelWithCovarianceStamped::SharedPtr) { ++received_count_; });
    odom_pub_ = helper_->create_publisher<Odometry>("input/odom", 10);
    twist_pub_ = helper_->create_publisher<TwistWithCovarianceStamped>("input/twist", 10);

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
    executor_->add_node(helper_);

    wait_for_discovery();
  }

  void wait_for_discovery()
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (
      (odom_pub_->get_subscription_count() == 0 || twist_pub_->get_subscription_count() == 0) &&
      std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some(std::chrono::milliseconds(10));
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  void wait_for_receive_output(std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (received_count_ < 1u && std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some(std::chrono::milliseconds(10));
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  std::shared_ptr<Twist2Accel> node_;
  std::shared_ptr<rclcpp::Node> helper_;
  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<TwistWithCovarianceStamped>::SharedPtr twist_pub_;
  rclcpp::Subscription<AccelWithCovarianceStamped>::SharedPtr sub_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::size_t received_count_ = 0;
};

TEST_F(Twist2AccelNodeTest, TwistInputProducesOutput)
{
  create_nodes(/*use_odom=*/false);

  twist_pub_->publish(TwistWithCovarianceStamped{});
  wait_for_receive_output(std::chrono::seconds(1));

  EXPECT_EQ(received_count_, 1u);
}

TEST_F(Twist2AccelNodeTest, OdomInputProducesOutputWhenUseOdomTrue)
{
  create_nodes(/*use_odom=*/true);

  odom_pub_->publish(Odometry{});
  wait_for_receive_output(std::chrono::seconds(1));

  EXPECT_EQ(received_count_, 1u);
}

TEST_F(Twist2AccelNodeTest, OdomInputIgnoredWhenUseOdomFalse)
{
  create_nodes(/*use_odom=*/false);

  odom_pub_->publish(Odometry{});
  wait_for_receive_output(std::chrono::milliseconds(200));

  EXPECT_EQ(received_count_, 0u);
}
}  // namespace
