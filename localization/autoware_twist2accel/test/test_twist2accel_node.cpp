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

#include <array>
#include <chrono>
#include <cstddef>
#include <memory>
#include <thread>
#include <vector>

namespace
{
using autoware::twist2accel::Twist2Accel;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using geometry_msgs::msg::TwistWithCovarianceStamped;
using nav_msgs::msg::Odometry;

// Spin both nodes until `predicate` holds or the timeout elapses, so the tests
// stay deterministic instead of relying on a fixed sleep.
template <typename Predicate>
void spin_until(
  rclcpp::executors::SingleThreadedExecutor & executor, const Predicate & predicate,
  std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!predicate() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(std::chrono::milliseconds(10));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

class Twist2AccelNodeTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }

  // Build the node under test with the given use_odom routing and spin it
  // together with a helper node that publishes inputs and captures outputs.
  std::shared_ptr<Twist2Accel> make_node(bool use_odom)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"use_odom", use_odom}, {"accel_lowpass_gain", 0.9}});
    return std::make_shared<Twist2Accel>(options);
  }
};

// First message (prev_twist_ == nullopt): the node publishes an
// AccelWithCovarianceStamped whose header is copied from the input but whose
// accel and covariance are left at their default zeros -- no estimate is run
// because there is no previous sample to difference against.
TEST_F(Twist2AccelNodeTest, FirstMessagePublishesZeroAccelWithHeader)
{
  auto node = make_node(/*use_odom=*/true);
  auto helper = std::make_shared<rclcpp::Node>("twist2accel_test_helper");

  std::vector<AccelWithCovarianceStamped> received;
  auto sub = helper->create_subscription<AccelWithCovarianceStamped>(
    "output/accel", 10,
    [&received](const AccelWithCovarianceStamped::SharedPtr msg) { received.push_back(*msg); });
  auto pub = helper->create_publisher<Odometry>("input/odom", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(helper);

  spin_until(executor, [&] { return pub->get_subscription_count() > 0; }, std::chrono::seconds(5));

  Odometry odom;
  odom.header.frame_id = "base_link";
  odom.header.stamp = rclcpp::Time(123, 456, RCL_ROS_TIME);
  odom.twist.twist.linear.x = 5.0;
  odom.twist.twist.angular.z = 1.0;
  pub->publish(odom);

  spin_until(executor, [&] { return !received.empty(); }, std::chrono::seconds(5));

  ASSERT_EQ(received.size(), 1u);
  const auto & out = received.front();
  // Header is forwarded from the input message.
  EXPECT_EQ(out.header.frame_id, "base_link");
  EXPECT_EQ(rclcpp::Time(out.header.stamp), rclcpp::Time(123, 456, RCL_ROS_TIME));
  // No estimate ran, so accel is all zeros ...
  EXPECT_DOUBLE_EQ(out.accel.accel.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(out.accel.accel.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(out.accel.accel.linear.z, 0.0);
  EXPECT_DOUBLE_EQ(out.accel.accel.angular.x, 0.0);
  EXPECT_DOUBLE_EQ(out.accel.accel.angular.y, 0.0);
  EXPECT_DOUBLE_EQ(out.accel.accel.angular.z, 0.0);
  // ... and the covariance is left untouched (all zeros).
  for (const auto & c : out.accel.covariance) {
    EXPECT_DOUBLE_EQ(c, 0.0);
  }
}

// With use_odom=true the odom callback drives estimation while the twist
// callback early-returns. The second odom sample (now that prev_twist_ is set)
// produces a non-zero accel, and the interleaved twist message produces no
// extra output.
TEST_F(Twist2AccelNodeTest, UseOdomTrueRoutesOdomAndIgnoresTwist)
{
  auto node = make_node(/*use_odom=*/true);
  auto helper = std::make_shared<rclcpp::Node>("twist2accel_test_helper");

  std::vector<AccelWithCovarianceStamped> received;
  auto sub = helper->create_subscription<AccelWithCovarianceStamped>(
    "output/accel", 10,
    [&received](const AccelWithCovarianceStamped::SharedPtr msg) { received.push_back(*msg); });
  auto odom_pub = helper->create_publisher<Odometry>("input/odom", 10);
  auto twist_pub = helper->create_publisher<TwistWithCovarianceStamped>("input/twist", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(helper);

  spin_until(
    executor,
    [&] {
      return odom_pub->get_subscription_count() > 0 && twist_pub->get_subscription_count() > 0;
    },
    std::chrono::seconds(5));

  // First odom: linear.x = 0 at t = 0. Primes prev_twist_, publishes zero accel.
  Odometry odom0;
  odom0.header.frame_id = "base_link";
  odom0.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  odom0.twist.twist.linear.x = 0.0;
  odom_pub->publish(odom0);
  spin_until(executor, [&] { return received.size() >= 1u; }, std::chrono::seconds(5));

  // A twist message must be ignored while use_odom=true: no new output.
  TwistWithCovarianceStamped twist;
  twist.header.frame_id = "base_link";
  twist.header.stamp = rclcpp::Time(0, 500000000, RCL_ROS_TIME);
  twist.twist.twist.linear.x = 100.0;
  twist_pub->publish(twist);
  spin_until(executor, [&] { return false; }, std::chrono::milliseconds(300));
  EXPECT_EQ(received.size(), 1u) << "twist input must be ignored when use_odom=true";

  // Second odom: linear.x = 1 at t = 1 s. dt = 1 s, raw accel = 1.0, first LPF
  // sample returns it unchanged -> linear.x accel == 1.0.
  Odometry odom1;
  odom1.header.frame_id = "base_link";
  odom1.header.stamp = rclcpp::Time(1, 0, RCL_ROS_TIME);
  odom1.twist.twist.linear.x = 1.0;
  odom_pub->publish(odom1);
  spin_until(executor, [&] { return received.size() >= 2u; }, std::chrono::seconds(5));

  ASSERT_EQ(received.size(), 2u);
  EXPECT_DOUBLE_EQ(received.back().accel.accel.linear.x, 1.0);

  // Characterization: the published covariance must stay byte-for-byte what the
  // node emitted before the covariance logic moved into the pure core, i.e. a
  // constant diagonal (linear variance 1.0, angular variance 0.05) with all
  // off-diagonal terms zero. The literals below are hand-written, not read back
  // from the estimator. Diagonal of a row-major 6x6 lives at row * 6 + row.
  const auto & cov = received.back().accel.covariance;
  std::array<double, 36> expected{};  // value-initialized to all zeros
  expected[0] = 1.0;                  // linear x variance
  expected[7] = 1.0;                  // linear y variance
  expected[14] = 1.0;                 // linear z variance
  expected[21] = 0.05;                // roll variance
  expected[28] = 0.05;                // pitch variance
  expected[35] = 0.05;                // yaw variance
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_DOUBLE_EQ(cov[i], expected[i]) << "covariance index " << i;
  }
}

// With use_odom=false the twist callback drives estimation while the odom
// callback early-returns. Symmetric to the use_odom=true case.
TEST_F(Twist2AccelNodeTest, UseOdomFalseRoutesTwistAndIgnoresOdom)
{
  auto node = make_node(/*use_odom=*/false);
  auto helper = std::make_shared<rclcpp::Node>("twist2accel_test_helper");

  std::vector<AccelWithCovarianceStamped> received;
  auto sub = helper->create_subscription<AccelWithCovarianceStamped>(
    "output/accel", 10,
    [&received](const AccelWithCovarianceStamped::SharedPtr msg) { received.push_back(*msg); });
  auto odom_pub = helper->create_publisher<Odometry>("input/odom", 10);
  auto twist_pub = helper->create_publisher<TwistWithCovarianceStamped>("input/twist", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(helper);

  spin_until(
    executor,
    [&] {
      return odom_pub->get_subscription_count() > 0 && twist_pub->get_subscription_count() > 0;
    },
    std::chrono::seconds(5));

  // First twist: linear.x = 0 at t = 0. Primes prev_twist_, publishes zero accel.
  TwistWithCovarianceStamped twist0;
  twist0.header.frame_id = "base_link";
  twist0.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  twist0.twist.twist.linear.x = 0.0;
  twist_pub->publish(twist0);
  spin_until(executor, [&] { return received.size() >= 1u; }, std::chrono::seconds(5));

  // An odom message must be ignored while use_odom=false: no new output.
  Odometry odom;
  odom.header.frame_id = "base_link";
  odom.header.stamp = rclcpp::Time(0, 500000000, RCL_ROS_TIME);
  odom.twist.twist.linear.x = 100.0;
  odom_pub->publish(odom);
  spin_until(executor, [&] { return false; }, std::chrono::milliseconds(300));
  EXPECT_EQ(received.size(), 1u) << "odom input must be ignored when use_odom=false";

  // Second twist: linear.x = 1 at t = 1 s. dt = 1 s, raw accel = 1.0.
  TwistWithCovarianceStamped twist1;
  twist1.header.frame_id = "base_link";
  twist1.header.stamp = rclcpp::Time(1, 0, RCL_ROS_TIME);
  twist1.twist.twist.linear.x = 1.0;
  twist_pub->publish(twist1);
  spin_until(executor, [&] { return received.size() >= 2u; }, std::chrono::seconds(5));

  ASSERT_EQ(received.size(), 2u);
  EXPECT_DOUBLE_EQ(received.back().accel.accel.linear.x, 1.0);
}
}  // namespace
