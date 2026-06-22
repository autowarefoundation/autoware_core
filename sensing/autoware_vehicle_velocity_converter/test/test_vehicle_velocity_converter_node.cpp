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
#include <vector>

using COV_IDX = autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;

namespace
{
autoware_vehicle_msgs::msg::VelocityReport make_velocity_report(
  const double longitudinal_velocity, const double lateral_velocity, const double heading_rate)
{
  autoware_vehicle_msgs::msg::VelocityReport report;
  report.header.frame_id = "base_link";
  report.longitudinal_velocity = static_cast<float>(longitudinal_velocity);
  report.lateral_velocity = static_cast<float>(lateral_velocity);
  report.heading_rate = static_cast<float>(heading_rate);
  return report;
}
}  // namespace

// Drives the node over real publish/subscribe so each test body stays a plain Arrange/Act/Assert:
// the fixture owns the ROS context, the executor thread and the test-side pub/sub wiring.
class VehicleVelocityConverterNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();

    test_control_node_ = std::make_shared<rclcpp::Node>("test_control_node");
    twist_subscription_ =
      test_control_node_->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "twist_with_covariance", 10,
        [this](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr message) {
          std::lock_guard<std::mutex> lock(message_mutex_);
          received_twist_ = message;
        });
    report_publisher_ =
      test_control_node_->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
        "velocity_status", 10);
    executor_->add_node(test_control_node_);
  }

  void TearDown() override
  {
    if (executor_) {
      executor_->cancel();
    }
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
    rclcpp::shutdown();
  }

  // Bring up the converter node with the given parameters and start spinning both nodes.
  void start_converter_node(const std::vector<rclcpp::Parameter> & parameter_overrides)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(parameter_overrides);
    converter_node_ =
      std::make_shared<autoware::vehicle_velocity_converter::VehicleVelocityConverterNode>(options);
    executor_->add_node(converter_node_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });

    // Poll until both endpoints have discovered each other instead of sleeping a fixed duration,
    // because DDS discovery time varies across environments and CI machines.
    const auto discovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < discovery_deadline) {
      if (
        report_publisher_->get_subscription_count() > 0 &&
        twist_subscription_->get_publisher_count() > 0) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  // Publish a report and return the converted twist, or nullptr if none arrives within the timeout.
  geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr publish_and_wait(
    const autoware_vehicle_msgs::msg::VelocityReport & report)
  {
    report_publisher_->publish(report);

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(message_mutex_);
        if (received_twist_) {
          return received_twist_;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::lock_guard<std::mutex> lock(message_mutex_);
    return received_twist_;
  }

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  rclcpp::Node::SharedPtr test_control_node_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    twist_subscription_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr report_publisher_;
  std::shared_ptr<autoware::vehicle_velocity_converter::VehicleVelocityConverterNode>
    converter_node_;

  std::mutex message_mutex_;
  geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr received_twist_;
};

TEST_F(VehicleVelocityConverterNodeTest, ConvertsVelocityReportToTwist)
{
  // Arrange
  start_converter_node(
    {{"frame_id", "base_link"},
     {"velocity_stddev_xx", 0.2},
     {"angular_velocity_stddev_zz", 0.1},
     {"speed_scale_factor", 1.5}});
  const auto report = make_velocity_report(2.0, 0.1, 0.3);

  // Act
  const auto twist = publish_and_wait(report);

  // Assert
  ASSERT_NE(twist, nullptr) << "Twist message was not received within timeout";

  // Longitudinal velocity scaled by speed_scale_factor; lateral and yaw mapped directly.
  // The report fields are float, so the converted values are compared at float precision.
  EXPECT_FLOAT_EQ(twist->twist.twist.linear.x, 2.0 * 1.5);
  EXPECT_FLOAT_EQ(twist->twist.twist.linear.y, 0.1);
  EXPECT_FLOAT_EQ(twist->twist.twist.angular.z, 0.3);

  // Diagonal covariance entries come from the configured standard deviations.
  EXPECT_DOUBLE_EQ(twist->twist.covariance[COV_IDX::X_X], 0.2 * 0.2);
  EXPECT_DOUBLE_EQ(twist->twist.covariance[COV_IDX::YAW_YAW], 0.1 * 0.1);

  // Header is copied verbatim from the input report.
  EXPECT_EQ(twist->header.frame_id, "base_link");
}
