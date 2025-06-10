// Copyright 2021 TierIV
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

#include "../src/vehicle_velocity_converter.hpp"

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

using autoware::vehicle_velocity_converter::VehicleVelocityConverter;
using autoware_vehicle_msgs::msg::VelocityReport;
using geometry_msgs::msg::TwistWithCovarianceStamped;
using std::chrono::milliseconds;
using std::chrono::system_clock;

class TestVehicleVelocityConverter : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize ROS context for test
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    // Create a test node
    node_ptr_ = std::make_shared<rclcpp::Node>("test_vehicle_velocity_converter");

    // Create a publisher for velocity report
    velocity_pub_ = node_ptr_->create_publisher<VelocityReport>("velocity_status", 10);

    // Create a subscription to receive the converted twist
    twist_sub_ = node_ptr_->create_subscription<TwistWithCovarianceStamped>(
      "twist_with_covariance", 10,
      [this](const TwistWithCovarianceStamped::SharedPtr msg) { received_twist_ = msg; });
  }

  void TearDown() override
  {
    node_ptr_.reset();
    // Do not shutdown ROS context here as it's shared between tests
  }

  rclcpp::Node::SharedPtr node_ptr_{nullptr};
  rclcpp::Publisher<VelocityReport>::SharedPtr velocity_pub_{nullptr};
  rclcpp::Subscription<TwistWithCovarianceStamped>::SharedPtr twist_sub_{nullptr};
  TwistWithCovarianceStamped::SharedPtr received_twist_{nullptr};

  void spinSome() { rclcpp::spin_some(node_ptr_); }

  bool waitForTwistMessage(
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds(1000))
  {
    const auto start_time = system_clock::now();
    received_twist_ = nullptr;

    while (rclcpp::ok() && !received_twist_) {
      spinSome();
      if (system_clock::now() - start_time > timeout) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return received_twist_ != nullptr;
  }
};

TEST_F(TestVehicleVelocityConverter, NodeInstantiation)
{
  // Test node creation with default parameters
  rclcpp::NodeOptions options;
  options.parameter_overrides().push_back(rclcpp::Parameter("frame_id", "base_link"));
  options.parameter_overrides().push_back(rclcpp::Parameter("velocity_stddev_xx", 0.2));
  options.parameter_overrides().push_back(rclcpp::Parameter("angular_velocity_stddev_zz", 0.1));
  options.parameter_overrides().push_back(rclcpp::Parameter("speed_scale_factor", 1.0));

  auto vehicle_velocity_converter = std::make_shared<VehicleVelocityConverter>(options);
  EXPECT_NE(vehicle_velocity_converter, nullptr);
}

TEST_F(TestVehicleVelocityConverter, MessageConversion)
{
  // Start the node with parameters
  rclcpp::NodeOptions options;
  options.parameter_overrides().push_back(rclcpp::Parameter("frame_id", "base_link"));
  options.parameter_overrides().push_back(rclcpp::Parameter("velocity_stddev_xx", 0.2));
  options.parameter_overrides().push_back(rclcpp::Parameter("angular_velocity_stddev_zz", 0.1));
  options.parameter_overrides().push_back(rclcpp::Parameter("speed_scale_factor", 1.5));

  auto vehicle_velocity_converter = std::make_shared<VehicleVelocityConverter>(options);

  // Create and publish a velocity report message
  auto velocity_msg = std::make_shared<VelocityReport>();
  velocity_msg->header.frame_id = "base_link";
  velocity_msg->header.stamp = node_ptr_->now();
  velocity_msg->longitudinal_velocity = 2.0;
  velocity_msg->lateral_velocity = 0.1;
  velocity_msg->heading_rate = 0.3;

  // Use a separate thread for executor to process messages
  std::thread executor_thread([&]() {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(vehicle_velocity_converter);
    auto start_time = system_clock::now();
    while (system_clock::now() - start_time < std::chrono::seconds(2)) {
      executor.spin_some(std::chrono::milliseconds(100));
    }
  });

  // Wait a moment for the executor to set up
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Publish the message
  velocity_pub_->publish(*velocity_msg);

  // Wait for the converted message
  EXPECT_TRUE(waitForTwistMessage());

  // Check that the conversion was correct
  EXPECT_EQ(received_twist_->header.frame_id, velocity_msg->header.frame_id);
  EXPECT_EQ(received_twist_->twist.twist.linear.x, velocity_msg->longitudinal_velocity * 1.5);
  EXPECT_EQ(received_twist_->twist.twist.linear.y, velocity_msg->lateral_velocity);
  EXPECT_EQ(received_twist_->twist.twist.angular.z, velocity_msg->heading_rate);

  // Check covariance values
  EXPECT_EQ(received_twist_->twist.covariance[0], 0.2 * 0.2);
  EXPECT_EQ(received_twist_->twist.covariance[7], 10000.0);
  EXPECT_EQ(received_twist_->twist.covariance[14], 10000.0);
  EXPECT_EQ(received_twist_->twist.covariance[21], 10000.0);
  EXPECT_EQ(received_twist_->twist.covariance[28], 10000.0);
  EXPECT_EQ(received_twist_->twist.covariance[35], 0.1 * 0.1);

  // Join the executor thread
  executor_thread.join();
}

TEST_F(TestVehicleVelocityConverter, DifferentFrameId)
{
  // Start the node with parameters
  rclcpp::NodeOptions options;
  options.parameter_overrides().push_back(rclcpp::Parameter("frame_id", "base_link"));
  options.parameter_overrides().push_back(rclcpp::Parameter("velocity_stddev_xx", 0.2));
  options.parameter_overrides().push_back(rclcpp::Parameter("angular_velocity_stddev_zz", 0.1));
  options.parameter_overrides().push_back(rclcpp::Parameter("speed_scale_factor", 1.0));

  auto vehicle_velocity_converter = std::make_shared<VehicleVelocityConverter>(options);

  // Create and publish a velocity report message with a different frame_id
  auto velocity_msg = std::make_shared<VelocityReport>();
  velocity_msg->header.frame_id = "different_frame";  // Not base_link
  velocity_msg->header.stamp = node_ptr_->now();
  velocity_msg->longitudinal_velocity = 2.0;
  velocity_msg->lateral_velocity = 0.1;
  velocity_msg->heading_rate = 0.3;

  // Use a separate thread for executor to process messages
  std::thread executor_thread([&]() {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(vehicle_velocity_converter);
    auto start_time = system_clock::now();
    while (system_clock::now() - start_time < std::chrono::seconds(2)) {
      executor.spin_some(std::chrono::milliseconds(100));
    }
  });

  // Wait a moment for the executor to set up
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Publish the message
  velocity_pub_->publish(*velocity_msg);

  // Wait for the converted message
  EXPECT_TRUE(waitForTwistMessage());

  // Even with different frame_id, the conversion should still work
  EXPECT_EQ(received_twist_->header.frame_id, velocity_msg->header.frame_id);
  EXPECT_EQ(received_twist_->twist.twist.linear.x, velocity_msg->longitudinal_velocity);
  EXPECT_EQ(received_twist_->twist.twist.linear.y, velocity_msg->lateral_velocity);
  EXPECT_EQ(received_twist_->twist.twist.angular.z, velocity_msg->heading_rate);

  // Join the executor thread
  executor_thread.join();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
