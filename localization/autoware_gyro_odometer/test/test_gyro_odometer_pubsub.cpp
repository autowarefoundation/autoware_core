// Copyright 2023 TIER IV, Inc.
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

#include "gyro_odometer_core.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

/*
 * This test checks if twist is published from gyro_odometer
 */
using geometry_msgs::msg::TwistWithCovarianceStamped;
using sensor_msgs::msg::Imu;

namespace
{
Imu generate_sample_imu()
{
  Imu imu;
  imu.header.frame_id = "base_link";
  imu.angular_velocity.x = 0.1;
  imu.angular_velocity.y = 0.2;
  imu.angular_velocity.z = 0.3;
  return imu;
}

TwistWithCovarianceStamped generate_sample_velocity()
{
  TwistWithCovarianceStamped twist;
  twist.header.frame_id = "base_link";
  twist.twist.twist.linear.x = 1.0;
  return twist;
}

rclcpp::NodeOptions get_node_options_with_default_params()
{
  rclcpp::NodeOptions node_options;

  // for gyro_odometer
  node_options.append_parameter_override("output_frame", "base_link");
  node_options.append_parameter_override("message_timeout_sec", 1e12);
  return node_options;
}
}  // namespace

bool is_twist_valid(
  const TwistWithCovarianceStamped & twist, const TwistWithCovarianceStamped & twist_ground_truth)
{
  if (twist.twist.twist.linear.x != twist_ground_truth.twist.twist.linear.x) {
    return false;
  }
  if (twist.twist.twist.linear.y != twist_ground_truth.twist.twist.linear.y) {
    return false;
  }
  if (twist.twist.twist.linear.z != twist_ground_truth.twist.twist.linear.z) {
    return false;
  }
  if (twist.twist.twist.angular.x != twist_ground_truth.twist.twist.angular.x) {
    return false;
  }
  if (twist.twist.twist.angular.y != twist_ground_truth.twist.twist.angular.y) {
    return false;
  }
  if (twist.twist.twist.angular.z != twist_ground_truth.twist.twist.angular.z) {
    return false;
  }
  return true;
}

class GyroOdometerNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();

    test_control_node_ = std::make_shared<rclcpp::Node>("test_control_node");
    twist_subscription_ = test_control_node_->create_subscription<TwistWithCovarianceStamped>(
      "/twist_with_covariance", 1, [this](const TwistWithCovarianceStamped::SharedPtr message) {
        std::lock_guard<std::mutex> lock(message_mutex_);
        received_twist_ = message;
      });
    imu_publisher_ = test_control_node_->create_publisher<Imu>(
      "/imu", rclcpp::QoS{1}.reliable().transient_local());
    vehicle_twist_publisher_ = test_control_node_->create_publisher<TwistWithCovarianceStamped>(
      "/vehicle/twist_with_covariance", rclcpp::QoS{1}.reliable().transient_local());
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

  // Bring up the gyro_odometer node and start spinning both nodes.
  void start_gyro_odometer_node()
  {
    gyro_odometer_node_ = std::make_shared<autoware::gyro_odometer::GyroOdometerNode>(
      get_node_options_with_default_params());
    executor_->add_node(gyro_odometer_node_->get_node_base_interface());
    executor_thread_ = std::thread([this]() { executor_->spin(); });

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  TwistWithCovarianceStamped::SharedPtr received_twist()
  {
    std::lock_guard<std::mutex> lock(message_mutex_);
    return received_twist_;
  }

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  rclcpp::Node::SharedPtr test_control_node_;
  rclcpp::Subscription<TwistWithCovarianceStamped>::SharedPtr twist_subscription_;
  rclcpp::Publisher<Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<TwistWithCovarianceStamped>::SharedPtr vehicle_twist_publisher_;
  std::shared_ptr<autoware::gyro_odometer::GyroOdometerNode> gyro_odometer_node_;

  std::mutex message_mutex_;
  TwistWithCovarianceStamped::SharedPtr received_twist_;
};

// IMU & Velocity test
// Verify that the gyro_odometer successfully publishes the fused twist message when both IMU and
// velocity data are provided
TEST_F(GyroOdometerNodeTest, TestGyroOdometerWithImuAndVelocity)
{
  Imu input_imu = generate_sample_imu();
  TwistWithCovarianceStamped input_velocity = generate_sample_velocity();

  TwistWithCovarianceStamped expected_output_twist;
  expected_output_twist.twist.twist.linear.x = input_velocity.twist.twist.linear.x;
  expected_output_twist.twist.twist.angular.x = input_imu.angular_velocity.x;
  expected_output_twist.twist.twist.angular.y = input_imu.angular_velocity.y;
  expected_output_twist.twist.twist.angular.z = input_imu.angular_velocity.z;

  start_gyro_odometer_node();

  // TODO(youtalk): Remove these after the refinement of the GyroOdometerNode
  vehicle_twist_publisher_->publish(input_velocity);
  imu_publisher_->publish(input_imu);

  vehicle_twist_publisher_->publish(input_velocity);
  imu_publisher_->publish(input_imu);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  EXPECT_FALSE(received_twist() == nullptr);
  EXPECT_TRUE(is_twist_valid(*received_twist(), expected_output_twist));
}

// IMU-only test
// Verify that the gyro_odometer does NOT publish any outputs when only IMU is provided
TEST_F(GyroOdometerNodeTest, TestGyroOdometerImuOnly)
{
  Imu input_imu = generate_sample_imu();

  start_gyro_odometer_node();

  imu_publisher_->publish(input_imu);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  EXPECT_TRUE(received_twist() == nullptr);
}
