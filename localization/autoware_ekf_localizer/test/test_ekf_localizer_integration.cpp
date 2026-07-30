// Copyright 2026 TIER IV, Inc.
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

#include "include/ekf_localizer.hpp"

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>
namespace autoware::ekf_localizer
{

class EKFLocalizerIntegrationHarness : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    // Force node into simulated time mode for test suite's ticks
    rclcpp::NodeOptions options;
    options.append_parameter_override("use_sim_time", true);

    // Stricter thresholds for stricter tests
    options.append_parameter_override("node.predict_frequency", 50.0);
    options.append_parameter_override("diagnostics.diagnostics_publish_frequency", 10.0);

    node_ = std::make_shared<EKFLocalizer>(options);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());

    // Integration test nodes for I/O
    test_node_ = std::make_shared<rclcpp::Node>("ekf_integration_test_node", options);
    executor_->add_node(test_node_->get_node_base_interface());

    clock_pub_ = test_node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    pub_initial_pose_ = test_node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10);
    pub_pose_ = test_node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/in_pose_with_covariance", 10);
    pub_twist_ = test_node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/in_twist_with_covariance", 10);

    client_trigger_ = test_node_->create_client<std_srvs::srv::SetBool>("/trigger_node_srv");

    sub_odom_ = test_node_->create_subscription<nav_msgs::msg::Odometry>(
      "/ekf_odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = msg;
        odom_count_++;
      });

    sub_diag_ = test_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10,
      [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) { latest_diag_ = msg; });

    // Start time at 100.0s to avoid 0.0s edge cases
    current_time_ = rclcpp::Time(100, 0, RCL_ROS_TIME);
    publish_clock();
  }

  void TearDown() override
  {
    executor_->cancel();
    executor_->remove_node(node_->get_node_base_interface());
    executor_->remove_node(test_node_->get_node_base_interface());
    node_.reset();
    test_node_.reset();
    rclcpp::shutdown();
  }

  void publish_clock()
  {
    rosgraph_msgs::msg::Clock msg;
    msg.clock = current_time_;
    clock_pub_->publish(msg);
    executor_->spin_some();
  }

  void step_time(double dt_seconds)
  {
    current_time_ = current_time_ + rclcpp::Duration::from_seconds(dt_seconds);
    publish_clock();
  }

  std::shared_ptr<EKFLocalizer> node_;
  std::shared_ptr<rclcpp::Node> test_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initial_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_trigger_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_diag_;

  rclcpp::Time current_time_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_ = nullptr;
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr latest_diag_ = nullptr;
  size_t odom_count_ = 0;
};

}  // namespace autoware::ekf_localizer
