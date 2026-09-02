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

#include "src/ekf_localizer_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace
{
// Floating point tolerance at EXPECT_NEAR and similar checks
constexpr float near_tol = 1e-2F;
}  // namespace

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
    options.parameter_overrides({
      {"use_sim_time", true},
      {"node.show_debug_info", false},
      {"node.enable_yaw_bias_estimation", true},
      {"node.predict_frequency", 50.0},
      {"node.tf_rate", 50.0},
      {"node.extend_state_step", 50},
      {"misc.pose_frame_id", std::string("map")},
      {"pose_measurement.pose_additional_delay", 0.0},
      {"pose_measurement.pose_measure_uncertainty_time", 0.01},
      {"pose_measurement.pose_smoothing_steps", 5},
      {"pose_measurement.max_pose_queue_size", 5},
      {"pose_measurement.pose_gate_dist", 49.5},
      {"twist_measurement.twist_additional_delay", 0.0},
      {"twist_measurement.twist_smoothing_steps", 2},
      {"twist_measurement.max_twist_queue_size", 2},
      {"twist_measurement.twist_gate_dist", 46.1},
      {"process_noise.proc_stddev_vx_c", 10.0},
      {"process_noise.proc_stddev_wz_c", 5.0},
      {"process_noise.proc_stddev_yaw_c", 0.005},
      {"simple_1d_filter_parameters.z_filter_proc_dev", 5.0},
      {"simple_1d_filter_parameters.roll_filter_proc_dev", 0.1},
      {"simple_1d_filter_parameters.pitch_filter_proc_dev", 0.1},
      {"diagnostics.pose_no_update_count_threshold_warn", 50},
      {"diagnostics.pose_no_update_count_threshold_error", 100},
      {"diagnostics.twist_no_update_count_threshold_warn", 50},
      {"diagnostics.twist_no_update_count_threshold_error", 100},
      {"diagnostics.ellipse_scale", 3.0},
      {"diagnostics.error_ellipse_size", 1.5},
      {"diagnostics.warn_ellipse_size", 1.2},
      {"diagnostics.error_ellipse_size_lateral_direction", 0.3},
      {"diagnostics.warn_ellipse_size_lateral_direction", 0.25},
      {"diagnostics.diagnostics_publish_frequency", 10.0},
      {"misc.threshold_observable_velocity_mps", 0.0},
    });

    node_ = std::make_shared<EKFLocalizerNode>(options);
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

    // Identity TF
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(test_node_);
    geometry_msgs::msg::TransformStamped static_tf;
    static_tf.header.stamp = test_node_->now();
    static_tf.header.frame_id = "earth";
    static_tf.child_frame_id = "map";
    static_tf.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(static_tf);

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

  void spin_executor()
  {
    for (int i = 0; i < 3; ++i) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  void publish_clock()
  {
    rosgraph_msgs::msg::Clock msg;
    msg.clock = current_time_;
    clock_pub_->publish(msg);
    spin_executor();
  }

  void step_time(double dt_seconds)
  {
    current_time_ = current_time_ + rclcpp::Duration::from_seconds(dt_seconds);
    publish_clock();
  }

  void trigger_node()
  {
    ASSERT_TRUE(client_trigger_->wait_for_service(std::chrono::seconds(1)));
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = true;
    auto future = client_trigger_->async_send_request(req);
    executor_->spin_until_future_complete(future);
    ASSERT_TRUE(future.get()->success);
    spin_executor();
  }

  geometry_msgs::msg::PoseWithCovarianceStamped make_pose(double x = 0.0, double y = 0.0)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = current_time_;
    pose.header.frame_id = "map";
    pose.pose.pose.position.x = x;
    pose.pose.pose.position.y = y;
    pose.pose.pose.orientation.w = 1.0;

    // Mathematically safe init array
    pose.pose.covariance.fill(0.0);
    pose.pose.covariance[0] = 0.01;   // X
    pose.pose.covariance[7] = 0.01;   // Y
    pose.pose.covariance[14] = 0.01;  // Z
    pose.pose.covariance[21] = 0.01;  // Roll
    pose.pose.covariance[28] = 0.01;  // Pitch
    pose.pose.covariance[35] = 0.01;  // Yaw
    return pose;
  }

  geometry_msgs::msg::TwistWithCovarianceStamped make_twist(double vx = 0.0, double wz = 0.0)
  {
    geometry_msgs::msg::TwistWithCovarianceStamped twist;
    twist.header.stamp = current_time_;
    twist.header.frame_id = "base_link";
    twist.twist.twist.linear.x = vx;
    twist.twist.twist.angular.z = wz;

    // Also a safe covariance array
    twist.twist.covariance.fill(0.0);
    twist.twist.covariance[0] = 0.1;
    twist.twist.covariance[35] = 0.1;
    return twist;
  }

  // Checks if latest diagnostics available (contains a substring)
  bool has_diagnostic(const std::string & substr)
  {
    if (!latest_diag_) return false;
    for (const auto & status : latest_diag_->status) {
      if (status.message.find(substr) != std::string::npos) {
        return true;
      }
    }
    return false;
  }

  // Tick some, wait for diagnostics containing certain substring
  bool poll_for_diagnostic(const std::string & substr, int max_ticks)
  {
    for (int i = 0; i < max_ticks; ++i) {
      step_time(0.02);
      if (has_diagnostic(substr)) {
        return true;
      }
    }
    return false;
  }

  void spin_once_tick_once()
  {
    spin_executor();  // Flush queue before ticking
    step_time(0.02);  // Tick once (50 Hz)
  }

  std::shared_ptr<EKFLocalizerNode> node_;
  std::shared_ptr<rclcpp::Node> test_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

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

// ================== TESTING AREA HERE ==================

// TEST 1. Confirms node correctly performs pose initialization properly.
// Expects:
// - Node should not publish odometry until it receives a trigger and an init pose.
// - After receiving a trigger and an init pose, node should publish odometry exactly at that init
// pose.
// This test will:
// 1. Brief step time 0.15 sec (expect no odometry published).
// 2. Trigger node init (still expect no odometry published).
// 3. Brief step time 0.1 sec (expect diagnostics to report error due to missing init pose).
// 4. Publish an init pose.
// 5. Very brief step time 0.02 sec (50 Hz) (expect odometry published at that pose).
TEST_F(EKFLocalizerIntegrationHarness, GatekeeperInitialization)
{
  // 1. Expects node should do nothing without trigger
  step_time(0.15);
  EXPECT_EQ(odom_count_, 0U);

  // 2. Trigger node init
  trigger_node();

  // 3. Diagnostics should report error due to missing init pose
  latest_diag_ = nullptr;
  EXPECT_TRUE(poll_for_diagnostic("[ERROR]initial pose is not set", 10))
    << "Node failed to guard against missing initial pose.";

  // 4. Send init pose
  pub_initial_pose_->publish(make_pose(10.0, 20.0));
  spin_once_tick_once();

  // 5. Expects odometry now being published at exact init coordinates
  ASSERT_NE(latest_odom_, nullptr);
  EXPECT_NEAR(latest_odom_->pose.pose.position.x, 10.0, near_tol);
  EXPECT_NEAR(latest_odom_->pose.pose.position.y, 20.0, near_tol);
}

// TEST 2. Confirms node correctly handles timeouts and cascaded WARN => ERROR diagnostics.
// Expects node to emit WARN at 50 ticks of no pose updates, and ERROR at 100 ticks of no pose
// updates. This test will:
// 1. Trigger node init.
// 2. Publish an init pose (0.0, 0.0, 0.0) in map frame.
// 3. Advance time 48 ticks (expect no WARN yet).
// 4. Advance time 1 more tick (expect WARN).
// 5. Advance time 50 more ticks (expect ERROR).
TEST_F(EKFLocalizerIntegrationHarness, TimeoutCascade)
{
  // Boot
  trigger_node();

  geometry_msgs::msg::PoseWithCovarianceStamped init_pose = make_pose(0.0, 0.0);
  pub_initial_pose_->publish(init_pose);
  spin_once_tick_once();

  // ============ 1. Advance 40 ticks (threshold is 50 for WARN) ============

  for (int i = 0; i < 40; ++i) {
    step_time(0.02);
  }

  // Expects node to not WARN yet
  EXPECT_FALSE(has_diagnostic("[WARN]pose is not updated"))
    << "Node failed to shut up before tick 50.";

  // ============ 2. Advance 30 more tick to exceed 50 but not yet 100 (WARN state) ============

  EXPECT_TRUE(poll_for_diagnostic("[WARN]pose is not updated", 30))
    << "Node failed to WARN at 50 missed updates.";

  // ============ 3. Advance much more ticks to exceed 100 (ERROR state) ============

  EXPECT_TRUE(poll_for_diagnostic("[ERROR]pose is not updated", 70))
    << "Node failed to ERROR at 100 missed updates.";
}

}  // namespace autoware::ekf_localizer
