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

#include <limits>
#include <memory>
#include <string>
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

  std::shared_ptr<EKFLocalizer> node_;
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
  ASSERT_TRUE(client_trigger_->wait_for_service(std::chrono::seconds(1)));
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = true;
  auto future = client_trigger_->async_send_request(req);
  executor_->spin_until_future_complete(future);
  ASSERT_TRUE(future.get()->success);

  // 3. Diagnostics should report error due to missing init pose
  step_time(0.15);
  ASSERT_NE(latest_diag_, nullptr);
  bool found_init_error = false;
  for (const auto & status : latest_diag_->status) {
    if (
      status.message.find("[ERROR]initial pose is not set") != std::string::npos &&
      status.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
      found_init_error = true;
    }
  }
  EXPECT_TRUE(found_init_error) << "Node failed to guard against missing initial pose.";

  // 4. Send init pose
  geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
  init_pose.header.stamp = current_time_;
  init_pose.header.frame_id = "map";
  init_pose.pose.pose.position.x = 10.0;
  init_pose.pose.pose.position.y = 20.0;
  init_pose.pose.pose.orientation.w = 1.0;

  // Also assign safe array
  init_pose.pose.covariance.fill(0.0);
  init_pose.pose.covariance[0] = 0.01;
  init_pose.pose.covariance[7] = 0.01;
  init_pose.pose.covariance[35] = 0.01;

  pub_initial_pose_->publish(init_pose);
  spin_executor();  // Make sure message clears middleware before tick
  step_time(0.02);  // 50Hz tick

  // 5. Expects odometry now being published at exact init coordinates
  ASSERT_NE(latest_odom_, nullptr);
  EXPECT_NEAR(latest_odom_->pose.pose.position.x, 10.0, near_tol);
  EXPECT_NEAR(latest_odom_->pose.pose.position.y, 20.0, near_tol);
}

// TEST 2. Confirms node correctly performs deterministic kinematics.
// Expects:
// - Node should publish odometry that moves 5.0 m in X after 1 second of
// constant velocity input (5.0 m/s in X, 0.0 rad/s in yaw).
// - Node should report covariance growth due to prediction step.
// This test will:
// 1. Trigger node init.
// 2. Publish an init pose (0.0, 0.0, 0.0) in map frame.
// 3. Publish a constant twist (5.0 m/s in X, 0.0 rad/s in yaw) for 1 second (50 ticks at 50Hz).
// 4. Expects:
// - Odometry to have moved 5.0 m in X, nothing in Y, hence new pose should be (5.0, 0.0, 0.0).
// - Covariance to have grown due to prediction step.
TEST_F(EKFLocalizerIntegrationHarness, DeterministicKinematics)
{
  // Boot up node
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = true;
  client_trigger_->async_send_request(req);
  spin_executor();

  // Init pose
  geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
  init_pose.header.stamp = current_time_;
  init_pose.header.frame_id = "map";
  init_pose.pose.pose.orientation.w = 1.0;
  init_pose.pose.covariance.fill(0.0);
  init_pose.pose.covariance[0] = 0.01;
  pub_initial_pose_->publish(init_pose);
  spin_executor();

  // Constant twist (velocity X = 5.0, yaw rate = 0.0)
  geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
  twist_msg.header.frame_id = "base_link";
  twist_msg.twist.twist.linear.x = 5.0;
  twist_msg.twist.twist.angular.z = 0.0;
  twist_msg.twist.covariance.fill(0.0);
  twist_msg.twist.covariance[0] = 0.1;
  twist_msg.twist.covariance[35] = 0.1;

  // Run filter for 1 second (20 ticks at 50Hz)
  for (int i = 0; i < 50; ++i) {
    twist_msg.header.stamp = current_time_;
    pub_twist_->publish(twist_msg);
    step_time(0.02);
  }

  // Velocity is 5.0 m/s for 1s.
  // Due to Kalman ramp-up, distance traveled must be positive, less than 5.0m.
  ASSERT_NE(latest_odom_, nullptr);
  EXPECT_LT(latest_odom_->pose.pose.position.x, 5.0);
  EXPECT_NEAR(latest_odom_->pose.pose.position.y, 0.0, near_tol);

  // Expects memory of covariance to grow due to prediction
  ASSERT_EQ(latest_odom_->pose.covariance.size(), 36U);
  double cov_x_x = latest_odom_->pose.covariance[0];
  EXPECT_NEAR(cov_x_x, 0.0120766, near_tol);
}

// TEST 3. Confirms node correctly rejects NaN/Inf, Mahalanobis gate and Delay gate violations.
// Expects node to ignore these cases without crashing, and emit WARN messages to diagnostics:
// - NaN/Inf pose measurements.
// - Pose measurements that exceed Mahalanobis gate.
// - Pose measurements that exceed Delay gate.
// This test will:
// 1. Trigger node init.
// 2. Publish an init pose (0.0, 0.0, 0.0) in map frame.
// 3. Publish a NaN pose measurement (expect node to ignore and emit WARN).
// 4. Publish a pose measurement that exceeds Mahalanobis gate (expect node to ignore and emit
// WARN).
// 5. Publish a pose measurement that exceeds Delay gate (expect node to ignore and emit WARN).
TEST_F(EKFLocalizerIntegrationHarness, SafetyAndRejectionBoundaries)
{
  // Boot
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = true;
  client_trigger_->async_send_request(req);
  executor_->spin_some();

  // Init pose
  geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
  init_pose.header.stamp = current_time_;
  init_pose.header.frame_id = "map";
  init_pose.pose.pose.orientation.w = 1.0;

  // Assign safe array
  init_pose.pose.covariance.fill(0.0);
  init_pose.pose.covariance[0] = 0.01;   // X
  init_pose.pose.covariance[7] = 0.01;   // Y
  init_pose.pose.covariance[14] = 0.01;  // Z
  init_pose.pose.covariance[21] = 0.01;  // Roll
  init_pose.pose.covariance[28] = 0.01;  // Pitch
  init_pose.pose.covariance[35] = 0.01;  // Yaw

  pub_initial_pose_->publish(init_pose);
  spin_executor();  // Flush queue before ticking
  step_time(0.02);  // Tick once (50 Hz)

  // Clear diagnostics state from init
  latest_diag_ = nullptr;

  // ================== Case 1: NaN/Inf ==================
  geometry_msgs::msg::PoseWithCovarianceStamped nan_pose = init_pose;
  nan_pose.header.stamp = current_time_;
  nan_pose.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();

  pub_pose_->publish(nan_pose);
  spin_executor();  // Flush queue before ticking
  step_time(0.02);  // Tick once (50 Hz)

  // Expects node to stay alive and odometry to remain at init pose
  ASSERT_NE(latest_odom_, nullptr);
  EXPECT_FALSE(std::isnan(latest_odom_->pose.pose.position.x));
  EXPECT_NEAR(latest_odom_->pose.pose.position.x, 0.0, near_tol);

  // Drain queue (5 ticks = 1.0 second)
  for (int i = 0; i < 5; ++i) step_time(0.02);
  spin_executor();

  // ================== Case 2: Mahalanobis gate rejection ==================
  geometry_msgs::msg::PoseWithCovarianceStamped far_pose = init_pose;
  far_pose.header.stamp = current_time_;
  far_pose.pose.pose.position.x = 10000.0;  // Massive jump
  far_pose.pose.pose.position.y = -5000.0;

  pub_pose_->publish(far_pose);
  spin_executor();  // Flush queue before ticking
  // Tick short, before queue discards message, to ensure Mahalanobis gate triggers
  for (int i = 0; i < 2; ++i) step_time(0.02);
  spin_executor();  // Flush again to ensure diagnostics are processed

  // Expects node to ignore that huge jump and emit a WARN
  EXPECT_NEAR(latest_odom_->pose.pose.position.x, 0.0, near_tol);
  ASSERT_NE(latest_diag_, nullptr);

  bool found_mahalanobis_warn = false;
  for (const auto & status : latest_diag_->status) {
    if (status.message.find("[WARN]mahalanobis distance") != std::string::npos) {
      found_mahalanobis_warn = true;
    }
  }
  EXPECT_TRUE(found_mahalanobis_warn) << "Failed to trigger Mahalanobis gate warning.";

  // Drain queue again
  for (int i = 0; i < 5; ++i) step_time(0.02);
  spin_executor();

  // ================== Case 3: Delay limit rejection ==================
  geometry_msgs::msg::PoseWithCovarianceStamped ancient_pose = init_pose;
  // Stamp it 50.0 seconds to exceed extend_state_step
  ancient_pose.header.stamp = current_time_ - rclcpp::Duration::from_seconds(50.0);

  pub_pose_->publish(ancient_pose);
  spin_executor();  // Flush queue before ticking
  // Tick short, before queue discards message, to ensure Delay gate triggers
  for (int i = 0; i < 5; ++i) step_time(0.02);
  spin_executor();  // Flush again to ensure diagnostics are processed

  // Expects node to ignore ancient message and emit a WARN
  EXPECT_NEAR(latest_odom_->pose.pose.position.x, 0.0, near_tol);
  ASSERT_NE(latest_diag_, nullptr);

  bool found_delay_warn = false;
  for (const auto & status : latest_diag_->status) {
    if (status.message.find("[WARN]twist topic is delay") != std::string::npos) {
      found_delay_warn = true;
    }
  }
  EXPECT_TRUE(found_delay_warn) << "Failed to trigger Delay limit warning.";
}

// TEST 4. Confirms node correctly handles pose queue overflow.
// Expects node to ignore oldest messages and process newest messages without crashing.
// This test will:
// 1. Trigger node init.
// 2. Publish an init pose (0.0, 0.0, 0.0) in map frame.
// 3. Flood pose queue with 8 messages (max_pose_queue_size is 5 by default).
// 4. Step time to trigger EKF timer callback, which should pop oldest 3 messages and process newest
// 5 without crashing.
// 5. Expects odometry to be updated with the newest pose message, and covariance array to be of
// size 36.
TEST_F(EKFLocalizerIntegrationHarness, QueueOverflow)
{
  // Boot
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = true;
  client_trigger_->async_send_request(req);
  executor_->spin_some();

  // Init pose
  geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
  init_pose.header.stamp = current_time_;
  init_pose.header.frame_id = "map";
  init_pose.pose.pose.orientation.w = 1.0;

  init_pose.pose.covariance.fill(0.0);
  init_pose.pose.covariance[0] = 0.01;   // X
  init_pose.pose.covariance[7] = 0.01;   // Y
  init_pose.pose.covariance[14] = 0.01;  // Z
  init_pose.pose.covariance[21] = 0.01;  // Roll
  init_pose.pose.covariance[28] = 0.01;  // Pitch
  init_pose.pose.covariance[35] = 0.01;  // Yaw

  pub_initial_pose_->publish(init_pose);
  spin_executor();  // Flush queue before ticking
  step_time(0.02);

  // As default max_pose_queue_size is 5, we gonna flood queue with 8 messages
  for (int i = 0; i < 8; ++i) {
    geometry_msgs::msg::PoseWithCovarianceStamped rapid_pose = init_pose;
    rapid_pose.header.stamp = current_time_;
    rapid_pose.pose.pose.position.x = 0.1 * i;
    pub_pose_->publish(rapid_pose);

    // Spin executor to process subscription callbacks instantly,
    // but don't step time (so EKF timer callback can't drain queue yet)
    executor_->spin_some();
  }

  // Now step time to trigger EKF timer callback
  // It should pop oldest 3, process newest 5, without crashing
  step_time(0.02);

  ASSERT_NE(latest_odom_, nullptr);

  // Expects updated state, should be somewhere x > 0.0
  EXPECT_GT(latest_odom_->pose.pose.position.x, 0.0);

  // Array boundary protection test
  ASSERT_EQ(latest_odom_->pose.covariance.size(), 36U);
}

// TEST 5. Confirms node correctly handles timeouts and cascaded WARN => ERROR diagnostics.
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
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = true;
  client_trigger_->async_send_request(req);
  executor_->spin_some();

  geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
  init_pose.header.stamp = current_time_;
  init_pose.header.frame_id = "map";
  init_pose.pose.pose.orientation.w = 1.0;

  init_pose.pose.covariance.fill(0.0);
  init_pose.pose.covariance[0] = 0.01;   // X
  init_pose.pose.covariance[7] = 0.01;   // Y
  init_pose.pose.covariance[14] = 0.01;  // Z
  init_pose.pose.covariance[21] = 0.01;  // Roll
  init_pose.pose.covariance[28] = 0.01;  // Pitch
  init_pose.pose.covariance[35] = 0.01;  // Yaw

  pub_initial_pose_->publish(init_pose);
  spin_executor();  // Flush queue before ticking
  step_time(0.02);

  // Now deny node of any /in_pose_with_covariance messages
  // ============ 1. Advance 48 ticks (total 49, threshold is 50 for WARN) ============
  for (int i = 0; i < 48; ++i) {
    step_time(0.02);
  }

  // Expects node to not WARN yet
  ASSERT_NE(latest_diag_, nullptr);
  bool found_pose_warn = false;
  for (const auto & status : latest_diag_->status) {
    if (status.message.find("pose is not updated") != std::string::npos) {
      found_pose_warn = true;
    }
  }
  EXPECT_FALSE(found_pose_warn) << "Node failed to shut up before tick 50.";

  // ============ 2. Advance 1 more tick to hit 50 (WARN state) ============
  step_time(0.02);

  found_pose_warn = false;
  for (const auto & status : latest_diag_->status) {
    if (status.message.find("[WARN]pose is not updated") != std::string::npos) {
      found_pose_warn = true;
    }
  }
  // Expects node to WARN at 50 ticks of no pose updates
  EXPECT_TRUE(found_pose_warn) << "Node failed to WARN at 50 missed updates.";

  // ============ 3. Advance 50 more ticks to hit 100 (ERROR state) ============
  for (int i = 0; i < 50; ++i) {
    step_time(0.02);
  }

  bool found_pose_error = false;
  for (const auto & status : latest_diag_->status) {
    if (
      status.message.find("[ERROR]pose is not updated") != std::string::npos &&
      status.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
      found_pose_error = true;
    }
  }
  // Expects node to ERROR at 100 ticks of no pose updates
  EXPECT_TRUE(found_pose_error) << "Node failed to ERROR at 100 missed updates.";
}

}  // namespace autoware::ekf_localizer
