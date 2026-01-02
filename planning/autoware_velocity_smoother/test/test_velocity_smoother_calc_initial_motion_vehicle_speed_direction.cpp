// Copyright 2025 Autonomous Systems sp. z o.o.
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

// Regression test for SDP-5220: calcInitialMotion() vehicle_speed direction handling
//
// This file intentionally focuses on ONE bug:
// `calcInitialMotion()` must compute `vehicle_speed` with trajectory direction in mind.
// Using `std::fabs(odom.twist.twist.linear.x)` loses direction and can prevent
// LARGE_DEVIATION_REPLAN from triggering in reverse-driving scenarios.
//
// ASCII sketch (1D along x):
//
//   map x-axis:   <----- negative x -----|----- positive x ----->
//
//   Reverse trajectory (desired):
//     yaw = pi (facing -X), v_ref < 0
//       p2  p1  p0
//       *---*---*   (points progress toward decreasing x)
//
//   Ego motion (stimulus):
//     phase1: ego moving backward (v_ego < 0)  -> matches reverse plan
//     phase2: ego moving forward  (v_ego > 0)  -> WRONG direction vs reverse plan
//
// Behavior difference:
// - Before fix: vehicle_speed = |v_ego| (always positive) -> error vs desired may look small.
// - After fix:  vehicle_speed keeps direction aligned to trajectory -> error becomes large,
//               triggering LARGE_DEVIATION_REPLAN as intended.

#include "autoware/velocity_smoother/node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <boost/math/constants/constants.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

using autoware::velocity_smoother::VelocitySmootherNode;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

namespace
{
using boost::math::constants::pi;

// Helper to create a trajectory point
TrajectoryPoint createTrajectoryPoint(
  double x, double y, double yaw, double velocity, double acceleration = 0.0)
{
  TrajectoryPoint p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.position.z = 0.0;

  // Create quaternion from yaw
  p.pose.orientation.x = 0.0;
  p.pose.orientation.y = 0.0;
  p.pose.orientation.z = std::sin(yaw / 2.0);
  p.pose.orientation.w = std::cos(yaw / 2.0);
  p.longitudinal_velocity_mps = static_cast<float>(velocity);
  p.acceleration_mps2 = static_cast<float>(acceleration);
  return p;
}

// Create a straight trajectory going backward.
Trajectory createBackwardTrajectory(size_t num_points, double spacing, double velocity)
{
  Trajectory traj;
  traj.header.frame_id = "map";

  // Start from far position and go towards origin
  double start_x = static_cast<double>(num_points - 1) * spacing;
  for (size_t i = 0; i < num_points; ++i) {
    traj.points.push_back(createTrajectoryPoint(
      start_x - static_cast<double>(i) * spacing,  // x decreasing
      0.0,                                         // y
      pi<double>(),  // yaw (facing -X for reverse, vehicle rear first)
      velocity));  // negative velocity for reverse
  }
  // Stop at the end
  if (!traj.points.empty()) {
    traj.points.back().longitudinal_velocity_mps = 0.0;
  }
  return traj;
}

// Create odometry message
nav_msgs::msg::Odometry createOdometry(double x, double y, double yaw, double linear_vel)
{
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "map";
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = std::sin(yaw / 2.0);
  odom.pose.pose.orientation.w = std::cos(yaw / 2.0);
  odom.twist.twist.linear.x = linear_vel;
  return odom;
}

std::shared_ptr<VelocitySmootherNode> createNode()
{
  auto node_options = rclcpp::NodeOptions{};
  node_options.append_parameter_override("algorithm_type", "JerkFiltered");
  node_options.append_parameter_override("publish_debug_trajs", false);
  const auto autoware_test_utils_dir =
    ament_index_cpp::get_package_share_directory("autoware_test_utils");
  const auto velocity_smoother_dir =
    ament_index_cpp::get_package_share_directory("autoware_velocity_smoother");
  node_options.arguments(
    {"--ros-args", "--params-file", autoware_test_utils_dir + "/config/test_common.param.yaml",
     "--params-file", autoware_test_utils_dir + "/config/test_nearest_search.param.yaml",
     "--params-file", autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml",
     "--params-file", velocity_smoother_dir + "/config/default_velocity_smoother.param.yaml",
     "--params-file", velocity_smoother_dir + "/config/default_common.param.yaml", "--params-file",
     velocity_smoother_dir + "/config/JerkFiltered.param.yaml"});

  return std::make_shared<VelocitySmootherNode>(node_options);
}
}  // namespace

class VelocitySmootherCalcInitialMotionVehicleSpeedDirectionTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

/**
 * @brief Test: vel_error threshold triggering different code paths (KEY REGRESSION TEST)
 *
 * This test MUST FAIL on buggy code and PASS on fixed code.
 *
 * Setup:
 * 1. Establish prev_output with backward driving at -3.0 m/s (internally: desired_vel = 3.0)
 * 2. Vehicle suddenly moves FORWARD at +2.0 m/s during reverse trajectory
 *
 * With replan_vel_deviation = 3.0 (default):
 *
 * BUG (std::fabs on vehicle_speed):
 *   vehicle_speed = fabs(+2.0) = 2.0  (positive, loses direction info!)
 *   vel_error = 2.0 - 3.0 = -1.0
 *   |vel_error| = 1.0 < 3.0 → NO LARGE_DEVIATION_REPLAN triggered
 *   Uses desired_vel path → output ≈ -3.0
 *
 * FIX ((is_reverse_ ? -1 : 1) * linear.x):
 *   vehicle_speed = -1 * 2.0 = -2.0  (negative, captures wrong direction!)
 *   vel_error = -2.0 - 3.0 = -5.0
 *   |vel_error| = 5.0 > 3.0 → LARGE_DEVIATION_REPLAN triggered!
 *   Uses vehicle_speed path → output ≈ -2.0
 *
 * The assertion checks that |first_vel| ≈ 2.0 (fix) not ≈ 3.0 (bug).
 */
TEST_F(VelocitySmootherCalcInitialMotionVehicleSpeedDirectionTest,
  CalcInitialMotionVehicleSpeedDirectionTriggersReplan)
{
  using std::chrono_literals::operator""ms;
  using std::chrono_literals::operator""s;

  auto node = createNode();

  // Create publishers
  auto traj_pub = node->create_publisher<Trajectory>("~/input/trajectory", 1);
  auto odom_pub =
    node->create_publisher<nav_msgs::msg::Odometry>("/localization/kinematic_state", 1);
  auto accel_pub = node->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
    "~/input/acceleration", 1);
  auto vel_limit_pub =
    node->create_publisher<autoware_internal_planning_msgs::msg::VelocityLimit>(
      "~/input/external_velocity_limit_mps", 1);
  auto op_mode_pub = node->create_publisher<autoware_adapi_v1_msgs::msg::OperationModeState>(
    "~/input/operation_mode_state", 1);

  Trajectory received_traj;
  bool received = false;
  auto traj_sub = node->create_subscription<Trajectory>(
    "~/output/trajectory", 1, [&](const Trajectory::SharedPtr msg) {
      received_traj = *msg;
      received = true;
    });

  // Static messages
  geometry_msgs::msg::AccelWithCovarianceStamped accel;
  accel.accel.accel.linear.x = 0.0;

  autoware_internal_planning_msgs::msg::VelocityLimit vel_limit;
  vel_limit.max_velocity = 10.0;

  autoware_adapi_v1_msgs::msg::OperationModeState op_mode;
  op_mode.mode = autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS;
  op_mode.is_autoware_control_enabled = true;  // Required for AUTONOMOUS mode path

  // Phase 1: Establish backward driving state at -3.0 m/s
  // Vehicle moving backward correctly
  for (int i = 0; i < 10; ++i) {
    auto backward_traj = createBackwardTrajectory(50, 1.0, -3.0);
    backward_traj.header.stamp = node->now();
    auto odom = createOdometry(45.0, 0.0, pi<double>(), -3.0);  // Moving backward correctly
    odom.header.stamp = node->now();
    accel.header.stamp = node->now();

    odom_pub->publish(odom);
    accel_pub->publish(accel);
    vel_limit_pub->publish(vel_limit);
    op_mode_pub->publish(op_mode);
    traj_pub->publish(backward_traj);

    rclcpp::spin_some(node);
    rclcpp::sleep_for(50ms);
  }

  // Clear any previous outputs
  received = false;

  // Phase 2: Vehicle suddenly moving FORWARD at +2.0 m/s (wrong direction!)
  // Trajectory still wants backward at -3.0 m/s
  for (int i = 0; i < 5; ++i) {
    auto backward_traj = createBackwardTrajectory(50, 1.0, -3.0);
    backward_traj.header.stamp = node->now();
    auto odom = createOdometry(45.0, 0.0, pi<double>(), +2.0);  // Moving FORWARD (wrong direction!)
    odom.header.stamp = node->now();
    accel.header.stamp = node->now();

    odom_pub->publish(odom);
    accel_pub->publish(accel);
    vel_limit_pub->publish(vel_limit);
    op_mode_pub->publish(op_mode);
    traj_pub->publish(backward_traj);

    rclcpp::spin_some(node);
    rclcpp::sleep_for(50ms);
  }

  // Wait for output
  const auto start_time = node->now();
  const auto timeout = rclcpp::Duration::from_seconds(3.0);
  while (!received && (node->now() - start_time) < timeout) {
    rclcpp::spin_some(node);
    rclcpp::sleep_for(10ms);
  }

  ASSERT_TRUE(received) << "Did not receive output trajectory";
  ASSERT_FALSE(received_traj.points.empty()) << "Output should not be empty";

  double first_vel = received_traj.points.front().longitudinal_velocity_mps;

  // With FIX: |first_vel| ≈ 2.0 (uses vehicle_speed after detecting large deviation)
  // With BUG: |first_vel| ≈ 3.0 (blindly uses desired_vel, no deviation detected)
  EXPECT_LT(std::fabs(first_vel), 2.5)
    << "First velocity magnitude should be ~2.0 (from vehicle_speed), not ~3.0 (from desired_vel). "
    << "Got: " << first_vel
    << ". With bug, smoother doesn't detect large deviation and blindly uses desired_vel.";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
