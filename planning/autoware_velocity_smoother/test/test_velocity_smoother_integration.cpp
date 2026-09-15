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

#include "autoware/velocity_smoother/node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <memory>

namespace autoware::velocity_smoother
{

using autoware_planning_msgs::msg::Trajectory;

class VelocitySmootherIntegrationHarness : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    // Load Parameter
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
       "--params-file", velocity_smoother_dir + "/config/default_common.param.yaml",
       "--params-file", velocity_smoother_dir + "/config/JerkFiltered.param.yaml"});

    // Init nodes
    node_ = std::make_shared<VelocitySmootherNode>(node_options);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    // Note: VelocitySmootherNode is inherited from agnocast_wrapper::Node not rclcpp::Node
    executor_->add_node(node_->get_rclcpp_node());

    // Setup I/O harness
    harness_node_ = std::make_shared<rclcpp::Node>("velocity_smoother_harness");
    executor_->add_node(harness_node_);

    // Harness Pub/Sub (Pub is Input and Sub is Output for VelocitySmootherNode)
    // Pubs
    pub_traj_ = harness_node_->create_publisher<Trajectory>(
      "/velocity_smoother/input/trajectory", rclcpp::QoS(1).transient_local());
    pub_odom_ =
      harness_node_->create_publisher<nav_msgs::msg::Odometry>("/localization/kinematic_state", 1);
    pub_ext_vel_limit_ =
      harness_node_->create_publisher<autoware_internal_planning_msgs::msg::VelocityLimit>(
        "/velocity_smoother/input/external_velocity_limit_mps", rclcpp::QoS(1).transient_local());

    pub_operation_mode_ =
      harness_node_->create_publisher<autoware_adapi_v1_msgs::msg::OperationModeState>(
        "/velocity_smoother/input/operation_mode_state", rclcpp::QoS(1).transient_local());

    pub_acceleration_ =
      harness_node_->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
        "/velocity_smoother/input/acceleration", rclcpp::QoS(1).transient_local());

    // Subs
    sub_traj_ = harness_node_->create_subscription<Trajectory>(
      "/velocity_smoother/output/trajectory", 1,
      [this](const Trajectory::ConstSharedPtr msg) { latest_traj_ = msg; });
  };

  void TearDown() override { rclcpp::shutdown(); }

  void spin_executor_for(std::chrono::milliseconds duration)
  {
    const auto end_time = std::chrono::steady_clock::now() + duration;

    while (std::chrono::steady_clock::now() < end_time && rclcpp::ok()) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  // Wait helper: spins executor until condition becomes true or timeout elapses
  template <typename Pred>
  bool wait_for(Pred pred, std::chrono::milliseconds timeout)
  {
    const auto end_time = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end_time && rclcpp::ok()) {
      if (pred()) {
        return true;
      }
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return pred();
  }

  // ======================= MOCK TRAJECTORY GENERATOR =========================
  // Straight Trajectory
  static Trajectory create_mock_straight_trajectory(const double velocity = 5.0)
  {
    return autoware::test_utils::generateTrajectory<Trajectory>(10, 1.0, velocity);
  }

  // Curved Trajectory
  static Trajectory create_mock_curved_trajectory(const double velocity = 5.0)
  {
    return autoware::test_utils::generateTrajectory<Trajectory>(10, 1.0, velocity, 0.0, M_PI / 18);
  }

  // Stopping Trajectory (with deceleration ramp)
  static Trajectory create_mock_stopping_trajectory(const double init_velocity = 5.0)
  {
    const size_t num_points = 10;
    const double point_interval = 1.0;
    const double final_velocity = 0.0;
    const double theta = 0.0;
    const double velocity_interval = (final_velocity - init_velocity) / num_points;
    Trajectory traj;
    traj.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
    for (size_t i = 0; i < num_points; ++i) {
      const double x = static_cast<double>(i) * point_interval * std::cos(theta);
      const double y = static_cast<double>(i) * point_interval * std::sin(theta);

      double velocity = init_velocity + velocity_interval * static_cast<double>(i);
      TrajectoryPoint p;
      p.pose = autoware::test_utils::createPose(x, y, 0.0, 0.0, 0.0, theta);
      p.longitudinal_velocity_mps = velocity;
      traj.points.push_back(p);
    }

    return traj;
  }

  // Self-intersecting Trajectory (TBD)

  // =========================== PUBLISH HELPERS ===============================

  static nav_msgs::msg::Odometry set_start_odom(double velocity = 5.0)
  {
    // set pose position at (0.0 ,0.0 ,0.0) with quaternion (0.0, 0.0, 0.0, 1.0)
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "map";

    // A bit forward velocity
    odom.twist.twist.linear.x = velocity;

    return odom;
  }

  void retrigger_pubs_spin(
    const std::optional<Trajectory> & traj, const std::optional<nav_msgs::msg::Odometry> & odom,
    const std::optional<autoware_internal_planning_msgs::msg::VelocityLimit> &
      external_velocity_limit,
    const std::optional<autoware_adapi_v1_msgs::msg::OperationModeState> & operation_mode,
    const std::optional<geometry_msgs::msg::AccelWithCovarianceStamped> & acceleration,
    std::chrono::milliseconds spin_time)
  {
    if (traj.has_value()) {
      pub_traj_->publish(traj.value());
    }
    if (odom.has_value()) {
      pub_odom_->publish(odom.value());
    }
    if (external_velocity_limit.has_value()) {
      pub_ext_vel_limit_->publish(external_velocity_limit.value());
    }
    if (operation_mode.has_value()) {
      pub_operation_mode_->publish(operation_mode.value());
    }
    if (acceleration.has_value()) {
      pub_acceleration_->publish(acceleration.value());
    }
    spin_executor_for(std::chrono::milliseconds(spin_time));
  }

  // Nodes
  std::shared_ptr<VelocitySmootherNode> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<rclcpp::Node> harness_node_;

  // Pubs
  rclcpp::Publisher<Trajectory>::SharedPtr pub_traj_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<autoware_internal_planning_msgs::msg::VelocityLimit>::SharedPtr
    pub_ext_vel_limit_;
  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr pub_operation_mode_;
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr pub_acceleration_;

  // Output Storages
  Trajectory::ConstSharedPtr latest_traj_{nullptr};

  // Subs
  rclcpp::Subscription<Trajectory>::SharedPtr sub_traj_;
};

// TEST 1:
TEST_F(VelocitySmootherIntegrationHarness, NominalSmoothing)
{
  autoware_adapi_v1_msgs::msg::OperationModeState operation_mode;
  operation_mode.mode = OperationModeState::AUTONOMOUS;
  operation_mode.is_autoware_control_enabled = true;
  geometry_msgs::msg::AccelWithCovarianceStamped current_acceleration;
  current_acceleration.accel.accel.linear.x = 0.0;

  {
    Trajectory input_traj = create_mock_straight_trajectory(10.0);
    auto odom = set_start_odom();

    retrigger_pubs_spin(
      input_traj, odom, std::nullopt, operation_mode, current_acceleration,
      std::chrono::milliseconds(100));

    ASSERT_TRUE(
      wait_for([this] { return latest_traj_ != nullptr; }, std::chrono::milliseconds(100)))
      << "Node failed to output Smoothed Trajectory";

    ASSERT_FALSE(latest_traj_->points.empty());
    for (const auto & pt : latest_traj_->points) {
      // less than ego velocity (odom)
      EXPECT_LE(pt.longitudinal_velocity_mps, 5.0);
    }
    // last value is 0.0
    EXPECT_NEAR(latest_traj_->points.back().longitudinal_velocity_mps, 0.0, 1e-2);
  }

  // check exceeding velocity
  {
    latest_traj_ = nullptr;
    auto odom = set_start_odom(15.0);
    Trajectory input_traj = create_mock_straight_trajectory(10.0);
    const double tol = 1e-2;
    retrigger_pubs_spin(
      input_traj, odom, std::nullopt, operation_mode, current_acceleration,
      std::chrono::milliseconds(100));

    ASSERT_TRUE(
      wait_for([this] { return latest_traj_ != nullptr; }, std::chrono::milliseconds(100)))
      << "Node failed to output Smoothed Trajectory";

    ASSERT_FALSE(latest_traj_->points.empty());
    for (const auto & pt : latest_traj_->points) {
      // less than maximum velocity (in config)
      EXPECT_LE(pt.longitudinal_velocity_mps, 11.1 + tol);
    }
    // last value is 0.0
    EXPECT_NEAR(latest_traj_->points.back().longitudinal_velocity_mps, 0.0, 1e-2);
  }
}

// TEST 2:
TEST_F(VelocitySmootherIntegrationHarness, VelocityConstraintRespect)
{
}
// TEST 3:
TEST_F(VelocitySmootherIntegrationHarness, StopPointPreserve)
{
}
// TEST 4:
TEST_F(VelocitySmootherIntegrationHarness, ExternalVelocityLimit)
{
}
// TEST 5:
TEST_F(VelocitySmootherIntegrationHarness, MultiCycleConsistency)
{
}
// TEST 6:
TEST_F(VelocitySmootherIntegrationHarness, AbnormalInputNoCrash)
{
}

}  // namespace autoware::velocity_smoother
