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

#include <algorithm>
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
    sub_vel_limit_ =
      harness_node_->create_subscription<autoware_internal_planning_msgs::msg::VelocityLimit>(
        "/velocity_smoother/output/current_velocity_limit_mps", rclcpp::QoS(1).transient_local(),
        [this](const autoware_internal_planning_msgs::msg::VelocityLimit::ConstSharedPtr msg) {
          latest_vel_limit_ = msg;
        });
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
    return autoware::test_utils::generateTrajectory<Trajectory>(100, 2.0, velocity);
  }

  // Curved Trajectory
  static Trajectory create_mock_curved_trajectory(const double velocity = 5.0)
  {
    return autoware::test_utils::generateTrajectory<Trajectory>(
      100, 2.0, velocity, 0.0, M_PI / 180);
  }

  // Stopping Trajectory (with deceleration ramp)
  static Trajectory create_mock_stopping_trajectory(
    const double init_velocity = 5.0, const size_t stopping_range = 0.0)
  {
    const size_t num_points = 100;
    const double point_interval = 2.0;
    const double final_velocity = 0.0;
    const double theta = 0.0;
    const double velocity_interval =
      (final_velocity - init_velocity) / (num_points - stopping_range);
    Trajectory traj;
    traj.header.frame_id = "map";
    traj.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
    for (size_t i = 0; i < num_points; ++i) {
      const double x = static_cast<double>(i) * point_interval * std::cos(theta);
      const double y = static_cast<double>(i) * point_interval * std::sin(theta);

      double velocity = std::max(init_velocity + velocity_interval * static_cast<double>(i), 0.0);
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

  static nav_msgs::msg::Odometry set_odom(const double x, const double velocity = 5.0)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "map";

    // set pose position at (x ,0.0 ,0.0) with quaternion (0.0, 0.0, 0.0, 1.0)
    odom.pose.pose.position.x = x;

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
  autoware_internal_planning_msgs::msg::VelocityLimit::ConstSharedPtr latest_vel_limit_{nullptr};

  // Subs
  rclcpp::Subscription<Trajectory>::SharedPtr sub_traj_;
  rclcpp::Subscription<autoware_internal_planning_msgs::msg::VelocityLimit>::SharedPtr
    sub_vel_limit_;
};

// =========================== TEST HELPER =================================
// Check velocity bound and terminal stop
static void check_velocity_bound(
  const Trajectory::ConstSharedPtr & traj, const double start_velocity, const double max_velocity,
  const std::optional<size_t> start_idx_opt = std::nullopt, const double tol = 1e-3)
{
  ASSERT_FALSE(traj->points.empty());
  if (!start_idx_opt.has_value()) {
    EXPECT_NEAR(traj->points.front().longitudinal_velocity_mps, start_velocity, tol);
  } else {
    const auto start_idx = *start_idx_opt;
    ASSERT_LT(start_idx, traj->points.size());
    EXPECT_NEAR(traj->points[start_idx].longitudinal_velocity_mps, start_velocity, tol);
  }
  for (const auto & pt : traj->points) {
    EXPECT_GE(pt.longitudinal_velocity_mps, 0.0);
    // less than input maximum velocity (odom velocity, or node's max velocity)
    EXPECT_LE(pt.longitudinal_velocity_mps, max_velocity + tol);
  }
  // last value is 0.0
  EXPECT_NEAR(traj->points.back().longitudinal_velocity_mps, 0.0, tol);
}

// Check acceleration bound
static void check_acceleration_bound(
  const Trajectory::ConstSharedPtr & traj, const double max_acc, const double min_acc)
{
  constexpr auto tol = 2e-3;
  ASSERT_FALSE(traj->points.empty());
  for (const auto & pt : traj->points) {
    // within acceleration bound
    EXPECT_GE(pt.acceleration_mps2, min_acc - tol);
    EXPECT_LE(pt.acceleration_mps2, max_acc + tol);
  }
}

static std::optional<size_t> findClosestIndex(
  const Trajectory::ConstSharedPtr & traj, const nav_msgs::msg::Odometry & odom)
{
  return autoware::motion_utils::findNearestIndex(traj->points, odom.pose.pose);
}

// TEST 1:
TEST_F(VelocitySmootherIntegrationHarness, NominalSmoothing)
{
  ASSERT_TRUE(
    wait_for([this] { return latest_vel_limit_ != nullptr; }, std::chrono::milliseconds(100)))
    << "Node failed to output latest velocity limit.";

  // check constructor max velocity (from config)
  EXPECT_NEAR(latest_vel_limit_->max_velocity, 11.1, 1e-3);

  autoware_adapi_v1_msgs::msg::OperationModeState operation_mode;
  operation_mode.mode = OperationModeState::AUTONOMOUS;
  operation_mode.is_autoware_control_enabled = true;
  geometry_msgs::msg::AccelWithCovarianceStamped current_acceleration;
  current_acceleration.accel.accel.linear.x = 0.0;

  {
    Trajectory input_traj = create_mock_straight_trajectory(10);
    auto odom = set_start_odom(5.0);

    retrigger_pubs_spin(
      input_traj, odom, std::nullopt, operation_mode, current_acceleration,
      std::chrono::milliseconds(100));

    ASSERT_TRUE(
      wait_for([this] { return latest_traj_ != nullptr; }, std::chrono::milliseconds(100)))
      << "Node failed to output Smoothed Trajectory.";

    // check start from 5.0 and less than 10.0 (target)
    check_velocity_bound(latest_traj_, 5.0, 10.0);

    // check within acceleration bound (from config)
    check_acceleration_bound(latest_traj_, 1.0, -0.5);
  }

  // check exceeding velocity
  {
    latest_traj_ = nullptr;
    auto odom = set_start_odom(5.0);
    Trajectory input_traj = create_mock_straight_trajectory(30.0);
    retrigger_pubs_spin(
      input_traj, odom, std::nullopt, operation_mode, current_acceleration,
      std::chrono::milliseconds(100));

    ASSERT_TRUE(
      wait_for([this] { return latest_traj_ != nullptr; }, std::chrono::milliseconds(100)))
      << "Node failed to output Smoothed Trajectory.";

    // check start from 5.0 and less than config maximum velocity (11.1)
    check_velocity_bound(latest_traj_, 5.0, 11.1);

    // check within acceleration bound (from config)
    check_acceleration_bound(latest_traj_, 1.0, -0.5);
  }

  // Curved Trajectory (v_target = 10)
  {
    latest_traj_ = nullptr;
    auto odom = set_start_odom(5.0);
    Trajectory input_traj = create_mock_curved_trajectory(10.0);
    retrigger_pubs_spin(
      input_traj, odom, std::nullopt, operation_mode, current_acceleration,
      std::chrono::milliseconds(100));

    ASSERT_TRUE(
      wait_for([this] { return latest_traj_ != nullptr; }, std::chrono::milliseconds(100)))
      << "Node failed to output Smoothed Trajectory.";

    // check start from 5.0 and less than target (10.0)
    check_velocity_bound(latest_traj_, 5.0, 10.0);

    // check within acceleration bound (from config)
    check_acceleration_bound(latest_traj_, 1.0, -0.5);
  }

  // Curved Trajectory (v_target = 30)
  {
    latest_traj_ = nullptr;
    auto odom = set_start_odom(5.0);
    Trajectory input_traj = create_mock_curved_trajectory(30.0);
    retrigger_pubs_spin(
      input_traj, odom, std::nullopt, operation_mode, current_acceleration,
      std::chrono::milliseconds(100));

    ASSERT_TRUE(
      wait_for([this] { return latest_traj_ != nullptr; }, std::chrono::milliseconds(100)))
      << "Node failed to output Smoothed Trajectory.";

    // check start from 5.0 and less than config maximum velocity (11.1)
    check_velocity_bound(latest_traj_, 5.0, 11.1);

    // check within acceleration bound (from config)
    check_acceleration_bound(latest_traj_, 1.0, -0.5);
  }

  // Stopping Trajectory (v_target = 10)
  {
    latest_traj_ = nullptr;
    auto odom = set_start_odom(5.0);
    Trajectory input_traj = create_mock_stopping_trajectory(10.0);
    retrigger_pubs_spin(
      input_traj, odom, std::nullopt, operation_mode, current_acceleration,
      std::chrono::milliseconds(100));

    ASSERT_TRUE(
      wait_for([this] { return latest_traj_ != nullptr; }, std::chrono::milliseconds(100)))
      << "Node failed to output Smoothed Trajectory.";
    EXPECT_EQ(latest_traj_->header.frame_id, "map");

    // check start from 5.0 and less than target (10.0)
    check_velocity_bound(latest_traj_, 5.0, 10.0);

    // check within acceleration bound (from config)
    check_acceleration_bound(latest_traj_, 1.0, -0.5);
  }

  // Stopping Trajectory (v_target = 30)
  {
    latest_traj_ = nullptr;
    auto odom = set_start_odom(5.0);
    Trajectory input_traj = create_mock_stopping_trajectory(30.0);
    retrigger_pubs_spin(
      input_traj, odom, std::nullopt, operation_mode, current_acceleration,
      std::chrono::milliseconds(100));

    ASSERT_TRUE(
      wait_for([this] { return latest_traj_ != nullptr; }, std::chrono::milliseconds(100)))
      << "Node failed to output Smoothed Trajectory.";
    EXPECT_EQ(latest_traj_->header.frame_id, "map");

    // check start from 5.0 and less than config maximum velocity (11.1)
    check_velocity_bound(latest_traj_, 5.0, 11.1);

    // check within acceleration bound (from config)
    check_acceleration_bound(latest_traj_, 1.0, -0.5);
  }
}

// TEST 2:
TEST_F(VelocitySmootherIntegrationHarness, ExternalVelocityConstraintRespect)
{
  ASSERT_TRUE(
    wait_for([this] { return latest_vel_limit_ != nullptr; }, std::chrono::milliseconds(100)))
    << "Node failed to output latest velocity limit for constructor.";

  // check constructor max velocity (from config)
  EXPECT_NEAR(latest_vel_limit_->max_velocity, 11.1, 1e-3);

  autoware_internal_planning_msgs::msg::VelocityLimit velocity_limit;

  velocity_limit.max_velocity = 7.5;
  velocity_limit.use_constraints = false;

  autoware_adapi_v1_msgs::msg::OperationModeState operation_mode;
  operation_mode.mode = OperationModeState::AUTONOMOUS;
  operation_mode.is_autoware_control_enabled = true;
  geometry_msgs::msg::AccelWithCovarianceStamped current_acceleration;
  current_acceleration.accel.accel.linear.x = 0.0;

  // Straight trajectory
  {
    Trajectory input_traj = create_mock_straight_trajectory(10.0);
    auto odom = set_start_odom(5.0);

    retrigger_pubs_spin(
      input_traj, odom, velocity_limit, operation_mode, current_acceleration,
      std::chrono::milliseconds(100));

    ASSERT_TRUE(
      wait_for([this] { return latest_traj_ != nullptr; }, std::chrono::milliseconds(100)))
      << "Node failed to output Smoothed Trajectory";

    ASSERT_TRUE(
      wait_for([this] { return latest_vel_limit_ != nullptr; }, std::chrono::milliseconds(100)))
      << "Node failed to output velocity_limit";
    EXPECT_NEAR(latest_vel_limit_->max_velocity, 7.5, 1e-3);

    // check start from 5.0 and less than external maximum velocity (7.5)
    check_velocity_bound(latest_traj_, 5.0, 7.5);

    // check within acceleration bound (from config)
    check_acceleration_bound(latest_traj_, 1.0, -0.5);
  }

  // Curved trajectory
  {
    latest_traj_ = nullptr;
    latest_vel_limit_ = nullptr;
    Trajectory input_traj = create_mock_curved_trajectory(10.0);

    retrigger_pubs_spin(
      input_traj, std::nullopt, velocity_limit, std::nullopt, std::nullopt,
      std::chrono::milliseconds(100));

    ASSERT_TRUE(
      wait_for([this] { return latest_traj_ != nullptr; }, std::chrono::milliseconds(100)))
      << "Node failed to output Smoothed Trajectory";

    ASSERT_TRUE(
      wait_for([this] { return latest_vel_limit_ != nullptr; }, std::chrono::milliseconds(100)))
      << "Node failed to output velocity_limit";
    EXPECT_NEAR(latest_vel_limit_->max_velocity, 7.5, 1e-3);

    // check start from 5.0 and less than external maximum velocity (7.5)
    check_velocity_bound(latest_traj_, 5.0, 7.5);

    // check within acceleration bound (from config)
    check_acceleration_bound(latest_traj_, 1.0, -0.5);
  }

  // Stopping Trajectory
  {
    latest_traj_ = nullptr;
    latest_vel_limit_ = nullptr;
    Trajectory input_traj = create_mock_stopping_trajectory(10.0);

    retrigger_pubs_spin(
      input_traj, std::nullopt, velocity_limit, std::nullopt, std::nullopt,
      std::chrono::milliseconds(100));

    ASSERT_TRUE(
      wait_for([this] { return latest_traj_ != nullptr; }, std::chrono::milliseconds(100)))
      << "Node failed to output Smoothed Trajectory";

    ASSERT_TRUE(
      wait_for([this] { return latest_vel_limit_ != nullptr; }, std::chrono::milliseconds(100)))
      << "Node failed to output velocity_limit";
    EXPECT_NEAR(latest_vel_limit_->max_velocity, 7.5, 1e-3);

    // check start from 5.0 and less than external maximum velocity (7.5)
    check_velocity_bound(latest_traj_, 5.0, 7.5);

    // check within acceleration bound (from config)
    check_acceleration_bound(latest_traj_, 1.0, -0.5);
  }
}

// TEST 3:
TEST_F(VelocitySmootherIntegrationHarness, StopPointPreserve)
{
  ASSERT_TRUE(
    wait_for([this] { return latest_vel_limit_ != nullptr; }, std::chrono::milliseconds(100)))
    << "Node failed to output latest velocity limit for constructor.";

  // check constructor max velocity (from config)
  EXPECT_NEAR(latest_vel_limit_->max_velocity, 11.1, 1e-3);

  autoware_adapi_v1_msgs::msg::OperationModeState operation_mode;
  operation_mode.mode = OperationModeState::AUTONOMOUS;
  operation_mode.is_autoware_control_enabled = true;
  geometry_msgs::msg::AccelWithCovarianceStamped current_acceleration;
  current_acceleration.accel.accel.linear.x = 0.0;

  {
    Trajectory input_traj = create_mock_stopping_trajectory(10.0, 80);
    auto odom = set_start_odom(5.0);

    retrigger_pubs_spin(
      input_traj, odom, std::nullopt, operation_mode, current_acceleration,
      std::chrono::milliseconds(100));

    ASSERT_TRUE(
      wait_for([this] { return latest_traj_ != nullptr; }, std::chrono::milliseconds(100)))
      << "Node failed to output Smoothed Trajectory";
    EXPECT_EQ(latest_traj_->header.frame_id, "map");

    // check start from 5.0 and less than  maximum velocity (11.1)
    check_velocity_bound(latest_traj_, 5.0, 11.1);

    // check within acceleration bound (from config)
    check_acceleration_bound(latest_traj_, 1.0, -0.5);

    // find stop point
    constexpr double stop_vel_threshold = 1e-3;
    const auto stop_idx = [&]() -> std::optional<size_t> {
      for (size_t i = 0; i < latest_traj_->points.size(); ++i) {
        if (latest_traj_->points[i].longitudinal_velocity_mps < stop_vel_threshold) {
          return i;
        }
      }
      return std::nullopt;
    }();
    ASSERT_TRUE(stop_idx.has_value()) << "No stop point found in output.";

    constexpr double tol = 1e-3;

    const double stop_x = 40.0;  // traj has 100 points with 2 m interval, stop_range is 80 points.
    EXPECT_NEAR(latest_traj_->points[*stop_idx].pose.position.x, stop_x, tol)
      << "Stop point is not preserved";

    // Stop is maintained to the end
    for (size_t i = *stop_idx; i < latest_traj_->points.size(); ++i) {
      EXPECT_NEAR(latest_traj_->points[i].longitudinal_velocity_mps, 0.0, stop_vel_threshold);
    }
    // last value is 0.0
    EXPECT_NEAR(latest_traj_->points.back().longitudinal_velocity_mps, 0.0, stop_vel_threshold);
  }
}

// TEST 4:
TEST_F(VelocitySmootherIntegrationHarness, MultiCycleConsistency)
{
  constexpr double tol = 1e-3;
  ASSERT_TRUE(
    wait_for([this] { return latest_vel_limit_ != nullptr; }, std::chrono::milliseconds(100)))
    << "Node failed to output latest velocity limit for constructor.";

  // check constructor max velocity (from config)
  EXPECT_NEAR(latest_vel_limit_->max_velocity, 11.1, 1e-3);

  autoware_adapi_v1_msgs::msg::OperationModeState operation_mode;
  operation_mode.mode = OperationModeState::AUTONOMOUS;
  operation_mode.is_autoware_control_enabled = true;
  geometry_msgs::msg::AccelWithCovarianceStamped current_acceleration;
  current_acceleration.accel.accel.linear.x = 0.0;

  Trajectory input_traj = create_mock_straight_trajectory(10.0);
  retrigger_pubs_spin(
    std::nullopt, std::nullopt, std::nullopt, operation_mode, current_acceleration,
    std::chrono::milliseconds(100));

  Trajectory::ConstSharedPtr prev_traj;
  // test for 4 cycles
  for (size_t k = 0; k < 4; ++k) {
    latest_traj_ = nullptr;
    auto odom = set_odom(5.0 * k, 5.0 + 2.0 * k);
    retrigger_pubs_spin(
      input_traj, odom, std::nullopt, std::nullopt, std::nullopt, std::chrono::milliseconds(100));
    ASSERT_TRUE(
      wait_for([this] { return latest_traj_ != nullptr; }, std::chrono::milliseconds(100)))
      << "Node failed to output Smoothed Trajectory";

    if (k > 0) {
      const auto start_idx_opt = findClosestIndex(latest_traj_, odom);
      ASSERT_TRUE(start_idx_opt.has_value());
      const auto prev_start_idx_opt = findClosestIndex(prev_traj, odom);
      ASSERT_TRUE(prev_start_idx_opt.has_value());
      // check start from that start_idx velocity and less than  maximum velocity (11.1) - with
      // loosen tol (0.1)
      check_velocity_bound(
        latest_traj_, prev_traj->points[*prev_start_idx_opt].longitudinal_velocity_mps, 11.1,
        start_idx_opt, 0.1);

      // check start point location (should be 5.0 * k)
      EXPECT_NEAR(latest_traj_->points[*start_idx_opt].pose.position.x, 5.0 * k, tol);
    } else {
      // check start from 5.0 and less than  maximum velocity (11.1)
      check_velocity_bound(latest_traj_, 5.0, 11.1);
    }

    // check within acceleration bound (from config)
    check_acceleration_bound(latest_traj_, 1.0, -0.5);
    prev_traj = latest_traj_;
  }
}

// TEST 5:
TEST_F(VelocitySmootherIntegrationHarness, AbnormalInputNoCrash)
{
}

}  // namespace autoware::velocity_smoother
