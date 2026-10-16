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

namespace
{
// ======================= MOCK TRAJECTORY GENERATOR =========================
// Straight Trajectory
Trajectory create_mock_straight_trajectory(const double velocity = 5.0)
{
  auto traj = autoware::test_utils::generateTrajectory<Trajectory>(100, 2.0, velocity);
  traj.header.frame_id = "map";
  return traj;
}

// Curved Trajectory
Trajectory create_mock_curved_trajectory(const double velocity = 5.0)
{
  auto traj =
    autoware::test_utils::generateTrajectory<Trajectory>(100, 2.0, velocity, 0.0, M_PI / 180);
  traj.header.frame_id = "map";
  return traj;
}

// Stopping Trajectory (with deceleration ramp)
Trajectory create_mock_stopping_trajectory(
  const double init_velocity = 5.0, const size_t stopping_range = 0)
{
  const size_t num_points = 100;
  const double point_interval = 2.0;
  const double final_velocity = 0.0;
  const double theta = 0.0;
  const double velocity_interval = (final_velocity - init_velocity) / (num_points - stopping_range);
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

nav_msgs::msg::Odometry set_odom(const double x, const double velocity = 5.0)
{
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "map";

  // set pose position at (x ,0.0 ,0.0) with quaternion (0.0, 0.0, 0.0, 1.0)
  odom.pose.pose.position.x = x;

  // A bit forward velocity
  odom.twist.twist.linear.x = velocity;

  return odom;
}

}  // namespace

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
  void spin_until(Pred pred, std::chrono::milliseconds timeout)
  {
    const auto end_time = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end_time && rclcpp::ok()) {
      if (pred()) {
        return;
      }
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  // =========================== PUBLISH HELPERS ===============================
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

  void publish_velocity_limit(const double max_velocity)
  {
    autoware_internal_planning_msgs::msg::VelocityLimit velocity_limit;

    velocity_limit.max_velocity = max_velocity;
    velocity_limit.use_constraints = false;
    retrigger_pubs_spin(
      std::nullopt, std::nullopt, velocity_limit, std::nullopt, std::nullopt,
      std::chrono::milliseconds(100));
  }

  void publish_ego_state(const nav_msgs::msg::Odometry & odom)
  {
    retrigger_pubs_spin(
      std::nullopt, odom, std::nullopt, std::nullopt, std::nullopt, std::chrono::milliseconds(100));
  }

  void publish_ego_state(const double x = 0.0, const double velocity = 5.0)
  {
    auto odom = set_odom(x, velocity);
    publish_ego_state(odom);
  }

  void publish_input_trajectory(const Trajectory & input_traj)
  {
    retrigger_pubs_spin(
      input_traj, std::nullopt, std::nullopt, std::nullopt, std::nullopt,
      std::chrono::milliseconds(100));
  }

  void publish_default_inputs()
  {
    autoware_adapi_v1_msgs::msg::OperationModeState operation_mode;
    operation_mode.mode = OperationModeState::AUTONOMOUS;
    operation_mode.is_autoware_control_enabled = true;
    geometry_msgs::msg::AccelWithCovarianceStamped current_acceleration;

    // publish operation_mode and acceleration (zero)
    retrigger_pubs_spin(
      std::nullopt, std::nullopt, std::nullopt, operation_mode, current_acceleration,
      std::chrono::milliseconds(100));
  }

  Trajectory::ConstSharedPtr receive_smoothed_trajectory()
  {
    spin_until([this] { return latest_traj_ != nullptr; }, std::chrono::milliseconds(100));
    return latest_traj_;
  }

  autoware_internal_planning_msgs::msg::VelocityLimit::ConstSharedPtr receive_velocity_limit()
  {
    spin_until([this] { return latest_vel_limit_ != nullptr; }, std::chrono::milliseconds(100));
    return latest_vel_limit_;
  }

  void reset_output_messages()
  {
    latest_traj_ = nullptr;
    latest_vel_limit_ = nullptr;
  }

  void reset_trajectory_output() { latest_traj_ = nullptr; }

  // =========================== CONFIG HELPERS ===============================

  double max_acc() const { return node_->get_parameter("normal.max_acc").as_double(); }
  double min_acc() const { return node_->get_parameter("normal.min_acc").as_double(); }
  double max_velocity() { return node_->get_parameter("max_vel").as_double(); }

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

// TEST 1: NominalSmoothing
class NominalSmoothing : public VelocitySmootherIntegrationHarness
{
};

// TEST 1.1: StraightTargetBelowMaxVel_OutputStaysBelowTargetWithinAccLimits
TEST_F(NominalSmoothing, StraightTargetBelowMaxVel_OutputStaysBelowTargetWithinAccLimits)
{
  // Publish all necessary inputs
  publish_default_inputs();
  publish_ego_state(0.0, 5.0);                                  // Ego start at x = 0, v = 5.0
  const auto input_traj = create_mock_straight_trajectory(10);  // Target traj of v = 10.0

  // Publish input trajectory and receive smoothed trajectory
  publish_input_trajectory(input_traj);
  const auto cur_vel_lim = receive_velocity_limit();
  const auto result_trajectory = receive_smoothed_trajectory();

  // Assert check
  ASSERT_NE(cur_vel_lim, nullptr);
  EXPECT_NEAR(
    cur_vel_lim->max_velocity, max_velocity(), 1e-3);  // Initial max_velocity (from config)
  ASSERT_NE(result_trajectory, nullptr) << "Node failed to output Smoothed Trajectory.";
  EXPECT_EQ(result_trajectory->header.frame_id, "map");

  // check start from 5.0 and less than 10.0 (target)
  check_velocity_bound(result_trajectory, 5.0, 10.0);

  // check within acceleration bound (from config)
  check_acceleration_bound(result_trajectory, max_acc(), min_acc());
}

// TEST 1.2: StraightTargetAboveMaxVel_OutputIsCappedAtMaxVelWithinAccLimits
TEST_F(NominalSmoothing, StraightTargetAboveMaxVel_OutputIsCappedAtMaxVelWithinAccLimits)
{
  // Publish all necessary inputs
  publish_default_inputs();
  publish_ego_state(0.0, 5.0);                                  // Ego start at x = 0, v = 5.0
  const auto input_traj = create_mock_straight_trajectory(30);  // Target traj of v = 30.0

  // Publish input trajectory and receive smoothed trajectory
  publish_input_trajectory(input_traj);
  const auto cur_vel_lim = receive_velocity_limit();
  const auto result_trajectory = receive_smoothed_trajectory();

  // Assert check
  ASSERT_NE(cur_vel_lim, nullptr);
  EXPECT_NEAR(
    cur_vel_lim->max_velocity, max_velocity(), 1e-3);  // Initial max_velocity (from config)
  ASSERT_NE(result_trajectory, nullptr) << "Node failed to output Smoothed Trajectory.";
  EXPECT_EQ(result_trajectory->header.frame_id, "map");

  // check start from 5.0 and less than config max_velocity (11.1)
  check_velocity_bound(result_trajectory, 5.0, max_velocity());

  // check within acceleration bound (from config)
  check_acceleration_bound(result_trajectory, max_acc(), min_acc());
}

// TEST 1.3: CurvedTargetBelowMaxVel_OutputStaysBelowTargetWithinAccLimits
TEST_F(NominalSmoothing, CurvedTargetBelowMaxVel_OutputStaysBelowTargetWithinAccLimits)
{
  // Publish all necessary inputs
  publish_default_inputs();
  publish_ego_state(0.0, 5.0);                                // Ego start at x = 0, v = 5.0
  const auto input_traj = create_mock_curved_trajectory(10);  // Target traj of v = 10.0

  // Publish input trajectory and receive smoothed trajectory
  publish_input_trajectory(input_traj);
  const auto cur_vel_lim = receive_velocity_limit();
  const auto result_trajectory = receive_smoothed_trajectory();

  // Assert check
  ASSERT_NE(cur_vel_lim, nullptr);
  EXPECT_NEAR(
    cur_vel_lim->max_velocity, max_velocity(), 1e-3);  // Initial max_velocity (from config)
  ASSERT_NE(result_trajectory, nullptr) << "Node failed to output Smoothed Trajectory.";
  EXPECT_EQ(result_trajectory->header.frame_id, "map");

  // check start from 5.0 and less than 10.0 (target)
  check_velocity_bound(result_trajectory, 5.0, 10.0);

  // check within acceleration bound (from config)
  check_acceleration_bound(result_trajectory, max_acc(), min_acc());
}

// TEST 1.4: CurvedTargetAboveMaxVel_OutputIsCappedAtMaxVelWithinAccLimits
TEST_F(NominalSmoothing, CurvedTargetAboveMaxVel_OutputIsCappedAtMaxVelWithinAccLimits)
{
  // Publish all necessary inputs
  publish_default_inputs();
  publish_ego_state(0.0, 5.0);                                // Ego start at x = 0, v = 5.0
  const auto input_traj = create_mock_curved_trajectory(30);  // Target traj of v = 30.0

  // Publish input trajectory and receive smoothed trajectory
  publish_input_trajectory(input_traj);
  const auto cur_vel_lim = receive_velocity_limit();
  const auto result_trajectory = receive_smoothed_trajectory();

  // Assert check
  ASSERT_NE(cur_vel_lim, nullptr);
  EXPECT_NEAR(
    cur_vel_lim->max_velocity, max_velocity(), 1e-3);  // Initial max_velocity (from config)
  ASSERT_NE(result_trajectory, nullptr) << "Node failed to output Smoothed Trajectory.";
  EXPECT_EQ(result_trajectory->header.frame_id, "map");

  // check start from 5.0 and less than config max_velocity (11.1)
  check_velocity_bound(result_trajectory, 5.0, max_velocity());

  // check within acceleration bound (from config)
  check_acceleration_bound(result_trajectory, max_acc(), min_acc());
}

// TEST 1.5: StoppingTargetBelowMaxVel_OutputStaysBelowTargetWithinAccLimits
TEST_F(NominalSmoothing, StoppingTargetBelowMaxVel_OutputStaysBelowTargetWithinAccLimits)
{
  // Publish all necessary inputs
  publish_default_inputs();
  publish_ego_state(0.0, 5.0);                                  // Ego start at x = 0, v = 5.0
  const auto input_traj = create_mock_stopping_trajectory(10);  // Target traj of v = 10.0

  // Publish input trajectory and receive smoothed trajectory
  publish_input_trajectory(input_traj);
  const auto cur_vel_lim = receive_velocity_limit();
  const auto result_trajectory = receive_smoothed_trajectory();

  // Assert check
  ASSERT_NE(cur_vel_lim, nullptr);
  EXPECT_NEAR(
    cur_vel_lim->max_velocity, max_velocity(), 1e-3);  // Initial max_velocity (from config)
  ASSERT_NE(result_trajectory, nullptr) << "Node failed to output Smoothed Trajectory.";
  EXPECT_EQ(result_trajectory->header.frame_id, "map");

  // check start from 5.0 and less than 10.0 (target)
  check_velocity_bound(result_trajectory, 5.0, 10.0);

  // check within acceleration bound (from config)
  check_acceleration_bound(result_trajectory, max_acc(), min_acc());
}

// TEST 1.6: StoppingTargetAboveMaxVel_OutputIsCappedAtMaxVelWithinAccLimits
TEST_F(NominalSmoothing, StoppingTargetAboveMaxVel_OutputIsCappedAtMaxVelWithinAccLimits)
{
  // Publish all necessary inputs
  publish_default_inputs();
  publish_ego_state(0.0, 5.0);                                  // Ego start at x = 0, v = 5.0
  const auto input_traj = create_mock_stopping_trajectory(30);  // Target traj of v = 30.0

  // Publish input trajectory and receive smoothed trajectory
  publish_input_trajectory(input_traj);
  const auto cur_vel_lim = receive_velocity_limit();
  const auto result_trajectory = receive_smoothed_trajectory();

  // Assert check
  ASSERT_NE(cur_vel_lim, nullptr);
  EXPECT_NEAR(
    cur_vel_lim->max_velocity, max_velocity(), 1e-3);  // Initial max_velocity (from config)
  ASSERT_NE(result_trajectory, nullptr) << "Node failed to output Smoothed Trajectory.";
  EXPECT_EQ(result_trajectory->header.frame_id, "map");

  // check start from 5.0 and less than config max_velocity (11.1)
  check_velocity_bound(result_trajectory, 5.0, max_velocity());

  // check within acceleration bound (from config)
  check_acceleration_bound(result_trajectory, max_acc(), min_acc());
}

// TEST 2: ExternalVelocityConstraintRespect
class ExternalVelocityConstraintRespect : public VelocitySmootherIntegrationHarness
{
};

// TEST 2.1: StraightTargetBelowMaxVelAboveExtMaxVel_OutputIsCappedAtExtMaxVelWithinAccLimits
TEST_F(
  ExternalVelocityConstraintRespect,
  StraightTargetBelowMaxVelAboveExtMaxVel_OutputIsCappedAtExtMaxVelWithinAccLimits)
{
  // Publish all necessary inputs
  publish_default_inputs();
  publish_ego_state(0.0, 5.0);                                  // Ego start at x = 0, v = 5.0
  const auto input_traj = create_mock_straight_trajectory(10);  // Target traj of v = 10.0
  constexpr double ext_max_velocity = 7.5;

  // Publish input trajectory and receive smoothed trajectory
  reset_output_messages();
  publish_velocity_limit(ext_max_velocity);
  publish_input_trajectory(input_traj);
  const auto result_trajectory = receive_smoothed_trajectory();
  const auto cur_vel_lim = receive_velocity_limit();

  // Assert check
  ASSERT_NE(cur_vel_lim, nullptr);
  EXPECT_NEAR(cur_vel_lim->max_velocity, ext_max_velocity, 1e-3);  // ext_max_velocity (7.5)
  ASSERT_NE(result_trajectory, nullptr) << "Node failed to output Smoothed Trajectory.";
  EXPECT_EQ(result_trajectory->header.frame_id, "map");

  // check start from 5.0 and less than ext_max_velocity (7.5)
  check_velocity_bound(result_trajectory, 5.0, ext_max_velocity);

  // check within acceleration bound (from config)
  check_acceleration_bound(result_trajectory, max_acc(), min_acc());
}

// TEST 2.2: CurvedTargetBelowMaxVelAboveExtMaxVel_OutputIsCappedAtExtMaxVelWithinAccLimits
TEST_F(
  ExternalVelocityConstraintRespect,
  CurvedTargetBelowMaxVelAboveExtMaxVel_OutputIsCappedAtExtMaxVelWithinAccLimits)
{
  // Publish all necessary inputs
  publish_default_inputs();
  publish_ego_state(0.0, 5.0);                                // Ego start at x = 0, v = 5.0
  const auto input_traj = create_mock_curved_trajectory(10);  // Target traj of v = 10.0
  constexpr double ext_max_velocity = 7.5;

  // Publish input trajectory and receive smoothed trajectory
  reset_output_messages();
  publish_velocity_limit(ext_max_velocity);
  publish_input_trajectory(input_traj);
  const auto result_trajectory = receive_smoothed_trajectory();
  const auto cur_vel_lim = receive_velocity_limit();

  // Assert check
  ASSERT_NE(cur_vel_lim, nullptr);
  EXPECT_NEAR(cur_vel_lim->max_velocity, ext_max_velocity, 1e-3);  // ext_max_velocity (7.5)
  ASSERT_NE(result_trajectory, nullptr) << "Node failed to output Smoothed Trajectory.";
  EXPECT_EQ(result_trajectory->header.frame_id, "map");

  // check start from 5.0 and less than ext_max_velocity (7.5)
  check_velocity_bound(result_trajectory, 5.0, ext_max_velocity);

  // check within acceleration bound (from config)
  check_acceleration_bound(result_trajectory, max_acc(), min_acc());
}

// TEST 2.3: StoppingTargetBelowMaxVelAboveExtMaxVel_OutputIsCappedAtExtMaxVelWithinAccLimits
TEST_F(
  ExternalVelocityConstraintRespect,
  StoppingTargetBelowMaxVelAboveExtMaxVel_OutputIsCappedAtExtMaxVelWithinAccLimits)
{
  // Publish all necessary inputs
  publish_default_inputs();
  publish_ego_state(0.0, 5.0);                                  // Ego start at x = 0, v = 5.0
  const auto input_traj = create_mock_stopping_trajectory(10);  // Target traj of v = 10.0
  constexpr double ext_max_velocity = 7.5;

  // Publish input trajectory and receive smoothed trajectory
  reset_output_messages();
  publish_velocity_limit(ext_max_velocity);
  publish_input_trajectory(input_traj);
  const auto result_trajectory = receive_smoothed_trajectory();
  const auto cur_vel_lim = receive_velocity_limit();

  // Assert check
  ASSERT_NE(cur_vel_lim, nullptr);
  EXPECT_NEAR(cur_vel_lim->max_velocity, ext_max_velocity, 1e-3);  // ext_max_velocity (7.5)
  ASSERT_NE(result_trajectory, nullptr) << "Node failed to output Smoothed Trajectory.";
  EXPECT_EQ(result_trajectory->header.frame_id, "map");

  // check start from 5.0 and less than ext_max_velocity (7.5)
  check_velocity_bound(result_trajectory, 5.0, ext_max_velocity);

  // check within acceleration bound (from config)
  check_acceleration_bound(result_trajectory, max_acc(), min_acc());
}

// TEST 3: StopPoint_OutputStopPointPreserved
TEST_F(VelocitySmootherIntegrationHarness, StopPoint_OutputStopPointPreserved)
{
  // Publish all necessary inputs
  publish_default_inputs();
  publish_ego_state(0.0, 5.0);                                        // Ego start at x = 0, v = 5.0
  const auto input_traj = create_mock_stopping_trajectory(10.0, 80);  // Target traj of v = 10.0

  // Publish input trajectory and receive smoothed trajectory
  publish_input_trajectory(input_traj);
  const auto result_trajectory = receive_smoothed_trajectory();

  // Assert check
  ASSERT_NE(result_trajectory, nullptr) << "Node failed to output Smoothed Trajectory.";
  EXPECT_EQ(result_trajectory->header.frame_id, "map");

  // check start from 5.0 and less than config max_velocity (11.1)
  check_velocity_bound(result_trajectory, 5.0, max_velocity());

  // check within acceleration bound (from config)
  check_acceleration_bound(result_trajectory, max_acc(), min_acc());

  // Check stop points
  {
    // find stop point
    constexpr double stop_vel_threshold = 1e-3;
    const auto stop_idx = [&]() -> std::optional<size_t> {
      for (size_t i = 0; i < result_trajectory->points.size(); ++i) {
        if (result_trajectory->points[i].longitudinal_velocity_mps < stop_vel_threshold) {
          return i;
        }
      }
      return std::nullopt;
    }();
    ASSERT_TRUE(stop_idx.has_value()) << "No stop point found in output.";

    // Setup value for testing
    constexpr double tol = 1e-3;
    const double stop_x = 40.0;  // traj has 100 points with 2 m interval, stop_range is 80 points.

    EXPECT_NEAR(result_trajectory->points[*stop_idx].pose.position.x, stop_x, tol)
      << "Stop point is not preserved";

    // Stop is maintained to the end
    for (size_t i = *stop_idx; i < result_trajectory->points.size(); ++i) {
      EXPECT_NEAR(result_trajectory->points[i].longitudinal_velocity_mps, 0.0, stop_vel_threshold);
    }
    // last value is 0.0
    EXPECT_NEAR(
      result_trajectory->points.back().longitudinal_velocity_mps, 0.0, stop_vel_threshold);
  }
}

// TEST 4: MultiCycleConsistency_OutputRemainsConsistent
TEST_F(VelocitySmootherIntegrationHarness, MultiCycleConsistency_OutputRemainsConsistent)
{
  constexpr double tol = 1e-3;
  constexpr double v_start = 5.0;
  constexpr double x_interval = 5.0;
  constexpr double v_interval = 2.0;

  // Publish all necessary inputs
  publish_default_inputs();
  publish_ego_state(0.0, v_start);                                // Ego start at x = 0, v = 5.0
  const auto input_traj = create_mock_straight_trajectory(10.0);  // Target traj of v = 10.0

  // Publish input trajectory and receive smoothed trajectory
  publish_input_trajectory(input_traj);
  const auto result_trajectory = receive_smoothed_trajectory();

  // Assert check
  ASSERT_NE(result_trajectory, nullptr) << "Node failed to output Smoothed Trajectory.";
  EXPECT_EQ(result_trajectory->header.frame_id, "map");

  Trajectory::ConstSharedPtr prev_traj;
  // test for 4 cycles
  for (size_t k = 0; k < 4; ++k) {
    // reset output
    reset_trajectory_output();

    // set next odom (ego state)
    auto odom = set_odom(x_interval * k, v_start + v_interval * k);
    publish_ego_state(odom);

    // update result_trajectory
    publish_input_trajectory(input_traj);
    const auto result_trajectory = receive_smoothed_trajectory();

    if (k > 0) {
      const auto start_idx_opt = findClosestIndex(result_trajectory, odom);
      ASSERT_TRUE(start_idx_opt.has_value());
      const auto prev_start_idx_opt = findClosestIndex(prev_traj, odom);
      ASSERT_TRUE(prev_start_idx_opt.has_value());
      // check start from that start_idx velocity and less than maximum velocity (11.1) - with
      // loosen tol (0.1)
      check_velocity_bound(
        result_trajectory, prev_traj->points[*prev_start_idx_opt].longitudinal_velocity_mps,
        max_velocity(), start_idx_opt, 0.1);

      // check start point location (should be 5.0 * k)
      EXPECT_NEAR(result_trajectory->points[*start_idx_opt].pose.position.x, 5.0 * k, tol);
    } else {
      // check start from 5.0 and less than maximum velocity (11.1)
      check_velocity_bound(result_trajectory, 5.0, max_velocity());
    }

    // check within acceleration bound (from config)
    check_acceleration_bound(result_trajectory, max_acc(), min_acc());
    prev_traj = result_trajectory;
  }
}

// TEST 5: AbnormalInputNoCrash
class AbnormalInputNoCrash : public VelocitySmootherIntegrationHarness
{
};

// TEST 5.1: EmptyInputTrajectory_NoOutput
TEST_F(AbnormalInputNoCrash, EmptyInputTrajectory_NoOutput)
{
  // Publish all necessary inputs
  publish_default_inputs();
  publish_ego_state(0.0, 5.0);  // Ego start at x = 0, v = 5.0
  // 0 point trajectory
  const auto input_traj = autoware::test_utils::generateTrajectory<Trajectory>(0, 2.0, 10.0);

  // Publish input trajectory and receive smoothed trajectory
  publish_input_trajectory(input_traj);
  const auto result_trajectory = receive_smoothed_trajectory();

  ASSERT_EQ(result_trajectory, nullptr);
}

// TEST 5.2: SinglePointInputTrajectory_NoOutput
TEST_F(AbnormalInputNoCrash, SinglePointInputTrajectory_NoOutput)
{
  // Publish all necessary inputs
  publish_default_inputs();
  publish_ego_state(0.0, 5.0);  // Ego start at x = 0, v = 5.0
  // 1 point trajectory
  const auto input_traj = autoware::test_utils::generateTrajectory<Trajectory>(1, 2.0, 10.0);

  // Publish input trajectory and receive smoothed trajectory
  publish_input_trajectory(input_traj);
  const auto result_trajectory = receive_smoothed_trajectory();

  ASSERT_EQ(result_trajectory, nullptr);
}

// TEST 5.3: OfftrackLongitudinalOdom_ProducesValidTrajectory
TEST_F(AbnormalInputNoCrash, OfftrackLongitudinalOdom_ProducesValidTrajectory)
{
  // Publish all necessary inputs
  publish_default_inputs();

  // Ego is longitudinally off-track from the trajectory
  publish_ego_state(-10.0, 5.0);  // Ego start at x = -10.0, v = 5.0

  // ordinary trajectory
  const auto input_traj = create_mock_straight_trajectory(10.0);

  // Publish input trajectory and receive smoothed trajectory
  publish_input_trajectory(input_traj);
  const auto result_trajectory = receive_smoothed_trajectory();

  ASSERT_NE(result_trajectory, nullptr) << "Node failed to output Smoothed Trajectory.";
  EXPECT_EQ(result_trajectory->header.frame_id, "map");

  // check start from 5.0 and less than config max_velocity (11.1)
  check_velocity_bound(result_trajectory, 5.0, max_velocity());

  // check within acceleration bound (from config)
  check_acceleration_bound(result_trajectory, max_acc(), min_acc());
}

// TEST 5.4: OfftrackLateralOdom_ProducesValidTrajectory
TEST_F(AbnormalInputNoCrash, OfftrackLateralOdom_ProducesValidTrajectory)
{
  // Publish all necessary inputs
  publish_default_inputs();
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.y = 10.0;
  odom.twist.twist.linear.x = 5.0;
  // Ego is laterally off-track from the trajectory
  publish_ego_state(odom);  // Ego start at x = 0.0, y = 10.0, v = 5.0

  // ordinary trajectory
  const auto input_traj = create_mock_straight_trajectory(10.0);

  // Publish input trajectory and receive smoothed trajectory
  publish_input_trajectory(input_traj);
  const auto result_trajectory = receive_smoothed_trajectory();

  ASSERT_NE(result_trajectory, nullptr) << "Node failed to output Smoothed Trajectory.";
  EXPECT_EQ(result_trajectory->header.frame_id, "map");

  // check start from 5.0 and less than config max_velocity (11.1)
  check_velocity_bound(result_trajectory, 5.0, max_velocity());

  // check within acceleration bound (from config)
  check_acceleration_bound(result_trajectory, max_acc(), min_acc());
}

}  // namespace autoware::velocity_smoother
