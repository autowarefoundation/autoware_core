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

#include "autoware/velocity_smoother/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.hpp"
#include "autoware/velocity_smoother/smoother/l2_pseudo_jerk_smoother.hpp"
#include "autoware/velocity_smoother/smoother/linf_pseudo_jerk_smoother.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>

#include <memory>
#include <string>
#include <vector>

using autoware::velocity_smoother::AnalyticalJerkConstrainedSmoother;
using autoware::velocity_smoother::L2PseudoJerkSmoother;
using autoware::velocity_smoother::LinfPseudoJerkSmoother;
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

namespace
{
constexpr float near_tol = 1e-4F;

TrajectoryPoint createPoint(double x, double y, double z, double yaw, double velocity)
{
  TrajectoryPoint p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.position.z = z;

  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, yaw);
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();

  p.longitudinal_velocity_mps = velocity;
  p.acceleration_mps2 = 0.0;
  return p;
}

TrajectoryPoints createTrajectory(double velocity, double length, double step)
{
  TrajectoryPoints traj;
  for (double x = 0.0; x <= length; x += step) {
    traj.push_back(createPoint(x, 0.0, 0.0, 0.0, velocity));
  }
  return traj;
}

TrajectoryPoints createTrajectoryWithStopAtEnd(double velocity, double length, double step)
{
  auto traj = createTrajectory(velocity, length, step);
  traj.back().longitudinal_velocity_mps = 0.0;
  return traj;
}

rclcpp::NodeOptions makeNodeOptions(const std::string & algorithm_param_file)
{
  rclcpp::NodeOptions options;
  const auto test_utils_dir = ament_index_cpp::get_package_share_directory("autoware_test_utils");
  const auto smoother_dir =
    ament_index_cpp::get_package_share_directory("autoware_velocity_smoother");

  options.arguments({
    "--ros-args",
    "--params-file",
    test_utils_dir + "/config/test_common.param.yaml",
    "--params-file",
    test_utils_dir + "/config/test_nearest_search.param.yaml",
    "--params-file",
    test_utils_dir + "/config/test_vehicle_info.param.yaml",
    "--params-file",
    smoother_dir + "/config/default_velocity_smoother.param.yaml",
    "--params-file",
    smoother_dir + "/config/default_common.param.yaml",
    "--params-file",
    smoother_dir + "/config/" + algorithm_param_file,
  });
  return options;
}
}  // namespace

class SmootherTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  template <typename SmootherType>
  std::unique_ptr<SmootherType> createSmoother(const std::string & algorithm_name)
  {
    auto node = std::make_shared<rclcpp::Node>(
      "test_" + algorithm_name + "_node", makeNodeOptions(algorithm_name + ".param.yaml"));
    auto time_keeper = std::make_shared<autoware_utils_debug::TimeKeeper>();
    return std::make_unique<SmootherType>(*node, time_keeper);
  }

  static void expect_initial_velocity_kept(
    const TrajectoryPoints & trajectory, const double initial_velocity)
  {
    ASSERT_FALSE(trajectory.empty());
    EXPECT_NEAR(trajectory.front().longitudinal_velocity_mps, initial_velocity, velocity_tolerance);
  }

  static void expect_within_velocity_limit(
    const TrajectoryPoints & input, const TrajectoryPoints & output)
  {
    ASSERT_EQ(input.size(), output.size());
    for (size_t i = 0; i < output.size(); ++i) {
      SCOPED_TRACE("point index " + std::to_string(i));
      EXPECT_LE(
        output.at(i).longitudinal_velocity_mps,
        input.at(i).longitudinal_velocity_mps + velocity_tolerance);
    }
  }

  static void expect_goal_point_velocity_is_zero(const TrajectoryPoints & trajectory)
  {
    ASSERT_FALSE(trajectory.empty());
    EXPECT_NEAR(trajectory.back().longitudinal_velocity_mps, 0.0, stop_velocity_tolerance);
  }

  static constexpr double velocity_tolerance = 0.05;
  static constexpr double stop_velocity_tolerance = 0.05;

  TrajectoryPoints output_;
  std::vector<TrajectoryPoints> debug_trajectories_;
};

// ========================== L2 pseudo jerk smoother tests ==========================

TEST_F(SmootherTest, L2SmoothsVelocityToStopPointKeepConstraint)
{
  // Arrange
  const auto smoother = createSmoother<L2PseudoJerkSmoother>("L2");
  const auto input = createTrajectoryWithStopAtEnd(10.0, 50.0, 1.0);

  // Act
  const bool is_success = smoother->apply(5.0, 0.0, input, output_, debug_trajectories_, true);

  // Assert
  ASSERT_TRUE(is_success);
  expect_initial_velocity_kept(output_, 5.0);
  expect_within_velocity_limit(input, output_);
  expect_goal_point_velocity_is_zero(output_);
  EXPECT_TRUE(debug_trajectories_.empty());
}

TEST_F(SmootherTest, L2RejectsInsufficientTrajectoryPoints)
{
  // Arrange
  const auto smoother = createSmoother<L2PseudoJerkSmoother>("L2");
  const TrajectoryPoints single_point = {createPoint(0.0, 0.0, 0.0, 0.0, 5.0)};

  // Act
  const bool is_success =
    smoother->apply(5.0, 0.0, single_point, output_, debug_trajectories_, false);

  // Assert
  EXPECT_FALSE(is_success);
}

TEST_F(SmootherTest, L2SucceedsForStoppedVehicle)
{
  // Arrange
  const auto smoother = createSmoother<L2PseudoJerkSmoother>("L2");
  const TrajectoryPoints stopped_input = {
    createPoint(0.0, 0.0, 0.0, 0.0, 0.0), createPoint(1.0, 0.0, 0.0, 0.0, 0.0)};

  // Act
  const bool is_success =
    smoother->apply(0.0, 0.0, stopped_input, output_, debug_trajectories_, false);

  // Assert
  EXPECT_TRUE(is_success);
  expect_goal_point_velocity_is_zero(output_);
}

TEST_F(SmootherTest, L2UpdatesPseudoJerkWeightParameter)
{
  // Arrange
  const auto smoother = createSmoother<L2PseudoJerkSmoother>("L2");
  auto input_param = smoother->getParam();
  constexpr double test_weight = 5.0;
  input_param.pseudo_jerk_weight = test_weight;

  // Act
  smoother->setParam(input_param);
  const auto output_param = smoother->getParam();

  // Assert
  EXPECT_NEAR(output_param.pseudo_jerk_weight, test_weight, near_tol);
}

// ========================== Linf pseudo jerk smoother tests ==========================

TEST_F(SmootherTest, LinfSmoothsVelocityToStopPointKeepConstraint)
{
  // Arrange
  const auto smoother = createSmoother<LinfPseudoJerkSmoother>("Linf");
  const auto input = createTrajectoryWithStopAtEnd(10.0, 50.0, 1.0);

  // Act
  const bool is_success = smoother->apply(5.0, 0.0, input, output_, debug_trajectories_, true);

  // Assert
  ASSERT_TRUE(is_success);
  expect_initial_velocity_kept(output_, 5.0);
  expect_within_velocity_limit(input, output_);
  expect_goal_point_velocity_is_zero(output_);
}

TEST_F(SmootherTest, LinfRejectsInsufficientTrajectoryPoints)
{
  // Arrange
  const auto smoother = createSmoother<LinfPseudoJerkSmoother>("Linf");
  const TrajectoryPoints single_point = {createPoint(0.0, 0.0, 0.0, 0.0, 5.0)};

  // Act
  const bool is_success =
    smoother->apply(5.0, 0.0, single_point, output_, debug_trajectories_, false);

  // Assert
  EXPECT_FALSE(is_success);
}

TEST_F(SmootherTest, LinfUpdatesPseudoJerkWeightParameter)
{
  // Arrange
  const auto smoother = createSmoother<LinfPseudoJerkSmoother>("Linf");
  auto input_param = smoother->getParam();
  constexpr double test_weight = 5.0;
  input_param.pseudo_jerk_weight = test_weight;

  // Act
  smoother->setParam(input_param);
  const auto output_param = smoother->getParam();

  // Assert
  EXPECT_NEAR(output_param.pseudo_jerk_weight, test_weight, near_tol);
}

// ========================== Analytical jerk constrained smoother tests ==========================

TEST_F(SmootherTest, AnalyticalSmoothsVelocityToStopPointKeepConstraint)
{
  // Arrange
  const auto smoother = createSmoother<AnalyticalJerkConstrainedSmoother>("Analytical");
  const auto input = createTrajectoryWithStopAtEnd(10.0, 50.0, 1.0);

  // Act
  const bool is_success = smoother->apply(5.0, 0.0, input, output_, debug_trajectories_, true);

  // Assert
  ASSERT_TRUE(is_success);
  expect_initial_velocity_kept(output_, 5.0);
  expect_within_velocity_limit(input, output_);
  expect_goal_point_velocity_is_zero(output_);
}

TEST_F(SmootherTest, AnalyticalHandlesSinglePointTrajectory)
{
  // Arrange
  const auto smoother = createSmoother<AnalyticalJerkConstrainedSmoother>("Analytical");
  const TrajectoryPoints single_point = {createPoint(0.0, 0.0, 0.0, 0.0, 5.0)};

  // Act
  const bool is_success =
    smoother->apply(5.0, 0.0, single_point, output_, debug_trajectories_, false);

  // Assert
  EXPECT_TRUE(is_success);
}

TEST_F(SmootherTest, AnalyticalFiltersLateralAcceleration)
{
  // Arrange
  const auto smoother = createSmoother<AnalyticalJerkConstrainedSmoother>("Analytical");
  const auto input = createTrajectory(10.0, 50.0, 1.0);

  // Act
  const auto filtered = smoother->applyLateralAccelerationFilter(input, 5.0, 0.0, true, true, 1.0);

  // Assert
  ASSERT_FALSE(filtered.empty());
  EXPECT_LE(filtered.front().longitudinal_velocity_mps, 10.0 + velocity_tolerance);
}

TEST_F(SmootherTest, AnalyticalUpdatesResampleParameter)
{
  // Arrange
  const auto smoother = createSmoother<AnalyticalJerkConstrainedSmoother>("Analytical");
  auto input_param = smoother->getParam();
  constexpr double test_ds = 0.5;
  input_param.resample.ds_resample = test_ds;

  // Act
  smoother->setParam(input_param);
  const auto output_param = smoother->getParam();

  // Assert
  EXPECT_NEAR(output_param.resample.ds_resample, test_ds, near_tol);
}

// ========================== Common trajectory resampling test ==========================

TEST_F(SmootherTest, ResamplesTrajectoryAroundPose)
{
  // Arrange
  const auto smoother = createSmoother<L2PseudoJerkSmoother>("L2");
  const auto input = createTrajectory(10.0, 50.0, 1.0);

  geometry_msgs::msg::Pose current_pose;
  current_pose.orientation.w = 1.0;

  // Act
  const auto resampled = smoother->resampleTrajectory(input, 5.0, current_pose, 3.0, 1.0);

  // Assert
  EXPECT_FALSE(resampled.empty());
}
