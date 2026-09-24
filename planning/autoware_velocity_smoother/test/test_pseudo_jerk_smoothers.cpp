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
// Floating point tolerance at EXPECT_NEAR and similar checks
constexpr float near_tol = 1e-4F;
}  // namespace

namespace
{
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

TEST(TestPseudoJerkSmoothers, L2PseudoJerkSmootherTest)
{
  rclcpp::init(0, nullptr);
  auto node =
    std::make_shared<rclcpp::Node>("test_l2_smoother_node", makeNodeOptions("L2.param.yaml"));
  auto time_keeper = std::make_shared<autoware_utils_debug::TimeKeeper>();

  L2PseudoJerkSmoother smoother(*node, time_keeper);

  // Test Param getters and setters
  auto p = smoother.getParam();
  EXPECT_GT(p.pseudo_jerk_weight, 0.0);
  p.pseudo_jerk_weight += 10.0;
  smoother.setParam(p);
  EXPECT_NEAR(smoother.getParam().pseudo_jerk_weight, p.pseudo_jerk_weight, near_tol);

  // Test apply with insufficient trajectory (single point)
  TrajectoryPoints single_point = {createPoint(0.0, 0.0, 0.0, 0.0, 5.0)};
  TrajectoryPoints output;
  std::vector<TrajectoryPoints> debug_trajectories;
  EXPECT_FALSE(smoother.apply(5.0, 0.0, single_point, output, debug_trajectories, false));

  // Test apply with stopped vehicle
  TrajectoryPoints stopped_input = {
    createPoint(0.0, 0.0, 0.0, 0.0, 0.0), createPoint(1.0, 0.0, 0.0, 0.0, 0.0)};
  EXPECT_TRUE(smoother.apply(0.0, 0.0, stopped_input, output, debug_trajectories, false));

  // Test apply with nominal trajectory
  TrajectoryPoints nominal_input = createTrajectory(10.0, 50.0, 1.0);
  nominal_input.back().longitudinal_velocity_mps = 0.0;  // Stop point
  EXPECT_TRUE(smoother.apply(5.0, 0.0, nominal_input, output, debug_trajectories, true));
  EXPECT_EQ(output.size(), nominal_input.size());
  EXPECT_TRUE(debug_trajectories.empty());

  // Test resampleTrajectory
  geometry_msgs::msg::Pose current_pose;
  current_pose.orientation.w = 1.0;
  const auto resampled = smoother.resampleTrajectory(nominal_input, 5.0, current_pose, 3.0, 1.0);
  EXPECT_GT(resampled.size(), 0u);

  rclcpp::shutdown();
}

TEST(TestPseudoJerkSmoothers, LinfPseudoJerkSmootherTest)
{
  rclcpp::init(0, nullptr);
  auto node =
    std::make_shared<rclcpp::Node>("test_linf_smoother_node", makeNodeOptions("Linf.param.yaml"));
  auto time_keeper = std::make_shared<autoware_utils_debug::TimeKeeper>();

  LinfPseudoJerkSmoother smoother(*node, time_keeper);

  // Test Param getters and setters
  auto p = smoother.getParam();
  EXPECT_GT(p.pseudo_jerk_weight, 0.0);
  p.pseudo_jerk_weight += 20.0;
  smoother.setParam(p);
  EXPECT_NEAR(smoother.getParam().pseudo_jerk_weight, p.pseudo_jerk_weight, near_tol);

  // Test apply with insufficient points
  TrajectoryPoints single_point = {createPoint(0.0, 0.0, 0.0, 0.0, 5.0)};
  TrajectoryPoints output;
  std::vector<TrajectoryPoints> debug_trajectories;
  EXPECT_FALSE(smoother.apply(5.0, 0.0, single_point, output, debug_trajectories, false));

  // Test apply with nominal trajectory
  TrajectoryPoints nominal_input = createTrajectory(10.0, 50.0, 1.0);
  nominal_input.back().longitudinal_velocity_mps = 0.0;  // Stop point
  EXPECT_TRUE(smoother.apply(5.0, 0.0, nominal_input, output, debug_trajectories, true));
  EXPECT_EQ(output.size(), nominal_input.size());

  // Test resampleTrajectory
  geometry_msgs::msg::Pose current_pose;
  current_pose.orientation.w = 1.0;
  const auto resampled = smoother.resampleTrajectory(nominal_input, 5.0, current_pose, 3.0, 1.0);
  EXPECT_GT(resampled.size(), 0u);

  rclcpp::shutdown();
}

TEST(TestPseudoJerkSmoothers, AnalyticalJerkConstrainedSmootherTest)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>(
    "test_analytical_smoother_node", makeNodeOptions("Analytical.param.yaml"));
  auto time_keeper = std::make_shared<autoware_utils_debug::TimeKeeper>();

  AnalyticalJerkConstrainedSmoother smoother(*node, time_keeper);

  // Test Param getters and setters
  auto p = smoother.getParam();
  p.resample.ds_resample = 0.5;
  smoother.setParam(p);
  EXPECT_NEAR(smoother.getParam().resample.ds_resample, 0.5, near_tol);

  // Test apply with single point
  TrajectoryPoints single_point = {createPoint(0.0, 0.0, 0.0, 0.0, 5.0)};
  TrajectoryPoints output;
  std::vector<TrajectoryPoints> debug_trajectories;
  EXPECT_TRUE(smoother.apply(5.0, 0.0, single_point, output, debug_trajectories, false));

  // Test apply with nominal trajectory
  TrajectoryPoints nominal_input = createTrajectory(10.0, 50.0, 1.0);
  nominal_input.back().longitudinal_velocity_mps = 0.0;  // Stop point
  EXPECT_TRUE(smoother.apply(5.0, 0.0, nominal_input, output, debug_trajectories, true));
  EXPECT_GT(output.size(), 0u);

  // Test applyLateralAccelerationFilter
  const auto filtered =
    smoother.applyLateralAccelerationFilter(nominal_input, 5.0, 0.0, true, true, 1.0);
  EXPECT_GT(filtered.size(), 0u);

  // Test resampleTrajectory
  geometry_msgs::msg::Pose current_pose;
  current_pose.orientation.w = 1.0;
  const auto resampled = smoother.resampleTrajectory(nominal_input, 5.0, current_pose, 3.0, 1.0);
  EXPECT_GT(resampled.size(), 0u);

  rclcpp::shutdown();
}
