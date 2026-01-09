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

#include "autoware/velocity_smoother/node.hpp"
#include "velocity_smoother_test_accessor.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

using autoware::velocity_smoother::VelocitySmootherNode;

namespace
{
std::shared_ptr<VelocitySmootherNode> createNode()
{
  auto node_options = rclcpp::NodeOptions{};
  node_options.append_parameter_override("algorithm_type", "JerkFiltered");
  node_options.append_parameter_override("publish_debug_trajs", false);

  const auto autoware_test_utils_dir =
    ament_index_cpp::get_package_share_directory("autoware_test_utils");
  const auto velocity_smoother_dir =
    ament_index_cpp::get_package_share_directory("autoware_velocity_smoother");

  node_options.arguments({
    "--ros-args",
    "--params-file",
    autoware_test_utils_dir + "/config/test_common.param.yaml",
    "--params-file",
    autoware_test_utils_dir + "/config/test_nearest_search.param.yaml",
    "--params-file",
    autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml",
    "--params-file",
    velocity_smoother_dir + "/config/default_velocity_smoother.param.yaml",
    "--params-file",
    velocity_smoother_dir + "/config/default_common.param.yaml",
    "--params-file",
    velocity_smoother_dir + "/config/JerkFiltered.param.yaml",
  });

  return std::make_shared<VelocitySmootherNode>(node_options);
}
}  // namespace

class VelocitySmootherCalcExternalVelocityLimit : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(VelocitySmootherCalcExternalVelocityLimit, PublishesExternalVelocityLimit)
{
  auto node = createNode();
  autoware::velocity_smoother::test::VelocitySmootherTestAccessor accessor{*node};

  // Subscribe to the published velocity limit so we can assert the published value.
  using autoware_internal_planning_msgs::msg::VelocityLimit;
  auto sub_node = std::make_shared<rclcpp::Node>("velocity_smoother_limit_test_sub");
  std::optional<VelocityLimit> last_published_limit;
  const auto qos = rclcpp::QoS(1).transient_local();
  auto sub = sub_node->create_subscription<VelocityLimit>(
    "/velocity_smoother/output/current_velocity_limit_mps", qos,
    [&last_published_limit](VelocityLimit::ConstSharedPtr msg) { last_published_limit = *msg; });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(sub_node);

  {
    const auto start = node->now();
    while ((node->now() - start).seconds() < 1.0 && rclcpp::ok()) {
      exec.spin_some();
    }
  }
  last_published_limit.reset();

  // Minimal setup: publishing happens even on the first call (no prev output / closest point).
  auto external = std::make_shared<autoware_internal_planning_msgs::msg::VelocityLimit>();
  external->max_velocity = 2.0;
  external->use_constraints = false;
  accessor.setExternalVelocityLimitPtr(external);

  accessor.calcExternalVelocityLimit();

  {
    const auto start = node->now();
    while (!last_published_limit.has_value() && (node->now() - start).seconds() < 1.0 &&
           rclcpp::ok()) {
      exec.spin_some();
    }
  }

  ASSERT_TRUE(last_published_limit.has_value())
    << "Expected a VelocityLimit message to be published.";
  EXPECT_NEAR(last_published_limit->max_velocity, external->max_velocity, 1e-6)
    << "Expected pub_velocity_limit_ to publish external_velocity_limit_ptr_->max_velocity.";
}

// Regression tests for calcExternalVelocityLimit() v0 velocity must be abs()
// calcExternalVelocityLimit() uses v0 from current_closest_point_from_prev_output_ to compute
// max_velocity_with_deceleration_ and distance to insert an external velocity limit.
//
// If v0 can be negative (e.g. numerical / edge-case sign), using it directly can propagate
// a negative max_velocity_with_deceleration_ and break downstream velocity limiting.
//
// Fix: v0 must be std::fabs(prev_output_velocity).
TEST_F(VelocitySmootherCalcExternalVelocityLimit, V0IsTreatedAsAbsoluteValue)
{
  auto node = createNode();
  autoware::velocity_smoother::test::VelocitySmootherTestAccessor accessor{*node};

  auto odom = std::make_shared<nav_msgs::msg::Odometry>();
  odom->twist.twist.linear.x = 10.0;
  accessor.setCurrentOdometry(odom);

  auto accel = std::make_shared<geometry_msgs::msg::AccelWithCovarianceStamped>();
  accel->accel.accel.linear.x = 0.0;
  accessor.setCurrentAcceleration(accel);
  auto external = std::make_shared<autoware_internal_planning_msgs::msg::VelocityLimit>();
  external->max_velocity = 2.0;
  external->use_constraints = false;
  accessor.setExternalVelocityLimitPtr(external);

  accessor.setExternalVelocityLimitVelocity(10.0);

  accessor.setPrevOutputSize(2);
  {
    auto closest = autoware_planning_msgs::msg::TrajectoryPoint{};
    closest.longitudinal_velocity_mps = -5.0;
    closest.acceleration_mps2 = 0.0;
    accessor.setCurrentClosestPointFromPrevOutput(closest);
  }

  accessor.calcExternalVelocityLimit();

  EXPECT_GT(accessor.maxVelocityWithDeceleration(), 0.0)
    << "maxVelocityWithDeceleration() must be positive when v0 is treated as absolute value. "
    << "Got: " << accessor.maxVelocityWithDeceleration();

  autoware::velocity_smoother::TrajectoryPoints traj(20);
  std::generate(traj.begin(), traj.end(), [n = 0]() mutable {
    autoware_planning_msgs::msg::TrajectoryPoint p;
    p.pose.position.x = static_cast<double>(n++);
    p.pose.position.y = 0.0;
    p.pose.position.z = 0.0;
    p.longitudinal_velocity_mps = 10.0F;
    return p;
  });

  accessor.applyExternalVelocityLimit(traj);

  ASSERT_FALSE(traj.empty());
  for (const auto & p : traj) {
    EXPECT_GE(p.longitudinal_velocity_mps, -1e-6F)
      << "External velocity limiting must not introduce negative velocities.";
  }

  ASSERT_GT(
    traj.front().longitudinal_velocity_mps, static_cast<float>(external->max_velocity) + 1e-3F)
    << "Expected the start of the trajectory not to be immediately clamped to the external max. "
    << "This indicates v0 is treated as |v0| when computing the pre-deceleration limit.";
}
