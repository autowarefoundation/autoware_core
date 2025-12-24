// Copyright 2024 Tier IV, Inc.
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

#include "../src/simple_pure_pursuit.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <memory>

namespace autoware::control::simple_pure_pursuit
{
Odometry makeOdometry(const double x, const double y, const double yaw)
{
  Odometry odom;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.orientation.z = std::sin(yaw / 2);
  odom.pose.pose.orientation.w = std::cos(yaw / 2);
  return odom;
}

class SimplePurePursuitNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    const auto autoware_test_utils_dir =
      ament_index_cpp::get_package_share_directory("autoware_test_utils");
    const auto autoware_simple_pure_pursuit_dir =
      ament_index_cpp::get_package_share_directory("autoware_simple_pure_pursuit");

    auto node_options = rclcpp::NodeOptions{};
    autoware::test_utils::updateNodeOptions(
      node_options, {autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml",
                     autoware_simple_pure_pursuit_dir + "/config/simple_pure_pursuit.param.yaml"});

    node_ = std::make_shared<SimplePurePursuitNode>(node_options);
  }

  void TearDown() override { rclcpp::shutdown(); }

  autoware_control_msgs::msg::Control create_control_command(
    const Odometry & odom, const Trajectory & traj) const
  {
    return node_->create_control_command(odom, traj);
  }

  autoware_control_msgs::msg::Longitudinal calc_longitudinal_control(
    const Odometry & odom, const double target_longitudinal_vel) const
  {
    return node_->calc_longitudinal_control(odom, target_longitudinal_vel);
  }

  autoware_control_msgs::msg::Lateral calc_lateral_control(
    const Odometry & odom, const Trajectory & traj, const double target_longitudinal_vel,
    const size_t closest_traj_point_idx) const
  {
    return node_->calc_lateral_control(odom, traj, target_longitudinal_vel, closest_traj_point_idx);
  }

  double speed_proportional_gain() const { return node_->speed_proportional_gain_; }

private:
  std::shared_ptr<SimplePurePursuitNode> node_;
};

TEST_F(SimplePurePursuitNodeTest, create_control_command)
{
  {  // normal case
    const auto odom = makeOdometry(0.0, 0.0, 0.0);
    const auto traj = autoware::test_utils::generateTrajectory<Trajectory>(10, 1.0, 1.0);

    const auto result = create_control_command(odom, traj);

    EXPECT_DOUBLE_EQ(result.longitudinal.velocity, 1.0);
    EXPECT_DOUBLE_EQ(result.longitudinal.acceleration, speed_proportional_gain() * 1.0);
  }

  {  // ego reached goal
    const auto odom = makeOdometry(10.0, 0.0, 0.0);
    const auto traj = autoware::test_utils::generateTrajectory<Trajectory>(10, 1.0, 1.0);

    const auto result = create_control_command(odom, traj);

    EXPECT_DOUBLE_EQ(result.longitudinal.velocity, 0.0);
    EXPECT_DOUBLE_EQ(result.longitudinal.acceleration, -10.0);
  }

  {  // reference trajectory is too short
    const auto odom = makeOdometry(0.0, 0.0, 0.0);
    const auto traj = autoware::test_utils::generateTrajectory<Trajectory>(5, 1.0, 1.0);

    const auto result = create_control_command(odom, traj);

    EXPECT_DOUBLE_EQ(result.longitudinal.velocity, 0.0);
    EXPECT_DOUBLE_EQ(result.longitudinal.acceleration, -10.0);
  }

  {  // empty trajectory
    const auto odom = makeOdometry(0.0, 0.0, 0.0);
    Trajectory traj;  // empty trajectory

    const auto result = create_control_command(odom, traj);

    EXPECT_DOUBLE_EQ(result.longitudinal.velocity, 0.0);
    EXPECT_DOUBLE_EQ(result.longitudinal.acceleration, 0.0);
  }
}

TEST_F(SimplePurePursuitNodeTest, calc_longitudinal_control)
{
  {  // normal case
    const auto odom = makeOdometry(0.0, 0.0, 0.0);
    const auto target_longitudinal_vel = 1.0;

    const auto result = calc_longitudinal_control(odom, target_longitudinal_vel);

    EXPECT_DOUBLE_EQ(result.velocity, 1.0);
    EXPECT_DOUBLE_EQ(result.acceleration, speed_proportional_gain() * 1.0);
  }
}

TEST_F(SimplePurePursuitNodeTest, calc_lateral_control)
{
  const auto traj = autoware::test_utils::generateTrajectory<Trajectory>(10, 1.0);

  {  // normal case
    const auto odom = makeOdometry(0.0, 0.0, 0.0);
    const auto target_longitudinal_vel = 1.0;
    const size_t closest_traj_point_idx = 0;

    const auto result =
      calc_lateral_control(odom, traj, target_longitudinal_vel, closest_traj_point_idx);

    EXPECT_DOUBLE_EQ(result.steering_tire_angle, 0.0f);
  }

  {  // lookahead distance exceeds remaining trajectory length
    const auto odom = makeOdometry(0.0, 0.0, 0.0);
    const auto target_longitudinal_vel = 2.0;
    const size_t closest_traj_point_idx = 8;

    const auto result =
      calc_lateral_control(odom, traj, target_longitudinal_vel, closest_traj_point_idx);

    EXPECT_DOUBLE_EQ(result.steering_tire_angle, 0.0f);
  }
}
}  // namespace autoware::control::simple_pure_pursuit
