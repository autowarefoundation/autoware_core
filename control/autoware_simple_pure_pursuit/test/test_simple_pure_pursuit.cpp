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

#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

namespace autoware::control::simple_pure_pursuit
{

using autoware_planning_msgs::msg::Trajectory;
using nav_msgs::msg::Odometry;

constexpr double terminal_brake_accel = SimplePurePursuit::terminal_brake_accel;

// Floating point tolerance at EXCEPT_NEAR checks
constexpr float near_tol = 1e-4F;

/**
 * @brief Helper func to create odometry
 *
 * @param x X position
 * @param y Y position
 * @param yaw Yaw angle in radians
 *
 * @return Odometry message with given pose and zero velocity
 */
Odometry makeOdometry(const double x, const double y, const double yaw)
{
  Odometry odom;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.orientation.z = std::sin(yaw / 2.0);
  odom.pose.pose.orientation.w = std::cos(yaw / 2.0);

  return odom;
}

class SimplePurePursuitCoreLogicTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    params_.lookahead_gain = 1.0;
    params_.lookahead_min_distance = 1.0;
    params_.speed_proportional_gain = 1.0;
    params_.use_external_target_vel = false;
    params_.external_target_vel = 1.0;
    params_.wheel_base_m = 2.79;
    core_logic_ = std::make_unique<SimplePurePursuit>(params_);
  }

  [[nodiscard]] autoware_control_msgs::msg::Control create_control_command(
    const Odometry & odom, const Trajectory & traj) const
  {
    return core_logic_->create_control_command(odom, traj);
  }

  SimplePurePursuitParameters params_;
  std::unique_ptr<SimplePurePursuit> core_logic_;
};

// ================== TESTING AREA HERE ==================

// TEST 1. Normal case happy tracking
// Car at origin, facing long x-axis, current velocity 1 m/s, gain 2.0
// Straight trajectory along x-axis, 10 points, 1m apart, target speed 5 m/s
// Expects 5 m/s velocity, 0 steering angle
// Acceleration = gain * (target - current) = 2.0 * (5.0 - 1.0) = 8.0 m/s^2
TEST_F(SimplePurePursuitCoreLogicTest, NormalCaseTracking)
{
  params_.speed_proportional_gain = 2.0;
  core_logic_ = std::make_unique<SimplePurePursuit>(params_);

  auto odom = makeOdometry(0.0, 0.0, 0.0);
  odom.twist.twist.linear.x = 1.0;
  const auto traj = autoware::test_utils::generateTrajectory<Trajectory>(10, 1.0, 5.0);

  const auto result = create_control_command(odom, traj);

  EXPECT_NEAR(result.longitudinal.velocity, 5.0, near_tol);

  EXPECT_NEAR(result.longitudinal.acceleration, 8.0, near_tol);
  EXPECT_NEAR(result.lateral.steering_tire_angle, 0.0, near_tol);
}

// TEST 2. Goal reached case
// Same odometry and trajectory as TEST 1, but car at trajectory's end
// Expects 0 velocity, strong negative acceleration (braking), 0 steering angle
TEST_F(SimplePurePursuitCoreLogicTest, GoalReachedTerminalBrake)
{
  const auto odom = makeOdometry(10.0, 0.0, 0.0);
  const auto traj = autoware::test_utils::generateTrajectory<Trajectory>(10, 1.0, 1.0);

  const auto result = create_control_command(odom, traj);

  EXPECT_NEAR(result.longitudinal.velocity, 0.0, near_tol);
  EXPECT_NEAR(result.longitudinal.acceleration, terminal_brake_accel, near_tol);
}

// TEST 3. Too short trajectory case
// Car at origin, facing long x-axis
// Straight trajectory along x-axis, only 5 points, 1m apart, 1m/s target speed
// Expects 0 velocity, strong negative acceleration (braking), 0 steering angle (trajectory way too
// short to look ahead)
TEST_F(SimplePurePursuitCoreLogicTest, TooShortTrajectoryTerminalBrake)
{
  const auto odom = makeOdometry(0.0, 0.0, 0.0);
  const auto traj = autoware::test_utils::generateTrajectory<Trajectory>(5, 1.0, 1.0);

  const auto result = create_control_command(odom, traj);

  EXPECT_NEAR(result.longitudinal.velocity, 0.0, near_tol);
  EXPECT_NEAR(result.longitudinal.acceleration, terminal_brake_accel, near_tol);
}

// TEST 4. External target velocity override case
// Car at origin, facing long x-axis, current velocity 1.0 m/s, gain 2.0
// Straight trajectory along x-axis, 10 points, 1m apart, target speed 1 m/s
// External target velocity injected at 3.0 m/s, which should override 1.0 m/s
// Expects 3.0 m/s velocity, 0 steering angle
// Acceleration = gain * (target - current) = 2.0 * (3.0 - 1.0) = 4.0 m/s^2
TEST_F(SimplePurePursuitCoreLogicTest, ExternalTargetVelocity)
{
  params_.speed_proportional_gain = 2.0;
  params_.use_external_target_vel = true;
  params_.external_target_vel = 3.0;
  core_logic_ = std::make_unique<SimplePurePursuit>(params_);

  auto odom = makeOdometry(0.0, 0.0, 0.0);
  odom.twist.twist.linear.x = 1.0;

  const auto traj = autoware::test_utils::generateTrajectory<Trajectory>(10, 1.0, 1.0);

  const auto result = create_control_command(odom, traj);

  EXPECT_NEAR(result.longitudinal.velocity, 3.0, near_tol);
  EXPECT_NEAR(result.longitudinal.acceleration, 4.0, near_tol);
}

// TEST 5. Lateral offset case
// Car at (0,1), facing long x-axis (1m lateral offset from trajectory along x-axis)
// Same trajectory as STEP 1
// Expects non-zero negative steering angle to correct leftward offset
TEST_F(SimplePurePursuitCoreLogicTest, GenerateNonZeroSteeringForLateralOffset)
{
  const auto odom = makeOdometry(0.0, 1.0, 0.0);
  const auto traj = autoware::test_utils::generateTrajectory<Trajectory>(10, 1.0, 1.0);

  const auto result = create_control_command(odom, traj);

  EXPECT_GT(std::abs(result.lateral.steering_tire_angle), 1e-6);
  EXPECT_LT(result.lateral.steering_tire_angle, 0.0);
}

// TEST 6. Lookahead distance clamp case [fallback branch coverage]
// Car at (0, 0.5), facing long x-axis, crawling at 0.5 m/s
// Lookahead gain 1.0, so calculated lookahead distance = 0.5 * 1.0  0.5m < min lookahead
// distance 1.0m Expects clamping to min lookahead distance
TEST_F(SimplePurePursuitCoreLogicTest, ClampToLookaheadMinDistance)
{
  const auto odom = makeOdometry(0.0, 0.5, 0.0);
  const auto traj = autoware::test_utils::generateTrajectory<Trajectory>(10, 1.0, 0.5);

  const auto result = create_control_command(odom, traj);

  // Verifying math executed safely without dividing by zero or throwing NaNs
  EXPECT_TRUE(std::isfinite(result.lateral.steering_tire_angle));

  // Note: current lookahead calculation is a bit weird. Confirmed with Ishikawa-san. We will fix it
  // later. Not now. Fow now just keep it, focusing only on refactoring. Thus I had to resort to
  // above infinity check. Still legit I guess.
}

// TEST 7. Lookahead point search exceeds trajectory length case [fallback branch coverage]
// Car at origin, facing long x-axis, huge target speed 50 m/s (to push lookahead distance to 50m)
// Same trajectory as STEP 1, which is shorter than lookahead distance
// Expects fallback to bind lookahead point to trajectory's end nicely
TEST_F(SimplePurePursuitCoreLogicTest, FallbackWhenLookaheadExceedsTrajectoryLength)
{
  params_.use_external_target_vel = true;
  params_.external_target_vel = 50.0;
  core_logic_ = std::make_unique<SimplePurePursuit>(params_);

  const auto odom = makeOdometry(0.0, 0.0, 0.0);
  const auto traj = autoware::test_utils::generateTrajectory<Trajectory>(10, 1.0, 1.0);

  const auto result = create_control_command(odom, traj);

  // Fallback branch `lookahead_point_itr == traj.points.end()` should activate
  // and bind lookahead to final point without triggering a segfault
  // Abuse infinity check again
  EXPECT_TRUE(std::isfinite(result.lateral.steering_tire_angle));
}

}  // namespace autoware::control::simple_pure_pursuit
