// Copyright 2023-2026 TIER IV, Inc.
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

#include "autoware/motion_utils/distance/distance.hpp"
#include "gtest/gtest.h"
namespace
{
using autoware::motion_utils::calcDecelDistWithJerkAndAccConstraints;
using autoware::motion_utils::calculate_stop_distance;

constexpr double epsilon = 1e-3;

TEST(distance, calcDecelDistWithJerkAndAccConstraints)
{
  // invalid velocity
  {
    constexpr double current_vel = 16.7;
    constexpr double target_vel = 20.0;
    constexpr double current_acc = 0.0;
    constexpr double acc_min = -0.5;
    constexpr double jerk_acc = 1.0;
    constexpr double jerk_dec = -0.5;

    const auto dist = calcDecelDistWithJerkAndAccConstraints(
      current_vel, target_vel, current_acc, acc_min, jerk_acc, jerk_dec);

    EXPECT_FALSE(dist);
  }

  // normal stop
  {
    constexpr double current_vel = 16.7;
    constexpr double target_vel = 0.0;
    constexpr double current_acc = 0.0;
    constexpr double acc_min = -0.5;
    constexpr double jerk_acc = 1.0;
    constexpr double jerk_dec = -0.5;

    constexpr double expected_dist = 287.224;
    const auto dist = calcDecelDistWithJerkAndAccConstraints(
      current_vel, target_vel, current_acc, acc_min, jerk_acc, jerk_dec);
    EXPECT_NEAR(expected_dist, *dist, epsilon);
  }

  // sudden stop
  {
    constexpr double current_vel = 16.7;
    constexpr double target_vel = 0.0;
    constexpr double current_acc = 0.0;
    constexpr double acc_min = -2.5;
    constexpr double jerk_acc = 1.5;
    constexpr double jerk_dec = -1.5;

    constexpr double expected_dist = 69.6947;
    const auto dist = calcDecelDistWithJerkAndAccConstraints(
      current_vel, target_vel, current_acc, acc_min, jerk_acc, jerk_dec);
    EXPECT_NEAR(expected_dist, *dist, epsilon);
  }

  // normal deceleration
  {
    constexpr double current_vel = 16.7;
    constexpr double target_vel = 10.0;
    constexpr double current_acc = 0.0;
    constexpr double acc_min = -0.5;
    constexpr double jerk_acc = 1.0;
    constexpr double jerk_dec = -0.5;

    constexpr double expected_dist = 189.724;
    const auto dist = calcDecelDistWithJerkAndAccConstraints(
      current_vel, target_vel, current_acc, acc_min, jerk_acc, jerk_dec);
    EXPECT_NEAR(expected_dist, *dist, epsilon);
  }

  // sudden deceleration
  {
    constexpr double current_vel = 16.7;
    constexpr double target_vel = 10.0;
    constexpr double current_acc = 0.0;
    constexpr double acc_min = -2.5;
    constexpr double jerk_acc = 1.5;
    constexpr double jerk_dec = -1.5;

    constexpr double expected_dist = 58.028;
    const auto dist = calcDecelDistWithJerkAndAccConstraints(
      current_vel, target_vel, current_acc, acc_min, jerk_acc, jerk_dec);
    EXPECT_NEAR(expected_dist, *dist, epsilon);
  }

  // current_acc is lower than acc_min
  {
    constexpr double current_vel = 16.7;
    constexpr double target_vel = 0.0;
    constexpr double current_acc = -2.5;
    constexpr double acc_min = -0.5;
    constexpr double jerk_acc = 1.0;
    constexpr double jerk_dec = -0.5;

    constexpr double expected_dist = 217.429;
    const auto dist = calcDecelDistWithJerkAndAccConstraints(
      current_vel, target_vel, current_acc, acc_min, jerk_acc, jerk_dec);
    EXPECT_NEAR(expected_dist, *dist, epsilon);
  }

  // need to decelerate
  {
    constexpr double current_vel = 10.0;
    constexpr double target_vel = 0.5;
    constexpr double current_acc = -2.0;
    constexpr double acc_min = -3.0;
    constexpr double jerk_acc = 1.0;
    constexpr double jerk_dec = -0.5;

    constexpr double expected_dist = 21.3333;
    const auto dist = calcDecelDistWithJerkAndAccConstraints(
      current_vel, target_vel, current_acc, acc_min, jerk_acc, jerk_dec);
    EXPECT_NEAR(expected_dist, *dist, epsilon);
  }

  // no need to decelerate
  {
    constexpr double current_vel = 16.7;
    constexpr double target_vel = 16.7;
    constexpr double current_acc = 0.0;
    constexpr double acc_min = -0.5;
    constexpr double jerk_acc = 1.0;
    constexpr double jerk_dec = -0.5;

    constexpr double expected_dist = 0.0;
    const auto dist = calcDecelDistWithJerkAndAccConstraints(
      current_vel, target_vel, current_acc, acc_min, jerk_acc, jerk_dec);
    EXPECT_NEAR(expected_dist, *dist, epsilon);
  }
}

TEST(CalculateStopDistanceTest, ReturnsZeroWhenAlreadyStopped)
{
  auto result = calculate_stop_distance(0.0, 0.0, -4.0, -10.0, 1.0);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result.value(), 0.0);
}

TEST(CalculateStopDistanceTest, ReturnsZeroWhenReversing)
{
  auto result = calculate_stop_distance(-2.5, 0.0, -4.0, -10.0, 1.0);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result.value(), 0.0);
}

TEST(CalculateStopDistanceTest, FailsOnInvalidKinematicLimits)
{
  // Positive acceleration limit (invalid for braking)
  EXPECT_FALSE(calculate_stop_distance(10.0, 0.0, 2.0, -10.0).has_value());

  // Positive jerk limit (invalid for braking)
  EXPECT_FALSE(calculate_stop_distance(10.0, 0.0, -4.0, 5.0).has_value());

  // Zero limits (would cause division by zero)
  EXPECT_FALSE(calculate_stop_distance(10.0, 0.0, 0.0, -10.0).has_value());
  EXPECT_FALSE(calculate_stop_distance(10.0, 0.0, -4.0, 0.0).has_value());
}

TEST(CalculateStopDistanceTest, Phase1Stop_StopsDuringDelayPhase)
{
  // v0 = 2.0 m/s, a0 = -2.0 m/s^2. Delay = 2.0s.
  // Vehicle will naturally stop at t = 1.0s, completely within the delay phase.
  // Distance = v0*t + 0.5*a0*t^2 = 2.0(1) + 0.5(-2)(1) = 1.0 m.
  auto result = calculate_stop_distance(2.0, -2.0, -5.0, -2.0, 2.0);

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result.value(), 1.0, epsilon);
}

TEST(CalculateStopDistanceTest, Phase2Stop_StopsDuringJerkPhase)
{
  // v0 = 1.0 m/s, a0 = 0.0. No delay. Limits: a = -5.0, j = -2.0.
  // Vehicle stops before reaching max decel (-5.0).
  // Solves to t = 1.0s. Distance = 1.0(1) + 1/6(-2)(1^3) = 1 - 0.3333 = 0.6666... m.
  auto result = calculate_stop_distance(1.0, 0.0, -5.0, -2.0, 0.0);

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result.value(), 2.0 / 3.0, epsilon);
}

TEST(CalculateStopDistanceTest, Phase3Stop_StandardThreePhaseStop)
{
  // Standard high-speed stop.
  // v0 = 10.0, a0 = 0.0. Delay = 1.0s. Limits: a = -2.0, j = -2.0.
  // Phase 1 (Delay): t = 1.0s, x1 = 10.0m, v1 = 10.0m/s.
  // Phase 2 (Jerk) : t = 1.0s to reach a=-2.0. x2 = 9.6666m, v2 = 9.0m/s.
  // Phase 3 (Decel): from v2=9.0 down to 0 at a=-2.0. x3 = 20.25m.
  // Total Expected: 10.0 + 9.6666... + 20.25 = 39.91666... m.
  auto result = calculate_stop_distance(10.0, 0.0, -2.0, -2.0, 1.0);

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result.value(), 39.9166, epsilon);
}

TEST(CalculateStopDistanceTest, HandlesAlreadyExceedingDecelerationLimit)
{
  // If the vehicle is braking at -6.0m/s² acceleration but the limit is -4.0m/s²,
  // the function uses a limit of -6.0 to maintain valid math.
  // v0 = 10.0, a0 = -6.0, a_limit = -4.0 (increased to -6.0). No delay.
  // Phase 2 is skipped (already at limit).
  // Phase 3: x = -v0^2 / (2*a_limit) = -100 / (2 * -6) = 8.333
  auto result = calculate_stop_distance(10.0, -6.0, -4.0, -10.0, 0.0);

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result.value(), 8.3333, epsilon);
}

}  // namespace
