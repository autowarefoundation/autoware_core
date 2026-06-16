// Copyright 2025 TIER IV
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

#include "../src/accel_estimator.hpp"

#include <geometry_msgs/msg/accel_with_covariance.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstddef>

using autoware::twist2accel::AccelEstimator;
using autoware::twist2accel::g_angular_accel_variance;
using autoware::twist2accel::g_linear_accel_variance;
using autoware::twist2accel::g_min_dt;

namespace
{
constexpr double kTol = 1e-9;

// Build a geometry_msgs Twist from its six scalar components for concise tests.
geometry_msgs::msg::Twist make_twist(
  const double linear_x, const double linear_y, const double linear_z, const double angular_x,
  const double angular_y, const double angular_z)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = linear_x;
  twist.linear.y = linear_y;
  twist.linear.z = linear_z;
  twist.angular.x = angular_x;
  twist.angular.y = angular_y;
  twist.angular.z = angular_z;
  return twist;
}
}  // namespace

// On the very first sample the per-component low-pass filters are uninitialized,
// so filter(u) returns u unchanged. The estimator therefore reports the raw
// finite difference (curr - prev) / dt on every channel.
TEST(AccelEstimatorTest, FiniteDifferenceNoSmoothingOnFirstSample)
{
  AccelEstimator estimator(0.9);
  const auto prev = make_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  const auto curr = make_twist(2.0, 4.0, 6.0, 0.2, 0.4, 0.6);
  const double dt = 0.5;

  const geometry_msgs::msg::AccelWithCovariance accel = estimator.estimate(prev, curr, dt);

  EXPECT_NEAR(accel.accel.linear.x, 4.0, kTol);   // (2.0 - 0.0) / 0.5
  EXPECT_NEAR(accel.accel.linear.y, 8.0, kTol);   // (4.0 - 0.0) / 0.5
  EXPECT_NEAR(accel.accel.linear.z, 12.0, kTol);  // (6.0 - 0.0) / 0.5
  EXPECT_NEAR(accel.accel.angular.x, 0.4, kTol);  // (0.2 - 0.0) / 0.5
  EXPECT_NEAR(accel.accel.angular.y, 0.8, kTol);  // (0.4 - 0.0) / 0.5
  EXPECT_NEAR(accel.accel.angular.z, 1.2, kTol);  // (0.6 - 0.0) / 0.5
}

// Equal successive twists give a zero finite difference on every channel,
// hence zero acceleration.
TEST(AccelEstimatorTest, ZeroVelocityChangeYieldsZeroAccel)
{
  AccelEstimator estimator(0.5);
  const auto twist = make_twist(1.0, -2.0, 3.0, 0.1, -0.2, 0.3);

  const geometry_msgs::msg::AccelWithCovariance accel = estimator.estimate(twist, twist, 0.1);

  EXPECT_NEAR(accel.accel.linear.x, 0.0, kTol);
  EXPECT_NEAR(accel.accel.linear.y, 0.0, kTol);
  EXPECT_NEAR(accel.accel.linear.z, 0.0, kTol);
  EXPECT_NEAR(accel.accel.angular.x, 0.0, kTol);
  EXPECT_NEAR(accel.accel.angular.y, 0.0, kTol);
  EXPECT_NEAR(accel.accel.angular.z, 0.0, kTol);
}

// The second and later samples smooth the raw finite difference with the
// first-order low-pass filter: out = gain * prev_out + (1 - gain) * raw.
TEST(AccelEstimatorTest, LowPassSmoothingAcrossSamples)
{
  const double gain = 0.9;
  AccelEstimator estimator(gain);
  const double dt = 1.0;

  // First sample: linear_x ramps 0 -> 1, raw accel = 1.0, LPF returns it as-is.
  const geometry_msgs::msg::AccelWithCovariance first =
    estimator.estimate(make_twist(0.0, 0, 0, 0, 0, 0), make_twist(1.0, 0, 0, 0, 0, 0), dt);
  EXPECT_NEAR(first.accel.linear.x, 1.0, kTol);

  // Second sample: linear_x ramps 1 -> 3, raw accel = 2.0.
  // Smoothed: 0.9 * 1.0 + 0.1 * 2.0 = 1.1.
  const geometry_msgs::msg::AccelWithCovariance second =
    estimator.estimate(make_twist(1.0, 0, 0, 0, 0, 0), make_twist(3.0, 0, 0, 0, 0, 0), dt);
  EXPECT_NEAR(second.accel.linear.x, gain * 1.0 + (1.0 - gain) * 2.0, kTol);
  EXPECT_NEAR(second.accel.linear.x, 1.1, kTol);

  // Third sample: linear_x ramps 3 -> 3, raw accel = 0.0.
  // Smoothed: 0.9 * 1.1 + 0.1 * 0.0 = 0.99.
  const geometry_msgs::msg::AccelWithCovariance third =
    estimator.estimate(make_twist(3.0, 0, 0, 0, 0, 0), make_twist(3.0, 0, 0, 0, 0, 0), dt);
  EXPECT_NEAR(third.accel.linear.x, gain * 1.1 + (1.0 - gain) * 0.0, kTol);
  EXPECT_NEAR(third.accel.linear.x, 0.99, kTol);
}

// dt below g_min_dt (including zero or negative, e.g. near-simultaneous or
// out-of-order stamps) is clamped up to g_min_dt before the division so the
// estimator never divides by zero or flips sign.
TEST(AccelEstimatorTest, DtClampForNearSimultaneousStamps)
{
  const auto prev = make_twist(0.0, 0, 0, 0, 0, 0);
  const auto curr = make_twist(1.0, 0, 0, 0, 0, 0);

  // dt == 0 is clamped to g_min_dt: accel = 1.0 / g_min_dt.
  AccelEstimator zero_dt_estimator(0.0);  // gain 0 -> filter passes raw value through
  const geometry_msgs::msg::AccelWithCovariance zero_dt =
    zero_dt_estimator.estimate(prev, curr, 0.0);
  EXPECT_NEAR(zero_dt.accel.linear.x, 1.0 / g_min_dt, kTol);

  // Negative dt is also clamped to g_min_dt, not used directly.
  AccelEstimator negative_dt_estimator(0.0);
  const geometry_msgs::msg::AccelWithCovariance negative_dt =
    negative_dt_estimator.estimate(prev, curr, -5.0);
  EXPECT_NEAR(negative_dt.accel.linear.x, 1.0 / g_min_dt, kTol);

  // dt above the threshold is used unchanged.
  AccelEstimator large_dt_estimator(0.0);
  const geometry_msgs::msg::AccelWithCovariance large_dt =
    large_dt_estimator.estimate(prev, curr, 2.0);
  EXPECT_NEAR(large_dt.accel.linear.x, 1.0 / 2.0, kTol);
}

// Each of the six channels is filtered independently (its own LPF state),
// so cross-talk cannot occur.
TEST(AccelEstimatorTest, ChannelsAreIndependent)
{
  AccelEstimator estimator(0.5);
  const double dt = 1.0;

  // Prime only linear_x with a non-zero history.
  estimator.estimate(make_twist(0, 0, 0, 0, 0, 0), make_twist(2.0, 0, 0, 0, 0, 0), dt);

  // Now move only angular_z; linear_x sees raw accel 0 but retains its history.
  const geometry_msgs::msg::AccelWithCovariance accel =
    estimator.estimate(make_twist(2.0, 0, 0, 0, 0, 0), make_twist(2.0, 0, 0, 0, 0, 1.0), dt);

  // linear_x: 0.5 * 2.0 (history) + 0.5 * 0.0 (raw) = 1.0. Its history is
  // preserved independently of the other channels.
  EXPECT_NEAR(accel.accel.linear.x, 1.0, kTol);
  // angular_z was initialized to 0.0 raw on the first call, so this second
  // sample smooths raw 1.0 against that: 0.5 * 0.0 + 0.5 * 1.0 = 0.5. This
  // proves angular_z carries its own state and is not affected by linear_x.
  EXPECT_NEAR(accel.accel.angular.z, 0.5, kTol);
  // Untouched channels stay at zero.
  EXPECT_NEAR(accel.accel.linear.y, 0.0, kTol);
  EXPECT_NEAR(accel.accel.angular.x, 0.0, kTol);
}

// The estimator fills in a constant diagonal covariance for the acceleration:
// g_linear_accel_variance on the three linear (x, y, z) channels and
// g_angular_accel_variance on the three angular (roll, pitch, yaw) channels,
// with every off-diagonal term zero. This test pins that structure -- which
// diagonal index carries which variance, and that every other entry stays zero.
// The covariance is a flat row-major 6x6 matrix, so element (row, col) lives at
// index row * 6 + col.
TEST(AccelEstimatorTest, CovarianceIsConstantDiagonal)
{
  AccelEstimator estimator(0.9);
  const auto prev = make_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  const auto curr = make_twist(2.0, 4.0, 6.0, 0.2, 0.4, 0.6);

  const geometry_msgs::msg::AccelWithCovariance accel = estimator.estimate(prev, curr, 0.5);

  // Expected: a 36-entry array that is zero everywhere except the six diagonal
  // variances, set to the named public constants. The diagonal of a row-major
  // 6x6 lives at indices 0, 7, 14, 21, 28, 35.
  std::array<double, 36> expected{};        // value-initialized to all zeros
  expected[0] = g_linear_accel_variance;    // X_X  (linear x)
  expected[7] = g_linear_accel_variance;    // Y_Y  (linear y)
  expected[14] = g_linear_accel_variance;   // Z_Z  (linear z)
  expected[21] = g_angular_accel_variance;  // R_R  (roll)
  expected[28] = g_angular_accel_variance;  // P_P  (pitch)
  expected[35] = g_angular_accel_variance;  // Y_Y  (yaw)

  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_NEAR(accel.covariance[i], expected[i], kTol) << "covariance index " << i;
  }
}
