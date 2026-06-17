// Copyright 2022 TIER IV
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

#ifndef ACCEL_ESTIMATOR_HPP_
#define ACCEL_ESTIMATOR_HPP_

#include "autoware/signal_processing/lowpass_filter_1d.hpp"

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <optional>

namespace autoware::twist2accel
{

/// Lower bound applied to the time delta between successive twist samples to
/// avoid division by (near-)zero when stamps are equal or out of order.
constexpr double g_min_dt = 1.0e-3;

/// Fixed covariance assigned to the estimated acceleration. The current
/// estimator does not propagate the input twist covariance through the
/// finite-difference / low-pass smoothing, so it reports these constant
/// diagonal variances (off-diagonal terms stay zero):
/// - linear x/y/z variance = g_linear_accel_variance
/// - angular roll/pitch/yaw variance = g_angular_accel_variance
constexpr double g_linear_accel_variance = 1.0;
constexpr double g_angular_accel_variance = 0.05;

/// @brief Pure acceleration estimator with no rclcpp::Node / spinning dependency.
///
/// Estimates acceleration by finite-differencing two successive twist samples
/// and smoothing each of the six components with a first-order low-pass filter.
/// It keeps the previous twist sample as internal state, so the caller only
/// supplies the current sample; the time delta is derived from the difference
/// between the current and previous header stamps. It also owns the six
/// LowpassFilter1d instances so the filter state persists across calls,
/// mirroring the per-component smoothing performed by the node.
///
/// In addition to the acceleration values it fills in the acceleration
/// covariance and copies the input header onto the output, keeping the full
/// estimation contract (stamp + value + uncertainty) in one pure place instead
/// of splitting it between the core and the node.
///
/// It operates directly on the plain geometry_msgs TwistStamped /
/// AccelWithCovarianceStamped data structs: those are header-only value types
/// that need no running ROS node, so the estimator stays unit-testable while
/// avoiding any conversion layer.
class AccelEstimator
{
public:
  explicit AccelEstimator(double lowpass_gain);

  /// @brief Estimate acceleration and its covariance from the finite difference
  ///   between the current twist sample and the previously supplied one.
  ///
  /// The previous twist sample is held as internal state and the time delta is
  /// computed from the difference between the current and previous header
  /// stamps; deltas below g_min_dt (including zero or negative ones from equal
  /// or out-of-order stamps) are clamped up to g_min_dt before the division.
  ///
  /// On the first call there is no previous sample to difference against, so the
  /// estimator reports a zero acceleration with a zero covariance and just
  /// stores the current sample as the baseline for the next call.
  ///
  /// @param curr_twist stamped twist sample at the current time step
  /// @return stamped acceleration whose header is copied from @p curr_twist and
  ///   whose value is the per-component, low-pass-filtered acceleration together
  ///   with its covariance (constant diagonal variances, see
  ///   g_linear_accel_variance / g_angular_accel_variance)
  geometry_msgs::msg::AccelWithCovarianceStamped estimate(
    const geometry_msgs::msg::TwistStamped & curr_twist);

  /// @brief Convenience overload that estimates from a TwistWithCovarianceStamped.
  ///
  /// The input covariance is not propagated; only the header and the nested
  /// twist are used before delegating to estimate(const TwistStamped &).
  geometry_msgs::msg::AccelWithCovarianceStamped estimate(
    const geometry_msgs::msg::TwistWithCovarianceStamped & curr_twist);

  /// @brief Convenience overload that estimates from an Odometry message.
  ///
  /// Only the header and the nested twist are used (the pose is ignored) before
  /// delegating to estimate(const TwistStamped &).
  geometry_msgs::msg::AccelWithCovarianceStamped estimate(
    const nav_msgs::msg::Odometry & curr_odom);

private:
  std::optional<geometry_msgs::msg::TwistStamped> prev_twist_;
  autoware::signal_processing::LowpassFilter1d lpf_alx_;
  autoware::signal_processing::LowpassFilter1d lpf_aly_;
  autoware::signal_processing::LowpassFilter1d lpf_alz_;
  autoware::signal_processing::LowpassFilter1d lpf_aax_;
  autoware::signal_processing::LowpassFilter1d lpf_aay_;
  autoware::signal_processing::LowpassFilter1d lpf_aaz_;
};
}  // namespace autoware::twist2accel
#endif  // ACCEL_ESTIMATOR_HPP_
