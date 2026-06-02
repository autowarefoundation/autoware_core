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

#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace autoware::twist2accel
{

/// Lower bound applied to the time delta between successive twist samples to
/// avoid division by (near-)zero when stamps are equal or out of order.
constexpr double g_min_dt = 1.0e-3;

/// @brief Pure acceleration estimator with no rclcpp::Node / spinning dependency.
///
/// Estimates acceleration by finite-differencing two successive twist samples
/// and smoothing each of the six components with a first-order low-pass filter.
/// Owns the six LowpassFilter1d instances so the filter state persists across
/// calls, mirroring the per-component smoothing performed by the node.
///
/// It operates directly on the plain geometry_msgs Twist/Accel data structs:
/// those are header-only value types that need no running ROS node, so the
/// estimator stays unit-testable while avoiding any conversion layer.
class AccelEstimator
{
public:
  explicit AccelEstimator(double lowpass_gain);

  /// @brief Estimate acceleration from a finite difference of two twists.
  /// @param prev_twist twist sample at the previous time step
  /// @param curr_twist twist sample at the current time step
  /// @param dt time delta in seconds between the two samples; values below
  ///   g_min_dt are clamped up to g_min_dt before the division
  /// @return per-component, low-pass-filtered acceleration
  geometry_msgs::msg::Accel estimate(
    const geometry_msgs::msg::Twist & prev_twist, const geometry_msgs::msg::Twist & curr_twist,
    double dt);

private:
  autoware::signal_processing::LowpassFilter1d lpf_alx_;
  autoware::signal_processing::LowpassFilter1d lpf_aly_;
  autoware::signal_processing::LowpassFilter1d lpf_alz_;
  autoware::signal_processing::LowpassFilter1d lpf_aax_;
  autoware::signal_processing::LowpassFilter1d lpf_aay_;
  autoware::signal_processing::LowpassFilter1d lpf_aaz_;
};
}  // namespace autoware::twist2accel
#endif  // ACCEL_ESTIMATOR_HPP_
