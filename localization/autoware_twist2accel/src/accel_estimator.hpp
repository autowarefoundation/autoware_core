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

constexpr double g_min_dt = 1.0e-3;
constexpr double g_linear_accel_variance = 1.0;
constexpr double g_angular_accel_variance = 0.05;

/// @brief Estimates acceleration by differentiating consecutive twist (velocity) samples over time
/// and low-pass filtering the result.
class AccelEstimator
{
public:
  /// @brief Construct the estimator.
  /// @param lowpass_gain Gain applied to every per-axis low-pass filter (0: no smoothing,
  /// closer to 1: stronger smoothing).
  explicit AccelEstimator(double lowpass_gain);

  /// @brief Estimate acceleration from the current twist.
  /// @param curr_twist Current velocity sample with timestamp.
  /// @return Estimated acceleration; zero on the first call since no previous sample exists yet.
  geometry_msgs::msg::AccelWithCovarianceStamped estimate(
    const geometry_msgs::msg::TwistStamped & curr_twist);

  /// @brief Estimate acceleration from a twist-with-covariance sample (covariance is ignored).
  geometry_msgs::msg::AccelWithCovarianceStamped estimate(
    const geometry_msgs::msg::TwistWithCovarianceStamped & curr_twist);

  /// @brief Estimate acceleration from the twist field of an odometry sample.
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
