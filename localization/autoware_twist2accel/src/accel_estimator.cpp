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

#include "accel_estimator.hpp"

#include <autoware_utils_geometry/msg/covariance.hpp>

#include <algorithm>

namespace autoware::twist2accel
{
using autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;

AccelEstimator::AccelEstimator(const double lowpass_gain)
: lpf_alx_(lowpass_gain),
  lpf_aly_(lowpass_gain),
  lpf_alz_(lowpass_gain),
  lpf_aax_(lowpass_gain),
  lpf_aay_(lowpass_gain),
  lpf_aaz_(lowpass_gain)
{
}

geometry_msgs::msg::AccelWithCovariance AccelEstimator::estimate(
  const geometry_msgs::msg::Twist & prev_twist, const geometry_msgs::msg::Twist & curr_twist,
  const double dt)
{
  const double clamped_dt = std::max(dt, g_min_dt);

  geometry_msgs::msg::AccelWithCovariance accel;
  accel.accel.linear.x = lpf_alx_.filter((curr_twist.linear.x - prev_twist.linear.x) / clamped_dt);
  accel.accel.linear.y = lpf_aly_.filter((curr_twist.linear.y - prev_twist.linear.y) / clamped_dt);
  accel.accel.linear.z = lpf_alz_.filter((curr_twist.linear.z - prev_twist.linear.z) / clamped_dt);
  accel.accel.angular.x =
    lpf_aax_.filter((curr_twist.angular.x - prev_twist.angular.x) / clamped_dt);
  accel.accel.angular.y =
    lpf_aay_.filter((curr_twist.angular.y - prev_twist.angular.y) / clamped_dt);
  accel.accel.angular.z =
    lpf_aaz_.filter((curr_twist.angular.z - prev_twist.angular.z) / clamped_dt);

  // Ideally speaking, this covariance should be properly estimated. Until that
  // is done, report constant diagonal variances; off-diagonal terms stay at the
  // default zero.
  accel.covariance[XYZRPY_COV_IDX::X_X] = g_linear_accel_variance;
  accel.covariance[XYZRPY_COV_IDX::Y_Y] = g_linear_accel_variance;
  accel.covariance[XYZRPY_COV_IDX::Z_Z] = g_linear_accel_variance;
  accel.covariance[XYZRPY_COV_IDX::ROLL_ROLL] = g_angular_accel_variance;
  accel.covariance[XYZRPY_COV_IDX::PITCH_PITCH] = g_angular_accel_variance;
  accel.covariance[XYZRPY_COV_IDX::YAW_YAW] = g_angular_accel_variance;

  return accel;
}
}  // namespace autoware::twist2accel
