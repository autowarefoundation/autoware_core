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
#include <rclcpp/time.hpp>

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

geometry_msgs::msg::AccelWithCovarianceStamped AccelEstimator::estimate(
  const geometry_msgs::msg::TwistStamped & curr_twist)
{
  geometry_msgs::msg::AccelWithCovarianceStamped accel_msg;
  accel_msg.header = curr_twist.header;

  if (!prev_twist_.has_value()) {
    prev_twist_ = curr_twist;
    return accel_msg;
  }

  const double dt =
    (rclcpp::Time(curr_twist.header.stamp) - rclcpp::Time(prev_twist_->header.stamp)).seconds();
  const double clamped_dt = std::max(dt, g_min_dt);

  const geometry_msgs::msg::Twist & prev = prev_twist_->twist;
  const geometry_msgs::msg::Twist & curr = curr_twist.twist;

  geometry_msgs::msg::AccelWithCovariance & accel = accel_msg.accel;
  accel.accel.linear.x = lpf_alx_.filter((curr.linear.x - prev.linear.x) / clamped_dt);
  accel.accel.linear.y = lpf_aly_.filter((curr.linear.y - prev.linear.y) / clamped_dt);
  accel.accel.linear.z = lpf_alz_.filter((curr.linear.z - prev.linear.z) / clamped_dt);
  accel.accel.angular.x = lpf_aax_.filter((curr.angular.x - prev.angular.x) / clamped_dt);
  accel.accel.angular.y = lpf_aay_.filter((curr.angular.y - prev.angular.y) / clamped_dt);
  accel.accel.angular.z = lpf_aaz_.filter((curr.angular.z - prev.angular.z) / clamped_dt);

  accel.covariance[XYZRPY_COV_IDX::X_X] = g_linear_accel_variance;
  accel.covariance[XYZRPY_COV_IDX::Y_Y] = g_linear_accel_variance;
  accel.covariance[XYZRPY_COV_IDX::Z_Z] = g_linear_accel_variance;
  accel.covariance[XYZRPY_COV_IDX::ROLL_ROLL] = g_angular_accel_variance;
  accel.covariance[XYZRPY_COV_IDX::PITCH_PITCH] = g_angular_accel_variance;
  accel.covariance[XYZRPY_COV_IDX::YAW_YAW] = g_angular_accel_variance;

  prev_twist_ = curr_twist;
  return accel_msg;
}

geometry_msgs::msg::AccelWithCovarianceStamped AccelEstimator::estimate(
  const geometry_msgs::msg::TwistWithCovarianceStamped & curr_twist)
{
  geometry_msgs::msg::TwistStamped twist;
  twist.header = curr_twist.header;
  twist.twist = curr_twist.twist.twist;
  return estimate(twist);
}

geometry_msgs::msg::AccelWithCovarianceStamped AccelEstimator::estimate(
  const nav_msgs::msg::Odometry & curr_odom)
{
  geometry_msgs::msg::TwistStamped twist;
  twist.header = curr_odom.header;
  twist.twist = curr_odom.twist.twist;
  return estimate(twist);
}
}  // namespace autoware::twist2accel
