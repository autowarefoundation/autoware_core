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

#include "autoware/twist2accel/twist2accel_processor.hpp"

#include <rclcpp/time.hpp>

#include <memory>

namespace autoware::twist2accel
{

Twist2AccelProcessor::Twist2AccelProcessor(double accel_lowpass_gain)
{
  lpf_alx_ptr_ = std::make_shared<LowpassFilter1d>(accel_lowpass_gain);
  lpf_aly_ptr_ = std::make_shared<LowpassFilter1d>(accel_lowpass_gain);
  lpf_alz_ptr_ = std::make_shared<LowpassFilter1d>(accel_lowpass_gain);
  lpf_aax_ptr_ = std::make_shared<LowpassFilter1d>(accel_lowpass_gain);
  lpf_aay_ptr_ = std::make_shared<LowpassFilter1d>(accel_lowpass_gain);
  lpf_aaz_ptr_ = std::make_shared<LowpassFilter1d>(accel_lowpass_gain);
}

void Twist2AccelProcessor::reset()
{
  prev_twist_.reset();
}

geometry_msgs::msg::AccelWithCovarianceStamped Twist2AccelProcessor::estimate_accel(
  const geometry_msgs::msg::TwistStamped & twist)
{
  geometry_msgs::msg::AccelWithCovarianceStamped accel_msg;
  accel_msg.header = twist.header;

  if (prev_twist_.has_value()) {
    const double dt = std::max(
      (rclcpp::Time(twist.header.stamp) - rclcpp::Time(prev_twist_->header.stamp)).seconds(),
      1.0e-3);

    const double alx = (twist.twist.linear.x - prev_twist_->twist.linear.x) / dt;
    const double aly = (twist.twist.linear.y - prev_twist_->twist.linear.y) / dt;
    const double alz = (twist.twist.linear.z - prev_twist_->twist.linear.z) / dt;
    const double aax = (twist.twist.angular.x - prev_twist_->twist.angular.x) / dt;
    const double aay = (twist.twist.angular.y - prev_twist_->twist.angular.y) / dt;
    const double aaz = (twist.twist.angular.z - prev_twist_->twist.angular.z) / dt;

    accel_msg.accel.accel.linear.x = lpf_alx_ptr_->filter(alx);
    accel_msg.accel.accel.linear.y = lpf_aly_ptr_->filter(aly);
    accel_msg.accel.accel.linear.z = lpf_alz_ptr_->filter(alz);
    accel_msg.accel.accel.angular.x = lpf_aax_ptr_->filter(aax);
    accel_msg.accel.accel.angular.y = lpf_aay_ptr_->filter(aay);
    accel_msg.accel.accel.angular.z = lpf_aaz_ptr_->filter(aaz);

    accel_msg.accel.covariance[0 * 6 + 0] = 1.0;
    accel_msg.accel.covariance[1 * 6 + 1] = 1.0;
    accel_msg.accel.covariance[2 * 6 + 2] = 1.0;
    accel_msg.accel.covariance[3 * 6 + 3] = 0.05;
    accel_msg.accel.covariance[4 * 6 + 4] = 0.05;
    accel_msg.accel.covariance[5 * 6 + 5] = 0.05;
  }

  prev_twist_ = twist;
  return accel_msg;
}

}  // namespace autoware::twist2accel
