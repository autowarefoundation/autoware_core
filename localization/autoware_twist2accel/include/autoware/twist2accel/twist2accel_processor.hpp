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

#ifndef AUTOWARE__TWIST2ACCEL__TWIST2ACCEL_PROCESSOR_HPP_
#define AUTOWARE__TWIST2ACCEL__TWIST2ACCEL_PROCESSOR_HPP_

#include "autoware/signal_processing/lowpass_filter_1d.hpp"

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <memory>
#include <optional>

namespace autoware::twist2accel
{

class Twist2AccelProcessor
{
public:
  explicit Twist2AccelProcessor(double accel_lowpass_gain);

  geometry_msgs::msg::AccelWithCovarianceStamped estimate_accel(
    const geometry_msgs::msg::TwistStamped & twist);

  void reset();

private:
  using LowpassFilter1d = autoware::signal_processing::LowpassFilter1d;

  std::optional<geometry_msgs::msg::TwistStamped> prev_twist_;
  std::shared_ptr<LowpassFilter1d> lpf_alx_ptr_;
  std::shared_ptr<LowpassFilter1d> lpf_aly_ptr_;
  std::shared_ptr<LowpassFilter1d> lpf_alz_ptr_;
  std::shared_ptr<LowpassFilter1d> lpf_aax_ptr_;
  std::shared_ptr<LowpassFilter1d> lpf_aay_ptr_;
  std::shared_ptr<LowpassFilter1d> lpf_aaz_ptr_;
};

}  // namespace autoware::twist2accel

#endif  // AUTOWARE__TWIST2ACCEL__TWIST2ACCEL_PROCESSOR_HPP_
