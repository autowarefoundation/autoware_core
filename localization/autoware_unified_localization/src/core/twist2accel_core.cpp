// Copyright 2025 Autoware Foundation
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

#include "autoware/unified_localization_core/twist2accel_core.hpp"

#include <algorithm>

namespace autoware::unified_localization_core
{

Twist2AccelCore::Twist2AccelCore(const Twist2AccelParams & params) : params_(params)
{
  const double g = params_.accel_lowpass_gain;
  lpf_alx_ = std::make_shared<LowpassFilter1d>(g);
  lpf_aly_ = std::make_shared<LowpassFilter1d>(g);
  lpf_alz_ = std::make_shared<LowpassFilter1d>(g);
  lpf_aax_ = std::make_shared<LowpassFilter1d>(g);
  lpf_aay_ = std::make_shared<LowpassFilter1d>(g);
  lpf_aaz_ = std::make_shared<LowpassFilter1d>(g);
}

void Twist2AccelCore::estimate(
  double prev_timestamp_sec, double curr_timestamp_sec, const Vector3 & prev_linear,
  const Vector3 & prev_angular, const Vector3 & curr_linear, const Vector3 & curr_angular,
  AccelerationOutput & out_accel)
{
  out_accel.timestamp_sec = curr_timestamp_sec;
  const double dt = std::max(curr_timestamp_sec - prev_timestamp_sec, 1e-3);

  out_accel.linear_x = lpf_alx_->filter((curr_linear.x - prev_linear.x) / dt);
  out_accel.linear_y = lpf_aly_->filter((curr_linear.y - prev_linear.y) / dt);
  out_accel.linear_z = lpf_alz_->filter((curr_linear.z - prev_linear.z) / dt);
  out_accel.angular_x = lpf_aax_->filter((curr_angular.x - prev_angular.x) / dt);
  out_accel.angular_y = lpf_aay_->filter((curr_angular.y - prev_angular.y) / dt);
  out_accel.angular_z = lpf_aaz_->filter((curr_angular.z - prev_angular.z) / dt);
}

}  // namespace autoware::unified_localization_core
