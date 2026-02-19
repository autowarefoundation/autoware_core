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

#ifndef AUTOWARE__UNIFIED_LOCALIZATION_CORE__TWIST2ACCEL_CORE_HPP_
#define AUTOWARE__UNIFIED_LOCALIZATION_CORE__TWIST2ACCEL_CORE_HPP_

#include "autoware/unified_localization_core/lowpass_filter_1d.hpp"
#include "autoware/unified_localization_core/types.hpp"

#include <memory>

namespace autoware::unified_localization_core
{

struct Twist2AccelParams
{
  double accel_lowpass_gain{0.2};
};

class Twist2AccelCore
{
public:
  explicit Twist2AccelCore(const Twist2AccelParams & params);

  /**
   * Compute acceleration from previous and current twist.
   * @param prev_timestamp_sec  previous twist time
   * @param curr_timestamp_sec  current twist time
   * @param prev_linear  previous linear velocity
   * @param prev_angular  previous angular velocity
   * @param curr_linear  current linear velocity
   * @param curr_angular  current angular velocity
   * @param out_accel  output linear and angular acceleration
   */
  void estimate(
    double prev_timestamp_sec, double curr_timestamp_sec,
    const Vector3 & prev_linear, const Vector3 & prev_angular,
    const Vector3 & curr_linear, const Vector3 & curr_angular,
    AccelerationOutput & out_accel);

private:
  Twist2AccelParams params_;
  std::shared_ptr<LowpassFilter1d> lpf_alx_;
  std::shared_ptr<LowpassFilter1d> lpf_aly_;
  std::shared_ptr<LowpassFilter1d> lpf_alz_;
  std::shared_ptr<LowpassFilter1d> lpf_aax_;
  std::shared_ptr<LowpassFilter1d> lpf_aay_;
  std::shared_ptr<LowpassFilter1d> lpf_aaz_;
};

}  // namespace autoware::unified_localization_core

#endif  // AUTOWARE__UNIFIED_LOCALIZATION_CORE__TWIST2ACCEL_CORE_HPP_
