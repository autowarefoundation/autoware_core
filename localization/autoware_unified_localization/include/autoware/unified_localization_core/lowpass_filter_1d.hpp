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
// Minimal first-order low-pass filter (ROS-free, no boost).

#ifndef AUTOWARE__UNIFIED_LOCALIZATION_CORE__LOWPASS_FILTER_1D_HPP_
#define AUTOWARE__UNIFIED_LOCALIZATION_CORE__LOWPASS_FILTER_1D_HPP_

#include <optional>

namespace autoware::unified_localization_core
{

class LowpassFilter1d
{
public:
  explicit LowpassFilter1d(double gain) : gain_(gain) {}
  double filter(double u)
  {
    if (x_.has_value()) {
      const double ret = gain_ * x_.value() + (1.0 - gain_) * u;
      x_ = ret;
      return ret;
    }
    x_ = u;
    return u;
  }

private:
  double gain_;
  std::optional<double> x_;
};

}  // namespace autoware::unified_localization_core

#endif  // AUTOWARE__UNIFIED_LOCALIZATION_CORE__LOWPASS_FILTER_1D_HPP_
