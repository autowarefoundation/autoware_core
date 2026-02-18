// Copyright 2025 Autoware Foundation
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
