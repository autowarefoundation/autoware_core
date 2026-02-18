// Copyright 2025 Autoware Foundation
// Licensed under the Apache License, Version 2.0.

#ifndef AUTOWARE__UNIFIED_LOCALIZATION_CORE__STOP_FILTER_CORE_HPP_
#define AUTOWARE__UNIFIED_LOCALIZATION_CORE__STOP_FILTER_CORE_HPP_

#include "autoware/unified_localization_core/types.hpp"

namespace autoware::unified_localization_core
{

struct StopFilterParams
{
  double linear_x_threshold{0.1};
  double angular_z_threshold{0.02};
};

class StopFilterCore
{
public:
  explicit StopFilterCore(const StopFilterParams & params);

  /** Apply stop filter: if stopped, output zeros; else pass through. */
  void apply(
    const Vector3 & linear_velocity, const Vector3 & angular_velocity,
    Vector3 & out_linear, Vector3 & out_angular, bool & was_stopped) const;

private:
  bool is_stopped(const Vector3 & linear_velocity, const Vector3 & angular_velocity) const;
  StopFilterParams params_;
};

}  // namespace autoware::unified_localization_core

#endif  // AUTOWARE__UNIFIED_LOCALIZATION_CORE__STOP_FILTER_CORE_HPP_
