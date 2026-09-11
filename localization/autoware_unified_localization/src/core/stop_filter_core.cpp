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

#include "autoware/unified_localization_core/stop_filter_core.hpp"

#include <cmath>

namespace autoware::unified_localization_core
{

StopFilterCore::StopFilterCore(const StopFilterParams & params) : params_(params)
{
}

bool StopFilterCore::is_stopped(
  const Vector3 & linear_velocity, const Vector3 & angular_velocity) const
{
  const bool linear_stopped = std::fabs(linear_velocity.x) < params_.linear_x_threshold;
  const bool angular_stopped = std::fabs(angular_velocity.z) < params_.angular_z_threshold;
  return linear_stopped && angular_stopped;
}

void StopFilterCore::apply(
  const Vector3 & linear_velocity, const Vector3 & angular_velocity, Vector3 & out_linear,
  Vector3 & out_angular, bool & was_stopped) const
{
  was_stopped = is_stopped(linear_velocity, angular_velocity);
  if (was_stopped) {
    out_linear = {0.0, 0.0, 0.0};
    out_angular = {0.0, 0.0, 0.0};
  } else {
    out_linear = linear_velocity;
    out_angular = angular_velocity;
  }
}

}  // namespace autoware::unified_localization_core
