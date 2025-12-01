// Copyright 2021-2025 TIER IV
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

#include "stop_filter.hpp"

#include <cmath>

namespace autoware::stop_filter
{
StopFilter::StopFilter(double linear_x_threshold, double angular_z_threshold)
: linear_x_threshold_(linear_x_threshold), angular_z_threshold_(angular_z_threshold)
{
}

bool StopFilter::is_stopped(
  const Vector3D & linear_velocity, const Vector3D & angular_velocity) const
{
  const bool linear_stopped = std::fabs(linear_velocity.x) < linear_x_threshold_;
  const bool angular_stopped = std::fabs(angular_velocity.z) < angular_z_threshold_;
  return linear_stopped && angular_stopped;
}

FilterResult StopFilter::apply_stop_filter(
  const Vector3D & linear_velocity, const Vector3D & angular_velocity) const
{
  FilterResult result;
  result.was_stopped = is_stopped(linear_velocity, angular_velocity);

  if (result.was_stopped) {
    result.linear_velocity = {0.0, 0.0, 0.0};
    result.angular_velocity = {0.0, 0.0, 0.0};
  } else {
    result.linear_velocity = linear_velocity;
    result.angular_velocity = angular_velocity;
  }

  return result;
}

}  // namespace autoware::stop_filter
