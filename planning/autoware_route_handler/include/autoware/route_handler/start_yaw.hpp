// Copyright 2026 The Autoware Foundation
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

#ifndef AUTOWARE__ROUTE_HANDLER__START_YAW_HPP_
#define AUTOWARE__ROUTE_HANDLER__START_YAW_HPP_

#include <cmath>

namespace autoware::route_handler
{
// Preserve the existing 90 degree limit for road starts. For a start verified
// inside a parking lot, permit the small reverse-facing yaw mismatch of a
// perpendicular stall without accepting nearly backwards (180 degree) headings.
inline bool is_start_yaw_acceptable(
  const double angle_diff, const bool start_inside_parking_lot)
{
  constexpr double pi = 3.14159265358979323846;
  constexpr double road_yaw_limit = pi / 2.0;
  constexpr double parking_yaw_limit = 3.0 * pi / 4.0;
  const double limit = start_inside_parking_lot ? parking_yaw_limit : road_yaw_limit;
  return std::isfinite(angle_diff) && angle_diff >= 0.0 && angle_diff <= limit;
}
}  // namespace autoware::route_handler

#endif  // AUTOWARE__ROUTE_HANDLER__START_YAW_HPP_
