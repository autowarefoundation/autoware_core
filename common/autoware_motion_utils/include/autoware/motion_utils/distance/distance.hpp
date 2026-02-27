// Copyright 2023-2026 TIER IV, Inc.
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

#ifndef AUTOWARE__MOTION_UTILS__DISTANCE__DISTANCE_HPP_
#define AUTOWARE__MOTION_UTILS__DISTANCE__DISTANCE_HPP_

#include <optional>

namespace autoware::motion_utils
{
[[nodiscard]] std::optional<double> calcDecelDistWithJerkAndAccConstraints(
  const double current_vel, const double target_vel, const double current_acc, const double acc_min,
  const double jerk_acc, const double jerk_dec);

/**
 * @brief Computes the minimum longitudinal distance required to bring a vehicle to a
 * complete stop, subject to jerk and acceleration constraints.
 *
 * This function models a three-phase stopping profile:
 * 1. A latency/delay phase where initial acceleration is maintained.
 * 2. A jerk-limited braking phase where deceleration increases to the maximum limit.
 * 3. A constant maximum-deceleration phase until the vehicle stops.
 * * If the vehicle reaches a full stop during phase 1 or phase 2, the calculation
 * terminates early and returns the exact distance covered up to the point where v = 0.
 *
 * @param v0 Current longitudinal speed v₀ [m/s].
 * @param a0 Current longitudinal acceleration a₀ [m/s²].
 * @param decel_limit        Maximum braking deceleration [m/s²]
 * @param jerk_limit         Maximum braking jerk [m/s³]
 * @param initial_time_delay Latency before any braking jerk is applied t₁ [s]. Defaults to 0.0.
 *
 * @return std::optional<double> Minimum longitudinal stopping distance [m].
 * Returns 0.0 if the current velocity is already non-positive.
 * Returns std::nullopt if the kinematic limits are invalid (e.g., limits are 0).
 */
[[nodiscard]] std::optional<double> calculate_stop_distance(
  const double v0, const double a0, const double decel_limit, const double jerk_limit,
  const double initial_time_delay = 0.0);

}  // namespace autoware::motion_utils

#endif  // AUTOWARE__MOTION_UTILS__DISTANCE__DISTANCE_HPP_
