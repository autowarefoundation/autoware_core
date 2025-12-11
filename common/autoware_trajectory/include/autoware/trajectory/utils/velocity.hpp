// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__TRAJECTORY__UTILS__VELOCITY_HPP_
#define AUTOWARE__TRAJECTORY__UTILS__VELOCITY_HPP_

#include "autoware/trajectory/forward.hpp"

#include "autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp"
#include "autoware_planning_msgs/msg/path_point.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"

#include <optional>

namespace autoware::experimental::trajectory
{

/**
 * @brief find zero velocity position in continuous trajectory
 * Searches for the arc-length position where velocity becomes zero in a continuous Trajectory.
 * Uses the velocity interpolator's underlying data directly with binary search refinement.
 * @param trajectory continuous trajectory to search
 * @param max_iterations maximum iterations for binary search refinement (default: 10)
 * @return arc-length distance where velocity is zero, or std::nullopt if not found
 */
template <class PointT>
[[nodiscard]] std::optional<double> searchZeroVelocityPosition(
  const Trajectory<PointT> & trajectory, const int max_iterations = 10)
{
  const auto [bases, velocities] = trajectory.longitudinal_velocity_mps().get_data();

  if (velocities.empty()) {
    return std::nullopt;
  }

  constexpr double epsilon = 1e-6;
  double prev_vel = velocities[0];

  if (std::abs(prev_vel) < epsilon) {
    return bases[0];
  }

  for (size_t i = 1; i < velocities.size(); ++i) {
    const double curr_vel = velocities[i];
    if (std::abs(curr_vel) < epsilon) {
      return bases[i];
    }
    if ((prev_vel > 0.0 && curr_vel < 0.0) || (prev_vel < 0.0 && curr_vel > 0.0)) {
      double s_low = bases[i - 1];
      double s_high = bases[i];

      for (int iter = 0; iter < max_iterations; ++iter) {
        const double s_mid = (s_low + s_high) / 2.0;
        const double mid_vel = trajectory.longitudinal_velocity_mps().compute(s_mid);

        if (std::abs(mid_vel) < epsilon) {
          return s_mid;
        }

        if ((prev_vel > 0.0 && mid_vel > 0.0) || (prev_vel < 0.0 && mid_vel < 0.0)) {
          s_low = s_mid;
        } else {
          s_high = s_mid;
        }
      }
      return (s_low + s_high) / 2.0;
    }
    prev_vel = curr_vel;
  }
  return std::nullopt;
}

}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__UTILS__VELOCITY_HPP_
