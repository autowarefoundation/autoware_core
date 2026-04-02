// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__TRAJECTORY__UTILS__TEMPORAL__FIND_STOP_POINTS_HPP_
#define AUTOWARE__TRAJECTORY__UTILS__TEMPORAL__FIND_STOP_POINTS_HPP_

#include "autoware/trajectory/temporal_trajectory.hpp"
#include "autoware/trajectory/threshold.hpp"

#include <vector>

namespace autoware::experimental::trajectory
{

struct StopPointInterval
{
  const double start_time;
  const double end_time;
  const double distance;
};

[[nodiscard]] std::vector<StopPointInterval> find_stop_points(
  const TemporalTrajectory & trajectory,
  const double velocity_threshold = k_zero_velocity_threshold, const int max_iter = 0);

}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__UTILS__TEMPORAL__FIND_STOP_POINTS_HPP_
