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

#include "autoware/trajectory/utils/temporal/find_stop_points.hpp"

#include "autoware/trajectory/utils/find_intervals.hpp"

#include <cmath>
#include <vector>

namespace autoware::experimental::trajectory
{

std::vector<StopPointInterval> find_stop_points(
  const TemporalTrajectory & trajectory, const double velocity_threshold, const int max_iter)
{
  const auto intervals = detail::impl::find_intervals_impl(
    trajectory.get_underlying_time_bases(),
    [&trajectory, velocity_threshold](const double & t) {
      return std::abs(trajectory.compute_from_time(t).longitudinal_velocity_mps) <=
             velocity_threshold;
    },
    max_iter);

  std::vector<StopPointInterval> stop_points;
  stop_points.reserve(intervals.size());
  for (const auto & interval : intervals) {
    stop_points.emplace_back(
      StopPointInterval{interval.start, interval.end, trajectory.time_to_distance(interval.start)});
  }
  return stop_points;
}

}  // namespace autoware::experimental::trajectory
