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

#include "autoware/trajectory/utils/crop.hpp"

#include "autoware/trajectory/detail/validate_range.hpp"
#include "autoware/trajectory/temporal_trajectory.hpp"

#include <algorithm>
#include <limits>

namespace autoware::experimental::trajectory
{

TemporalTrajectory crop_time(
  TemporalTrajectory trajectory, const double start_time, const double duration)
{
  detail::throw_if_out_of_range(
    start_time, trajectory.start_time(), trajectory.end_time(), "start_time");
  detail::throw_if_out_of_range(duration, 0.0, std::numeric_limits<double>::infinity(), "duration");
  const auto clamped_duration = std::min(duration, trajectory.end_time() - start_time);

  const auto absolute_start_distance = trajectory.time_distance_mapping_.distance_at(start_time);
  const auto absolute_end_distance =
    trajectory.time_distance_mapping_.distance_at(start_time + clamped_duration);
  trajectory.spatial_trajectory_.crop(
    absolute_start_distance - trajectory.distance_offset_,
    absolute_end_distance - absolute_start_distance);
  trajectory.time_distance_mapping_.set_time_range(start_time, start_time + clamped_duration);
  trajectory.distance_offset_ = absolute_start_distance;
  return trajectory;
}

TemporalTrajectory crop_distance(
  TemporalTrajectory trajectory, const double start_distance, const double length)
{
  detail::throw_if_out_of_range(start_distance, 0.0, trajectory.length(), "start_distance");
  detail::throw_if_out_of_range(length, 0.0, std::numeric_limits<double>::infinity(), "length");
  const auto clamped_length = std::min(length, trajectory.length() - start_distance);

  const auto start_time = trajectory.distance_to_time(start_distance);
  const auto end_time = trajectory.distance_to_time(start_distance + clamped_length, true);

  trajectory.spatial_trajectory_.crop(start_distance, clamped_length);
  trajectory.time_distance_mapping_.set_time_range(start_time, end_time);
  trajectory.distance_offset_ = start_distance + trajectory.distance_offset_;
  return trajectory;
}

}  // namespace autoware::experimental::trajectory
