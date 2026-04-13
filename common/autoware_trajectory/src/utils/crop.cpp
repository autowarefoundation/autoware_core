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

#include "autoware/trajectory/temporal_trajectory.hpp"

#include <algorithm>

namespace autoware::experimental::trajectory
{

TemporalTrajectory crop_time(
  TemporalTrajectory trajectory, const double start_time, const double duration)
{
  const auto clamped_start_time = trajectory.time_distance_mapping_.clamp_time(start_time, false);
  const auto clamped_end_time =
    trajectory.time_distance_mapping_.clamp_time(start_time + duration, false);

  const auto absolute_start_distance =
    trajectory.time_distance_mapping_.distance_at(clamped_start_time);
  const auto absolute_end_distance =
    trajectory.time_distance_mapping_.distance_at(clamped_end_time);
  trajectory.spatial_trajectory_.crop(
    absolute_start_distance - trajectory.distance_offset_,
    absolute_end_distance - absolute_start_distance);
  trajectory.time_distance_mapping_.set_time_range(clamped_start_time, clamped_end_time);
  trajectory.distance_offset_ = absolute_start_distance;
  return trajectory;
}

TemporalTrajectory crop_distance(
  TemporalTrajectory trajectory, const double start_distance, const double length)
{
  const auto clamped_start_distance = std::clamp(start_distance, 0.0, trajectory.length());
  const auto clamped_end_distance = std::clamp(start_distance + length, 0.0, trajectory.length());

  const auto start_time = trajectory.distance_to_time(clamped_start_distance);
  const auto end_time = trajectory.distance_to_time(clamped_end_distance);

  trajectory.spatial_trajectory_.crop(
    clamped_start_distance, clamped_end_distance - clamped_start_distance);
  trajectory.time_distance_mapping_.set_time_range(start_time, end_time);
  trajectory.distance_offset_ = clamped_start_distance + trajectory.distance_offset_;
  return trajectory;
}

}  // namespace autoware::experimental::trajectory
