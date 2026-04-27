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

#include "autoware/trajectory/utils/set_stopline.hpp"

#include "autoware/trajectory/detail/validate_range.hpp"
#include "autoware/trajectory/temporal_trajectory.hpp"

#include <algorithm>
#include <limits>

namespace autoware::experimental::trajectory
{

TemporalTrajectory set_stopline(TemporalTrajectory trajectory, const double arc_length)
{
  detail::throw_if_out_of_range(arc_length, 0.0, trajectory.length(), "arc_length");

  const auto stop_time = trajectory.distance_to_time(arc_length);
  trajectory.spatial_trajectory_.crop(0.0, arc_length);
  trajectory.spatial_trajectory_.longitudinal_velocity_mps()
    .at(trajectory.spatial_trajectory_.length())
    .set(0.0);
  trajectory.time_distance_mapping_.set_distance_range(
    stop_time, trajectory.time_distance_mapping_.end_time(),
    arc_length + trajectory.distance_offset_);
  return trajectory;
}

TemporalTrajectory insert_stop_duration(
  TemporalTrajectory trajectory, const double arc_length, const double duration)
{
  detail::throw_if_out_of_range(arc_length, 0.0, trajectory.length(), "arc_length");
  detail::throw_if_out_of_range(duration, 0.0, std::numeric_limits<double>::infinity(), "duration");

  const auto stop_time = trajectory.distance_to_time(arc_length);

  trajectory.spatial_trajectory_.longitudinal_velocity_mps().at(arc_length).set(0.0);
  trajectory.time_distance_mapping_.extend_time_at(stop_time, duration);
  return trajectory;
}

}  // namespace autoware::experimental::trajectory
