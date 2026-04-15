// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__TRAJECTORY__UTILS__CROP_HPP_
#define AUTOWARE__TRAJECTORY__UTILS__CROP_HPP_

#include "autoware/trajectory/forward.hpp"

namespace autoware::experimental::trajectory
{

template <class PointType>
Trajectory<PointType> crop(
  Trajectory<PointType> trajectory, const double & start, const double & end)
{
  trajectory.crop(start, end);
  return trajectory;
}

/**
 * @brief Crop a temporal trajectory to a time window.
 * @param[in] trajectory Input temporal trajectory.
 * @param[in] start_time Window start time in seconds.
 * @param[in] duration Window duration in seconds.
 * @return Cropped temporal trajectory.
 * @throw std::out_of_range If start_time or start_time + duration is outside the trajectory time
 * range.
 */
TemporalTrajectory crop_time(TemporalTrajectory trajectory, double start_time, double duration);

/**
 * @brief Crop a temporal trajectory to a distance window.
 * @param[in] trajectory Input temporal trajectory.
 * @param[in] start_distance Window start distance in meters.
 * @param[in] length Window length in meters.
 * @return Cropped temporal trajectory.
 * @throw std::out_of_range If start_distance or start_distance + length is outside [0, length()].
 */
TemporalTrajectory crop_distance(
  TemporalTrajectory trajectory, double start_distance, double length);

}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__UTILS__CROP_HPP_
