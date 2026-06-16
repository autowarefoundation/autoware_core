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

#ifndef AUTOWARE__TRAJECTORY__UTILS__ALIGN_ORIENTATION_HPP_
#define AUTOWARE__TRAJECTORY__UTILS__ALIGN_ORIENTATION_HPP_

#include "autoware/trajectory/temporal_trajectory.hpp"

namespace autoware::experimental::trajectory
{

/**
 * @brief Align the spatial trajectory orientation with the trajectory direction.
 * @param[in] trajectory Temporal trajectory to modify.
 * @return Modified temporal trajectory.
 */
TemporalTrajectory align_orientation_with_trajectory_direction(TemporalTrajectory trajectory);

}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__UTILS__ALIGN_ORIENTATION_HPP_
