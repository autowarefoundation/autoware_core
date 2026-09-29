// Copyright 2023 TIER IV, Inc.
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

#ifndef PATH_TO_TRAJECTORY_HPP_
#define PATH_TO_TRAJECTORY_HPP_

#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

namespace autoware::planning_topic_converter::path_to_trajectory
{

/// @brief Convert a Path into a Trajectory by mapping each point's pose, longitudinal/lateral
/// velocity and heading rate one-to-one. TrajectoryPoint-only fields (time_from_start,
/// acceleration, wheel angles) are left at their default (zero) value.
autoware_planning_msgs::msg::Trajectory convert(const autoware_planning_msgs::msg::Path & path);

}  // namespace autoware::planning_topic_converter::path_to_trajectory

#endif  // PATH_TO_TRAJECTORY_HPP_
