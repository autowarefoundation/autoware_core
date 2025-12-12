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

#include "autoware/trajectory/utils/velocity.hpp"

#include "autoware/trajectory/forward.hpp"
#include "autoware/trajectory/trajectory_point.hpp"

#include "autoware_planning_msgs/msg/trajectory_point.hpp"

namespace autoware::experimental::trajectory
{

template std::optional<double>
search_zero_velocity_position<autoware_planning_msgs::msg::TrajectoryPoint>(
  const Trajectory<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const double start_distance, const double end_distance);

template std::optional<double>
search_zero_velocity_position<autoware_planning_msgs::msg::TrajectoryPoint>(
  const Trajectory<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const double start_distance);

template std::optional<double>
search_zero_velocity_position<autoware_planning_msgs::msg::TrajectoryPoint>(
  const Trajectory<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory);

template bool has_zero_velocity<autoware_planning_msgs::msg::TrajectoryPoint>(
  const Trajectory<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory);

}  // namespace autoware::experimental::trajectory
