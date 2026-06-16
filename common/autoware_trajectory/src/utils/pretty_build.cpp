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

#include "autoware/trajectory/utils/pretty_build.hpp"

#include <vector>

namespace autoware::experimental::trajectory
{

template std::optional<Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>>
pretty_build(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const bool use_akima);

template std::optional<Trajectory<autoware_planning_msgs::msg::PathPoint>> pretty_build(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points, const bool use_akima);

template std::optional<Trajectory<autoware_planning_msgs::msg::TrajectoryPoint>> pretty_build(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points, const bool use_akima);

std::optional<TemporalTrajectory> pretty_build_temporal(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points, const bool use_akima)
{
  if (use_akima) {
    const auto try_trajectory =
      TemporalTrajectory::Builder{}.set_xy_interpolator<interpolator::AkimaSpline>().build(points);
    if (!try_trajectory) {
      return std::nullopt;
    }
    return try_trajectory.value();
  }

  const auto try_trajectory = TemporalTrajectory::Builder{}.build(points);
  if (!try_trajectory) {
    return std::nullopt;
  }
  if (try_trajectory->length() < k_epsilon_distance) {
    return std::nullopt;
  }
  return try_trajectory.value();
}

}  // namespace autoware::experimental::trajectory
