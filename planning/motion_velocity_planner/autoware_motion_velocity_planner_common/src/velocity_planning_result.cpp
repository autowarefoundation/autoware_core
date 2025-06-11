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

#include "autoware/motion_velocity_planner_common/velocity_planning_result.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <vector>

namespace autoware::motion_velocity_planner
{
void add_planning_results_to_trajectory(
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const VelocityPlanningResult & planning_result)
{
  for (const auto & stop_point : planning_result.stop_points) insert_stop(trajectory, stop_point);
  for (const auto & slowdown_interval : planning_result.slowdown_intervals)
    insert_slowdown(trajectory, slowdown_interval);
}

void insert_stop(
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const geometry_msgs::msg::Point & stop_point)
{
  const auto length = autoware::motion_utils::calcSignedArcLength(trajectory, 0, stop_point);
  autoware::motion_utils::insertStopPoint(length, trajectory);
}

void insert_slowdown(
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const autoware::motion_velocity_planner::SlowdownInterval & slowdown_interval)
{
  const auto from_seg_idx =
    autoware::motion_utils::findNearestSegmentIndex(trajectory, slowdown_interval.from);
  const auto from_insert_idx =
    autoware::motion_utils::insertTargetPoint(from_seg_idx, slowdown_interval.from, trajectory);
  const auto to_seg_idx =
    autoware::motion_utils::findNearestSegmentIndex(trajectory, slowdown_interval.to);
  const auto to_insert_idx =
    autoware::motion_utils::insertTargetPoint(to_seg_idx, slowdown_interval.to, trajectory);
  if (from_insert_idx && to_insert_idx) {
    for (auto idx = *from_insert_idx; idx <= *to_insert_idx; ++idx) {
      trajectory[idx].longitudinal_velocity_mps =
        std::min(  // prevent the node from increasing the velocity
          trajectory[idx].longitudinal_velocity_mps,
          static_cast<float>(slowdown_interval.velocity));
    }
  }
}
}  // namespace autoware::motion_velocity_planner
