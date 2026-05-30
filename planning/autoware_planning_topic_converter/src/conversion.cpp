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

#include <autoware/planning_topic_converter/conversion.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <vector>

namespace autoware::planning_topic_converter
{

TrajectoryPoint convertToTrajectoryPoint(const PathPoint & point)
{
  TrajectoryPoint traj_point;
  traj_point.pose = autoware_utils_geometry::get_pose(point);
  traj_point.longitudinal_velocity_mps = point.longitudinal_velocity_mps;
  traj_point.lateral_velocity_mps = point.lateral_velocity_mps;
  traj_point.heading_rate_rps = point.heading_rate_rps;
  return traj_point;
}

std::vector<TrajectoryPoint> convertToTrajectoryPoints(const std::vector<PathPoint> & points)
{
  std::vector<TrajectoryPoint> traj_points;
  traj_points.reserve(points.size());
  for (const auto & point : points) {
    traj_points.emplace_back(convertToTrajectoryPoint(point));
  }
  return traj_points;
}

}  // namespace autoware::planning_topic_converter
