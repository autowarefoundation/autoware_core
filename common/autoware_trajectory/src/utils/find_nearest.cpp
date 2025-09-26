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

#include "autoware/trajectory/utils/find_nearest.hpp"

#include "autoware/trajectory/detail/types.hpp"
#include "autoware/trajectory/forward.hpp"
#include "autoware/trajectory/threshold.hpp"
#include "autoware_utils_geometry/geometry.hpp"
#include "autoware_utils_geometry/pose_deviation.hpp"

namespace autoware::experimental::trajectory
{

// Extern template for Point =====================================================================
template double find_nearest_index<autoware_planning_msgs::msg::PathPoint>(
  const Trajectory<autoware_planning_msgs::msg::PathPoint> & trajectory,
  const geometry_msgs::msg::Point & point);

template double find_nearest_index<autoware_internal_planning_msgs::msg::PathPointWithLaneId>(
  const Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & trajectory,
  const geometry_msgs::msg::Point & point);

template double find_nearest_index<geometry_msgs::msg::Pose>(
  const Trajectory<geometry_msgs::msg::Pose> & trajectory, const geometry_msgs::msg::Point & point);

template double find_nearest_index<autoware_planning_msgs::msg::TrajectoryPoint>(
  const Trajectory<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const geometry_msgs::msg::Point & point);

// Extern template for Pose =====================================================================
template std::optional<double> find_first_nearest_index<autoware_planning_msgs::msg::PathPoint>(
  const Trajectory<autoware_planning_msgs::msg::PathPoint> & trajectory,
  const geometry_msgs::msg::Pose & pose, const double max_dist, const double max_yaw);

template std::optional<double>
find_first_nearest_index<autoware_internal_planning_msgs::msg::PathPointWithLaneId>(
  const Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & trajectory,
  const geometry_msgs::msg::Pose & pose, const double max_dist, const double max_yaw);

template std::optional<double> find_first_nearest_index<geometry_msgs::msg::Pose>(
  const Trajectory<geometry_msgs::msg::Pose> & trajectory, const geometry_msgs::msg::Pose & pose,
  const double max_dist, const double max_yaw);

template std::optional<double>
find_first_nearest_index<autoware_planning_msgs::msg::TrajectoryPoint>(
  const Trajectory<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const geometry_msgs::msg::Pose & pose, const double max_dist, const double max_yaw);

}  // namespace autoware::experimental::trajectory
