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

#include "autoware/motion_utils/trajectory/conversion.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/trajectory_utils.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <autoware/velocity_smoother/trajectory_utils.hpp>

#include <iostream>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using TrajectoryPointWithIdx = std::pair<TrajectoryPoint, size_t>;

//! smooth path point with lane id starts from ego position on path to the path end
bool smoothPath(
  const PathWithLaneId & in_path, PathWithLaneId & out_path,
  const std::shared_ptr<const PlannerData> & planner_data)
{
  return smoothPath(in_path, out_path, *planner_data);
}

bool smoothPath(
  const PathWithLaneId & in_path, PathWithLaneId & out_path, const PlannerData & planner_data)
{
  const geometry_msgs::msg::Pose current_pose = planner_data.current_odometry->pose;
  const double v0 = planner_data.current_velocity->twist.linear.x;
  const double a0 = planner_data.current_acceleration->accel.accel.linear.x;
  const auto & external_v_limit = planner_data.external_velocity_limit;
  const auto & smoother = planner_data.velocity_smoother_;

  auto trajectory = autoware::motion_utils::convertToTrajectoryPoints<
    autoware_internal_planning_msgs::msg::PathWithLaneId>(in_path);
  const auto traj_lateral_acc_filtered = smoother->applyLateralAccelerationFilter(trajectory);

  const auto traj_steering_rate_limited =
    smoother->applySteeringRateLimit(traj_lateral_acc_filtered, false);

  // Resample trajectory with ego-velocity based interval distances
  auto traj_resampled = smoother->resampleTrajectory(
    traj_steering_rate_limited, v0, current_pose, planner_data.ego_nearest_dist_threshold,
    planner_data.ego_nearest_yaw_threshold);
  const size_t traj_resampled_closest =
    autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
      traj_resampled, current_pose, planner_data.ego_nearest_dist_threshold,
      planner_data.ego_nearest_yaw_threshold);
  std::vector<TrajectoryPoints> debug_trajectories;
  // Clip trajectory from closest point
  TrajectoryPoints clipped;
  TrajectoryPoints traj_smoothed;
  clipped.insert(
    clipped.end(), traj_resampled.begin() + static_cast<std::ptrdiff_t>(traj_resampled_closest),
    traj_resampled.end());
  if (!smoother->apply(v0, a0, clipped, traj_smoothed, debug_trajectories, false)) {
    std::cerr << "[behavior_velocity][trajectory_utils]: failed to smooth" << std::endl;
    return false;
  }
  traj_smoothed.insert(
    traj_smoothed.begin(), traj_resampled.begin(),
    traj_resampled.begin() + static_cast<std::ptrdiff_t>(traj_resampled_closest));

  if (external_v_limit) {
    autoware::velocity_smoother::trajectory_utils::applyMaximumVelocityLimit(
      traj_resampled_closest, traj_smoothed.size(), external_v_limit->max_velocity, traj_smoothed);
  }
  out_path = autoware::motion_utils::convertToPathWithLaneId<TrajectoryPoints>(traj_smoothed);
  return true;
}

std::optional<Trajectory> smoothPath(const Trajectory & in_path, const PlannerData & planner_data)
{
  const auto & current_pose = planner_data.current_odometry->pose;
  const auto & v0 = planner_data.current_velocity->twist.linear.x;
  const auto & a0 = planner_data.current_acceleration->accel.accel.linear.x;
  const auto & external_v_limit = planner_data.external_velocity_limit;
  const auto & smoother = planner_data.velocity_smoother_;

  const auto in_path_points = in_path.restore();
  TrajectoryPoints traj(in_path_points.size());
  std::transform(
    in_path_points.begin(), in_path_points.end(), traj.begin(), [](const PathPointWithLaneId & p) {
      TrajectoryPoint tp;
      tp.pose = p.point.pose;
      tp.longitudinal_velocity_mps = p.point.longitudinal_velocity_mps;
      tp.acceleration_mps2 = 0;
      return tp;
    });

  const auto traj_lateral_acc_filtered = smoother->applyLateralAccelerationFilter(traj);

  const auto traj_steering_rate_limited =
    smoother->applySteeringRateLimit(traj_lateral_acc_filtered, false);

  // Resample trajectory with ego-velocity based interval distances
  auto traj_resampled = smoother->resampleTrajectory(
    traj_steering_rate_limited, v0, current_pose, planner_data.ego_nearest_dist_threshold,
    planner_data.ego_nearest_yaw_threshold);

  const auto traj_resampled_closest =
    autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
      traj_resampled, current_pose, planner_data.ego_nearest_dist_threshold,
      planner_data.ego_nearest_yaw_threshold);

  // Clip trajectory from closest point
  TrajectoryPoints clipped{
    traj_resampled.begin() + static_cast<std::ptrdiff_t>(traj_resampled_closest),
    traj_resampled.end()};
  TrajectoryPoints traj_smoothed;
  std::vector<TrajectoryPoints> debug_trajectories;

  if (!smoother->apply(v0, a0, clipped, traj_smoothed, debug_trajectories, false)) {
    std::cerr << "[behavior_velocity][trajectory_utils]: failed to smooth" << std::endl;
    return std::nullopt;
  }

  traj_smoothed.insert(
    traj_smoothed.begin(), traj_resampled.begin(),
    traj_resampled.begin() + static_cast<std::ptrdiff_t>(traj_resampled_closest));

  if (external_v_limit) {
    autoware::velocity_smoother::trajectory_utils::applyMaximumVelocityLimit(
      traj_resampled_closest, traj_smoothed.size(), external_v_limit->max_velocity, traj_smoothed);
  }

  std::vector<PathPointWithLaneId> out_path_points(traj_smoothed.size());
  std::transform(
    traj_smoothed.begin(), traj_smoothed.end(), out_path_points.begin(),
    [](const TrajectoryPoint & tp) {
      PathPointWithLaneId p;
      p.point.pose = tp.pose;
      p.point.longitudinal_velocity_mps = tp.longitudinal_velocity_mps;
      return p;
    });

  return autoware::experimental::trajectory::pretty_build(out_path_points);
}

}  // namespace autoware::behavior_velocity_planner
