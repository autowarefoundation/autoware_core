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

#include "simple_pure_pursuit_core_logics.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <tf2/utils.hpp>

#include <algorithm>

namespace autoware::control::simple_pure_pursuit
{

using autoware::motion_utils::findNearestIndex;

// Constructor
SimplePurePursuitCoreLogics::SimplePurePursuitCoreLogics(const SimplePurePursuitParameters & params)
: params_(params) {}

// Params setter
void SimplePurePursuitCoreLogics::set_params(const SimplePurePursuitParameters & params)
{
  params_ = params;
};

// Core logic for creating control command based on current odom and traj
autoware_control_msgs::msg::Control SimplePurePursuitCoreLogics::create_control_command(
  const nav_msgs::msg::Odometry & odom, const autoware_planning_msgs::msg::Trajectory & traj) const
{
  const size_t closest_traj_point_idx = findNearestIndex(traj.points, odom.pose.pose.position);

  // When ego reaches goal or traj is too short, stop vehicle
  if ((closest_traj_point_idx == traj.points.size() - 1) || (traj.points.size() <= 5)) {
    autoware_control_msgs::msg::Control control_command;
    control_command.stamp = odom.header.stamp;
    control_command.longitudinal.velocity = 0.0;
    control_command.longitudinal.acceleration = -10.0;
    control_command.longitudinal.is_defined_acceleration = true;

    return control_command;
  }

  // Calculate target longitudinal velocity
  const double target_longitudinal_vel =
    params_.use_external_target_vel
      ? params_.external_target_vel
      : traj.points.at(closest_traj_point_idx).longitudinal_velocity_mps;

  // Calculate control command
  autoware_control_msgs::msg::Control control_command;
  control_command.stamp = odom.header.stamp;
  control_command.longitudinal = calc_longitudinal_control(odom, target_longitudinal_vel);
  control_command.lateral =
    calc_lateral_control(odom, traj, target_longitudinal_vel, closest_traj_point_idx);

  return control_command;
};

// Core logic for calculating longitudinal control command
autoware_control_msgs::msg::Longitudinal SimplePurePursuitCoreLogics::calc_longitudinal_control(
  const nav_msgs::msg::Odometry & odom, const double target_longitudinal_vel) const
{
  autoware_control_msgs::msg::Longitudinal longitudinal_control_command;
  const double current_velocity = odom.twist.twist.linear.x;

  if (params_.use_external_target_vel) {
    longitudinal_control_command.velocity = static_cast<float>(params_.external_target_vel);
    longitudinal_control_command.acceleration = static_cast<float>(
      (params_.external_target_vel - current_velocity) * params_.speed_proportional_gain);

  } else {
    longitudinal_control_command.velocity = static_cast<float>(target_longitudinal_vel);
    longitudinal_control_command.acceleration = static_cast<float>(
      (target_longitudinal_vel - current_velocity) * params_.speed_proportional_gain);
  }

  return longitudinal_control_command;
};

// Core logic for calculating lateral control command
autoware_control_msgs::msg::Lateral SimplePurePursuitCoreLogics::calc_lateral_control(
  const nav_msgs::msg::Odometry & odom, const autoware_planning_msgs::msg::Trajectory & traj,
  const double target_longitudinal_vel, const size_t closest_traj_point_idx) const
{
  // Calculate lookahead distance
  const double lookahead_distance =
    params_.lookahead_gain * target_longitudinal_vel + params_.lookahead_min_distance;

  // Calculate center coordinate of rear wheel
  const double vehicle_heading = tf2::getYaw(odom.pose.pose.orientation);
  const double rear_x =
    odom.pose.pose.position.x - params_.wheel_base_m / 2.0 * std::cos(vehicle_heading);
  const double rear_y =
    odom.pose.pose.position.y - params_.wheel_base_m / 2.0 * std::sin(vehicle_heading);

  // Search for the lookahead point on the trajectory
  auto lookahead_point_itr = std::find_if(
    traj.points.begin() + static_cast<std::ptrdiff_t>(closest_traj_point_idx), traj.points.end(),
    [&](const autoware_planning_msgs::msg::TrajectoryPoint & point) {
      return std::hypot(point.pose.position.x - rear_x, point.pose.position.y - rear_y) >=
             lookahead_distance;
    });

  if (lookahead_point_itr == traj.points.end()) {
    lookahead_point_itr = traj.points.end() - 1;
  }

  const double lookahead_point_x = lookahead_point_itr->pose.position.x;
  const double lookahead_point_y = lookahead_point_itr->pose.position.y;

  // Calculate pure pursuit steering angle
  autoware_control_msgs::msg::Lateral lateral_control_command;
  const double alpha = std::atan2(lookahead_point_y - rear_y, lookahead_point_x - rear_x) -
                       tf2::getYaw(odom.pose.pose.orientation);

  lateral_control_command.steering_tire_angle = static_cast<float>(
    std::atan2(2.0 * params_.wheel_base_m * std::sin(alpha), lookahead_distance));

  return lateral_control_command;
}

};  // namespace autoware::control::simple_pure_pursuit
