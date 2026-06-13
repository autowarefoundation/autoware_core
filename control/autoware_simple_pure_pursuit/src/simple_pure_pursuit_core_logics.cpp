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
    : params_(params)
    {
    };

    // Params setter
    void SimplePurePursuitCoreLogics::set_params(const SimplePurePursuitParameters & params)
    {
        params_ = params;
    };

    // Core logic for creating control command based on current odom and traj
    autoware_control_msgs::msg::Control SimplePurePursuitCoreLogics::create_control_command(
        const nav_msgs::msg::Odometry & odom, 
        const autoware_planning_msgs::msg::Trajectory & traj) const
    {

        const size_t closest_traj_point_idx = findNearestIndex(traj.points, odom.pose.pose.position);

        // When ego reaches goal or traj is too short, stop vehicle
        if (
            (closest_traj_point_idx == traj.points.size() - 1) || 
            (traj.points.size() <= 5)
        ) {
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
        control_command.longitudinal = calc_longitudinal_control(
            odom, 
            target_longitudinal_vel
        );
        control_command.lateral = calc_lateral_control(
            odom, 
            traj, 
            target_longitudinal_vel, 
            closest_traj_point_idx
        );

        return control_command;

    }

}; // namespace autoware::control::simple_pure_pursuit