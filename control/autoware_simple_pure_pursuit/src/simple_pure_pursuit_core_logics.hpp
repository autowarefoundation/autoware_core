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

#ifndef SIMPLE_PURE_PURSUIT_CORE_LOGICS_HPP_
#define SIMPLE_PURE_PURSUIT_CORE_LOGICS_HPP_

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace autoware::control::simple_pure_pursuit
{

/**
@ brief Params for SimplePurePursuit class
*
* @param lookahead_gain Gain for calculating lookahead distance
* @param lookahead_min_distance Minimum lookahead distance
* @param speed_proportional_gain Proportional gain for longitudinal control
* @param use_external_target_vel Whether to use external target velocity or not
* @param external_target_vel External target velocity to use when use_external_target_vel is true
* @param wheel_base_m Wheel base of vehicle in meters
*/
struct SimplePurePursuitParameters {
  double lookahead_gain;
  double lookahead_min_distance;
  double speed_proportional_gain;
  bool use_external_target_vel;
  double external_target_vel;
  double wheel_base_m;
};

class SimplePurePursuitCoreLogics
{

    public:
  
    // Constructor
    explicit SimplePurePursuitCoreLogics(const SimplePurePursuitParameters & parameters);
    
    // Set param struct for algorithm
    void set_parameters(const SimplePurePursuitParameters & parameters);

    /**
    * @brief Create control command based on current odom and traj
    *
    * @param odom Current odometry of vehicle
    * @param traj Current trajectory to follow
    *
    * @return Control command to be published
    */
    autoware_control_msgs::msg::Control create_control_command(
        const nav_msgs::msg::Odometry & odom, 
        const autoware_planning_msgs::msg::Trajectory & traj
    ) const;

};

}; // namespace autoware::control::simple_pure_pursuit