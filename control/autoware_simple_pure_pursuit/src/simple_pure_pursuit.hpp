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

#ifndef SIMPLE_PURE_PURSUIT_HPP_
#define SIMPLE_PURE_PURSUIT_HPP_

#include "simple_pure_pursuit_core_logics.hpp"

#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>

namespace autoware::control::simple_pure_pursuit
{
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::Odometry;

class SimplePurePursuitNode : public rclcpp::Node
{
public:
  explicit SimplePurePursuitNode(const rclcpp::NodeOptions & node_options);

private:
  // subscribers
  autoware_utils_rclcpp::InterProcessPollingSubscriber<Odometry> odom_sub_{
    this, "~/input/odometry"};
  autoware_utils_rclcpp::InterProcessPollingSubscriber<Trajectory> traj_sub_{
    this, "~/input/trajectory"};

  // publishers
  rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr pub_control_command_;

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  // simple_pure_pursuit core logics
  std::unique_ptr<SimplePurePursuitCoreLogics> core_logics_;

  // pure pursuit parameters
  const double lookahead_gain_;
  const double lookahead_min_distance_;
  const double speed_proportional_gain_;
  const bool use_external_target_vel_;
  const double external_target_vel_;

  // functions
  void on_timer();

};

}  // namespace autoware::control::simple_pure_pursuit

#endif  // SIMPLE_PURE_PURSUIT_HPP_
