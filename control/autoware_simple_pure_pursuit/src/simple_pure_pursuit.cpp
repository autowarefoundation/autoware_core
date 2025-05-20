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

#include "simple_pure_pursuit.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <tf2/utils.hpp>

#include <algorithm>

namespace autoware::control::simple_pure_pursuit
{
using autoware::motion_utils::findNearestIndex;

SimplePurePursuitNode::SimplePurePursuitNode(const rclcpp::NodeOptions & node_options)
: Node("simple_pure_pursuit", node_options),
  pub_control_command_(create_publisher<autoware_control_msgs::msg::Control>(
    "~/output/control_command", rclcpp::QoS(1).transient_local())),
  vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo()),
  lookahead_gain_(declare_parameter<float>("lookahead_gain")),
  lookahead_min_distance_(declare_parameter<float>("lookahead_min_distance")),
  speed_proportional_gain_(declare_parameter<float>("speed_proportional_gain")),
  use_external_target_vel_(declare_parameter<bool>("use_external_target_vel")),
  external_target_vel_(declare_parameter<float>("external_target_vel"))
{
  using namespace std::literals::chrono_literals;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 30ms, std::bind(&SimplePurePursuitNode::on_timer, this));
}

void SimplePurePursuitNode::on_timer()
{
  // 1. subscribe data
  const auto odom_ptr = odom_sub_.take_data();
  const auto traj_ptr = traj_sub_.take_data();
  if (!odom_ptr || !traj_ptr) {
    return;
  }

  // 2. extract subscribed data
  const auto odom = *odom_ptr;
  const auto traj = *traj_ptr;

  // 3. create control command
  const auto control_command = create_control_command(odom, traj);

  // 4. publish control command
  pub_control_command_->publish(control_command);
}

autoware_control_msgs::msg::Control SimplePurePursuitNode::create_control_command(
  const Odometry & odom, const Trajectory & traj)
{
  const size_t closest_traj_point_idx = findNearestIndex(traj.points, odom.pose.pose.position);

  // when the ego reaches the goal
  if (closest_traj_point_idx == traj.points.size() - 1 || traj.points.size() <= 5) {
    autoware_control_msgs::msg::Control control_command;
    control_command.stamp = odom.header.stamp;
    control_command.longitudinal.velocity = 0.0;
    control_command.longitudinal.acceleration = -10.0;
    control_command.longitudinal.is_defined_acceleration = true;
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000, "reached to the goal");
    return control_command;
  }

  // calculate target longitudinal velocity
  const double target_longitudinal_vel =
    use_external_target_vel_ ? external_target_vel_
                             : traj.points.at(closest_traj_point_idx).longitudinal_velocity_mps;

  // calculate control command
  autoware_control_msgs::msg::Control control_command;
  control_command.stamp = odom.header.stamp;
  control_command.longitudinal = calc_longitudinal_control(odom, target_longitudinal_vel);
  control_command.lateral =
    calc_lateral_control(odom, traj, target_longitudinal_vel, closest_traj_point_idx);

  return control_command;
}

autoware_control_msgs::msg::Longitudinal SimplePurePursuitNode::calc_longitudinal_control(
  const Odometry & odom, const double target_longitudinal_vel) const
{
  const double current_longitudinal_vel = odom.twist.twist.linear.x;

  autoware_control_msgs::msg::Longitudinal longitudinal_control_command;
  longitudinal_control_command.velocity = target_longitudinal_vel;
  longitudinal_control_command.acceleration =
    speed_proportional_gain_ * (target_longitudinal_vel - current_longitudinal_vel);

  return longitudinal_control_command;
}

autoware_control_msgs::msg::Lateral SimplePurePursuitNode::calc_lateral_control(
  const Odometry & odom, const Trajectory & traj, const double target_longitudinal_vel,
  const size_t closest_traj_point_idx) const
{
  // calculate lookahead distance
  const double lookahead_distance =
    lookahead_gain_ * target_longitudinal_vel + lookahead_min_distance_;

  // calculate center coordinate of rear wheel
  const double vehicle_heading = tf2::getYaw(odom.pose.pose.orientation);
  const double rear_x =
    odom.pose.pose.position.x - vehicle_info_.wheel_base_m / 2.0 * std::cos(vehicle_heading);
  const double rear_y =
    odom.pose.pose.position.y - vehicle_info_.wheel_base_m / 2.0 * std::sin(vehicle_heading);

  // search lookahead point
  auto lookahead_point_itr = std::find_if(
    traj.points.begin() + closest_traj_point_idx, traj.points.end(),
    [&](const TrajectoryPoint & point) {
      return std::hypot(point.pose.position.x - rear_x, point.pose.position.y - rear_y) >=
             lookahead_distance;
    });
  if (lookahead_point_itr == traj.points.end()) {
    lookahead_point_itr = traj.points.end() - 1;
  }
  const double lookahead_point_x = lookahead_point_itr->pose.position.x;
  const double lookahead_point_y = lookahead_point_itr->pose.position.y;

  // calculate steering angle
  autoware_control_msgs::msg::Lateral lateral_control_command;
  const double alpha = std::atan2(lookahead_point_y - rear_y, lookahead_point_x - rear_x) -
                       tf2::getYaw(odom.pose.pose.orientation);
  lateral_control_command.steering_tire_angle =
    std::atan2(2.0 * vehicle_info_.wheel_base_m * std::sin(alpha), lookahead_distance);

  return lateral_control_command;
}
}  // namespace autoware::control::simple_pure_pursuit

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::control::simple_pure_pursuit::SimplePurePursuitNode)
