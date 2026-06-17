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

#include "simple_pure_pursuit_node.hpp"

#include "simple_pure_pursuit.hpp"

#include <memory>

namespace autoware::control::simple_pure_pursuit
{

SimplePurePursuitNode::SimplePurePursuitNode(const rclcpp::NodeOptions & node_options)
: Node("simple_pure_pursuit_node", node_options),
  pub_control_command_(
    create_publisher<autoware_control_msgs::msg::Control>(
      "~/output/control_command", rclcpp::QoS(1).transient_local()))
{
  // Vehicle info is now fetch locally
  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();

  // Read ROS params, fetch them into struct
  SimplePurePursuitParameters params{};
  params.lookahead_gain = declare_parameter<float>("lookahead_gain");
  params.lookahead_min_distance = declare_parameter<float>("lookahead_min_distance");
  params.speed_proportional_gain = declare_parameter<float>("speed_proportional_gain");
  params.use_external_target_vel = declare_parameter<bool>("use_external_target_vel");
  params.external_target_vel = declare_parameter<float>("external_target_vel");
  params.wheel_base_m = vehicle_info.wheel_base_m;

  // Init core logic
  core_logic_ = std::make_unique<SimplePurePursuit>(params);

  using namespace std::literals::chrono_literals;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 30ms, std::bind(&SimplePurePursuitNode::on_timer, this));
}

void SimplePurePursuitNode::on_timer()
{
  // 1. Subscribe data
  const auto odom_ptr = odom_sub_.take_data();
  const auto traj_ptr = traj_sub_.take_data();
  if (!odom_ptr || !traj_ptr) {
    return;
  }

  // 2. Extract subscribed data
  const auto & odom = *odom_ptr;
  const auto & traj = *traj_ptr;

  // 3. Input validation
  if (traj.points.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Received empty trajectory. Delegate handling to core logic.");
  }

  // 4. Delegate to core logic
  const auto control_command = core_logic_->create_control_command(odom, traj);

  // 5. Goal reached ROS check and notify
  if (
    (control_command.longitudinal.velocity == 0.0) &&
    (control_command.longitudinal.acceleration == -10.0)) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000, "reached to the goal");
  }

  // 6. Publish control command
  pub_control_command_->publish(control_command);
}

}  // namespace autoware::control::simple_pure_pursuit

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::control::simple_pure_pursuit::SimplePurePursuitNode)
