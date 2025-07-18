// Copyright 2025 TIER IV
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

#include "stop_filter_node.hpp"

namespace autoware::stop_filter
{

StopFilterProcessor::StopFilterProcessor(double linear_x_threshold, double angular_z_threshold)
: stop_filter_(linear_x_threshold, angular_z_threshold)
{
}

FilterResult StopFilterProcessor::apply_filter(const nav_msgs::msg::Odometry::SharedPtr input) const
{
  Vector3D linear_velocity{
    input->twist.twist.linear.x, input->twist.twist.linear.y, input->twist.twist.linear.z};
  Vector3D angular_velocity{
    input->twist.twist.angular.x, input->twist.twist.angular.y, input->twist.twist.angular.z};

  return stop_filter_.apply_stop_filter(linear_velocity, angular_velocity);
}

autoware_internal_debug_msgs::msg::BoolStamped StopFilterProcessor::create_stop_flag_msg(
  const nav_msgs::msg::Odometry::SharedPtr input) const
{
  autoware_internal_debug_msgs::msg::BoolStamped stop_flag_msg;
  stop_flag_msg.stamp = input->header.stamp;

  FilterResult result = apply_filter(input);
  stop_flag_msg.data = result.was_stopped;

  return stop_flag_msg;
}

nav_msgs::msg::Odometry StopFilterProcessor::create_filtered_msg(
  const nav_msgs::msg::Odometry::SharedPtr input) const
{
  nav_msgs::msg::Odometry filtered_msg = *input;

  FilterResult result = apply_filter(input);
  filtered_msg.twist.twist.linear.x = result.linear_velocity.x;
  filtered_msg.twist.twist.linear.y = result.linear_velocity.y;
  filtered_msg.twist.twist.linear.z = result.linear_velocity.z;
  filtered_msg.twist.twist.angular.x = result.angular_velocity.x;
  filtered_msg.twist.twist.angular.y = result.angular_velocity.y;
  filtered_msg.twist.twist.angular.z = result.angular_velocity.z;

  return filtered_msg;
}

StopFilterNode::StopFilterNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("stop_filter", node_options),
  message_processor_(
    declare_parameter<double>("vx_threshold"), declare_parameter<double>("wz_threshold"))
{
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "input/odom", 1, std::bind(&StopFilterNode::callback_odometry, this, std::placeholders::_1));

  pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("output/odom", 1);
  pub_stop_flag_ =
    create_publisher<autoware_internal_debug_msgs::msg::BoolStamped>("debug/stop_flag", 1);
}

void StopFilterNode::callback_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  pub_stop_flag_->publish(message_processor_.create_stop_flag_msg(msg));
  pub_odom_->publish(message_processor_.create_filtered_msg(msg));
}
}  // namespace autoware::stop_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::stop_filter::StopFilterNode)
