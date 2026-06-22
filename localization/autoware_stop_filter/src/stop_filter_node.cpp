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

#include <utility>

namespace autoware::stop_filter
{

StopFilterNode::StopFilterNode(const rclcpp::NodeOptions & node_options)
: autoware::agnocast_wrapper::Node("stop_filter", node_options),
  stop_filter_(declare_parameter<double>("vx_threshold"), declare_parameter<double>("wz_threshold"))
{
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "input/odom", 1, std::bind(&StopFilterNode::callback_odometry, this, std::placeholders::_1));

  pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("output/odom", 1);
  pub_stop_flag_ =
    create_publisher<autoware_internal_debug_msgs::msg::BoolStamped>("debug/stop_flag", 1);
}

void StopFilterNode::callback_odometry(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(nav_msgs::msg::Odometry) & msg)
{
  auto stop_flag_msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_stop_flag_);
  *stop_flag_msg = stop_filter_.create_stop_flag_msg(*msg);
  pub_stop_flag_->publish(std::move(stop_flag_msg));

  auto filtered_msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_odom_);
  *filtered_msg = stop_filter_.create_filtered_msg(*msg);
  pub_odom_->publish(std::move(filtered_msg));
}
}  // namespace autoware::stop_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::stop_filter::StopFilterNode)
