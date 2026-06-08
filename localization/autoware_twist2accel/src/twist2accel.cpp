// Copyright 2022 TIER IV
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

#include "twist2accel.hpp"

#include <functional>
#include <memory>

namespace autoware::twist2accel
{
using std::placeholders::_1;

Twist2Accel::Twist2Accel(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("twist2accel", node_options),
  processor_(declare_parameter<double>("accel_lowpass_gain")),
  use_odom_(declare_parameter<bool>("use_odom"))
{
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "input/odom", 1, std::bind(&Twist2Accel::callback_odometry, this, _1));
  sub_twist_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "input/twist", 1, std::bind(&Twist2Accel::callback_twist_with_covariance, this, _1));

  pub_accel_ = create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>("output/accel", 1);
}

void Twist2Accel::callback_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!use_odom_) {
    return;
  }

  geometry_msgs::msg::TwistStamped twist;
  twist.header = msg->header;
  twist.twist = msg->twist.twist;

  pub_accel_->publish(processor_.estimate_accel(twist));
}

void Twist2Accel::callback_twist_with_covariance(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  if (use_odom_) {
    return;
  }

  geometry_msgs::msg::TwistStamped twist;
  twist.header = msg->header;
  twist.twist = msg->twist.twist;

  pub_accel_->publish(processor_.estimate_accel(twist));
}

}  // namespace autoware::twist2accel

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::twist2accel::Twist2Accel)
