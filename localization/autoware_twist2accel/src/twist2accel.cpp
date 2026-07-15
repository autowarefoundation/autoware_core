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

#include "accel_estimator.hpp"

#include <functional>
#include <memory>
#include <utility>

namespace autoware::twist2accel
{
using std::placeholders::_1;

Twist2Accel::Twist2Accel(const rclcpp::NodeOptions & node_options)
: autoware::agnocast_wrapper::Node("twist2accel", node_options)
{
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "input/odom", 1, std::bind(&Twist2Accel::callback_odometry, this, _1));
  sub_twist_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "input/twist", 1, std::bind(&Twist2Accel::callback_twist_with_covariance, this, _1));

  pub_accel_ = create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>("output/accel", 1);

  const double accel_lowpass_gain = declare_parameter<double>("accel_lowpass_gain");
  use_odom_ = declare_parameter<bool>("use_odom");

  accel_estimator_ = std::make_unique<AccelEstimator>(accel_lowpass_gain);
}

void Twist2Accel::callback_odometry(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(nav_msgs::msg::Odometry) msg)
{
  if (!use_odom_) return;
  pub_accel_->publish(accel_estimator_->estimate(*msg));
}

void Twist2Accel::callback_twist_with_covariance(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(geometry_msgs::msg::TwistWithCovarianceStamped) msg)
{
  if (use_odom_) return;
  pub_accel_->publish(accel_estimator_->estimate(*msg));
}
}  // namespace autoware::twist2accel

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::twist2accel::Twist2Accel)
