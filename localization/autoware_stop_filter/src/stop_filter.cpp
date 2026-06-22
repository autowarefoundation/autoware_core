// Copyright 2021-2025 TIER IV
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

#include "stop_filter.hpp"

#include <cmath>

namespace autoware::stop_filter
{
namespace
{
bool is_stopped(
  const nav_msgs::msg::Odometry & input, const double linear_x_threshold,
  const double angular_z_threshold)
{
  const bool linear_stopped = std::fabs(input.twist.twist.linear.x) < linear_x_threshold;
  const bool angular_stopped = std::fabs(input.twist.twist.angular.z) < angular_z_threshold;
  return linear_stopped && angular_stopped;
}
}  // namespace

StopFilter::StopFilter(const double linear_x_threshold, const double angular_z_threshold)
: linear_x_threshold_(linear_x_threshold), angular_z_threshold_(angular_z_threshold)
{
}

autoware_internal_debug_msgs::msg::BoolStamped StopFilter::create_stop_flag_msg(
  const nav_msgs::msg::Odometry::SharedPtr input) const
{
  autoware_internal_debug_msgs::msg::BoolStamped stop_flag_msg;
  stop_flag_msg.stamp = input->header.stamp;
  stop_flag_msg.data = is_stopped(*input, linear_x_threshold_, angular_z_threshold_);
  return stop_flag_msg;
}

nav_msgs::msg::Odometry StopFilter::create_filtered_msg(
  const nav_msgs::msg::Odometry::SharedPtr input) const
{
  nav_msgs::msg::Odometry filtered_msg = *input;

  if (is_stopped(*input, linear_x_threshold_, angular_z_threshold_)) {
    filtered_msg.twist.twist.linear.x = 0.0;
    filtered_msg.twist.twist.linear.y = 0.0;
    filtered_msg.twist.twist.linear.z = 0.0;
    filtered_msg.twist.twist.angular.x = 0.0;
    filtered_msg.twist.twist.angular.y = 0.0;
    filtered_msg.twist.twist.angular.z = 0.0;
  }

  return filtered_msg;
}

}  // namespace autoware::stop_filter
