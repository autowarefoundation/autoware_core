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

#ifndef STOP_FILTER_NODE_HPP_
#define STOP_FILTER_NODE_HPP_

#include "stop_filter.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/bool_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace autoware::stop_filter
{

class StopFilterProcessor
{
public:
  StopFilterProcessor(double linear_x_threshold, double angular_z_threshold);
  autoware_internal_debug_msgs::msg::BoolStamped create_stop_flag_msg(
    const nav_msgs::msg::Odometry::SharedPtr input) const;
  nav_msgs::msg::Odometry create_filtered_msg(const nav_msgs::msg::Odometry::SharedPtr input) const;

private:
  StopFilter stop_filter_;
  FilterResult apply_filter(const nav_msgs::msg::Odometry::SharedPtr input) const;
};

class StopFilterNode : public rclcpp::Node
{
public:
  explicit StopFilterNode(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;  //!< @brief odom publisher
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::BoolStamped>::SharedPtr
    pub_stop_flag_;  //!< @brief stop flag publisher
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    sub_odom_;  //!< @brief measurement odometry subscriber

  StopFilterProcessor message_processor_;

  /**
   * @brief set odometry measurement
   */
  void callback_odometry(const nav_msgs::msg::Odometry::SharedPtr msg);
};
}  // namespace autoware::stop_filter
#endif  // STOP_FILTER_NODE_HPP_
