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

#include <autoware/agnocast_wrapper/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/bool_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace autoware::stop_filter
{

class StopFilterNode : public autoware::agnocast_wrapper::Node
{
public:
  explicit StopFilterNode(const rclcpp::NodeOptions & node_options);

private:
  AUTOWARE_PUBLISHER_PTR(nav_msgs::msg::Odometry) pub_odom_;  //!< @brief odom publisher
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::BoolStamped)
  pub_stop_flag_;  //!< @brief stop flag publisher
  AUTOWARE_SUBSCRIPTION_PTR(nav_msgs::msg::Odometry)
  sub_odom_;  //!< @brief measurement odometry subscriber

  StopFilter stop_filter_;

  /**
   * @brief set odometry measurement
   */
  void callback_odometry(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(nav_msgs::msg::Odometry) & msg);
};
}  // namespace autoware::stop_filter
#endif  // STOP_FILTER_NODE_HPP_
