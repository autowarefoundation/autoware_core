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

#ifndef TWIST2ACCEL_HPP_
#define TWIST2ACCEL_HPP_

#include "accel_estimator.hpp"

#include <autoware/agnocast_wrapper/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <optional>

namespace autoware::twist2accel
{
class Twist2Accel : public autoware::agnocast_wrapper::Node
{
public:
  explicit Twist2Accel(const rclcpp::NodeOptions & node_options);

private:
  AUTOWARE_PUBLISHER_PTR(geometry_msgs::msg::AccelWithCovarianceStamped)
  pub_accel_;  //!< @brief stop flag publisher
  AUTOWARE_SUBSCRIPTION_PTR(nav_msgs::msg::Odometry)
  sub_odom_;  //!< @brief measurement odometry subscriber
  AUTOWARE_SUBSCRIPTION_PTR(geometry_msgs::msg::TwistWithCovarianceStamped)
  sub_twist_;  //!< @brief measurement odometry subscriber

  std::optional<geometry_msgs::msg::TwistStamped> prev_twist_;
  bool use_odom_;
  std::unique_ptr<AccelEstimator> accel_estimator_;

  /**
   * @brief set odometry measurement
   */
  void callback_twist_with_covariance(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(geometry_msgs::msg::TwistWithCovarianceStamped) msg);
  void callback_odometry(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(nav_msgs::msg::Odometry) msg);
  void estimate_accel(const geometry_msgs::msg::TwistStamped & twist);
};
}  // namespace autoware::twist2accel
#endif  // TWIST2ACCEL_HPP_
