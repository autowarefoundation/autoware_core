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

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>

namespace autoware::twist2accel
{
class Twist2Accel : public rclcpp::Node
{
public:
  explicit Twist2Accel(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr
    pub_accel_;  //!< @brief stop flag publisher
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    sub_odom_;  //!< @brief measurement odometry subscriber
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    sub_twist_;  //!< @brief measurement odometry subscriber

  bool use_odom_;
  std::unique_ptr<AccelEstimator> accel_estimator_;

  /**
   * @brief set odometry measurement
   */
  void callback_twist_with_covariance(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
  void callback_odometry(const nav_msgs::msg::Odometry::SharedPtr msg);
};
}  // namespace autoware::twist2accel
#endif  // TWIST2ACCEL_HPP_
