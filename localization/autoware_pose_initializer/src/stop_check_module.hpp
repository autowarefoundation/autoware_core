// Copyright 2022 The Autoware Contributors
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

#ifndef STOP_CHECK_MODULE_HPP_
#define STOP_CHECK_MODULE_HPP_

#include <autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp>
#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/motion_utils/vehicle/vehicle_state_checker.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

namespace autoware::pose_initializer
{
class StopCheckModule : public autoware::motion_utils::VehicleStopCheckerBase
{
public:
  StopCheckModule(autoware::agnocast_wrapper::Node * node, double buffer_duration);

private:
  using TwistWithCovarianceStamped = geometry_msgs::msg::TwistWithCovarianceStamped;
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  AUTOWARE_SUBSCRIPTION_PTR(TwistWithCovarianceStamped) sub_twist_;
  void on_twist(TwistWithCovarianceStamped::ConstSharedPtr msg);
};
}  // namespace autoware::pose_initializer

#endif  // STOP_CHECK_MODULE_HPP_
