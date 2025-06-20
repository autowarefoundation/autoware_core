// Copyright 2022 TIER IV, Inc.
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

#ifndef ROUTING_ADAPTOR_HPP_
#define ROUTING_ADAPTOR_HPP_

#include <autoware/adapi_specs/routing.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <string>

namespace autoware::adapi_adaptors
{

class RoutingAdaptor : public rclcpp::Node
{
public:
  explicit RoutingAdaptor(const rclcpp::NodeOptions & options);

private:
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using SetRoutePoints = autoware::adapi_specs::routing::SetRoutePoints;
  using ChangeRoutePoints = autoware::adapi_specs::routing::ChangeRoutePoints;
  using ClearRoute = autoware::adapi_specs::routing::ClearRoute;
  using RouteState = autoware::adapi_specs::routing::RouteState;
  rclcpp::Client<ChangeRoutePoints::Service>::SharedPtr cli_reroute_;
  rclcpp::Client<SetRoutePoints::Service>::SharedPtr cli_route_;
  rclcpp::Client<ClearRoute::Service>::SharedPtr cli_clear_;
  rclcpp::Subscription<RouteState::Message>::SharedPtr sub_state_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_fixed_goal_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_rough_goal_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_waypoint_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_reroute_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool calling_service_ = false;
  int request_timing_control_ = 0;
  SetRoutePoints::Service::Request::SharedPtr route_;
  RouteState::Message::_state_type state_;

  void on_timer();
  void on_fixed_goal(const PoseStamped::ConstSharedPtr pose);
  void on_rough_goal(const PoseStamped::ConstSharedPtr pose);
  void on_waypoint(const PoseStamped::ConstSharedPtr pose);
  void on_reroute(const PoseStamped::ConstSharedPtr pose);
};

}  // namespace autoware::adapi_adaptors

#endif  // ROUTING_ADAPTOR_HPP_
