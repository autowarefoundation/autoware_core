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
#include <autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp>
#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <string>

namespace autoware::adapi_adaptors
{

class RoutingAdaptor : public autoware::agnocast_wrapper::Node
{
public:
  explicit RoutingAdaptor(const rclcpp::NodeOptions & options);

private:
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using SetRoutePoints = autoware::adapi_specs::routing::SetRoutePoints;
  using ChangeRoutePoints = autoware::adapi_specs::routing::ChangeRoutePoints;
  using ClearRoute = autoware::adapi_specs::routing::ClearRoute;
  using RouteState = autoware::adapi_specs::routing::RouteState;
  // NodeAdaptor deduces its constructor argument separately from NodeT, so the node type has to
  // be named explicitly. The Cli alias keeps the endpoint declarations and the response-future
  // spellings in on_timer() in step.
  using NodeT = autoware::agnocast_wrapper::Node;
  template <class SpecT>
  using Cli = autoware::component_interface_utils::Client<SpecT, NodeT>;

  autoware::component_interface_utils::NodeAdaptor<NodeT> adaptor_{this};
  Cli<ChangeRoutePoints>::SharedPtr cli_reroute_;
  Cli<SetRoutePoints>::SharedPtr cli_route_;
  Cli<ClearRoute>::SharedPtr cli_clear_;
  autoware::component_interface_utils::Subscription<RouteState, NodeT>::SharedPtr sub_state_;
  AUTOWARE_SUBSCRIPTION_PTR(PoseStamped) sub_fixed_goal_;
  AUTOWARE_SUBSCRIPTION_PTR(PoseStamped) sub_rough_goal_;
  AUTOWARE_SUBSCRIPTION_PTR(PoseStamped) sub_waypoint_;
  AUTOWARE_SUBSCRIPTION_PTR(PoseStamped) sub_reroute_;
  AUTOWARE_TIMER_PTR timer_;

  bool calling_service_ = false;
  int elapsed_count_from_last_request_ = 0;
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
