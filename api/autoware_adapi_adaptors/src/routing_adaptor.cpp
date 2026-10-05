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

#include "routing_adaptor.hpp"

#include "request_state_machine.hpp"
#include "route_builder.hpp"

#include <memory>

namespace autoware::adapi_adaptors
{

RoutingAdaptor::RoutingAdaptor(const rclcpp::NodeOptions & options)
: autoware::agnocast_wrapper::Node("autoware_routing_adaptor", options)
{
  using std::placeholders::_1;

  sub_fixed_goal_ = create_subscription<PoseStamped>(
    "~/input/fixed_goal", 3, std::bind(&RoutingAdaptor::on_fixed_goal, this, _1));
  sub_rough_goal_ = create_subscription<PoseStamped>(
    "~/input/rough_goal", 3, std::bind(&RoutingAdaptor::on_rough_goal, this, _1));
  sub_reroute_ = create_subscription<PoseStamped>(
    "~/input/reroute", 3, std::bind(&RoutingAdaptor::on_reroute, this, _1));
  sub_waypoint_ = create_subscription<PoseStamped>(
    "~/input/waypoint", 10, std::bind(&RoutingAdaptor::on_waypoint, this, _1));

  cli_reroute_ = adaptor_.create_client<ChangeRoutePoints>();
  cli_route_ = adaptor_.create_client<SetRoutePoints>();
  cli_clear_ = adaptor_.create_client<ClearRoute>();

  sub_state_ = adaptor_.create_subscription<RouteState>(
    [this](const RouteState::Message::ConstSharedPtr msg) { state_ = msg->state; });

  const auto rate = rclcpp::Rate(5.0);
  timer_ = autoware::agnocast_wrapper::create_timer(
    this, get_clock(), rate.period(), std::bind(&RoutingAdaptor::on_timer, this));

  state_ = RouteState::Message::UNKNOWN;
  route_ = std::make_shared<SetRoutePoints::Service::Request>();
}

void RoutingAdaptor::on_timer()
{
  const auto decision = decide_routing_action(
    elapsed_count_from_last_request_, calling_service_, state_ == RouteState::Message::UNSET);
  elapsed_count_from_last_request_ = decision.elapsed_count_from_last_request;

  switch (decision.action) {
    case RoutingAction::None:
      break;
    case RoutingAction::CallClear: {
      const auto request = std::make_shared<ClearRoute::Service::Request>();
      calling_service_ = true;
      cli_clear_->async_send_request(
        request, [this](Cli<ClearRoute>::SharedFuture) { calling_service_ = false; });
      break;
    }
    case RoutingAction::CallRoute: {
      calling_service_ = true;
      cli_route_->async_send_request(
        route_, [this](Cli<SetRoutePoints>::SharedFuture) { calling_service_ = false; });
      break;
    }
  }
}

void RoutingAdaptor::on_fixed_goal(const PoseStamped::ConstSharedPtr pose)
{
  elapsed_count_from_last_request_ = 1;
  set_goal(*route_, *pose, false);
}

void RoutingAdaptor::on_rough_goal(const PoseStamped::ConstSharedPtr pose)
{
  elapsed_count_from_last_request_ = 1;
  set_goal(*route_, *pose, true);
}

void RoutingAdaptor::on_waypoint(const PoseStamped::ConstSharedPtr pose)
{
  if (!append_waypoint(*route_, *pose)) {
    RCLCPP_ERROR_STREAM(get_logger(), "The waypoint frame does not match the goal.");
    return;
  }
  elapsed_count_from_last_request_ = 1;
}

void RoutingAdaptor::on_reroute(const PoseStamped::ConstSharedPtr pose)
{
  const auto route = std::make_shared<SetRoutePoints::Service::Request>();
  route->header = pose->header;
  route->goal = pose->pose;
  cli_reroute_->async_send_request(route);
}

}  // namespace autoware::adapi_adaptors

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::adapi_adaptors::RoutingAdaptor)
