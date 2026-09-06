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

#ifndef ROUTING_HPP_
#define ROUTING_HPP_

#include <autoware/adapi_specs/routing.hpp>
#include <autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp>
#include <autoware/agnocast_wrapper/diagnostic_updater.hpp>
#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/component_interface_specs/planning.hpp>
#include <autoware/component_interface_specs/system.hpp>
#include <autoware/component_interface_utils/rclcpp.hpp>
#include <autoware/motion_utils/vehicle/vehicle_state_checker.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>

namespace autoware::default_adapi
{

class RoutingNode : public autoware::agnocast_wrapper::Node
{
public:
  explicit RoutingNode(const rclcpp::NodeOptions & options);

private:
  using OperationModeState = autoware::component_interface_specs::system::OperationModeState;
  using State = autoware::component_interface_specs::planning::RouteState;
  using Route = autoware::component_interface_specs::planning::LaneletRoute;

  // NodeAdaptor deduces its constructor argument separately from NodeT, so the node type has
  // to be named explicitly here and on every endpoint below.
  using NodeT = autoware::agnocast_wrapper::Node;
  autoware::component_interface_utils::NodeAdaptor<NodeT> adaptor_{this};
  rclcpp::CallbackGroup::SharedPtr group_cli_;

  // AD API Interface
  autoware::component_interface_utils::Publisher<
    autoware::adapi_specs::routing::RouteState, NodeT>::SharedPtr pub_state_;
  autoware::component_interface_utils::Publisher<
    autoware::adapi_specs::routing::Route, NodeT>::SharedPtr pub_route_;
  autoware::component_interface_utils::Service<
    autoware::adapi_specs::routing::SetRoutePoints, NodeT>::SharedPtr srv_set_route_points_;
  autoware::component_interface_utils::Service<
    autoware::adapi_specs::routing::SetRoute, NodeT>::SharedPtr srv_set_route_;
  autoware::component_interface_utils::Service<
    autoware::adapi_specs::routing::ChangeRoutePoints, NodeT>::SharedPtr srv_change_route_points_;
  autoware::component_interface_utils::Service<
    autoware::adapi_specs::routing::ChangeRoute, NodeT>::SharedPtr srv_change_route_;
  autoware::component_interface_utils::Service<
    autoware::adapi_specs::routing::ClearRoute, NodeT>::SharedPtr srv_clear_route_;

  // Component Interface
  autoware::component_interface_utils::Subscription<
    autoware::component_interface_specs::planning::RouteState, NodeT>::SharedPtr sub_state_;
  autoware::component_interface_utils::Subscription<
    autoware::component_interface_specs::planning::LaneletRoute, NodeT>::SharedPtr sub_route_;
  autoware::component_interface_utils::Client<
    autoware::component_interface_specs::planning::SetWaypointRoute, NodeT>::SharedPtr
    cli_set_waypoint_route_;
  autoware::component_interface_utils::Client<
    autoware::component_interface_specs::planning::SetLaneletRoute, NodeT>::SharedPtr
    cli_set_lanelet_route_;
  autoware::component_interface_utils::Client<
    autoware::component_interface_specs::planning::ClearRoute, NodeT>::SharedPtr cli_clear_route_;
  autoware::component_interface_utils::Subscription<
    autoware::component_interface_specs::system::OperationModeState, NodeT>::SharedPtr
    sub_operation_mode_;
  autoware::component_interface_utils::Client<
    autoware::component_interface_specs::system::ChangeOperationMode, NodeT>::SharedPtr
    cli_operation_mode_;

  void diagnose_state(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void change_stop_mode();
  void on_operation_mode(const OperationModeState::Message::ConstSharedPtr msg);
  void on_state(const State::Message::ConstSharedPtr msg);
  void on_route(const Route::Message::ConstSharedPtr msg);
  void on_clear_route(
    const autoware::adapi_specs::routing::ClearRoute::Service::Request::SharedPtr req,
    const autoware::adapi_specs::routing::ClearRoute::Service::Response::SharedPtr res);
  void on_set_route_points(
    const autoware::adapi_specs::routing::SetRoutePoints::Service::Request::SharedPtr req,
    const autoware::adapi_specs::routing::SetRoutePoints::Service::Response::SharedPtr res);
  void on_set_route(
    const autoware::adapi_specs::routing::SetRoute::Service::Request::SharedPtr req,
    const autoware::adapi_specs::routing::SetRoute::Service::Response::SharedPtr res);
  void on_change_route_points(
    const autoware::adapi_specs::routing::SetRoutePoints::Service::Request::SharedPtr req,
    const autoware::adapi_specs::routing::SetRoutePoints::Service::Response::SharedPtr res);
  void on_change_route(
    const autoware::adapi_specs::routing::SetRoute::Service::Request::SharedPtr req,
    const autoware::adapi_specs::routing::SetRoute::Service::Response::SharedPtr res);

  bool is_autoware_control_;
  bool is_auto_mode_;
  State::Message state_;
  autoware::agnocast_wrapper::diagnostic_updater::Updater diagnostics_;

  // Stop check for route clear. VehicleStopChecker owns an rclcpp odometry subscription, so use
  // its base -- already templated on the node type -- and keep the subscription here, where it
  // follows this node's backend.
  autoware::motion_utils::VehicleStopCheckerBase vehicle_stop_checker_;
  AUTOWARE_SUBSCRIPTION_PTR(nav_msgs::msg::Odometry) sub_kinematic_state_;
  double stop_check_duration_;
};

}  // namespace autoware::default_adapi

#endif  // ROUTING_HPP_
