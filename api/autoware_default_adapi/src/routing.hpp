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
#include <autoware/component_interface_specs/planning.hpp>
#include <autoware/component_interface_specs/system.hpp>
#include <autoware/motion_utils/vehicle/vehicle_state_checker.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

// This file should be included after messages.
#include "utils/types.hpp"

namespace autoware::default_adapi
{

class RoutingNode : public rclcpp::Node
{
public:
  explicit RoutingNode(const rclcpp::NodeOptions & options);

private:
  using OperationModeState = autoware::component_interface_specs::system::OperationModeState;
  using State = autoware::component_interface_specs::planning::RouteState;
  using Route = autoware::component_interface_specs::planning::LaneletRoute;

  rclcpp::CallbackGroup::SharedPtr group_cli_;

  // AD API Interface
  Pub<autoware::adapi_specs::routing::RouteState::Message> pub_state_;
  Pub<autoware::adapi_specs::routing::Route::Message> pub_route_;
  Srv<autoware::adapi_specs::routing::SetRoutePoints::Service> srv_set_route_points_;
  Srv<autoware::adapi_specs::routing::SetRoute::Service> srv_set_route_;
  Srv<autoware::adapi_specs::routing::ChangeRoutePoints::Service> srv_change_route_points_;
  Srv<autoware::adapi_specs::routing::ChangeRoute::Service> srv_change_route_;
  Srv<autoware::adapi_specs::routing::ClearRoute::Service> srv_clear_route_;

  // Component Interface
  Sub<autoware::component_interface_specs::planning::RouteState::Message> sub_state_;
  Sub<autoware::component_interface_specs::planning::LaneletRoute::Message> sub_route_;
  Cli<autoware::component_interface_specs::planning::SetWaypointRoute::Service>
    cli_set_waypoint_route_;
  Cli<autoware::component_interface_specs::planning::SetLaneletRoute::Service>
    cli_set_lanelet_route_;
  Cli<autoware::component_interface_specs::planning::ClearRoute::Service> cli_clear_route_;
  Sub<autoware::component_interface_specs::system::OperationModeState::Message> sub_operation_mode_;

  Cli<autoware::component_interface_specs::system::ChangeOperationMode::Service>
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
  diagnostic_updater::Updater diagnostics_;

  // Stop check for route clear.
  autoware::motion_utils::VehicleStopChecker vehicle_stop_checker_;
  double stop_check_duration_;
};

}  // namespace autoware::default_adapi

#endif  // ROUTING_HPP_
