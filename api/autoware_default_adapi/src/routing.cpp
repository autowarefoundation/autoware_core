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

#include "routing.hpp"

#include "utils/route_conversion.hpp"

#include <memory>

namespace
{

using autoware_adapi_v1_msgs::msg::ResponseStatus;

template <class InterfaceT>
ResponseStatus route_already_set()
{
  ResponseStatus status;
  status.success = false;
  status.code = InterfaceT::Service::Response::ERROR_INVALID_STATE;
  status.message = "The route is already set.";
  return status;
}

template <class InterfaceT>
ResponseStatus route_is_not_set()
{
  ResponseStatus status;
  status.success = false;
  status.code = InterfaceT::Service::Response::ERROR_INVALID_STATE;
  status.message = "The route is not set yet.";
  return status;
}

}  // namespace

namespace autoware::default_adapi
{

RoutingNode::RoutingNode(const rclcpp::NodeOptions & options)
: Node("routing", options), diagnostics_(this), vehicle_stop_checker_(this)
{
  stop_check_duration_ = declare_parameter<double>("stop_check_duration");

  diagnostics_.setHardwareID("none");
  diagnostics_.add("state", this, &RoutingNode::diagnose_state);

  group_cli_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // AD API Interface
  pub_state_ = create_publisher<autoware::adapi_specs::routing::RouteState::Message>(
    autoware::adapi_specs::routing::RouteState::name,
    autoware::component_interface_specs::get_qos<autoware::adapi_specs::routing::RouteState>());
  pub_route_ = create_publisher<autoware::adapi_specs::routing::Route::Message>(
    autoware::adapi_specs::routing::Route::name,
    autoware::component_interface_specs::get_qos<autoware::adapi_specs::routing::Route>());
  srv_clear_route_ = create_service<autoware::adapi_specs::routing::ClearRoute::Service>(
    autoware::adapi_specs::routing::ClearRoute::name,
    std::bind(&RoutingNode::on_clear_route, this, std::placeholders::_1, std::placeholders::_2));
  srv_set_route_ = create_service<autoware::adapi_specs::routing::SetRoute::Service>(
    autoware::adapi_specs::routing::SetRoute::name,
    std::bind(&RoutingNode::on_set_route, this, std::placeholders::_1, std::placeholders::_2));
  srv_set_route_points_ = create_service<autoware::adapi_specs::routing::SetRoutePoints::Service>(
    autoware::adapi_specs::routing::SetRoutePoints::name,
    std::bind(
      &RoutingNode::on_set_route_points, this, std::placeholders::_1, std::placeholders::_2));
  srv_change_route_ = create_service<autoware::adapi_specs::routing::ChangeRoute::Service>(
    autoware::adapi_specs::routing::ChangeRoute::name,
    std::bind(&RoutingNode::on_change_route, this, std::placeholders::_1, std::placeholders::_2));
  srv_change_route_points_ =
    create_service<autoware::adapi_specs::routing::ChangeRoutePoints::Service>(
      autoware::adapi_specs::routing::ChangeRoutePoints::name,
      std::bind(
        &RoutingNode::on_change_route_points, this, std::placeholders::_1, std::placeholders::_2));

  // Component Interface
  sub_state_ =
    create_subscription<autoware::component_interface_specs::planning::RouteState::Message>(
      autoware::component_interface_specs::planning::RouteState::name,
      autoware::component_interface_specs::get_qos<
        autoware::component_interface_specs::planning::RouteState>(),
      std::bind(&RoutingNode::on_state, this, std::placeholders::_1));
  sub_route_ =
    create_subscription<autoware::component_interface_specs::planning::LaneletRoute::Message>(
      autoware::component_interface_specs::planning::LaneletRoute::name,
      autoware::component_interface_specs::get_qos<
        autoware::component_interface_specs::planning::LaneletRoute>(),
      std::bind(&RoutingNode::on_route, this, std::placeholders::_1));
  cli_clear_route_ =
    create_client<autoware::component_interface_specs::planning::ClearRoute::Service>(
      autoware::component_interface_specs::planning::ClearRoute::name,
      rmw_qos_profile_services_default, group_cli_);
  cli_set_waypoint_route_ =
    create_client<autoware::component_interface_specs::planning::SetWaypointRoute::Service>(
      autoware::component_interface_specs::planning::SetWaypointRoute::name,
      rmw_qos_profile_services_default, group_cli_);
  cli_set_lanelet_route_ =
    create_client<autoware::component_interface_specs::planning::SetLaneletRoute::Service>(
      autoware::component_interface_specs::planning::SetLaneletRoute::name,
      rmw_qos_profile_services_default, group_cli_);
  sub_operation_mode_ =
    create_subscription<autoware::component_interface_specs::system::OperationModeState::Message>(
      autoware::component_interface_specs::system::OperationModeState::name,
      autoware::component_interface_specs::get_qos<
        autoware::component_interface_specs::system::OperationModeState>(),
      std::bind(&RoutingNode::on_operation_mode, this, std::placeholders::_1));

  cli_operation_mode_ =
    create_client<autoware::component_interface_specs::system::ChangeOperationMode::Service>(
      autoware::component_interface_specs::system::ChangeOperationMode::name,
      rmw_qos_profile_services_default, group_cli_);

  is_autoware_control_ = false;
  is_auto_mode_ = false;
  state_.state = State::Message::UNKNOWN;
}

void RoutingNode::diagnose_state(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  const auto message = std::to_string(state_.state);

  switch (state_.state) {
    case State::Message::SET:
    case State::Message::REROUTING:
    case State::Message::ARRIVED:
      stat.summary(DiagnosticStatus::OK, message);
      break;
    case State::Message::UNKNOWN:
    case State::Message::INITIALIZING:
    case State::Message::UNSET:
    case State::Message::ROUTING:
    case State::Message::ABORTED:
    case State::Message::INTERRUPTED:
    default:
      stat.summary(DiagnosticStatus::ERROR, message);
      break;
  }
}

void RoutingNode::change_stop_mode()
{
  using OperationModeRequest =
    autoware::component_interface_specs::system::ChangeOperationMode::Service::Request;
  if (is_auto_mode_) {
    if (!cli_operation_mode_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "Operation mode service is not ready");
      return;
    }

    const auto req = std::make_shared<OperationModeRequest>();
    req->mode = OperationModeRequest::STOP;
    cli_operation_mode_->async_send_request(req);
  }
}

void RoutingNode::on_operation_mode(const OperationModeState::Message::ConstSharedPtr msg)
{
  is_autoware_control_ = msg->is_autoware_control_enabled;
  is_auto_mode_ = msg->mode == OperationModeState::Message::AUTONOMOUS;
}

void RoutingNode::on_state(const State::Message::ConstSharedPtr msg)
{
  // TODO(Takagi, Isamu): Add adapi initializing state.
  // Represent initializing state by not publishing the topic for now.
  if (msg->state == State::Message::INITIALIZING) {
    return;
  }

  state_ = *msg;
  pub_state_->publish(conversion::convert_state(*msg));

  // Change operation mode to stop when the vehicle arrives.
  if (msg->state == State::Message::ARRIVED) {
    change_stop_mode();
  }

  // TODO(Takagi, Isamu): Remove when the mission planner supports an empty route.
  if (msg->state == State::Message::UNSET) {
    pub_route_->publish(conversion::create_empty_route(msg->stamp));
  }
}

void RoutingNode::on_route(const Route::Message::ConstSharedPtr msg)
{
  pub_route_->publish(conversion::convert_route(*msg));
}

void RoutingNode::on_clear_route(
  const autoware::adapi_specs::routing::ClearRoute::Service::Request::SharedPtr req,
  const autoware::adapi_specs::routing::ClearRoute::Service::Response::SharedPtr res)
{
  // For safety, do not clear the route while it is in use.
  // https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/list/api/routing/clear_route/
  if (is_auto_mode_ && is_autoware_control_) {
    if (!vehicle_stop_checker_.isVehicleStopped(stop_check_duration_)) {
      res->status.success = false;
      res->status.code = ResponseStatus::UNKNOWN;
      res->status.message = "The route cannot be cleared while it is in use.";
      return;
    }
  }

  if (!cli_clear_route_->service_is_ready()) {
    RCLCPP_ERROR(get_logger(), "Clear route service is not ready");
    return;
  }
  res->status = conversion::convert_call(cli_clear_route_, req);
}

void RoutingNode::on_set_route_points(
  const autoware::adapi_specs::routing::SetRoutePoints::Service::Request::SharedPtr req,
  const autoware::adapi_specs::routing::SetRoutePoints::Service::Response::SharedPtr res)
{
  if (state_.state != State::Message::UNSET) {
    res->status = route_already_set<autoware::adapi_specs::routing::SetRoutePoints>();
    return;
  }
  if (!cli_set_waypoint_route_->service_is_ready()) {
    RCLCPP_ERROR(get_logger(), "Set waypoint route service is not ready");
    return;
  }
  res->status = conversion::convert_call(cli_set_waypoint_route_, req);
}

void RoutingNode::on_set_route(
  const autoware::adapi_specs::routing::SetRoute::Service::Request::SharedPtr req,
  const autoware::adapi_specs::routing::SetRoute::Service::Response::SharedPtr res)
{
  if (state_.state != State::Message::UNSET) {
    res->status = route_already_set<autoware::adapi_specs::routing::SetRoute>();
    return;
  }
  if (!cli_set_lanelet_route_->service_is_ready()) {
    RCLCPP_ERROR(get_logger(), "Set lanelet route service is not ready");
    return;
  }
  res->status = conversion::convert_call(cli_set_lanelet_route_, req);
}

void RoutingNode::on_change_route_points(
  const autoware::adapi_specs::routing::SetRoutePoints::Service::Request::SharedPtr req,
  const autoware::adapi_specs::routing::SetRoutePoints::Service::Response::SharedPtr res)
{
  if (state_.state != State::Message::SET) {
    res->status = route_is_not_set<autoware::adapi_specs::routing::SetRoutePoints>();
    return;
  }
  if (!cli_set_waypoint_route_->service_is_ready()) {
    RCLCPP_ERROR(get_logger(), "Set waypoint route service is not ready");
    return;
  }
  res->status = conversion::convert_call(cli_set_waypoint_route_, req);
}

void RoutingNode::on_change_route(
  const autoware::adapi_specs::routing::SetRoute::Service::Request::SharedPtr req,
  const autoware::adapi_specs::routing::SetRoute::Service::Response::SharedPtr res)
{
  if (state_.state != State::Message::SET) {
    res->status = route_is_not_set<autoware::adapi_specs::routing::SetRoute>();
    return;
  }
  if (!cli_set_lanelet_route_->service_is_ready()) {
    RCLCPP_ERROR(get_logger(), "Set lanelet route service is not ready");
    return;
  }
  res->status = conversion::convert_call(cli_set_lanelet_route_, req);
}

}  // namespace autoware::default_adapi

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::default_adapi::RoutingNode)
