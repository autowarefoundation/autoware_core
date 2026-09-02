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

// Mirrors VehicleStopChecker's own buffer length, which is private to that class.
constexpr double velocity_buffer_time_sec = 10.0;

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
: autoware::agnocast_wrapper::Node("routing", options),
  diagnostics_(this),
  vehicle_stop_checker_(this, velocity_buffer_time_sec)
{
  stop_check_duration_ = declare_parameter<double>("stop_check_duration");

  // Replaces VehicleStopChecker's own subscription so it follows this node's backend.
  sub_kinematic_state_ = create_subscription<nav_msgs::msg::Odometry>(
    "/localization/kinematic_state", rclcpp::QoS(1), [this](const nav_msgs::msg::Odometry & msg) {
      geometry_msgs::msg::TwistStamped current_velocity;
      current_velocity.header = msg.header;
      current_velocity.twist = msg.twist.twist;
      vehicle_stop_checker_.addTwist(current_velocity);
    });

  diagnostics_.setHardwareID("none");
  diagnostics_.add("state", this, &RoutingNode::diagnose_state);

  group_cli_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // AD API Interface
  pub_state_ = adaptor_.create_publisher<autoware::adapi_specs::routing::RouteState>();
  pub_route_ = adaptor_.create_publisher<autoware::adapi_specs::routing::Route>();
  srv_clear_route_ = adaptor_.create_service<autoware::adapi_specs::routing::ClearRoute>(
    this, &RoutingNode::on_clear_route);
  srv_set_route_ = adaptor_.create_service<autoware::adapi_specs::routing::SetRoute>(
    this, &RoutingNode::on_set_route);
  srv_set_route_points_ = adaptor_.create_service<autoware::adapi_specs::routing::SetRoutePoints>(
    this, &RoutingNode::on_set_route_points);
  srv_change_route_ = adaptor_.create_service<autoware::adapi_specs::routing::ChangeRoute>(
    this, &RoutingNode::on_change_route);
  srv_change_route_points_ =
    adaptor_.create_service<autoware::adapi_specs::routing::ChangeRoutePoints>(
      this, &RoutingNode::on_change_route_points);

  // Component Interface
  sub_state_ =
    adaptor_.create_subscription<autoware::component_interface_specs::planning::RouteState>(
      this, &RoutingNode::on_state);
  sub_route_ =
    adaptor_.create_subscription<autoware::component_interface_specs::planning::LaneletRoute>(
      this, &RoutingNode::on_route);
  cli_clear_route_ =
    adaptor_.create_client<autoware::component_interface_specs::planning::ClearRoute>(group_cli_);
  cli_set_waypoint_route_ =
    adaptor_.create_client<autoware::component_interface_specs::planning::SetWaypointRoute>(
      group_cli_);
  cli_set_lanelet_route_ =
    adaptor_.create_client<autoware::component_interface_specs::planning::SetLaneletRoute>(
      group_cli_);
  sub_operation_mode_ =
    adaptor_.create_subscription<autoware::component_interface_specs::system::OperationModeState>(
      this, &RoutingNode::on_operation_mode);

  cli_operation_mode_ =
    adaptor_.create_client<autoware::component_interface_specs::system::ChangeOperationMode>(
      group_cli_);

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
  // https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/interfaces/ad-api/list/api/routing/clear_route/
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
