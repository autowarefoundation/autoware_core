// Copyright 2019 Autoware Foundation
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

#include "mission_planner.hpp"

#include "reroute_safety.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>

#include <autoware_adapi_v1_msgs/srv/set_route.hpp>
#include <autoware_adapi_v1_msgs/srv/set_route_points.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace autoware::mission_planner
{

namespace
{
void set_fail_response(
  SetLaneletRoute::Response & res, const uint16_t code, const std::string & message)
{
  res.status.success = false;
  res.status.code = code;
  res.status.message = message;
}

void set_fail_response(
  SetWaypointRoute::Response & res, const uint16_t code, const std::string & message)
{
  res.status.success = false;
  res.status.code = code;
  res.status.message = message;
}

void set_success_response(
  ClearRoute::Response & res, const uint16_t code, const std::string & message)
{
  res.status.success = true;
  res.status.code = code;
  res.status.message = message;
}

Pose transform_pose(const Pose & pose, const geometry_msgs::msg::TransformStamped & transform)
{
  Pose result;
  tf2::doTransform(pose, result, transform);
  return result;
}

}  // namespace

MissionPlanner::MissionPlanner(
  const MissionPlannerConfig & config, ChangeStateCallback on_change_state)
: arrival_checker_(config.arrival_checker_threshold),
  planner_(
    std::make_shared<lanelet2::DefaultPlanner>(
      config.default_planner_parameters, config.vehicle_info)),
  map_frame_(config.map_frame),
  odometry_(nullptr),
  map_ptr_(nullptr),
  on_change_state_(std::move(on_change_state)),
  is_mission_planner_ready_(false),
  reroute_time_threshold_(config.reroute_time_threshold),
  minimum_reroute_length_(config.minimum_reroute_length),
  allow_reroute_in_autonomous_mode_(config.allow_reroute_in_autonomous_mode)
{
}

InitializationCheckResult MissionPlanner::check_initialization()
{
  if (!planner_->ready()) {
    return {false, std::string("waiting lanelet map... Route API is not ready.")};
  }
  if (!odometry_) {
    return {false, std::string("waiting odometry... Route API is not ready.")};
  }

  // All data is ready. Now API is available.
  is_mission_planner_ready_ = true;
  change_state(RouteState::UNSET);
  return {true, std::nullopt};
}

void MissionPlanner::on_odometry(const Odometry::ConstSharedPtr msg)
{
  odometry_ = msg;
  arrival_checker_.update(*msg);

  // NOTE: Do not check in the other states as goal may change.
  if (state_ == RouteState::SET) {
    if (arrival_checker_.is_arrived()) {
      change_state(RouteState::ARRIVED);
    }
  }
}

void MissionPlanner::on_operation_mode_state(const OperationModeState::ConstSharedPtr msg)
{
  operation_mode_state_ = msg;
}

void MissionPlanner::on_map(const LaneletMapBin::ConstSharedPtr msg)
{
  map_ptr_ = msg;
  lanelet_map_ptr_ = autoware::experimental::lanelet2_utils::remove_const(
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*map_ptr_));
  planner_->set_map(*map_ptr_);
}

void MissionPlanner::change_state(RouteState::_state_type state)
{
  state_ = state;
  if (on_change_state_) {
    on_change_state_(state);
  }
}

ClearRoute::Response MissionPlanner::clear_route()
{
  ClearRoute::Response res;
  if (!is_mission_planner_ready_) {
    using ResponseCode = autoware_adapi_v1_msgs::msg::ResponseStatus;
    set_success_response(res, ResponseCode::NO_EFFECT, "The mission planner is not ready.");
    return res;
  }

  process_clear_route();
  change_state(RouteState::UNSET);
  res.status.success = true;
  return res;
}

SetLaneletRouteResult MissionPlanner::set_lanelet_route(
  const SetLaneletRoute::Request & req,
  const std::optional<geometry_msgs::msg::TransformStamped> & transform_to_map)
{
  using ResponseCode = autoware_adapi_v1_msgs::srv::SetRoute::Response;
  const auto is_reroute = state_ == RouteState::SET;

  SetLaneletRouteResult result;
  auto & res = result.response;

  if (state_ != RouteState::UNSET && state_ != RouteState::SET) {
    set_fail_response(
      res, ResponseCode::ERROR_INVALID_STATE, "The route cannot be set in the current state.");
    return result;
  }
  if (!is_mission_planner_ready_) {
    set_fail_response(
      res, ResponseCode::ERROR_PLANNER_UNREADY, "The mission planner is not ready.");
    return result;
  }
  if (is_reroute && !operation_mode_state_) {
    set_fail_response(
      res, ResponseCode::ERROR_PLANNER_UNREADY, "Operation mode state is not received.");
    return result;
  }

  const bool is_autonomous_driving =
    operation_mode_state_ ? operation_mode_state_->mode == OperationModeState::AUTONOMOUS &&
                              operation_mode_state_->is_autoware_control_enabled
                          : false;

  if (is_reroute && !allow_reroute_in_autonomous_mode_ && is_autonomous_driving) {
    set_fail_response(
      res, ResponseCode::ERROR_INVALID_STATE, "Reroute is not allowed in autonomous mode.");
    return result;
  }

  change_state(is_reroute ? RouteState::REROUTING : RouteState::ROUTING);
  if (!transform_to_map) {
    set_fail_response(
      res, autoware_common_msgs::msg::ResponseStatus::TRANSFORM_ERROR,
      "Failed to transform pose to map frame.");
    return result;
  }

  const auto route = create_lanelet_route(req, *transform_to_map);

  if (route.segments.empty()) {
    cancel_route();
    change_state(is_reroute ? RouteState::SET : RouteState::UNSET);
    set_fail_response(res, ResponseCode::ERROR_PLANNER_FAILED, "The planned route is empty.");
    return result;
  }

  if (is_reroute && is_autonomous_driving) {
    const auto reroute_safety_result = check_reroute_safety(*current_route_, route);
    if (!reroute_safety_result.is_safe) {
      cancel_route();
      change_state(RouteState::SET);
      set_fail_response(
        res, ResponseCode::ERROR_REROUTE_FAILED, "New route is not safe. Reroute failed.");
      result.error_message = reroute_safety_result.reason;
      return result;
    }
  }

  change_route(route);
  change_state(RouteState::SET);
  res.status.success = true;

  result.route = route;
  result.route_marker = planner_->visualize(route);
  result.initial_pose = odometry_->pose.pose;
  return result;
}

SetWaypointRouteResult MissionPlanner::set_waypoint_route(
  const SetWaypointRoute::Request & req,
  const std::optional<geometry_msgs::msg::TransformStamped> & transform_to_map)
{
  using ResponseCode = autoware_adapi_v1_msgs::srv::SetRoutePoints::Response;
  const auto is_reroute = state_ == RouteState::SET;

  SetWaypointRouteResult result;
  auto & res = result.response;

  if (state_ != RouteState::UNSET && state_ != RouteState::SET) {
    set_fail_response(
      res, ResponseCode::ERROR_INVALID_STATE, "The route cannot be set in the current state.");
    return result;
  }
  if (!is_mission_planner_ready_) {
    set_fail_response(
      res, ResponseCode::ERROR_PLANNER_UNREADY, "The mission planner is not ready.");
    return result;
  }
  if (is_reroute && !operation_mode_state_) {
    set_fail_response(
      res, ResponseCode::ERROR_PLANNER_UNREADY, "Operation mode state is not received.");
    return result;
  }

  const bool is_autonomous_driving =
    operation_mode_state_ ? operation_mode_state_->mode == OperationModeState::AUTONOMOUS &&
                              operation_mode_state_->is_autoware_control_enabled
                          : false;

  change_state(is_reroute ? RouteState::REROUTING : RouteState::ROUTING);
  if (!transform_to_map) {
    set_fail_response(
      res, autoware_common_msgs::msg::ResponseStatus::TRANSFORM_ERROR,
      "Failed to transform pose to map frame.");
    return result;
  }

  const auto waypoint_plan_result = create_waypoint_route(req, *transform_to_map);
  const auto & route = waypoint_plan_result.route;

  if (waypoint_plan_result.goal_footprint) {
    result.goal_footprint_marker =
      lanelet2::DefaultPlanner::visualize_debug_footprint(*waypoint_plan_result.goal_footprint);
  }
  result.planner_warning_message = waypoint_plan_result.warning_message;

  if (route.segments.empty()) {
    cancel_route();
    change_state(is_reroute ? RouteState::SET : RouteState::UNSET);
    set_fail_response(res, ResponseCode::ERROR_PLANNER_FAILED, "The planned route is empty.");
    return result;
  }

  if (is_reroute && is_autonomous_driving) {
    const auto reroute_safety_result = check_reroute_safety(*current_route_, route);
    if (!reroute_safety_result.is_safe) {
      cancel_route();
      change_state(RouteState::SET);
      set_fail_response(
        res, ResponseCode::ERROR_REROUTE_FAILED, "New route is not safe. Reroute failed.");
      result.error_message = reroute_safety_result.reason;
      return result;
    }
  }

  change_route(route);
  change_state(RouteState::SET);
  res.status.success = true;

  result.route = route;
  result.route_marker = planner_->visualize(route);
  result.initial_pose = odometry_->pose.pose;
  return result;
}

void MissionPlanner::process_clear_route()
{
  current_route_ = nullptr;
  planner_->clearRoute();
  arrival_checker_.clear_goal();

  // TODO(Takagi, Isamu): publish an empty route here
  // pub_route_->publish();
  // pub_marker_->publish();
}

void MissionPlanner::change_route(const LaneletRoute & route)
{
  PoseWithUuidStamped goal;
  goal.header = route.header;
  goal.pose = route.goal_pose;
  goal.uuid = route.uuid;

  current_route_ = std::make_shared<LaneletRoute>(route);
  planner_->updateRoute(route);
  arrival_checker_.set_goal(goal);
}

void MissionPlanner::cancel_route()
{
  // Restore planner state that changes with create_route function.
  if (current_route_) {
    planner_->updateRoute(*current_route_);
  }
}

LaneletRoute MissionPlanner::create_lanelet_route(
  const SetLaneletRoute::Request & req,
  const geometry_msgs::msg::TransformStamped & transform_to_map)
{
  LaneletRoute route;
  route.header.stamp = req.header.stamp;
  route.header.frame_id = map_frame_;
  route.start_pose = odometry_->pose.pose;
  route.goal_pose = transform_pose(req.goal_pose, transform_to_map);
  route.segments = req.segments;
  route.uuid = req.uuid;
  route.allow_modification = req.allow_modification;
  return route;
}

lanelet2::DefaultPlanner::PlanResult MissionPlanner::create_waypoint_route(
  const SetWaypointRoute::Request & req,
  const geometry_msgs::msg::TransformStamped & transform_to_map)
{
  lanelet2::DefaultPlanner::RoutePoints points;
  points.push_back(odometry_->pose.pose);
  for (const auto & waypoint : req.waypoints) {
    points.push_back(transform_pose(waypoint, transform_to_map));
  }
  points.push_back(transform_pose(req.goal_pose, transform_to_map));

  auto plan_result = planner_->plan(points);

  plan_result.route.header.stamp = req.header.stamp;
  plan_result.route.header.frame_id = map_frame_;
  plan_result.route.uuid = req.uuid;
  plan_result.route.allow_modification = req.allow_modification;

  return plan_result;
}

RerouteSafetyResult MissionPlanner::check_reroute_safety(
  const LaneletRoute & original_route, const LaneletRoute & target_route)
{
  // The pure check_reroute_safety free function validates the routes and the map, but this class
  // owns the odometry, so guard it here to keep the original observable behavior (same failure
  // reason and early return when odometry or map is not yet available).
  if (!map_ptr_ || !lanelet_map_ptr_ || !odometry_) {
    return {false, "Check reroute safety failed. Route, map or odometry is not set."};
  }

  const auto current_velocity = odometry_->twist.twist.linear.x;

  return autoware::mission_planner::check_reroute_safety(
    original_route, target_route, lanelet_map_ptr_, current_velocity, reroute_time_threshold_,
    minimum_reroute_length_);
}
}  // namespace autoware::mission_planner
