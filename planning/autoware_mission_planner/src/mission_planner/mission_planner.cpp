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

#include <autoware_common_msgs/msg/response_status.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::mission_planner
{

MissionPlanner::MissionPlanner(const rclcpp::NodeOptions & options)
: Node("mission_planner", options),
  arrival_checker_(this),
  plugin_loader_("autoware_mission_planner", "autoware::mission_planner::PlannerPlugin"),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  odometry_(nullptr),
  map_ptr_(nullptr)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // cppcheck-suppress useInitializationList
  map_frame_ = declare_parameter<std::string>("map_frame");
  reroute_time_threshold_ = declare_parameter<double>("reroute_time_threshold");
  minimum_reroute_length_ = declare_parameter<double>("minimum_reroute_length");
  allow_reroute_in_autonomous_mode_ = declare_parameter<bool>("allow_reroute_in_autonomous_mode");

  planner_ =
    plugin_loader_.createSharedInstance("autoware::mission_planner::lanelet2::DefaultPlanner");
  planner_->initialize(this);

  const auto durable_qos = rclcpp::QoS(1).transient_local();
  sub_odometry_ = create_subscription<Odometry>(
    "~/input/odometry", rclcpp::QoS(1), std::bind(&MissionPlanner::on_odometry, this, _1));
  sub_operation_mode_state_ = create_subscription<OperationModeState>(
    "~/input/operation_mode_state", rclcpp::QoS(1).transient_local(),
    std::bind(&MissionPlanner::on_operation_mode_state, this, _1));
  sub_vector_map_ = create_subscription<LaneletMapBin>(
    "~/input/vector_map", durable_qos, std::bind(&MissionPlanner::on_map, this, _1));
  pub_marker_ = create_publisher<MarkerArray>("~/debug/route_marker", durable_qos);

  // NOTE: The route interface should be mutually exclusive by callback group.
  srv_clear_route = create_service<ClearRouteSpecs::Service>(
    "~/clear_route", std::bind(&MissionPlanner::on_clear_route, this, _1, _2));
  srv_set_lanelet_route = create_service<SetLaneletRouteSpecs::Service>(
    "~/set_lanelet_route", std::bind(&MissionPlanner::on_set_lanelet_route, this, _1, _2));
  srv_set_waypoint_route = create_service<SetWaypointRouteSpecs::Service>(
    "~/set_waypoint_route", std::bind(&MissionPlanner::on_set_waypoint_route, this, _1, _2));
  pub_route_ = create_publisher<LaneletRouteSpecs::Message>(
    "~/route", autoware::component_interface_specs::get_qos<LaneletRouteSpecs>());
  pub_state_ = create_publisher<RouteStateSpecs::Message>(
    "~/state", autoware::component_interface_specs::get_qos<RouteStateSpecs>());

  // Route state will be published when the node gets ready for route api after initialization,
  // otherwise the mission planner rejects the request for the API.
  const auto period = rclcpp::Rate(10).period();
  data_check_timer_ = create_wall_timer(period, [this] { check_initialization(); });
  is_mission_planner_ready_ = false;

  logger_configure_ = std::make_unique<autoware_utils_logging::LoggerLevelConfigure>(this);
  pub_processing_time_ = this->create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "~/debug/processing_time_ms", 1);
}

void MissionPlanner::publish_processing_time(
  autoware_utils_system::StopWatch<std::chrono::milliseconds> stop_watch)
{
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch.toc();
  pub_processing_time_->publish(processing_time_msg);
}

void MissionPlanner::publish_pose_log(const Pose & pose, const std::string & pose_type)
{
  const auto & p = pose.position;
  RCLCPP_INFO(
    this->get_logger(), "%s pose - x: %f, y: %f, z: %f", pose_type.c_str(), p.x, p.y, p.z);
  const auto & quaternion = pose.orientation;
  RCLCPP_INFO(
    this->get_logger(), "%s orientation - qx: %f, qy: %f, qz: %f, qw: %f", pose_type.c_str(),
    quaternion.x, quaternion.y, quaternion.z, quaternion.w);
}

void MissionPlanner::check_initialization()
{
  auto logger = get_logger();
  auto clock = *get_clock();

  if (!planner_->ready()) {
    RCLCPP_INFO_THROTTLE(logger, clock, 5000, "waiting lanelet map... Route API is not ready.");
    return;
  }
  if (!odometry_) {
    RCLCPP_INFO_THROTTLE(logger, clock, 5000, "waiting odometry... Route API is not ready.");
    return;
  }

  // All data is ready. Now API is available.
  is_mission_planner_ready_ = true;
  RCLCPP_DEBUG(logger, "Route API is ready.");
  change_state(RouteState::UNSET);

  // Stop timer callback.
  data_check_timer_->cancel();
  data_check_timer_ = nullptr;
}

void MissionPlanner::on_odometry(const Odometry::ConstSharedPtr msg)
{
  odometry_ = msg;

  // NOTE: Do not check in the other states as goal may change.
  if (state_.state == RouteState::SET) {
    PoseStamped pose;
    pose.header = odometry_->header;
    pose.pose = odometry_->pose.pose;
    if (arrival_checker_.is_arrived(pose)) {
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
}

Pose MissionPlanner::transform_pose(const Pose & pose, const Header & header)
{
  const auto transform =
    tf_buffer_.lookupTransform(map_frame_, header.frame_id, tf2::TimePointZero);
  geometry_msgs::msg::Pose result;
  tf2::doTransform(pose, result, transform);
  return result;
}

void MissionPlanner::change_state(RouteState::_state_type state)
{
  state_.stamp = now();
  state_.state = state;
  pub_state_->publish(state_);
}

void MissionPlanner::on_clear_route(
  const ClearRoute::Request::SharedPtr, const ClearRoute::Response::SharedPtr res)
{
  ScopedProcessingTimePublisher processing_time_publisher(*this);

  if (!is_mission_planner_ready_) {
    using ResponseCode = autoware_adapi_v1_msgs::msg::ResponseStatus;
    res->status.success = true;
    res->status.code = ResponseCode::NO_EFFECT;
    res->status.message = "The mission planner is not ready.";
    return;
  }

  change_route();
  change_state(RouteState::UNSET);
  res->status.success = true;
}

void MissionPlanner::on_set_lanelet_route(
  const SetLaneletRoute::Request::SharedPtr req, const SetLaneletRoute::Response::SharedPtr res)
{
  ScopedProcessingTimePublisher processing_time_publisher(*this);
  using ResponseCode = autoware_adapi_v1_msgs::srv::SetRoute::Response;
  const auto is_reroute = state_.state == RouteState::SET;

  if (state_.state != RouteState::UNSET && state_.state != RouteState::SET) {
    res->status.success = false;
    res->status.code = ResponseCode::ERROR_INVALID_STATE;
    res->status.message = "The route cannot be set in the current state.";
    return;
  }
  if (!is_mission_planner_ready_) {
    res->status.success = false;
    res->status.code = ResponseCode::ERROR_PLANNER_UNREADY;
    res->status.message = "The mission planner is not ready.";
    return;
  }
  if (is_reroute && !operation_mode_state_) {
    res->status.success = false;
    res->status.code = ResponseCode::ERROR_PLANNER_UNREADY;
    res->status.message = "Operation mode state is not received.";
    return;
  }

  const bool is_autonomous_driving =
    operation_mode_state_ ? operation_mode_state_->mode == OperationModeState::AUTONOMOUS &&
                              operation_mode_state_->is_autoware_control_enabled
                          : false;

  if (is_reroute && !allow_reroute_in_autonomous_mode_ && is_autonomous_driving) {
    res->status.success = false;
    res->status.code = ResponseCode::ERROR_INVALID_STATE;
    res->status.message = "Reroute is not allowed in autonomous mode.";
    return;
  }

  change_state(is_reroute ? RouteState::REROUTING : RouteState::ROUTING);
  LaneletRoute route;
  try {
    route = create_route(*req);
  } catch (const tf2::TransformException & error) {
    res->status.success = false;
    res->status.code = autoware_common_msgs::msg::ResponseStatus::TRANSFORM_ERROR;
    res->status.message = error.what();
    return;
  }

  if (route.segments.empty()) {
    cancel_route();
    change_state(is_reroute ? RouteState::SET : RouteState::UNSET);
    res->status.success = false;
    res->status.code = ResponseCode::ERROR_PLANNER_FAILED;
    res->status.message = "The planned route is empty.";
    return;
  }

  if (is_reroute && is_autonomous_driving && !check_reroute_safety(*current_route_, route)) {
    cancel_route();
    change_state(RouteState::SET);
    res->status.success = false;
    res->status.code = ResponseCode::ERROR_REROUTE_FAILED;
    res->status.message = "New route is not safe. Reroute failed.";
    return;
  }

  change_route(route);
  change_state(RouteState::SET);
  res->status.success = true;

  publish_pose_log(odometry_->pose.pose, "initial");
  publish_pose_log(req->goal_pose, "goal");
}

void MissionPlanner::on_set_waypoint_route(
  const SetWaypointRoute::Request::SharedPtr req, const SetWaypointRoute::Response::SharedPtr res)
{
  ScopedProcessingTimePublisher processing_time_publisher(*this);
  using ResponseCode = autoware_adapi_v1_msgs::srv::SetRoutePoints::Response;
  const auto is_reroute = state_.state == RouteState::SET;

  if (state_.state != RouteState::UNSET && state_.state != RouteState::SET) {
    res->status.success = false;
    res->status.code = ResponseCode::ERROR_INVALID_STATE;
    res->status.message = "The route cannot be set in the current state.";
    return;
  }
  if (!is_mission_planner_ready_) {
    res->status.success = false;
    res->status.code = ResponseCode::ERROR_PLANNER_UNREADY;
    res->status.message = "The mission planner is not ready.";
    return;
  }
  if (is_reroute && !operation_mode_state_) {
    res->status.success = false;
    res->status.code = ResponseCode::ERROR_PLANNER_UNREADY;
    res->status.message = "Operation mode state is not received.";
    return;
  }

  const bool is_autonomous_driving =
    operation_mode_state_ ? operation_mode_state_->mode == OperationModeState::AUTONOMOUS &&
                              operation_mode_state_->is_autoware_control_enabled
                          : false;

  change_state(is_reroute ? RouteState::REROUTING : RouteState::ROUTING);
  LaneletRoute route;
  try {
    route = create_route(*req);
  } catch (const tf2::TransformException & error) {
    res->status.success = false;
    res->status.code = autoware_common_msgs::msg::ResponseStatus::TRANSFORM_ERROR;
    res->status.message = error.what();
    return;
  }

  if (route.segments.empty()) {
    cancel_route();
    change_state(is_reroute ? RouteState::SET : RouteState::UNSET);
    res->status.success = false;
    res->status.code = ResponseCode::ERROR_PLANNER_FAILED;
    res->status.message = "The planned route is empty.";
    return;
  }

  if (is_reroute && is_autonomous_driving && !check_reroute_safety(*current_route_, route)) {
    cancel_route();
    change_state(RouteState::SET);
    res->status.success = false;
    res->status.code = ResponseCode::ERROR_REROUTE_FAILED;
    res->status.message = "New route is not safe. Reroute failed.";
    return;
  }

  change_route(route);
  change_state(RouteState::SET);
  res->status.success = true;

  publish_pose_log(odometry_->pose.pose, "initial");
  publish_pose_log(req->goal_pose, "goal");
}

void MissionPlanner::change_route()
{
  current_route_ = nullptr;
  planner_->clearRoute();
  arrival_checker_.set_goal();

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

  pub_route_->publish(route);
  pub_marker_->publish(planner_->visualize(route));
}

void MissionPlanner::cancel_route()
{
  // Restore planner state that changes with create_route function.
  if (current_route_) {
    planner_->updateRoute(*current_route_);
  }
}

LaneletRoute MissionPlanner::create_route(const SetLaneletRoute::Request & req)
{
  const auto & header = req.header;
  const auto & segments = req.segments;
  const auto & goal_pose = req.goal_pose;
  const auto & uuid = req.uuid;
  const auto & allow_goal_modification = req.allow_modification;

  return create_route(header, segments, goal_pose, uuid, allow_goal_modification);
}

LaneletRoute MissionPlanner::create_route(const SetWaypointRoute::Request & req)
{
  const auto & header = req.header;
  const auto & waypoints = req.waypoints;
  const auto & goal_pose = req.goal_pose;
  const auto & uuid = req.uuid;
  const auto & allow_goal_modification = req.allow_modification;

  return create_route(
    header, waypoints, odometry_->pose.pose, goal_pose, uuid, allow_goal_modification);
}

LaneletRoute MissionPlanner::create_route(
  const Header & header, const std::vector<LaneletSegment> & segments, const Pose & goal_pose,
  const UUID & uuid, const bool allow_goal_modification)
{
  LaneletRoute route;
  route.header.stamp = header.stamp;
  route.header.frame_id = map_frame_;
  route.start_pose = odometry_->pose.pose;
  route.goal_pose = transform_pose(goal_pose, header);
  route.segments = segments;
  route.uuid = uuid;
  route.allow_modification = allow_goal_modification;
  return route;
}

LaneletRoute MissionPlanner::create_route(
  const Header & header, const std::vector<Pose> & waypoints, const Pose & start_pose,
  const Pose & goal_pose, const UUID & uuid, const bool allow_goal_modification)
{
  PlannerPlugin::RoutePoints points;
  points.push_back(start_pose);
  for (const auto & waypoint : waypoints) {
    points.push_back(transform_pose(waypoint, header));
  }
  points.push_back(transform_pose(goal_pose, header));

  LaneletRoute route = planner_->plan(points);
  route.header.stamp = header.stamp;
  route.header.frame_id = map_frame_;
  route.uuid = uuid;
  route.allow_modification = allow_goal_modification;
  return route;
}

bool MissionPlanner::check_reroute_safety(
  const LaneletRoute & original_route, const LaneletRoute & target_route)
{
  // The pure check_reroute_safety free function validates the routes and the map, but the node
  // owns the odometry, so guard it here to keep the original observable behavior (same log
  // message and early return when odometry or map is not yet available).
  if (!map_ptr_ || !lanelet_map_ptr_ || !odometry_) {
    RCLCPP_ERROR(get_logger(), "Check reroute safety failed. Route, map or odometry is not set.");
    return false;
  }

  const auto current_velocity = odometry_->twist.twist.linear.x;

  return autoware::mission_planner::check_reroute_safety(
    original_route, target_route, lanelet_map_ptr_, current_velocity, reroute_time_threshold_,
    minimum_reroute_length_, get_logger());
}
}  // namespace autoware::mission_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::mission_planner::MissionPlanner)
