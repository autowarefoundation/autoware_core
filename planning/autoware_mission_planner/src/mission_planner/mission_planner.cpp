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
#include <autoware_utils_math/unit_conversion.hpp>

#include <autoware_common_msgs/msg/response_status.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::mission_planner
{

namespace
{
void set_fail_response(
  const SetLaneletRoute::Response::SharedPtr & res, const uint16_t code,
  const std::string & message)
{
  res->status.success = false;
  res->status.code = code;
  res->status.message = message;
}

void set_fail_response(
  const SetWaypointRoute::Response::SharedPtr & res, const uint16_t code,
  const std::string & message)
{
  res->status.success = false;
  res->status.code = code;
  res->status.message = message;
}

void set_success_response(
  const ClearRoute::Response::SharedPtr & res, const uint16_t code, const std::string & message)
{
  res->status.success = true;
  res->status.code = code;
  res->status.message = message;
}

ArrivalCheckerThreshold get_arrival_checker_threshold(rclcpp::Node & node)
{
  ArrivalCheckerThreshold threshold;
  threshold.angle =
    autoware_utils_math::deg2rad(node.declare_parameter<double>("arrival_check_angle_deg"));
  threshold.distance = node.declare_parameter<double>("arrival_check_distance");
  threshold.duration = node.declare_parameter<double>("arrival_check_duration");
  return threshold;
}

Pose transform_pose(const Pose & pose, const geometry_msgs::msg::TransformStamped & transform)
{
  Pose result;
  tf2::doTransform(pose, result, transform);
  return result;
}
}  // namespace

MissionPlanner::MissionPlanner(const rclcpp::NodeOptions & options)
: Node("mission_planner", options),
  arrival_checker_(get_arrival_checker_threshold(*this)),
  plugin_loader_("autoware_mission_planner", "autoware::mission_planner::PlannerPlugin"),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  odometry_(nullptr),
  map_ptr_(nullptr)
{
  using std::placeholders::_1;

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
  srv_clear_route = adaptor_.create_service<ClearRouteSpecs>(this, &MissionPlanner::on_clear_route);
  srv_set_lanelet_route =
    adaptor_.create_service<SetLaneletRouteSpecs>(this, &MissionPlanner::on_set_lanelet_route);
  srv_set_waypoint_route =
    adaptor_.create_service<SetWaypointRouteSpecs>(this, &MissionPlanner::on_set_waypoint_route);
  pub_route_ = adaptor_.create_publisher<LaneletRouteSpecs>();
  pub_state_ = adaptor_.create_publisher<RouteStateSpecs>();
  on_change_state_ = [this](RouteState::_state_type state) {
    RouteState msg;
    msg.stamp = now();
    msg.state = state;
    pub_state_->publish(msg);
  };

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
}

void MissionPlanner::change_state(RouteState::_state_type state)
{
  state_ = state;
  if (on_change_state_) {
    on_change_state_(state);
  }
}

void MissionPlanner::on_clear_route(
  const ClearRoute::Request::SharedPtr, const ClearRoute::Response::SharedPtr res)
{
  ScopedProcessingTimePublisher processing_time_publisher(*this);

  if (!is_mission_planner_ready_) {
    using ResponseCode = autoware_adapi_v1_msgs::msg::ResponseStatus;
    set_success_response(res, ResponseCode::NO_EFFECT, "The mission planner is not ready.");
    return;
  }

  clear_route();
  change_state(RouteState::UNSET);
  res->status.success = true;
}

void MissionPlanner::on_set_lanelet_route(
  const SetLaneletRoute::Request::SharedPtr req, const SetLaneletRoute::Response::SharedPtr res)
{
  ScopedProcessingTimePublisher processing_time_publisher(*this);
  using ResponseCode = autoware_adapi_v1_msgs::srv::SetRoute::Response;
  const auto is_reroute = state_ == RouteState::SET;

  std::optional<geometry_msgs::msg::TransformStamped> transform_to_map;
  try {
    transform_to_map =
      tf_buffer_.lookupTransform(map_frame_, req->header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    // Explicit assignment to avoid clang-tidy's check.
    // Failure is handled by the transform_to_map check below.
    transform_to_map = std::nullopt;
  }

  if (state_ != RouteState::UNSET && state_ != RouteState::SET) {
    set_fail_response(
      res, ResponseCode::ERROR_INVALID_STATE, "The route cannot be set in the current state.");
    return;
  }
  if (!is_mission_planner_ready_) {
    set_fail_response(
      res, ResponseCode::ERROR_PLANNER_UNREADY, "The mission planner is not ready.");
    return;
  }
  if (is_reroute && !operation_mode_state_) {
    set_fail_response(
      res, ResponseCode::ERROR_PLANNER_UNREADY, "Operation mode state is not received.");
    return;
  }

  const bool is_autonomous_driving =
    operation_mode_state_ ? operation_mode_state_->mode == OperationModeState::AUTONOMOUS &&
                              operation_mode_state_->is_autoware_control_enabled
                          : false;

  if (is_reroute && !allow_reroute_in_autonomous_mode_ && is_autonomous_driving) {
    set_fail_response(
      res, ResponseCode::ERROR_INVALID_STATE, "Reroute is not allowed in autonomous mode.");
    return;
  }

  change_state(is_reroute ? RouteState::REROUTING : RouteState::ROUTING);
  if (!transform_to_map) {
    set_fail_response(
      res, autoware_common_msgs::msg::ResponseStatus::TRANSFORM_ERROR,
      "Failed to transform pose to map frame.");
    return;
  }
  const auto route = create_lanelet_route(*req, *transform_to_map);

  if (route.segments.empty()) {
    cancel_route();
    change_state(is_reroute ? RouteState::SET : RouteState::UNSET);
    set_fail_response(res, ResponseCode::ERROR_PLANNER_FAILED, "The planned route is empty.");
    return;
  }

  if (is_reroute && is_autonomous_driving) {
    const auto reroute_safety_result = check_reroute_safety(*current_route_, route);
    if (!reroute_safety_result.is_safe) {
      RCLCPP_ERROR(get_logger(), "%s", reroute_safety_result.reason.c_str());
      cancel_route();
      change_state(RouteState::SET);
      set_fail_response(
        res, ResponseCode::ERROR_REROUTE_FAILED, "New route is not safe. Reroute failed.");
      return;
    }
  }

  change_route(route);
  change_state(RouteState::SET);
  res->status.success = true;

  publish_route(route);
  publish_pose_log(odometry_->pose.pose, "initial");
  publish_pose_log(req->goal_pose, "goal");
}

void MissionPlanner::on_set_waypoint_route(
  const SetWaypointRoute::Request::SharedPtr req, const SetWaypointRoute::Response::SharedPtr res)
{
  ScopedProcessingTimePublisher processing_time_publisher(*this);
  using ResponseCode = autoware_adapi_v1_msgs::srv::SetRoutePoints::Response;
  const auto is_reroute = state_ == RouteState::SET;

  std::optional<geometry_msgs::msg::TransformStamped> transform_to_map;
  try {
    transform_to_map =
      tf_buffer_.lookupTransform(map_frame_, req->header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    // Explicit assignment to avoid clang-tidy's check.
    // Failure is handled by the transform_to_map check below.
    transform_to_map = std::nullopt;
  }

  if (state_ != RouteState::UNSET && state_ != RouteState::SET) {
    set_fail_response(
      res, ResponseCode::ERROR_INVALID_STATE, "The route cannot be set in the current state.");
    return;
  }
  if (!is_mission_planner_ready_) {
    set_fail_response(
      res, ResponseCode::ERROR_PLANNER_UNREADY, "The mission planner is not ready.");
    return;
  }
  if (is_reroute && !operation_mode_state_) {
    set_fail_response(
      res, ResponseCode::ERROR_PLANNER_UNREADY, "Operation mode state is not received.");
    return;
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
    return;
  }
  const auto route = create_waypoint_route(*req, *transform_to_map);

  if (route.segments.empty()) {
    cancel_route();
    change_state(is_reroute ? RouteState::SET : RouteState::UNSET);
    set_fail_response(res, ResponseCode::ERROR_PLANNER_FAILED, "The planned route is empty.");
    return;
  }

  if (is_reroute && is_autonomous_driving) {
    const auto reroute_safety_result = check_reroute_safety(*current_route_, route);
    if (!reroute_safety_result.is_safe) {
      RCLCPP_ERROR(get_logger(), "%s", reroute_safety_result.reason.c_str());
      cancel_route();
      change_state(RouteState::SET);
      set_fail_response(
        res, ResponseCode::ERROR_REROUTE_FAILED, "New route is not safe. Reroute failed.");
      return;
    }
  }

  change_route(route);
  change_state(RouteState::SET);
  res->status.success = true;

  publish_route(route);
  publish_pose_log(odometry_->pose.pose, "initial");
  publish_pose_log(req->goal_pose, "goal");
}

void MissionPlanner::clear_route()
{
  current_route_ = nullptr;
  planner_->clearRoute();
  arrival_checker_.clear_goal();

  // TODO(Takagi, Isamu): publish an empty route here
  // pub_route_->publish();
  // pub_marker_->publish();
}

void MissionPlanner::publish_route(const LaneletRoute & route)
{
  pub_route_->publish(route);
  pub_marker_->publish(planner_->visualize(route));
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

LaneletRoute MissionPlanner::create_waypoint_route(
  const SetWaypointRoute::Request & req,
  const geometry_msgs::msg::TransformStamped & transform_to_map)
{
  PlannerPlugin::RoutePoints points;
  points.push_back(odometry_->pose.pose);
  for (const auto & waypoint : req.waypoints) {
    points.push_back(transform_pose(waypoint, transform_to_map));
  }
  points.push_back(transform_pose(req.goal_pose, transform_to_map));

  LaneletRoute route = planner_->plan(points);
  route.header.stamp = req.header.stamp;
  route.header.frame_id = map_frame_;
  route.uuid = req.uuid;
  route.allow_modification = req.allow_modification;
  return route;
}

RerouteSafetyResult MissionPlanner::check_reroute_safety(
  const LaneletRoute & original_route, const LaneletRoute & target_route)
{
  // The pure check_reroute_safety free function validates the routes and the map, but the node
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

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::mission_planner::MissionPlanner)
