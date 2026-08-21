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
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

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

ArrivalCheckerThreshold get_arrival_checker_threshold(rclcpp::Node & node)
{
  ArrivalCheckerThreshold threshold;
  threshold.angle =
    autoware_utils_math::deg2rad(node.declare_parameter<double>("arrival_check_angle_deg"));
  threshold.distance = node.declare_parameter<double>("arrival_check_distance");
  threshold.duration = node.declare_parameter<double>("arrival_check_duration");
  return threshold;
}

MissionPlannerConfig create_mission_planner_config(rclcpp::Node & node)
{
  MissionPlannerConfig config;
  config.map_frame = node.declare_parameter<std::string>("map_frame");
  config.reroute_time_threshold = node.declare_parameter<double>("reroute_time_threshold");
  config.minimum_reroute_length = node.declare_parameter<double>("minimum_reroute_length");
  config.allow_reroute_in_autonomous_mode =
    node.declare_parameter<bool>("allow_reroute_in_autonomous_mode");
  config.arrival_checker_threshold = get_arrival_checker_threshold(node);

  config.default_planner_parameters.goal_angle_threshold_deg =
    node.declare_parameter<double>("goal_angle_threshold_deg");
  config.default_planner_parameters.enable_correct_goal_pose =
    node.declare_parameter<bool>("enable_correct_goal_pose");
  config.default_planner_parameters.consider_no_drivable_lanes =
    node.declare_parameter<bool>("consider_no_drivable_lanes");
  config.default_planner_parameters.check_footprint_inside_lanes =
    node.declare_parameter<bool>("check_footprint_inside_lanes");

  config.vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
  return config;
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

MissionPlannerNode::MissionPlannerNode(const rclcpp::NodeOptions & options)
: Node("mission_planner", options),
  mission_planner_(
    create_mission_planner_config(*this),
    [this](RouteState::_state_type state) {
      RouteState msg;
      msg.stamp = now();
      msg.state = state;
      pub_state_->publish(msg);
    }),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_)
{
  using std::placeholders::_1;

  // NOTE: "map_frame" is already declared by create_mission_planner_config() above; the tf
  // lookup in this node also needs the value, so read it back instead of re-declaring it.
  // cppcheck-suppress useInitializationList
  map_frame_ = get_parameter("map_frame").as_string();

  const auto durable_qos = rclcpp::QoS(1).transient_local();
  sub_odometry_ = create_subscription<Odometry>(
    "~/input/odometry", rclcpp::QoS(1), std::bind(&MissionPlannerNode::on_odometry, this, _1));
  sub_operation_mode_state_ = create_subscription<OperationModeState>(
    "~/input/operation_mode_state", rclcpp::QoS(1).transient_local(),
    std::bind(&MissionPlannerNode::on_operation_mode_state, this, _1));
  sub_vector_map_ = create_subscription<LaneletMapBin>(
    "~/input/vector_map", durable_qos, std::bind(&MissionPlannerNode::on_map, this, _1));
  pub_marker_ = create_publisher<MarkerArray>("~/debug/route_marker", durable_qos);
  pub_goal_footprint_marker_ = create_publisher<MarkerArray>("~/debug/goal_footprint", durable_qos);

  // NOTE: The route interface should be mutually exclusive by callback group.
  srv_clear_route =
    adaptor_.create_service<ClearRouteSpecs>(this, &MissionPlannerNode::on_clear_route);
  srv_set_lanelet_route =
    adaptor_.create_service<SetLaneletRouteSpecs>(this, &MissionPlannerNode::on_set_lanelet_route);
  srv_set_waypoint_route = adaptor_.create_service<SetWaypointRouteSpecs>(
    this, &MissionPlannerNode::on_set_waypoint_route);
  pub_route_ = adaptor_.create_publisher<LaneletRouteSpecs>();
  pub_state_ = adaptor_.create_publisher<RouteStateSpecs>();

  // Route state will be published when the node gets ready for route api after initialization,
  // otherwise the mission planner rejects the request for the API.
  const auto period = rclcpp::Rate(10).period();
  data_check_timer_ = create_wall_timer(period, [this] { check_initialization(); });

  logger_configure_ = std::make_unique<autoware_utils_logging::LoggerLevelConfigure>(this);
  pub_processing_time_ = this->create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "~/debug/processing_time_ms", 1);
}

void MissionPlannerNode::publish_processing_time(
  autoware_utils_system::StopWatch<std::chrono::milliseconds> stop_watch)
{
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch.toc();
  pub_processing_time_->publish(processing_time_msg);
}

void MissionPlannerNode::publish_pose_log(const Pose & pose, const std::string & pose_type)
{
  const auto & p = pose.position;
  RCLCPP_INFO(
    this->get_logger(), "%s pose - x: %f, y: %f, z: %f", pose_type.c_str(), p.x, p.y, p.z);
  const auto & quaternion = pose.orientation;
  RCLCPP_INFO(
    this->get_logger(), "%s orientation - qx: %f, qy: %f, qz: %f, qw: %f", pose_type.c_str(),
    quaternion.x, quaternion.y, quaternion.z, quaternion.w);
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

void MissionPlannerNode::check_initialization()
{
  auto logger = get_logger();
  auto clock = *get_clock();

  const auto result = mission_planner_.check_initialization();
  if (!result.became_ready) {
    RCLCPP_INFO_THROTTLE(logger, clock, 5000, "%s", result.waiting_message->c_str());
    return;
  }

  RCLCPP_DEBUG(logger, "Route API is ready.");

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

void MissionPlannerNode::on_odometry(const Odometry::ConstSharedPtr msg)
{
  mission_planner_.on_odometry(msg);
}

void MissionPlanner::on_operation_mode_state(const OperationModeState::ConstSharedPtr msg)
{
  operation_mode_state_ = msg;
}

void MissionPlannerNode::on_operation_mode_state(const OperationModeState::ConstSharedPtr msg)
{
  mission_planner_.on_operation_mode_state(msg);
}

void MissionPlanner::on_map(const LaneletMapBin::ConstSharedPtr msg)
{
  map_ptr_ = msg;
  lanelet_map_ptr_ = autoware::experimental::lanelet2_utils::remove_const(
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*map_ptr_));
  planner_->set_map(*map_ptr_);
}

void MissionPlannerNode::on_map(const LaneletMapBin::ConstSharedPtr msg)
{
  mission_planner_.on_map(msg);
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

void MissionPlannerNode::on_clear_route(
  const ClearRoute::Request::SharedPtr, const ClearRoute::Response::SharedPtr res)
{
  ScopedProcessingTimePublisher processing_time_publisher(*this);

  *res = mission_planner_.clear_route();
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

void MissionPlannerNode::on_set_lanelet_route(
  const SetLaneletRoute::Request::SharedPtr req, const SetLaneletRoute::Response::SharedPtr res)
{
  ScopedProcessingTimePublisher processing_time_publisher(*this);

  std::optional<geometry_msgs::msg::TransformStamped> transform_to_map;
  try {
    transform_to_map =
      tf_buffer_.lookupTransform(map_frame_, req->header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    // Explicit assignment to avoid clang-tidy's check.
    // Failure is handled by the transform_to_map check below.
    transform_to_map = std::nullopt;
  }

  const auto result = mission_planner_.set_lanelet_route(*req, transform_to_map);
  *res = result.response;

  if (!res->status.success) {
    if (result.error_message) {
      RCLCPP_ERROR(get_logger(), "%s", result.error_message->c_str());
    }
    return;
  }

  pub_route_->publish(*result.route);
  pub_marker_->publish(*result.route_marker);
  publish_pose_log(result.initial_pose, "initial");
  publish_pose_log(req->goal_pose, "goal");
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

void MissionPlannerNode::on_set_waypoint_route(
  const SetWaypointRoute::Request::SharedPtr req, const SetWaypointRoute::Response::SharedPtr res)
{
  ScopedProcessingTimePublisher processing_time_publisher(*this);

  std::optional<geometry_msgs::msg::TransformStamped> transform_to_map;
  try {
    transform_to_map =
      tf_buffer_.lookupTransform(map_frame_, req->header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    // Explicit assignment to avoid clang-tidy's check.
    // Failure is handled by the transform_to_map check below.
    transform_to_map = std::nullopt;
  }

  const auto result = mission_planner_.set_waypoint_route(*req, transform_to_map);
  *res = result.response;

  if (result.planner_warning_message) {
    RCLCPP_WARN(get_logger(), "%s", result.planner_warning_message->c_str());
  }
  if (result.goal_footprint_marker) {
    pub_goal_footprint_marker_->publish(*result.goal_footprint_marker);
  }

  if (!res->status.success) {
    if (result.error_message) {
      RCLCPP_ERROR(get_logger(), "%s", result.error_message->c_str());
    }
    return;
  }

  pub_route_->publish(*result.route);
  pub_marker_->publish(*result.route_marker);
  publish_pose_log(result.initial_pose, "initial");
  publish_pose_log(req->goal_pose, "goal");
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

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::mission_planner::MissionPlannerNode)
