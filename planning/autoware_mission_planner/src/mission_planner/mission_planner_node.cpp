// Copyright 2026 Autoware Foundation
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

#include "mission_planner_node.hpp"

#include <autoware_utils_math/unit_conversion.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <memory>
#include <optional>
#include <string>

namespace autoware::mission_planner
{

namespace
{
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
}  // namespace

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

void MissionPlannerNode::on_odometry(const Odometry::ConstSharedPtr msg)
{
  mission_planner_.on_odometry(msg);
}

void MissionPlannerNode::on_operation_mode_state(const OperationModeState::ConstSharedPtr msg)
{
  mission_planner_.on_operation_mode_state(msg);
}

void MissionPlannerNode::on_map(const LaneletMapBin::ConstSharedPtr msg)
{
  mission_planner_.on_map(msg);
}

void MissionPlannerNode::on_clear_route(
  const ClearRoute::Request::SharedPtr, const ClearRoute::Response::SharedPtr res)
{
  ScopedProcessingTimePublisher processing_time_publisher(*this);

  *res = mission_planner_.clear_route();
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

  if (result.warning_message) {
    RCLCPP_WARN(get_logger(), "%s", result.warning_message->c_str());
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

}  // namespace autoware::mission_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::mission_planner::MissionPlannerNode)
