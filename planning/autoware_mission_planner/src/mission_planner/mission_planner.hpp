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

#ifndef MISSION_PLANNER__MISSION_PLANNER_HPP_
#define MISSION_PLANNER__MISSION_PLANNER_HPP_

#include "../lanelet2_plugins/default_planner.hpp"
#include "arrival_checker.hpp"
#include "reroute_safety.hpp"

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_common_msgs/msg/response_status.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/route_state.hpp>
#include <autoware_planning_msgs/srv/clear_route.hpp>
#include <autoware_planning_msgs/srv/set_lanelet_route.hpp>
#include <autoware_planning_msgs/srv/set_waypoint_route.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace autoware::mission_planner
{
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::PoseWithUuidStamped;
using autoware_planning_msgs::msg::RouteState;
using autoware_planning_msgs::srv::ClearRoute;
using autoware_planning_msgs::srv::SetLaneletRoute;
using autoware_planning_msgs::srv::SetWaypointRoute;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::Odometry;
using visualization_msgs::msg::MarkerArray;

struct MissionPlannerConfig
{
  std::string map_frame;
  double reroute_time_threshold;
  double minimum_reroute_length;
  bool allow_reroute_in_autonomous_mode;
  ArrivalCheckerThreshold arrival_checker_threshold;
  lanelet2::DefaultPlannerParameters default_planner_parameters;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info;
};

struct InitializationCheckResult
{
  bool became_ready;
  std::optional<std::string> waiting_message;
};

struct SetLaneletRouteResult
{
  SetLaneletRoute::Response response;
  std::optional<LaneletRoute> route;
  std::optional<MarkerArray> route_marker;
  std::optional<std::string> error_message;
  Pose initial_pose;
};

struct SetWaypointRouteResult
{
  SetWaypointRoute::Response response;
  std::optional<LaneletRoute> route;
  std::optional<MarkerArray> route_marker;
  std::optional<MarkerArray> goal_footprint_marker;
  std::optional<std::string> planner_warning_message;
  std::optional<std::string> error_message;
  Pose initial_pose;
};

class MissionPlanner
{
public:
  using ChangeStateCallback = std::function<void(RouteState::_state_type)>;

  MissionPlanner(const MissionPlannerConfig & config, ChangeStateCallback on_change_state);

  void on_odometry(const Odometry::ConstSharedPtr msg);
  void on_operation_mode_state(const OperationModeState::ConstSharedPtr msg);
  void on_map(const LaneletMapBin::ConstSharedPtr msg);

  InitializationCheckResult check_initialization();

  ClearRoute::Response clear_route();

  SetLaneletRouteResult set_lanelet_route(
    const SetLaneletRoute::Request & req,
    const std::optional<geometry_msgs::msg::TransformStamped> & transform_to_map);
  SetWaypointRouteResult set_waypoint_route(
    const SetWaypointRoute::Request & req,
    const std::optional<geometry_msgs::msg::TransformStamped> & transform_to_map);

private:
  void change_state(RouteState::_state_type state);

  void process_clear_route();
  void change_route(const LaneletRoute & route);
  void cancel_route();

  LaneletRoute create_lanelet_route(
    const SetLaneletRoute::Request & req,
    const geometry_msgs::msg::TransformStamped & transform_to_map);
  lanelet2::DefaultPlanner::PlanResult create_waypoint_route(
    const SetWaypointRoute::Request & req,
    const geometry_msgs::msg::TransformStamped & transform_to_map);

  ArrivalChecker arrival_checker_;
  std::shared_ptr<lanelet2::DefaultPlanner> planner_;

  std::string map_frame_;

  Odometry::ConstSharedPtr odometry_;
  OperationModeState::ConstSharedPtr operation_mode_state_;
  LaneletMapBin::ConstSharedPtr map_ptr_;
  RouteState::_state_type state_{};
  ChangeStateCallback on_change_state_;
  LaneletRoute::ConstSharedPtr current_route_;
  lanelet::LaneletMapPtr lanelet_map_ptr_{nullptr};

  bool is_mission_planner_ready_;

  double reroute_time_threshold_;
  double minimum_reroute_length_;
  // flag to allow reroute in autonomous driving mode.
  // if false, reroute fails. if true, only safe reroute is allowed.
  bool allow_reroute_in_autonomous_mode_;
  RerouteSafetyResult check_reroute_safety(
    const LaneletRoute & original_route, const LaneletRoute & target_route);
};

}  // namespace autoware::mission_planner

#endif  // MISSION_PLANNER__MISSION_PLANNER_HPP_
