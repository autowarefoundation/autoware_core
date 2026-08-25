// Copyright 2025 Tier IV, Inc.
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

#include "scene.hpp"

#include "../stop_line_util.hpp"

#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{

StopLineModule::StopLineModule(
  const int64_t module_id,                                                //
  const lanelet::ConstLineString3d & stop_line,                           //
  const lanelet::Id & linked_lanelet_id,                                  //
  const PlannerParam & planner_param,                                     //
  const rclcpp::Logger & logger,                                          //
  const rclcpp::Clock::SharedPtr clock,                                   //
  const std::shared_ptr<autoware_utils_debug::TimeKeeper> & time_keeper,  //
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface> &
    planning_factor_interface)
: SceneModuleInterface(module_id, logger, clock, time_keeper, planning_factor_interface),
  stop_line_(stop_line),
  linked_lanelet_id_(linked_lanelet_id),
  planner_param_(planner_param),
  state_(State::APPROACH),
  debug_data_()
{
  logInfo("Module initialized");
}

bool StopLineModule::modifyPathVelocity(
  Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound, const PlannerData & planner_data)
{
  auto [ego_s, stop_point] = getEgoAndStopPoint(
    path, left_bound, right_bound, planner_data.current_odometry->pose, planner_data);

  if (!stop_point) {
    if (state_ == State::APPROACH) {
      logWarnThrottle(
        5000, "No stop point found | ego_s: %.2f | path_length: %.2f", ego_s, path.length());
    }
    return true;
  }

  path.set_stopline(*stop_point);

  // TODO(soblin): PlanningFactorInterface use trajectory class
  planning_factor_interface_->add(
    path.restore(), planner_data.current_odometry->pose, path.compute(*stop_point).point.pose,
    autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
    autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/, 0.0,
    0.0 /*shift distance*/, "stopline");

  updateStateAndStoppedTime(
    clock_->now(), *stop_point - ego_s,
    planner_data.isVehicleStopped(planner_param_.vehicle_stopped_duration_threshold));

  const auto stop_pose = path.compute(*stop_point).point.pose;

  updateDebugData(stop_pose, planner_data);

  return true;
}

std::pair<double, std::optional<double>> StopLineModule::getEgoAndStopPoint(
  const Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound,
  const geometry_msgs::msg::Pose & ego_pose, const PlannerData & planner_data) const
{
  lanelet::Ids connected_lanelet_ids;
  if (planner_data.route_handler_) {
    connected_lanelet_ids =
      planning_utils::collectConnectedLaneIds(linked_lanelet_id_, planner_data.route_handler_);
  } else {
    connected_lanelet_ids = {linked_lanelet_id_};
  }

  return stop_line_utils::compute_ego_and_stop_point(
    path, left_bound, right_bound, stop_line_, ego_pose, state_, connected_lanelet_ids,
    planner_data.vehicle_info_.max_longitudinal_offset_m, planner_param_.stop_margin);
}

void StopLineModule::updateStateAndStoppedTime(
  const rclcpp::Time & now, const double & distance_to_stop_point, const bool & is_vehicle_stopped)
{
  const auto result = stop_line_utils::advance_state(
    state_, stopped_time_, now, distance_to_stop_point, is_vehicle_stopped,
    planner_param_.hold_stop_margin_distance, planner_param_.required_stop_duration_sec);

  switch (result.transition) {
    case stop_line_utils::StateTransition::APPROACH_TO_STOPPED: {
      logInfo("State transition: APPROACH -> STOPPED | Distance: %.2fm", distance_to_stop_point);
      if (distance_to_stop_point < 0.0) {
        logWarn("Vehicle stopped after stop line | Distance: %.2fm", distance_to_stop_point);
      }
      break;
    }
    case stop_line_utils::StateTransition::APPROACH_STAY: {
      logInfoThrottle(10000, "State: APPROACH | Distance: %.2fm", distance_to_stop_point);
      break;
    }
    case stop_line_utils::StateTransition::STOPPED_TIME_RECOVERY: {
      logWarn("stopped_time_ has no value in STOPPED state");
      break;
    }
    case stop_line_utils::StateTransition::STOPPED_TO_START: {
      logInfo("State transition: STOPPED -> START | Duration: %.2fs", result.stop_duration);
      break;
    }
    case stop_line_utils::StateTransition::STOPPED_STAY: {
      logInfoThrottle(
        5000, "State: STOPPED | Distance: %.2fm | Duration: %.2fs", distance_to_stop_point,
        result.stop_duration);
      break;
    }
    case stop_line_utils::StateTransition::START: {
      logDebug("State: START | Distance: %.2fm", distance_to_stop_point);
      break;
    }
  }
}

void StopLineModule::updateDebugData(
  const geometry_msgs::msg::Pose & stop_pose, const PlannerData & planner_data)
{
  debug_data_.base_link2front = planner_data.vehicle_info_.max_longitudinal_offset_m;
  debug_data_.stop_pose = stop_pose;
  if (state_ == State::START) {
    debug_data_.stop_pose = std::nullopt;
  }
}

autoware::motion_utils::VirtualWalls StopLineModule::createVirtualWalls()
{
  autoware::motion_utils::VirtualWalls virtual_walls;

  if (debug_data_.stop_pose && (state_ == State::APPROACH || state_ == State::STOPPED)) {
    autoware::motion_utils::VirtualWall wall;
    wall.text = "stopline";
    wall.style = autoware::motion_utils::VirtualWallType::stop;
    wall.ns = std::to_string(module_id_) + "_";
    wall.pose = autoware_utils_geometry::calc_offset_pose(
      *debug_data_.stop_pose, debug_data_.base_link2front, 0.0, 0.0);
    virtual_walls.push_back(wall);
  }
  return virtual_walls;
}

}  // namespace autoware::behavior_velocity_planner::experimental
