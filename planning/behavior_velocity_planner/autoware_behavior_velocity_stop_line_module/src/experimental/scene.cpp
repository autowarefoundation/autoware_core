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

#include <autoware/trajectory/utils/closest.hpp>
#include <autoware/trajectory/utils/crossed.hpp>

#include <memory>
#include <set>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{

namespace
{
bool hasIntersection(const std::set<lanelet::Id> & a, const std::set<lanelet::Id> & b)
{
  for (const auto & id : a) {
    if (b.find(id) != b.end()) {
      return true;
    }
  }
  return false;
}
}  // namespace

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

  path.longitudinal_velocity_mps().range(*stop_point, path.length()).set(0.0);

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
  const double ego_s = autoware::experimental::trajectory::closest(path, ego_pose);
  std::optional<double> stop_point_s;

  switch (state_) {
    case State::APPROACH: {
      const double base_link2front = planner_data.vehicle_info_.max_longitudinal_offset_m;
      const LineString2d stop_line = planning_utils::extendSegmentToBounds(
        lanelet::utils::to2D(stop_line_).basicLineString(), left_bound, right_bound);

      lanelet::Ids connected_lanelet_ids;

      if (planner_data.route_handler_) {
        connected_lanelet_ids =
          planning_utils::collectConnectedLaneIds(linked_lanelet_id_, planner_data.route_handler_);
      } else {
        connected_lanelet_ids = {linked_lanelet_id_};
      }

      // Calculate intersection with stop line
      const auto trajectory_stop_line_intersection =
        autoware::experimental::trajectory::crossed_with_constraint(
          path, stop_line,
          [&](const autoware_internal_planning_msgs::msg::PathPointWithLaneId & point) {
            return hasIntersection(
              {connected_lanelet_ids.begin(), connected_lanelet_ids.end()},
              {point.lane_ids.begin(), point.lane_ids.end()});
          });

      // If no collision found, do nothing
      if (trajectory_stop_line_intersection.size() == 0) {
        stop_point_s = std::nullopt;
        break;
      }

      stop_point_s =
        trajectory_stop_line_intersection.at(0) -
        (base_link2front + planner_param_.stop_margin);  // consider vehicle length and stop margin

      if (*stop_point_s < 0.0) {
        stop_point_s = std::nullopt;
      }
      break;
    }

    case State::STOPPED: {
      stop_point_s = ego_s;
      break;
    }

    case State::START: {
      stop_point_s = std::nullopt;
      break;
    }
  }
  return {ego_s, stop_point_s};
}

void StopLineModule::updateStateAndStoppedTime(
  const rclcpp::Time & now, const double & distance_to_stop_point, const bool & is_vehicle_stopped)
{
  switch (state_) {
    case State::APPROACH: {
      if (distance_to_stop_point < planner_param_.hold_stop_margin_distance && is_vehicle_stopped) {
        state_ = State::STOPPED;
        stopped_time_ = now;
        logInfo("State transition: APPROACH -> STOPPED | Distance: %.2fm", distance_to_stop_point);
        if (distance_to_stop_point < 0.0) {
          logWarn("Vehicle stopped after stop line | Distance: %.2fm", distance_to_stop_point);
        }
      } else {
        logInfoThrottle(10000, "State: APPROACH | Distance: %.2fm", distance_to_stop_point);
      }
      break;
    }
    case State::STOPPED: {
      double stop_duration = (now - *stopped_time_).seconds();
      if (stop_duration > planner_param_.required_stop_duration_sec) {
        state_ = State::START;
        stopped_time_.reset();
        logInfo("State transition: STOPPED -> START | Duration: %.2fs", stop_duration);
      } else {
        logInfoThrottle(
          5000, "State: STOPPED | Distance: %.2fm | Duration: %.2fs", distance_to_stop_point,
          stop_duration);
      }
      break;
    }
    case State::START: {
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
