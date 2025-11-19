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

#include "manager.hpp"

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <memory>
#include <set>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
using lanelet::TrafficSign;

StopLineModuleManager::StopLineModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName()), planner_param_()
{
  const std::string ns(StopLineModuleManager::getModuleName());
  auto & p = planner_param_;
  p.stop_margin = get_or_declare_parameter<double>(node, ns + ".stop_margin");
  p.hold_stop_margin_distance =
    get_or_declare_parameter<double>(node, ns + ".hold_stop_margin_distance");
  p.required_stop_duration_sec =
    get_or_declare_parameter<double>(node, ns + ".required_stop_duration_sec");
  p.vehicle_stopped_duration_threshold =
    get_or_declare_parameter<double>(node, ns + ".vehicle_stopped_duration_threshold");
}

std::vector<StopLineWithLaneId> StopLineModuleManager::getStopLinesWithLaneIdOnPath(
  const Trajectory & path, const lanelet::LaneletMapPtr lanelet_map,
  const PlannerData & planner_data)
{
  std::vector<StopLineWithLaneId> stop_lines_with_lane_id;

  PathWithLaneId path_msg;
  path_msg.points = path.restore();

  for (const auto & [traffic_sign_reg_elem, lanelet] :
       planning_utils::getRegElemMapOnPath<TrafficSign>(
         path_msg, lanelet_map, planner_data.current_odometry->pose)) {
    if (traffic_sign_reg_elem->type() != "stop_sign") {
      continue;
    }

    for (const auto & stop_line : traffic_sign_reg_elem->refLines()) {
      stop_lines_with_lane_id.emplace_back(stop_line, lanelet.id());
    }
  }

  return stop_lines_with_lane_id;
}

std::set<lanelet::Id> StopLineModuleManager::getStopLineIdSetOnPath(
  const Trajectory & path, const lanelet::LaneletMapPtr lanelet_map,
  const PlannerData & planner_data)
{
  std::set<lanelet::Id> stop_line_id_set;

  for (const auto & [stop_line, linked_lane_id] :
       getStopLinesWithLaneIdOnPath(path, lanelet_map, planner_data)) {
    stop_line_id_set.insert(stop_line.id());
  }

  return stop_line_id_set;
}

void StopLineModuleManager::launchNewModules(
  const Trajectory & path, [[maybe_unused]] const rclcpp::Time & stamp,
  const PlannerData & planner_data)
{
  for (const auto & [stop_line, linked_lane_id] : getStopLinesWithLaneIdOnPath(
         path, planner_data.route_handler_->getLaneletMapPtr(), planner_data)) {
    const auto module_id = stop_line.id();
    if (!isModuleRegistered(module_id)) {
      registerModule(
        std::make_shared<StopLineModule>(
          module_id,                              //
          stop_line,                              //
          linked_lane_id,                         //
          planner_param_,                         //
          logger_.get_child("stop_line_module"),  //
          clock_,                                 //
          time_keeper_,                           //
          planning_factor_interface_),
        planner_data);
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
StopLineModuleManager::getModuleExpiredFunction(
  const Trajectory & path, const PlannerData & planner_data)
{
  const auto stop_line_id_set =
    getStopLineIdSetOnPath(path, planner_data.route_handler_->getLaneletMapPtr(), planner_data);

  return [stop_line_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return stop_line_id_set.count(scene_module->getModuleId()) == 0;
  };
}

}  // namespace autoware::behavior_velocity_planner::experimental

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::experimental::StopLineModulePlugin,
  autoware::behavior_velocity_planner::experimental::PluginInterface)
