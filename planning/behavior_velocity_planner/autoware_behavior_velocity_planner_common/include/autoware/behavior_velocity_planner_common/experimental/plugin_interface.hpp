// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_VELOCITY_PLANNER_COMMON__EXPERIMENTAL__PLUGIN_INTERFACE_HPP_
#define AUTOWARE__BEHAVIOR_VELOCITY_PLANNER_COMMON__EXPERIMENTAL__PLUGIN_INTERFACE_HPP_

#include <autoware/behavior_velocity_planner_common/planner_data.hpp>
#include <autoware/trajectory/path_point_with_lane_id.hpp>
#include <rclcpp/rclcpp.hpp>

#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
using Trajectory = autoware::experimental::trajectory::Trajectory<
  autoware_internal_planning_msgs::msg::PathPointWithLaneId>;

class PluginInterface
{
public:
  virtual ~PluginInterface() = default;
  virtual void init(rclcpp::Node & node) = 0;
  virtual RequiredSubscriptionInfo getRequiredSubscriptions() = 0;
  virtual void plan(
    Trajectory & path, const std_msgs::msg::Header & header,
    const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound,
    const PlannerData & planner_data) = 0;
  virtual void updateSceneModuleInstances(
    const Trajectory & path, const rclcpp::Time & stamp, const PlannerData & planner_data) = 0;
  virtual const char * getModuleName() = 0;
};

}  // namespace autoware::behavior_velocity_planner::experimental

#endif  // AUTOWARE__BEHAVIOR_VELOCITY_PLANNER_COMMON__EXPERIMENTAL__PLUGIN_INTERFACE_HPP_
