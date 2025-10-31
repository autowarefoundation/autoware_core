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

#ifndef AUTOWARE__BEHAVIOR_VELOCITY_PLANNER_COMMON__EXPERIMENTAL__PLUGIN_WRAPPER_HPP_
#define AUTOWARE__BEHAVIOR_VELOCITY_PLANNER_COMMON__EXPERIMENTAL__PLUGIN_WRAPPER_HPP_

#include <autoware/behavior_velocity_planner_common/experimental/plugin_interface.hpp>

#include <memory>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{

template <class T>
class PluginWrapper : public PluginInterface
{
public:
  void init(rclcpp::Node & node) override { scene_manager_ = std::make_unique<T>(node); }
  RequiredSubscriptionInfo getRequiredSubscriptions() override
  {
    return scene_manager_->getRequiredSubscriptions();
  };
  void plan(
    Trajectory & path, const std_msgs::msg::Header & header,
    const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound,
    const PlannerData & planner_data) override
  {
    scene_manager_->plan(path, header, left_bound, right_bound, planner_data);
  };
  void updateSceneModuleInstances(
    const Trajectory & path, const rclcpp::Time & stamp, const PlannerData & planner_data) override
  {
    scene_manager_->updateSceneModuleInstances(path, stamp, planner_data);
  }
  const char * getModuleName() override { return scene_manager_->getModuleName(); }

private:
  std::unique_ptr<T> scene_manager_;
};

}  // namespace autoware::behavior_velocity_planner::experimental

#endif  // AUTOWARE__BEHAVIOR_VELOCITY_PLANNER_COMMON__EXPERIMENTAL__PLUGIN_WRAPPER_HPP_
