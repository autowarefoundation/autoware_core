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

#ifndef AUTOWARE__BEHAVIOR_VELOCITY_PLANNER__EXPERIMENTAL__PLANNER_MANAGER_HPP_
#define AUTOWARE__BEHAVIOR_VELOCITY_PLANNER__EXPERIMENTAL__PLANNER_MANAGER_HPP_

#include <autoware/behavior_velocity_planner_common/experimental/plugin_interface.hpp>
#include <pluginlib/class_loader.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{

class BehaviorVelocityPlannerManager
{
public:
  BehaviorVelocityPlannerManager();
  void launchScenePlugin(rclcpp::Node & node, const std::string & name);
  void removeScenePlugin(rclcpp::Node & node, const std::string & name);

  // cppcheck-suppress functionConst
  Trajectory planPathVelocity(
    const PlannerData & planner_data, const Trajectory & input_path,
    const std_msgs::msg::Header & header, const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound);

  RequiredSubscriptionInfo getRequiredSubscriptions() const { return required_subscriptions_; }

private:
  pluginlib::ClassLoader<PluginInterface> plugin_loader_;
  std::vector<std::shared_ptr<PluginInterface>> scene_manager_plugins_;
  RequiredSubscriptionInfo required_subscriptions_;
};
}  // namespace autoware::behavior_velocity_planner::experimental

#endif  // AUTOWARE__BEHAVIOR_VELOCITY_PLANNER__EXPERIMENTAL__PLANNER_MANAGER_HPP_
