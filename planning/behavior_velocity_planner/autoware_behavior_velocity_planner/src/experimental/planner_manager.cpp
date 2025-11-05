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

#include "autoware/behavior_velocity_planner/experimental/planner_manager.hpp"

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
BehaviorVelocityPlannerManager::BehaviorVelocityPlannerManager()
: plugin_loader_(
    "autoware_behavior_velocity_planner",
    "autoware::behavior_velocity_planner::experimental::PluginInterface")
{
}

void BehaviorVelocityPlannerManager::launchScenePlugin(
  rclcpp::Node & node, const std::string & name)
{
  if (!plugin_loader_.isClassAvailable(name)) {
    RCLCPP_ERROR_STREAM(node.get_logger(), "The scene plugin '" << name << "' is not available.");
    return;
  }

  const auto plugin = plugin_loader_.createSharedInstance(name);
  plugin->init(node);

  // Check if the plugin is already registered.
  for (const auto & running_plugin : scene_manager_plugins_) {
    if (plugin->getModuleName() == running_plugin->getModuleName()) {
      RCLCPP_WARN_STREAM(node.get_logger(), "The plugin '" << name << "' is already loaded.");
      return;
    }
  }

  // register
  scene_manager_plugins_.push_back(plugin);
  RCLCPP_INFO_STREAM(node.get_logger(), "The scene plugin '" << name << "' is loaded.");

  // update the subscription
  const auto required_subscriptions = plugin->getRequiredSubscriptions();
  required_subscriptions_.concat(required_subscriptions);
}

void BehaviorVelocityPlannerManager::removeScenePlugin(
  rclcpp::Node & node, const std::string & name)
{
  const auto it = std::remove_if(
    scene_manager_plugins_.begin(), scene_manager_plugins_.end(),
    [&](const std::shared_ptr<PluginInterface> plugin) { return plugin->getModuleName() == name; });

  if (it == scene_manager_plugins_.end()) {
    RCLCPP_WARN_STREAM(
      node.get_logger(),
      "The scene plugin '" << name << "' is not found in the registered modules.");
    return;
  }
  scene_manager_plugins_.erase(it, scene_manager_plugins_.end());
  RCLCPP_INFO_STREAM(node.get_logger(), "The scene plugin '" << name << "' is unloaded.");
}

Trajectory BehaviorVelocityPlannerManager::planPathVelocity(
  const PlannerData & planner_data, const Trajectory & input_path,
  const std_msgs::msg::Header & header, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound)
{
  auto output_path = input_path;

  for (const auto & plugin : scene_manager_plugins_) {
    plugin->updateSceneModuleInstances(input_path, header.stamp, planner_data);
    plugin->plan(output_path, header, left_bound, right_bound, planner_data);
  }

  return output_path;
}

}  // namespace autoware::behavior_velocity_planner::experimental
