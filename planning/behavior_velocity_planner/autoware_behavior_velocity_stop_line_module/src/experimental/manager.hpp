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

#ifndef EXPERIMENTAL__MANAGER_HPP_
#define EXPERIMENTAL__MANAGER_HPP_

#include "scene.hpp"

#include <autoware/behavior_velocity_planner_common/experimental/plugin_wrapper.hpp>

#include <memory>
#include <set>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
using StopLineWithLaneId = std::pair<lanelet::ConstLineString3d, lanelet::Id>;

class StopLineModuleManager : public SceneModuleManagerInterface<>
{
public:
  explicit StopLineModuleManager(rclcpp::Node & node);

  const char * getModuleName() override { return "stop_line"; }

  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    return RequiredSubscriptionInfo{};
  }

private:
  StopLineModule::PlannerParam planner_param_;

  std::vector<StopLineWithLaneId> getStopLinesWithLaneIdOnPath(
    const Trajectory & path, const lanelet::LaneletMapPtr lanelet_map,
    const PlannerData & planner_data);

  std::set<lanelet::Id> getStopLineIdSetOnPath(
    const Trajectory & path, const lanelet::LaneletMapPtr lanelet_map,
    const PlannerData & planner_data);

  void launchNewModules(
    const Trajectory & path, const rclcpp::Time & stamp, const PlannerData & planner_data) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterface> &)> getModuleExpiredFunction(
    const Trajectory & path, const PlannerData & planner_data) override;
};

class StopLineModulePlugin : public PluginWrapper<StopLineModuleManager>
{
};

}  // namespace autoware::behavior_velocity_planner::experimental

#endif  // EXPERIMENTAL__MANAGER_HPP_
