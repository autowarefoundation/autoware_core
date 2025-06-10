// Copyright 2023 TIER IV, Inc.
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

#include <autoware/behavior_velocity_planner_common/scene_module_interface.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_debug/time_keeper.hpp>

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner
{

namespace
{
std::string formatIds(
  const std::vector<int64_t> & regulatory_element_ids, const std::vector<int64_t> & lanelet_ids,
  const std::vector<int64_t> & line_ids)
{
  const auto formatIdGroup =
    [](const std::vector<int64_t> & ids, const char * prefix) -> std::string {
    if (ids.empty()) {
      return "";
    }
    std::string result = "[";
    result += prefix;
    for (size_t i = 0; i < ids.size(); ++i) {
      result += (i == 0 ? ": " : ", ") + std::to_string(ids[i]);
    }
    result += "]";
    return result;
  };

  return formatIdGroup(regulatory_element_ids, "Reg") + formatIdGroup(lanelet_ids, "Lane") +
         formatIdGroup(line_ids, "Line");
}
}  // namespace

SceneModuleInterface::SceneModuleInterface(
  const int64_t module_id, rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: module_id_(module_id),
  logger_(logger),
  clock_(clock),
  time_keeper_(time_keeper),
  planning_factor_interface_(planning_factor_interface)
{
}

size_t SceneModuleInterface::findEgoSegmentIndex(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points) const
{
  const auto & p = planner_data_;
  return autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    points, p->current_odometry->pose, p->ego_nearest_dist_threshold);
}

std::string SceneModuleInterface::formatLogMessage(const char * format, va_list args) const
{
  char buffer[1024];
  vsnprintf(buffer, sizeof(buffer), format, args);
  std::string id_info = formatIds(getRegulatoryElementIds(), getLaneletIds(), getLineIds());
  return "[Module ID: " + std::to_string(module_id_) + "]" + id_info + " " + buffer;
}

#define DEFINE_LOG_FUNCTION(custom_log_func, rclcpp_log_func)                \
  void SceneModuleInterface::custom_log_func(const char * format, ...) const \
  {                                                                          \
    va_list args;                                                            \
    va_start(args, format);                                                  \
    std::string message = formatLogMessage(format, args);                    \
    va_end(args);                                                            \
    rclcpp_log_func(logger_, "%s", message.c_str());                         \
  }
DEFINE_LOG_FUNCTION(logInfo, RCLCPP_INFO)
DEFINE_LOG_FUNCTION(logWarn, RCLCPP_WARN)
DEFINE_LOG_FUNCTION(logDebug, RCLCPP_DEBUG)
#undef DEFINE_LOG_FUNCTION

#define DEFINE_LOG_THROTTLE_FUNCTION(custom_log_func, rclcpp_log_func)                     \
  void SceneModuleInterface::custom_log_func(int duration, const char * format, ...) const \
  {                                                                                        \
    va_list args;                                                                          \
    va_start(args, format);                                                                \
    std::string message = formatLogMessage(format, args);                                  \
    va_end(args);                                                                          \
    rclcpp_log_func(logger_, *clock_, duration, "%s", message.c_str());                    \
  }
DEFINE_LOG_THROTTLE_FUNCTION(logInfoThrottle, RCLCPP_INFO_THROTTLE)
DEFINE_LOG_THROTTLE_FUNCTION(logWarnThrottle, RCLCPP_WARN_THROTTLE)
DEFINE_LOG_THROTTLE_FUNCTION(logDebugThrottle, RCLCPP_DEBUG_THROTTLE)
#undef DEFINE_LOG_THROTTLE_FUNCTION

template SceneModuleManagerInterface<SceneModuleInterface>::SceneModuleManagerInterface(
  rclcpp::Node & node, [[maybe_unused]] const char * module_name);
template size_t SceneModuleManagerInterface<SceneModuleInterface>::findEgoSegmentIndex(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points) const;
template void SceneModuleManagerInterface<SceneModuleInterface>::updateSceneModuleInstances(
  const std::shared_ptr<const PlannerData> & planner_data,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path);
template void SceneModuleManagerInterface<SceneModuleInterface>::modifyPathVelocity(
  autoware_internal_planning_msgs::msg::PathWithLaneId * path);
template void SceneModuleManagerInterface<SceneModuleInterface>::deleteExpiredModules(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path);
template void SceneModuleManagerInterface<SceneModuleInterface>::registerModule(
  const std::shared_ptr<SceneModuleInterface> & scene_module);
}  // namespace autoware::behavior_velocity_planner
