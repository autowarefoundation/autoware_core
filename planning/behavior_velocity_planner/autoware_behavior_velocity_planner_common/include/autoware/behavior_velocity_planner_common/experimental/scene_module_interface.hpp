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

#ifndef AUTOWARE__BEHAVIOR_VELOCITY_PLANNER_COMMON__EXPERIMENTAL__SCENE_MODULE_INTERFACE_HPP_
#define AUTOWARE__BEHAVIOR_VELOCITY_PLANNER_COMMON__EXPERIMENTAL__SCENE_MODULE_INTERFACE_HPP_

#include "autoware/behavior_velocity_planner_common/planner_data.hpp"
#include "autoware/behavior_velocity_planner_common/utilization/util.hpp"

#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware/objects_of_interest_marker_interface/objects_of_interest_marker_interface.hpp>
#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware_utils_debug/debug_publisher.hpp>
#include <autoware_utils_rclcpp/parameter.hpp>

#include <memory>
#include <set>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{

using autoware::objects_of_interest_marker_interface::ColorName;
using autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface;
using autoware_internal_debug_msgs::msg::Float64Stamped;
using autoware_utils_debug::DebugPublisher;
using autoware_utils_rclcpp::get_or_declare_parameter;
using autoware_utils_system::StopWatch;
using builtin_interfaces::msg::Time;
using unique_identifier_msgs::msg::UUID;

using Trajectory = autoware::experimental::trajectory::Trajectory<
  autoware_internal_planning_msgs::msg::PathPointWithLaneId>;

struct ObjectOfInterest
{
  geometry_msgs::msg::Pose pose;
  autoware_perception_msgs::msg::Shape shape;
  ColorName color;
  ObjectOfInterest(
    const geometry_msgs::msg::Pose & pose, const autoware_perception_msgs::msg::Shape & shape,
    const ColorName & color_name)
  : pose(pose), shape(shape), color(color_name)
  {
  }
};

class SceneModuleInterface
{
public:
  explicit SceneModuleInterface(
    const int64_t module_id, const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr clock,
    const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
    const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
      planning_factor_interface);
  virtual ~SceneModuleInterface() = default;

  virtual bool modifyPathVelocity(
    Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound,
    const PlannerData & planner_data) = 0;

  virtual visualization_msgs::msg::MarkerArray createDebugMarkerArray() = 0;
  virtual std::vector<autoware::motion_utils::VirtualWall> createVirtualWalls() = 0;

  int64_t getModuleId() const { return module_id_; }

  std::vector<ObjectOfInterest> getObjectsOfInterestData() const { return objects_of_interest_; }
  void clearObjectsOfInterestData() { objects_of_interest_.clear(); }

protected:
  const int64_t module_id_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::vector<ObjectOfInterest> objects_of_interest_;
  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  std::shared_ptr<planning_factor_interface::PlanningFactorInterface> planning_factor_interface_;

  void setObjectsOfInterestData(
    const geometry_msgs::msg::Pose & pose, const autoware_perception_msgs::msg::Shape & shape,
    const ColorName & color_name)
  {
    objects_of_interest_.emplace_back(pose, shape, color_name);
  }

  size_t findEgoSegmentIndex(
    const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
    const PlannerData & planner_data) const
  {
    const auto & p = planner_data;
    return autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      points, p.current_odometry->pose, p.ego_nearest_dist_threshold, p.ego_nearest_yaw_threshold);
  }

  void logInfo(const char * format, ...) const;
  void logWarn(const char * format, ...) const;
  void logDebug(const char * format, ...) const;
  void logInfoThrottle(const int duration, const char * format, ...) const;
  void logWarnThrottle(const int duration, const char * format, ...) const;
  void logDebugThrottle(const int duration, const char * format, ...) const;

  virtual std::vector<int64_t> getRegulatoryElementIds() const { return {}; }
  virtual std::vector<int64_t> getLaneletIds() const { return {}; }
  virtual std::vector<int64_t> getLineIds() const { return {}; }

private:
  std::string formatLogMessage(const char * format, va_list args) const;
};

template <class T = SceneModuleInterface>
class SceneModuleManagerInterface
{
public:
  SceneModuleManagerInterface(rclcpp::Node & node, [[maybe_unused]] const char * module_name)
  : node_(node), clock_(node.get_clock()), logger_(node.get_logger())
  {
    const auto ns = std::string("~/debug/") + module_name;
    pub_debug_ = node.create_publisher<visualization_msgs::msg::MarkerArray>(ns, 1);
    if (!node.has_parameter("is_publish_debug_path")) {
      is_publish_debug_path_ = node.declare_parameter<bool>("is_publish_debug_path");
    } else {
      is_publish_debug_path_ = node.get_parameter("is_publish_debug_path").as_bool();
    }
    if (is_publish_debug_path_) {
      pub_debug_path_ = node.create_publisher<autoware_internal_planning_msgs::msg::PathWithLaneId>(
        std::string("~/debug/path_with_lane_id/") + module_name, 1);
    }
    pub_virtual_wall_ = node.create_publisher<visualization_msgs::msg::MarkerArray>(
      std::string("~/virtual_wall/") + module_name, 5);

    const bool enable_console_output =
      get_or_declare_parameter<bool>(node, "planning_factor_console_output.enable");
    const int throttle_duration_ms =
      get_or_declare_parameter<int>(node, "planning_factor_console_output.duration");

    planning_factor_interface_ =
      std::make_shared<planning_factor_interface::PlanningFactorInterface>(
        &node, module_name, enable_console_output, throttle_duration_ms);

    processing_time_publisher_ = std::make_shared<DebugPublisher>(&node, "~/debug");

    pub_processing_time_detail_ = node.create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
      "~/debug/processing_time_detail_ms/" + std::string(module_name), 1);

    time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>(pub_processing_time_detail_);
  }

  virtual ~SceneModuleManagerInterface() = default;

  virtual const char * getModuleName() = 0;

  void updateSceneModuleInstances(
    const Trajectory & path, const rclcpp::Time & stamp, const PlannerData & planner_data)
  {
    launchNewModules(path, stamp, planner_data);
    deleteExpiredModules(path, planner_data);
  }

  virtual void plan(
    Trajectory & path, const std_msgs::msg::Header & header,
    const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound, const PlannerData & planner_data)
  {
    modifyPathVelocity(path, header, left_bound, right_bound, planner_data);
  }

  virtual RequiredSubscriptionInfo getRequiredSubscriptions() const = 0;

protected:
  virtual void modifyPathVelocity(
    Trajectory & path, const std_msgs::msg::Header & header,
    const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound, const PlannerData & planner_data)
  {
    autoware_utils_debug::ScopedTimeTrack st(
      "SceneModuleManagerInterface::modifyPathVelocity", *time_keeper_);
    StopWatch<std::chrono::milliseconds> stop_watch;
    stop_watch.tic("Total");
    visualization_msgs::msg::MarkerArray debug_marker_array;

    for (const auto & scene_module : scene_modules_) {
      scene_module->modifyPathVelocity(path, left_bound, right_bound, planner_data);

      // The velocity factor must be called after modifyPathVelocity.

      for (const auto & marker : scene_module->createDebugMarkerArray().markers) {
        debug_marker_array.markers.push_back(marker);
      }

      virtual_wall_marker_creator_.add_virtual_walls(scene_module->createVirtualWalls());
    }

    planning_factor_interface_->publish();
    pub_debug_->publish(debug_marker_array);
    if (is_publish_debug_path_) {
      autoware_internal_planning_msgs::msg::PathWithLaneId debug_path;
      debug_path.header = header;
      debug_path.points = path.restore();
      pub_debug_path_->publish(debug_path);
    }
    pub_virtual_wall_->publish(virtual_wall_marker_creator_.create_markers(clock_->now()));
    processing_time_publisher_->publish<Float64Stamped>(
      std::string(getModuleName()) + "/processing_time_ms", stop_watch.toc("Total"));
  }

  virtual void launchNewModules(
    const Trajectory & path, const rclcpp::Time & stamp, const PlannerData & planner_data) = 0;

  virtual std::function<bool(const std::shared_ptr<T> &)> getModuleExpiredFunction(
    const Trajectory & path, const PlannerData & planner_data) = 0;

  virtual void deleteExpiredModules(const Trajectory & path, const PlannerData & planner_data)
  {
    const auto isModuleExpired = getModuleExpiredFunction(path, planner_data);
    std::vector<int64_t> expired_module_ids;

    auto itr = scene_modules_.begin();
    while (itr != scene_modules_.end()) {
      if (isModuleExpired(*itr)) {
        expired_module_ids.push_back((*itr)->getModuleId());
        registered_module_id_set_.erase((*itr)->getModuleId());
        itr = scene_modules_.erase(itr);
      } else {
        itr++;
      }
    }

    if (!expired_module_ids.empty()) {
      printDeletionInfo(expired_module_ids, planner_data);
    }
  }

  bool isModuleRegistered(const int64_t module_id)
  {
    return registered_module_id_set_.count(module_id) != 0;
  }

  void registerModule(const std::shared_ptr<T> & scene_module, const PlannerData & planner_data)
  {
    registered_module_id_set_.emplace(scene_module->getModuleId());
    scene_modules_.insert(scene_module);

    printRegistrationInfo(scene_module->getModuleId(), planner_data);
  }

  std::set<std::shared_ptr<T>> scene_modules_;
  std::set<int64_t> registered_module_id_set_;

  autoware::motion_utils::VirtualWallMarkerCreator virtual_wall_marker_creator_;

  rclcpp::Node & node_;
  rclcpp::Clock::SharedPtr clock_;
  // Debug
  bool is_publish_debug_path_ = {false};  // note : this is very heavy debug topic option
  rclcpp::Logger logger_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_virtual_wall_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_;
  rclcpp::Publisher<autoware_internal_planning_msgs::msg::PathWithLaneId>::SharedPtr
    pub_debug_path_;

  std::shared_ptr<DebugPublisher> processing_time_publisher_;

  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr
    pub_processing_time_detail_;

  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;

  std::shared_ptr<planning_factor_interface::PlanningFactorInterface> planning_factor_interface_;

private:
  void appendCommonInfo(std::ostringstream & log, const PlannerData & planner_data)
  {
    if (planner_data.current_odometry) {
      const auto & ego_pose = planner_data.current_odometry->pose;
      const auto & ego_velocity = planner_data.current_velocity;

      log << std::fixed << std::setprecision(2) << "Ego position: (" << ego_pose.position.x << ", "
          << ego_pose.position.y << ", " << ego_pose.position.z << "), ";

      if (ego_velocity) {
        log << "velocity: (" << ego_velocity->twist.linear.x << ", " << ego_velocity->twist.linear.y
            << ") m/s";
        if (planner_data.isVehicleStopped()) {
          log << " (stopped)";
        }
      }
      log << "\n";
    }

    log << "Registered Module IDs: [";
    bool first = true;
    for (const auto & module : scene_modules_) {
      if (!first) {
        log << ", ";
      }
      log << module->getModuleId();
      first = false;
    }
    log << "]\n";
  }

  void printRegistrationInfo(const int64_t module_id, const PlannerData & planner_data)
  {
    std::ostringstream log;

    log << "\n=== BEHAVIOR VELOCITY PLANNER MODULE REGISTRATION ===\n"
        << "Module Name: " << getModuleName() << "\n"
        << "Module ID: " << module_id << "\n";

    appendCommonInfo(log, planner_data);
    log << "========================================================\n";

    RCLCPP_INFO(logger_, "%s", log.str().c_str());
  }

  void printDeletionInfo(
    const std::vector<int64_t> & expired_module_ids, const PlannerData & planner_data)
  {
    std::ostringstream log;

    log << "\n=== BEHAVIOR VELOCITY PLANNER MODULE DELETION ===\n"
        << "Module Name: " << getModuleName() << "\n"
        << "Expired Module IDs: [";

    for (size_t i = 0; i < expired_module_ids.size(); ++i) {
      if (i > 0) log << ", ";
      log << expired_module_ids[i];
    }
    log << "]\n";

    appendCommonInfo(log, planner_data);
    log << "========================================================\n";

    RCLCPP_INFO(logger_, "%s", log.str().c_str());
  }
};

extern template SceneModuleManagerInterface<SceneModuleInterface>::SceneModuleManagerInterface(
  rclcpp::Node & node, [[maybe_unused]] const char * module_name);
extern template void SceneModuleManagerInterface<SceneModuleInterface>::updateSceneModuleInstances(
  const Trajectory & path, const rclcpp::Time & stamp, const PlannerData & planner_data);
extern template void SceneModuleManagerInterface<SceneModuleInterface>::modifyPathVelocity(
  Trajectory & path, const std_msgs::msg::Header & header,
  const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound, const PlannerData & planner_data);
extern template void SceneModuleManagerInterface<SceneModuleInterface>::deleteExpiredModules(
  const Trajectory & path, const PlannerData & planner_data);
extern template void SceneModuleManagerInterface<SceneModuleInterface>::registerModule(
  const std::shared_ptr<SceneModuleInterface> & scene_module, const PlannerData & planner_data);

}  // namespace autoware::behavior_velocity_planner::experimental

#endif  // AUTOWARE__BEHAVIOR_VELOCITY_PLANNER_COMMON__EXPERIMENTAL__SCENE_MODULE_INTERFACE_HPP_
