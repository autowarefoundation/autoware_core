// Copyright 2026 Autoware Foundation
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

#ifndef MISSION_PLANNER__MISSION_PLANNER_NODE_HPP_
#define MISSION_PLANNER__MISSION_PLANNER_NODE_HPP_

#include "mission_planner.hpp"

#include <autoware/component_interface_specs/planning.hpp>
#include <autoware/component_interface_utils/rclcpp.hpp>
#include <autoware_utils_logging/logger_level_configure.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>

namespace autoware::mission_planner
{
using RouteStateSpecs = autoware::component_interface_specs::planning::RouteState;
using ClearRouteSpecs = autoware::component_interface_specs::planning::ClearRoute;
using SetLaneletRouteSpecs = autoware::component_interface_specs::planning::SetLaneletRoute;
using SetWaypointRouteSpecs = autoware::component_interface_specs::planning::SetWaypointRoute;
using LaneletRouteSpecs = autoware::component_interface_specs::planning::LaneletRoute;
using autoware_planning_msgs::srv::ClearRoute;

class MissionPlannerNode : public rclcpp::Node
{
public:
  explicit MissionPlannerNode(const rclcpp::NodeOptions & options);

private:
  // Publishes the processing time on destruction, regardless of which return path is taken.
  class ScopedProcessingTimePublisher
  {
  public:
    explicit ScopedProcessingTimePublisher(MissionPlannerNode & node) : node_(node) {}
    ~ScopedProcessingTimePublisher() { node_.publish_processing_time(stop_watch_); }

  private:
    MissionPlannerNode & node_;
    autoware_utils_system::StopWatch<std::chrono::milliseconds> stop_watch_;
  };

  void publish_processing_time(
    autoware_utils_system::StopWatch<std::chrono::milliseconds> stop_watch);

  MissionPlanner mission_planner_;

  std::string map_frame_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  autoware::component_interface_utils::NodeAdaptor<rclcpp::Node> adaptor_{this};
  autoware::component_interface_utils::Service<ClearRouteSpecs>::SharedPtr srv_clear_route;
  autoware::component_interface_utils::Service<SetLaneletRouteSpecs>::SharedPtr
    srv_set_lanelet_route;
  autoware::component_interface_utils::Service<SetWaypointRouteSpecs>::SharedPtr
    srv_set_waypoint_route;
  autoware::component_interface_utils::Publisher<RouteStateSpecs>::SharedPtr pub_state_;
  autoware::component_interface_utils::Publisher<LaneletRouteSpecs>::SharedPtr pub_route_;

  rclcpp::Subscription<PoseWithUuidStamped>::SharedPtr sub_modified_goal_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<OperationModeState>::SharedPtr sub_operation_mode_state_;

  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_vector_map_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_goal_footprint_marker_;

  void on_odometry(const Odometry::ConstSharedPtr msg);
  void on_operation_mode_state(const OperationModeState::ConstSharedPtr msg);
  void on_map(const LaneletMapBin::ConstSharedPtr msg);

  void on_clear_route(
    const ClearRoute::Request::SharedPtr req, const ClearRoute::Response::SharedPtr res);
  void on_set_lanelet_route(
    const SetLaneletRoute::Request::SharedPtr req, const SetLaneletRoute::Response::SharedPtr res);
  void on_set_waypoint_route(
    const SetWaypointRoute::Request::SharedPtr req,
    const SetWaypointRoute::Response::SharedPtr res);

  void publish_pose_log(const Pose & pose, const std::string & pose_type);

  rclcpp::TimerBase::SharedPtr data_check_timer_;
  void check_initialization();

  std::unique_ptr<autoware_utils_logging::LoggerLevelConfigure> logger_configure_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    pub_processing_time_;
};

}  // namespace autoware::mission_planner

#endif  // MISSION_PLANNER__MISSION_PLANNER_NODE_HPP_
