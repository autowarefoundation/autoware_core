// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__PATH_GENERATOR__NODE_HPP_
#define AUTOWARE__PATH_GENERATOR__NODE_HPP_

#include "autoware/path_generator/common_structs.hpp"

#include <autoware/trajectory/path_point_with_lane_id.hpp>
#include <autoware_path_generator/path_generator_parameters.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>

namespace autoware::path_generator
{
using autoware_internal_debug_msgs::msg::Float64Stamped;
using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_vehicle_msgs::msg::HazardLightsCommand;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using nav_msgs::msg::Odometry;
using ::path_generator::Params;
using Trajectory = autoware::experimental::trajectory::Trajectory<PathPointWithLaneId>;

class PathGenerator : public rclcpp::Node
{
public:
  explicit PathGenerator(const rclcpp::NodeOptions & node_options);

  // NOTE: This is for the static_centerline_generator package which utilizes the following
  // instance.
  struct InputData
  {
    LaneletRoute::ConstSharedPtr route_ptr{nullptr};
    LaneletMapBin::ConstSharedPtr lanelet_map_bin_ptr{nullptr};
    Odometry::ConstSharedPtr odometry_ptr{nullptr};
  };
  bool is_data_ready(const InputData & input_data);
  void set_planner_data(const InputData & input_data);
  std::optional<PathWithLaneId> generate_path(
    const geometry_msgs::msg::Pose & current_pose, const Params & params);

private:
  // subscriber
  autoware_utils::InterProcessPollingSubscriber<
    LaneletRoute, autoware_utils::polling_policy::Newest>
    route_subscriber_{this, "~/input/route", rclcpp::QoS{1}.transient_local()};
  autoware_utils::InterProcessPollingSubscriber<
    LaneletMapBin, autoware_utils::polling_policy::Newest>
    vector_map_subscriber_{this, "~/input/vector_map", rclcpp::QoS{1}.transient_local()};
  autoware_utils::InterProcessPollingSubscriber<Odometry> odometry_subscriber_{
    this, "~/input/odometry"};

  // publisher
  rclcpp::Publisher<PathWithLaneId>::SharedPtr path_publisher_;
  rclcpp::Publisher<TurnIndicatorsCommand>::SharedPtr turn_signal_publisher_;
  rclcpp::Publisher<HazardLightsCommand>::SharedPtr hazard_signal_publisher_;
  rclcpp::Publisher<Float64Stamped>::SharedPtr debug_calculation_time_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<::path_generator::ParamListener> param_listener_;

  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  PlannerData planner_data_;

  std::optional<lanelet::ConstLanelet> current_lanelet_{std::nullopt};

  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_{nullptr};

  autoware_utils_system::StopWatch<std::chrono::milliseconds> stop_watch_;

  void run();

  InputData take_data();

  void set_route(const LaneletRoute::ConstSharedPtr & route_ptr);

  std::optional<PathWithLaneId> plan_path(const InputData & input_data, const Params & params);

  std::optional<PathWithLaneId> generate_path(
    const lanelet::LaneletSequence & lanelet_sequence, const double s_start, const double s_end,
    const std::optional<lanelet::ConstLanelet> & goal_lanelet_for_path,
    const Params & params) const;

  bool update_current_lanelet(const geometry_msgs::msg::Pose & current_pose, const Params & params);

  void publishStopWatchTime();
};
}  // namespace autoware::path_generator

#endif  // AUTOWARE__PATH_GENERATOR__NODE_HPP_
