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

#ifndef EXPERIMENTAL__SCENE_HPP_
#define EXPERIMENTAL__SCENE_HPP_

#define EIGEN_MPL2_ONLY

#include <autoware/behavior_velocity_planner_common/experimental/scene_module_interface.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
class StopLineModule : public SceneModuleInterface
{
public:
  using StopLineWithLaneId = std::pair<lanelet::ConstLineString3d, int64_t>;

  enum class State { APPROACH, STOPPED, START };

  struct DebugData
  {
    double base_link2front;  ///< Distance from the base link to the vehicle front.
    std::optional<geometry_msgs::msg::Pose> stop_pose;  ///< Pose of the stop position.
  };

  struct PlannerParam
  {
    double stop_margin;                         ///< Margin to the stop line.
    double required_stop_duration_sec;          ///< Required stop duration at the stop line.
    double vehicle_stopped_duration_threshold;  ///< Duration threshold for determining if the
                                                ///< vehicle is stopped.
    double
      hold_stop_margin_distance;  ///< Distance threshold for transitioning to the STOPPED state
  };

  /**
   * @brief Constructor for StopLineModule.
   * @param module_id Unique ID for the module.
   * @param stop_line Stop line data.
   * @param linked_lanelet_id ID of the linked lanelet.
   * @param planner_param Planning parameters.
   * @param logger Logger for output messages.
   * @param clock Shared clock instance.
   * @param time_keeper Time keeper for the module.
   * @param planning_factor_interface Planning factor interface.
   */
  StopLineModule(
    const int64_t module_id,                                                //
    const lanelet::ConstLineString3d & stop_line,                           //
    const lanelet::Id & linked_lanelet_id,                                  //
    const PlannerParam & planner_param,                                     //
    const rclcpp::Logger & logger,                                          //
    const rclcpp::Clock::SharedPtr clock,                                   //
    const std::shared_ptr<autoware_utils_debug::TimeKeeper> & time_keeper,  //
    const std::shared_ptr<planning_factor_interface::PlanningFactorInterface> &
      planning_factor_interface);

  bool modifyPathVelocity(
    Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound,
    const PlannerData & planner_data) override;

  /**
   * @brief Calculate ego position and stop point.
   * @param path Current path.
   * @param left_bound Left bound of the path.
   * @param right_bound Right bound of the path.
   * @param ego_pose Current pose of the vehicle.
   * @param state Current state of the stop line module.
   * @return Pair of ego position and optional stop point.
   */
  std::pair<double, std::optional<double>> getEgoAndStopPoint(
    const Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound,
    const geometry_msgs::msg::Pose & ego_pose, const PlannerData & planner_data) const;

  /**
   * @brief Update the state and stopped time of the module.
   * @param now Current time.
   * @param distance_to_stop_point Distance to the stop point.
   * @param is_vehicle_stopped Flag indicating if the vehicle is stopped.
   */
  void updateStateAndStoppedTime(
    const rclcpp::Time & now, const double & distance_to_stop_point,
    const bool & is_vehicle_stopped);

  void updateDebugData(
    const geometry_msgs::msg::Pose & stop_pose, const PlannerData & planner_data);

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override
  {
    return visualization_msgs::msg::MarkerArray{};
  }

  autoware::motion_utils::VirtualWalls createVirtualWalls() override;

  std::vector<int64_t> getLaneletIds() const override { return {linked_lanelet_id_}; }
  std::vector<int64_t> getLineIds() const override { return {stop_line_.id()}; }

private:
  const lanelet::ConstLineString3d stop_line_;  ///< Stop line geometry.
  const lanelet::Id linked_lanelet_id_;         ///< ID of the linked lanelet.
  const PlannerParam planner_param_;            ///< Parameters for the planner.
  State state_;                                 ///< Current state of the module.
  std::optional<rclcpp::Time> stopped_time_;    ///< Time when the vehicle stopped.
  DebugData debug_data_;                        ///< Debug information.
};
}  // namespace autoware::behavior_velocity_planner::experimental

#endif  // EXPERIMENTAL__SCENE_HPP_
