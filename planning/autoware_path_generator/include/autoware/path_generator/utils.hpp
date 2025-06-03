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

#ifndef AUTOWARE__PATH_GENERATOR__UTILS_HPP_
#define AUTOWARE__PATH_GENERATOR__UTILS_HPP_

#include "autoware/path_generator/common_structs.hpp"
#include "autoware/trajectory/path_point_with_lane_id.hpp"

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>

#include <optional>
#include <utility>
#include <vector>

namespace autoware::path_generator
{
using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;

template <typename T>
struct PathRange
{
  T left;
  T right;
};

namespace utils
{
/**
 * @brief get previous lanelet within route
 * @param lanelet target lanelet
 * @param planner_data planner data
 * @return lanelets in range (std::nullopt if previous lanelet is not found or not
 * within route)
 */
std::optional<lanelet::ConstLanelet> get_previous_lanelet_within_route(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data);

/**
 * @brief get next lanelet within route
 * @param lanelet target lanelet
 * @param planner_data planner data
 * @return lanelets in range (std::nullopt if next lanelet is not found or not
 * within route)
 */
std::optional<lanelet::ConstLanelet> get_next_lanelet_within_route(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data);

/**
 * @brief get path bounds for PathWithLaneId
 * @param path_points path points
 * @param lanelet_sequence lanelet sequence
 * @return path bounds (left / right, std::nullopt if bounds cannot be determined)
 */
std::optional<PathRange<std::vector<geometry_msgs::msg::Point>>> get_path_bounds(
  const std::vector<PathPointWithLaneId> & path_points,
  const lanelet::LaneletSequence & lanelet_sequence);

/**
 * @brief crop line string
 * @param line_string line string
 * @param s_start longitudinal distance to crop from
 * @param s_end longitudinal distance to crop to
 * @return cropped line string
 */
std::vector<geometry_msgs::msg::Point> crop_line_string(
  const lanelet::BasicLineString3d & line_string, const double s_start, const double s_end);

/**
 * @brief get positions of given path point projected to left / right bound in arc length
 * @param lanelet_sequence lanelet sequence
 * @param path_point path point
 * @return longitudinal distance of projected point (left / right)
 */
std::optional<PathRange<double>> get_arc_length_on_bounds(
  const lanelet::LaneletSequence & lanelet_sequence, const PathPointWithLaneId & path_point);

/**
 * @brief Recreate the path with a given goal pose.
 * @param input Input path.
 * @param refined_goal Goal pose.
 * @param planner_data Planner data.
 * @return Recreated path
 */
experimental::trajectory::Trajectory<PathPointWithLaneId> refine_path_for_goal(
  const experimental::trajectory::Trajectory<PathPointWithLaneId> & input,
  const geometry_msgs::msg::Pose & goal_pose, const lanelet::Id goal_lane_id,
  const double search_radius_range, const double pre_goal_offset);

/**
 * @brief Extract lanelets from the trajectory.
 * @param trajectory Input trajectory.
 * @param planner_data Planner data.
 * @return Extracted lanelets
 */
lanelet::ConstLanelets extract_lanelets_from_trajectory(
  const experimental::trajectory::Trajectory<PathPointWithLaneId> & trajectory,
  const PlannerData & planner_data);

/**
 * @brief Check if the pose is in the lanelets.
 * @param pose Pose.
 * @param lanes Lanelets.
 * @return True if the pose is in the lanelets, false otherwise
 */
bool is_in_lanelets(const geometry_msgs::msg::Pose & pose, const lanelet::ConstLanelets & lanes);

/**
 * @brief Check if the trajectory is inside the lanelets.
 * @param refined_path Input trajectory.
 * @param lanelets Lanelets to check against.
 * @return True if the trajectory is inside the lanelets, false otherwise
 */
bool is_trajectory_inside_lanelets(
  const experimental::trajectory::Trajectory<PathPointWithLaneId> & refined_path,
  const lanelet::ConstLanelets & lanelets);

std::optional<experimental::trajectory::Trajectory<PathPointWithLaneId>>
modify_path_for_smooth_goal_connection(
  const experimental::trajectory::Trajectory<PathPointWithLaneId> & trajectory,
  const PlannerData & planner_data, const double search_radius_range, const double pre_goal_offset);

/**
 * @brief get earliest turn signal based on turn direction specified for lanelets
 * @param path target path
 * @param planner_data planner data
 * @param current_pose current pose of ego vehicle
 * @param current_vel current longitudinal velocity of ego vehicle
 * @param search_distance base search distance
 * @param search_time time to extend search distance
 * @param angle_threshold_deg angle threshold for required end point determination
 * @param base_link_to_front distance from base link to front of ego vehicle
 * @return turn signal
 */
TurnIndicatorsCommand get_turn_signal(
  const PathWithLaneId & path, const PlannerData & planner_data,
  const geometry_msgs::msg::Pose & current_pose, const double current_vel,
  const double search_distance, const double search_time, const double angle_threshold_deg,
  const double base_link_to_front);

/**
 * @brief get required end point for turn signal activation
 * @param lanelet target lanelet
 * @param angle_threshold_deg  yaw angle difference threshold
 * @return required end point
 */
std::optional<lanelet::ConstPoint2d> get_turn_signal_required_end_point(
  const lanelet::ConstLanelet & lanelet, const double angle_threshold_deg);
}  // namespace utils
}  // namespace autoware::path_generator

#endif  // AUTOWARE__PATH_GENERATOR__UTILS_HPP_
