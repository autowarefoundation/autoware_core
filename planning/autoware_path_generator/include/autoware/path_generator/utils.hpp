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

struct WaypointGroup
{
  struct Waypoint
  {
    lanelet::ConstPoint3d point;
    lanelet::Id lane_id;
  };

  struct Interval
  {
    double start;
    double end;
  };

  std::vector<Waypoint> waypoints;
  Interval interval;
};

template <typename T>
struct PathRange
{
  T left;
  T right;
};

namespace utils
{
/**
 * @brief get lanelets within route that are in specified distance backward from target
 * lanelet
 * @param lanelet target lanelet
 * @param planner_data planner data
 * @param distance backward distance from beginning of target lanelet
 * @return lanelets in range (std::nullopt if target lanelet is not within route)
 */
std::optional<lanelet::ConstLanelets> get_lanelets_within_route_up_to(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data, const double distance);

/**
 * @brief get lanelets within route that are in specified distance forward from target
 * lanelet
 * @param lanelet target lanelet
 * @param planner_data planner data
 * @param distance forward distance from end of target lanelet
 * @return lanelets in range (std::nullopt if target lanelet is not within route)
 */
std::optional<lanelet::ConstLanelets> get_lanelets_within_route_after(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data, const double distance);

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
 * @brief get waypoints in lanelet sequence and group them
 * @param lanelet_sequence lanelet sequence
 * @param lanelet_map lanelet map to get waypoints
 * @param group_separation_threshold maximum distance between waypoints to belong to same
 * group (see figure in README)
 * @param interval_margin_ratio ratio to expand interval bound of group according to the
 * lateral distance of first and last point of group
 * @return waypoint groups
 */
std::vector<WaypointGroup> get_waypoint_groups(
  const lanelet::LaneletSequence & lanelet_sequence, const lanelet::LaneletMap & lanelet_map,
  const double group_separation_threshold, const double interval_margin_ratio);

/**
 * @brief get position of first intersection (including self-intersection) in lanelet sequence in
 * arc length
 * @param lanelet_sequence target lanelet sequence
 * @param s_start longitudinal distance of point to start searching for intersections
 * @param s_end longitudinal distance of point to end search
 * @param vehicle_length vehicle length
 * @return longitudinal distance of intersecting point (std::nullopt if no intersection)
 */
std::optional<double> get_first_intersection_arc_length(
  const lanelet::LaneletSequence & lanelet_sequence, const double s_start, const double s_end,
  const double vehicle_length);

/**
 * @brief get position of first self-intersection (point where return
 * path intersects outward path) of line string in arc length
 * @param line_string target line string
 * @return longitudinal distance of self-intersecting point (std::nullopt if no
 * self-intersection)
 */
std::optional<double> get_first_self_intersection_arc_length(
  const lanelet::BasicLineString2d & line_string);

/**
 * @brief get position of given point on centerline projected to path in arc length
 * @param lanelet_sequence lanelet sequence
 * @param path target path
 * @param s_centerline longitudinal distance of point on centerline
 * @return longitudinal distance of projected point
 */
double get_arc_length_on_path(
  const lanelet::LaneletSequence & lanelet_sequence, const std::vector<PathPointWithLaneId> & path,
  const double s_centerline);

/**
 * @brief get path bounds for PathWithLaneId cropped within specified range
 * @param lanelet_sequence lanelet sequence
 * @param s_start longitudinal distance of start of bound on centerline
 * @param s_end longitudinal distance of end of bound on centerline
 * @return cropped bounds (left / right)
 */
PathRange<std::vector<geometry_msgs::msg::Point>> get_path_bounds(
  const lanelet::LaneletSequence & lanelet_sequence, const double s_start, const double s_end);

/**
 * @brief crop line string
 * @param line_string line string
 * @param s_start longitudinal distance to crop from
 * @param s_end longitudinal distance to crop to
 * @return cropped line string
 */
std::vector<geometry_msgs::msg::Point> crop_line_string(
  const std::vector<geometry_msgs::msg::Point> & line_string, const double s_start,
  const double s_end);

/**
 * @brief get positions of given point on centerline projected to left / right bound in arc length
 * @param lanelet_sequence lanelet sequence
 * @param s_centerline longitudinal distance of point on centerline
 * @return longitudinal distance of projected point (left / right)
 */
PathRange<double> get_arc_length_on_bounds(
  const lanelet::LaneletSequence & lanelet_sequence, const double s_centerline);

/**
 * @brief get positions of given point on left / right bound projected to centerline in arc length
 * @param lanelet_sequence lanelet sequence
 * @param s_left_bound longitudinal distance of point on left bound
 * @param s_right_bound longitudinal distance of point on left bound
 * @return longitudinal distance of projected point (left / right)
 */
PathRange<std::optional<double>> get_arc_length_on_centerline(
  const lanelet::LaneletSequence & lanelet_sequence, const std::optional<double> & s_left_bound,
  const std::optional<double> & s_right_bound);

/**
 * @brief Recreate the goal pose to prevent the goal point being too far from the lanelet, which
 *  causes the path to twist near the goal.
 * @details Return the goal point projected on the straight line of the segment of lanelet
 *  closest to the original goal.
 * @param [in] goal original goal pose
 * @param [in] goal_lanelet lanelet containing the goal pose
 */
geometry_msgs::msg::Pose refine_goal(
  const geometry_msgs::msg::Pose & goal, const lanelet::ConstLanelet & goal_lanelet);

/**
 * @brief Prepare the point before the goal point.
 * @param goal Goal pose.
 * @param lanes Lanelets.
 * @return Pre-goal point.
 */
PathPointWithLaneId prepare_pre_goal(
  const geometry_msgs::msg::Pose & goal, const lanelet::ConstLanelets & lanes);

/**
 * @brief Get the index of the point closest to the circumference of the circle whose center is the
 * goal and outside of it.
 * @param points Points to search.
 * @param goal Goal pose.
 * @param goal_lane_id Lane ID of the goal.
 * @param max_dist Maximum distance to search.
 * @return Index of the point closest to the circumference of the circle whose center is the goal
 * and outside of it.
 */
std::optional<size_t> find_index_out_of_goal_search_range(
  const std::vector<PathPointWithLaneId> & points, const geometry_msgs::msg::Pose & goal,
  const int64_t goal_lane_id, const double max_dist);

/**
 * @brief Get the path up to just before the pre_goal.
 * @param input Input path.
 * @param refined_goal Goal pose.
 * @return Recreated path
 */
std::optional<PathWithLaneId> get_path_up_to_just_before_pre_goal(
  const PathWithLaneId & input, const geometry_msgs::msg::Pose & goal,
  const lanelet::Id goal_lane_id, const double search_radius_range);

/**
 * @brief Recreate the path with a given goal pose.
 * @param input Input path.
 * @param refined_goal Goal pose.
 * @param planner_data Planner data.
 * @return Recreated path
 */
PathWithLaneId refine_path_for_goal(
  const PathWithLaneId & input, const geometry_msgs::msg::Pose & goal,
  const PlannerData & planner_data);

/**
 * @brief Extract lanelets from the path.
 * @param path Input path.
 * @param planner_data Planner data.
 * @return Extracted lanelets
 */
std::optional<lanelet::ConstLanelets> extract_lanelets_from_path(
  const PathWithLaneId & refined_path, const PlannerData & planner_data);

/**
 * @brief Check if the pose is in the lanelets.
 * @param pose Pose.
 * @param lanes Lanelets.
 * @return True if the pose is in the lanelets, false otherwise
 */
bool is_in_lanelets(const geometry_msgs::msg::Pose & pose, const lanelet::ConstLanelets & lanes);

std::optional<experimental::trajectory::Trajectory<PathPointWithLaneId>>
modify_path_for_smooth_goal_connection(
  const experimental::trajectory::Trajectory<PathPointWithLaneId> & trajectory,
  const PlannerData & planner_data, const double refine_goal_search_radius_range);

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
