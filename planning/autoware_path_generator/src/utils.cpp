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

#include "autoware/path_generator/utils.hpp"

#include "autoware/trajectory/interpolator/linear.hpp"
#include "autoware/trajectory/utils/closest.hpp"
#include "autoware/trajectory/utils/crop.hpp"
#include "autoware/trajectory/utils/find_intervals.hpp"

#include <autoware/motion_utils/constants.hpp>
#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory/forward.hpp>
#include <autoware/trajectory/path_point_with_lane_id.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/unit_conversion.hpp>

#include <autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::path_generator
{
namespace utils
{
namespace
{
const std::unordered_map<std::string, uint8_t> turn_signal_command_map = {
  {"left", TurnIndicatorsCommand::ENABLE_LEFT},
  {"right", TurnIndicatorsCommand::ENABLE_RIGHT},
  {"straight", TurnIndicatorsCommand::DISABLE}};

template <typename T>
bool exists(const std::vector<T> & vec, const T & item)
{
  return std::find(vec.begin(), vec.end(), item) != vec.end();
}

template <typename LineStringT>
std::vector<geometry_msgs::msg::Point> to_geometry_msgs_points(const LineStringT & line_string)
{
  std::vector<geometry_msgs::msg::Point> geometry_msgs_points{};
  geometry_msgs_points.reserve(line_string.size());
  std::transform(
    line_string.begin(), line_string.end(), std::back_inserter(geometry_msgs_points),
    [](const auto & point) { return lanelet::utils::conversion::toGeomMsgPt(point); });
  return geometry_msgs_points;
}
}  // namespace

std::optional<lanelet::ConstLanelet> get_previous_lanelet_within_route(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data)
{
  if (exists(planner_data.start_lanelets, lanelet)) {
    // If the lanelet is a start lanelet, we cannot go backward
    return std::nullopt;
  }

  const auto prev_lanelets = planner_data.routing_graph_ptr->previous(lanelet);
  if (prev_lanelets.empty()) {
    return std::nullopt;
  }

  // Pick the first lanelet that is part of the route
  const auto prev_lanelet_itr = std::find_if(
    prev_lanelets.cbegin(), prev_lanelets.cend(),
    [&](const lanelet::ConstLanelet & l) { return exists(planner_data.route_lanelets, l); });
  if (prev_lanelet_itr == prev_lanelets.cend()) {
    return std::nullopt;
  }
  return *prev_lanelet_itr;
}

std::optional<lanelet::ConstLanelet> get_next_lanelet_within_route(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data)
{
  if (planner_data.preferred_lanelets.empty()) {
    return std::nullopt;
  }

  if (exists(planner_data.goal_lanelets, lanelet)) {
    // If the lanelet is a goal lanelet, we cannot go forward
    return std::nullopt;
  }

  const auto next_lanelets = planner_data.routing_graph_ptr->following(lanelet);
  if (
    next_lanelets.empty() ||
    next_lanelets.front().id() == planner_data.preferred_lanelets.front().id()) {
    return std::nullopt;
  }

  // Pick the first lanelet that is part of the route and not in a loop
  const auto next_lanelet_itr = std::find_if(
    next_lanelets.cbegin(), next_lanelets.cend(),
    [&](const lanelet::ConstLanelet & l) { return exists(planner_data.route_lanelets, l); });
  if (next_lanelet_itr == next_lanelets.cend()) {
    return std::nullopt;
  }
  return *next_lanelet_itr;
}

double get_arc_length_on_path(
  const lanelet::LaneletSequence & lanelet_sequence,
  const experimental::trajectory::Trajectory<PathPointWithLaneId> & path, const double s_centerline)
{
  std::optional<lanelet::Id> target_lanelet_id = std::nullopt;
  std::optional<geometry_msgs::msg::Point> point_on_centerline = std::nullopt;

  if (lanelet_sequence.empty()) {
    RCLCPP_WARN(
      rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
      "Input lanelet sequence is empty, returning 0.");
    return 0.;
  }

  if (s_centerline < 0.) {
    RCLCPP_WARN(
      rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
      "Input arc length is negative, returning 0.");
    return 0.;
  }

  for (auto [it, s] = std::make_tuple(lanelet_sequence.begin(), 0.); it != lanelet_sequence.end();
       ++it) {
    const double centerline_length = lanelet::geometry::length(it->centerline2d());
    if (s + centerline_length < s_centerline) {
      s += centerline_length;
      continue;
    }

    target_lanelet_id = it->id();
    const auto lanelet_point_on_centerline =
      lanelet::geometry::interpolatedPointAtDistance(it->centerline2d(), s_centerline - s);
    point_on_centerline = lanelet::utils::conversion::toGeomMsgPt(
      Eigen::Vector3d{lanelet_point_on_centerline.x(), lanelet_point_on_centerline.y(), 0.});
    break;
  }

  if (!target_lanelet_id || !point_on_centerline) {
    RCLCPP_WARN(
      rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
      "No lanelet found for input arc length, returning input as is");
    return s_centerline;
  }

  const auto s_path = autoware::experimental::trajectory::closest_with_constraint(
    path, *point_on_centerline,
    [&](const PathPointWithLaneId & point) { return exists(point.lane_ids, *target_lanelet_id); });
  if (s_path) {
    return *s_path;
  }

  RCLCPP_WARN(
    rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
    "Path does not contain point with target lane id, falling back to constraint-free closest "
    "point search");

  const auto s_path_free = autoware::experimental::trajectory::closest(path, *point_on_centerline);

  return s_path_free;
}

PathRange<std::vector<geometry_msgs::msg::Point>> get_path_bounds(
  const lanelet::LaneletSequence & lanelet_sequence, const double s_start, const double s_end)
{
  if (lanelet_sequence.empty()) {
    RCLCPP_WARN(
      rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
      "Input lanelet sequence is empty");
    return {};
  }

  const auto [s_left_start, s_right_start] = get_arc_length_on_bounds(lanelet_sequence, s_start);
  const auto [s_left_end, s_right_end] = get_arc_length_on_bounds(lanelet_sequence, s_end);

  const auto left_path_bound = build_cropped_trajectory(
    to_geometry_msgs_points(lanelet_sequence.leftBound()), s_left_start, s_left_end);
  const auto right_path_bound = build_cropped_trajectory(
    to_geometry_msgs_points(lanelet_sequence.rightBound()), s_right_start, s_right_end);
  if (!left_path_bound || !right_path_bound) {
    RCLCPP_WARN(
      rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
      "Failed to get path bounds");
    return {};
  }

  return {left_path_bound->restore(), right_path_bound->restore()};
}

std::optional<autoware::experimental::trajectory::Trajectory<geometry_msgs::msg::Point>>
build_cropped_trajectory(
  const std::vector<geometry_msgs::msg::Point> & line_string, const double s_start,
  const double s_end)
{
  if (s_start < 0.) {
    RCLCPP_WARN(
      rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
      "Start of crop range is negative");
    return std::nullopt;
  }

  if (s_start > s_end) {
    RCLCPP_WARN(
      rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
      "Start of crop range is larger than end");
    return std::nullopt;
  }

  auto trajectory =
    autoware::experimental::trajectory::Trajectory<geometry_msgs::msg::Point>::Builder()
      .set_xy_interpolator<autoware::experimental::trajectory::interpolator::Linear>()
      .build(line_string);
  if (!trajectory) {
    RCLCPP_WARN(
      rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
      "Failed to build trajectory from line string");
    return std::nullopt;
  }

  trajectory->crop(s_start, s_end - s_start);
  return *trajectory;
}

PathRange<double> get_arc_length_on_bounds(
  const lanelet::LaneletSequence & lanelet_sequence, const double s_centerline)
{
  if (s_centerline < 0.) {
    RCLCPP_WARN(
      rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
      "Input arc length is negative, returning 0.");
    return {0., 0.};
  }

  auto s = 0.;
  auto s_left = 0.;
  auto s_right = 0.;

  for (auto it = lanelet_sequence.begin(); it != lanelet_sequence.end(); ++it) {
    const double centerline_length = lanelet::geometry::length(it->centerline2d());
    const double left_bound_length = lanelet::geometry::length(it->leftBound2d());
    const double right_bound_length = lanelet::geometry::length(it->rightBound2d());

    if (s + centerline_length < s_centerline) {
      s += centerline_length;
      s_left += left_bound_length;
      s_right += right_bound_length;
      continue;
    }

    const auto point_on_centerline =
      lanelet::geometry::interpolatedPointAtDistance(it->centerline2d(), s_centerline - s);
    s_left += lanelet::geometry::toArcCoordinates(it->leftBound2d(), point_on_centerline).length;
    s_right += lanelet::geometry::toArcCoordinates(it->rightBound2d(), point_on_centerline).length;

    return {s_left, s_right};
  }

  // If the loop ends without returning, it means that the lanelet_sequence is too short.
  // In this case, we return the original arc length on the centerline.
  return {s_centerline, s_centerline};
}

std::optional<experimental::trajectory::Trajectory<PathPointWithLaneId>>
connect_path_to_goal_inside_lanelet_sequence(
  const experimental::trajectory::Trajectory<PathPointWithLaneId> & path,
  const lanelet::LaneletSequence & lanelet_sequence, const geometry_msgs::msg::Pose & goal_pose,
  const lanelet::ConstLanelet & goal_lanelet, const double s_goal, const PlannerData & planner_data,
  const double connection_section_length, const double pre_goal_offset)
{
  if (lanelet_sequence.empty()) {
    RCLCPP_WARN(
      rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
      "Input lanelet sequence is empty");
    return std::nullopt;
  }

  // TODO(mitukou1109): make delta configurable
  constexpr auto connection_section_length_delta = 0.1;
  for (auto m = connection_section_length; m > 0.0; m -= connection_section_length_delta) {
    auto path_to_goal = connect_path_to_goal(
      path, lanelet_sequence, goal_pose, goal_lanelet, s_goal, planner_data, m, pre_goal_offset);
    if (!is_path_inside_lanelets(path_to_goal, lanelet_sequence.lanelets())) {
      continue;
    }
    path_to_goal.align_orientation_with_trajectory_direction();
    return path_to_goal;
  }
  return std::nullopt;
}

experimental::trajectory::Trajectory<PathPointWithLaneId> connect_path_to_goal(
  const experimental::trajectory::Trajectory<PathPointWithLaneId> & path,
  const lanelet::LaneletSequence & lanelet_sequence, const geometry_msgs::msg::Pose & goal_pose,
  const lanelet::ConstLanelet & goal_lanelet, const double s_goal, const PlannerData & planner_data,
  const double connection_section_length, const double pre_goal_offset)
{
  if (goal_lanelet.id() == lanelet::InvalId) {
    RCLCPP_WARN(
      rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
      "Input goal lanelet is invalid, returning input path as is");
    return path;
  }

  const auto pre_goal_pose =
    autoware_utils_geometry::calc_offset_pose(goal_pose, -pre_goal_offset, 0.0, 0.0);
  auto pre_goal_lanelet = goal_lanelet;
  while (rclcpp::ok() && !lanelet::utils::isInLanelet(pre_goal_pose, pre_goal_lanelet)) {
    const auto prev_lanelet = get_previous_lanelet_within_route(pre_goal_lanelet, planner_data);
    if (!prev_lanelet) {
      RCLCPP_WARN(
        rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
        "Pre-goal is outside route, returning input path as is");
      return path;
    }
    pre_goal_lanelet = *prev_lanelet;
  }

  const auto s_pre_goal =
    autoware::experimental::trajectory::closest_with_constraint(
      path, pre_goal_pose,
      [&](const PathPointWithLaneId & point) {
        return exists(point.lane_ids, pre_goal_lanelet.id());
      })
      .value_or(autoware::experimental::trajectory::closest(path, pre_goal_pose));

  PathPointWithLaneId pre_goal;
  pre_goal.lane_ids = {pre_goal_lanelet.id()};
  pre_goal.point.pose = pre_goal_pose;
  pre_goal.point.longitudinal_velocity_mps =
    path.compute(s_pre_goal).point.longitudinal_velocity_mps;

  std::vector<PathPointWithLaneId> path_points_to_goal;

  if (s_goal <= connection_section_length) {
    // If distance from start to goal is smaller than connection_section_length and start is
    // farther from goal than pre-goal, we just connect start, pre-goal, and goal.
    path_points_to_goal = {path.compute(0)};
  } else {
    const auto s_connection_section_start =
      get_arc_length_on_path(lanelet_sequence, path, s_goal - connection_section_length);
    const auto cropped_path =
      autoware::experimental::trajectory::crop(path, 0, s_connection_section_start);
    path_points_to_goal = cropped_path.restore(1);
  }

  if (s_goal > pre_goal_offset) {
    // add pre-goal only if goal is farther than pre-goal
    path_points_to_goal.push_back(pre_goal);
  }

  PathPointWithLaneId goal;
  goal.lane_ids = {goal_lanelet.id()};
  goal.point.pose = goal_pose;
  goal.point.longitudinal_velocity_mps = 0.0;
  path_points_to_goal.push_back(goal);

  if (const auto output = autoware::experimental::trajectory::pretty_build(path_points_to_goal)) {
    return *output;
  }

  RCLCPP_WARN(
    rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
    "Failed to connect path to goal, returning input path as is");
  return path;
}

bool is_pose_inside_lanelets(
  const geometry_msgs::msg::Pose & pose, const lanelet::ConstLanelets & lanelets)
{
  return std::any_of(lanelets.begin(), lanelets.end(), [&](const lanelet::ConstLanelet & l) {
    return lanelet::utils::isInLanelet(pose, l);
  });
}

bool is_path_inside_lanelets(
  const experimental::trajectory::Trajectory<PathPointWithLaneId> & path,
  const lanelet::ConstLanelets & lanelets)
{
  for (double s = 0.0; s < path.length(); s += 0.1) {
    const auto point = path.compute(s);
    if (!is_pose_inside_lanelets(point.point.pose, lanelets)) {
      return false;
    }
  }
  return true;
}

TurnIndicatorsCommand get_turn_signal(
  const PathWithLaneId & path, const PlannerData & planner_data,
  const geometry_msgs::msg::Pose & current_pose, const double current_vel,
  const double search_distance, const double search_time, const double angle_threshold_deg,
  const double base_link_to_front)
{
  TurnIndicatorsCommand turn_signal;
  turn_signal.command = TurnIndicatorsCommand::NO_COMMAND;

  const lanelet::BasicPoint2d current_point{current_pose.position.x, current_pose.position.y};
  const auto base_search_distance = search_distance + current_vel * search_time;

  const auto calc_arc_length =
    [&](const lanelet::ConstLanelet & lanelet, const lanelet::BasicPoint2d & point) -> double {
    return lanelet::geometry::toArcCoordinates(lanelet.centerline2d(), point).length;
  };

  std::vector<lanelet::Id> searched_lanelet_ids = {};

  // arc length from vehicle front to start of first lanelet with turn signal
  std::optional<double> arc_length_from_vehicle_front_to_lanelet_start = std::nullopt;

  for (const auto & point : path.points) {
    for (const auto & lane_id : point.lane_ids) {
      if (exists(searched_lanelet_ids, lane_id)) {
        // Skip already searched lanelets
        continue;
      }
      searched_lanelet_ids.push_back(lane_id);

      const auto lanelet = planner_data.lanelet_map_ptr->laneletLayer.get(lane_id);
      if (!get_next_lanelet_within_route(lanelet, planner_data)) {
        // If lanelet is at end of route, no need to publish turn signal
        continue;
      }

      if (
        !arc_length_from_vehicle_front_to_lanelet_start &&
        !lanelet::geometry::inside(lanelet, current_point)) {
        // Skip until we find first lanelet that contains current point
        continue;
      }

      if (lanelet.hasAttribute("turn_direction")) {
        turn_signal.command =
          turn_signal_command_map.at(lanelet.attribute("turn_direction").value());

        if (arc_length_from_vehicle_front_to_lanelet_start) {
          // Ego is in front of lanelet with turn signal
          if (
            *arc_length_from_vehicle_front_to_lanelet_start >
            lanelet.attributeOr("turn_signal_distance", base_search_distance)) {
            // Ego is still too far from lanelet to publish turn signal
            turn_signal.command = TurnIndicatorsCommand::NO_COMMAND;
          }
          return turn_signal;
        }

        // Ego is inside lanelet with turn signal
        const auto required_end_point =
          get_turn_signal_required_end_point(lanelet, angle_threshold_deg);
        if (!required_end_point) {
          continue;
        }
        if (
          calc_arc_length(lanelet, current_point) <=
          calc_arc_length(lanelet, *required_end_point)) {
          // Ego is in front of required end point, so we continue to publish current turn signal
          return turn_signal;
        }
      }

      const auto lanelet_length = lanelet::utils::getLaneletLength2d(lanelet);
      if (arc_length_from_vehicle_front_to_lanelet_start) {
        // Accumulate lanelet length
        *arc_length_from_vehicle_front_to_lanelet_start += lanelet_length;
      } else {
        // Initialize arc length from vehicle front to lanelet start
        arc_length_from_vehicle_front_to_lanelet_start =
          lanelet_length - calc_arc_length(lanelet, current_point) - base_link_to_front;
      }
      break;
    }
  }

  return turn_signal;
}

std::optional<lanelet::ConstPoint2d> get_turn_signal_required_end_point(
  const lanelet::ConstLanelet & lanelet, const double angle_threshold_deg)
{
  // Convert lanelet centerline to set of geometry_msgs::msg::Pose for autoware_trajectory
  std::vector<geometry_msgs::msg::Pose> centerline_poses(lanelet.centerline().size());
  std::transform(
    lanelet.centerline().begin(), lanelet.centerline().end(), centerline_poses.begin(),
    [](const auto & point) {
      geometry_msgs::msg::Pose pose{};
      pose.position = lanelet::utils::conversion::toGeomMsgPt(point);
      return pose;
    });

  // Trajectory cannot be built from less than 4 points, so we resample centerline if necessary.
  // This implementation should be replaced once pretty_build() supports geometry_msgs::msg::Pose.
  if (centerline_poses.size() < 4) {
    const auto lanelet_length = autoware::motion_utils::calcArcLength(centerline_poses);
    const auto resampling_interval = lanelet_length / 4.0;
    std::vector<double> resampled_arclength;
    for (double s = 0.0; s < lanelet_length; s += resampling_interval) {
      resampled_arclength.push_back(s);
    }
    if (
      !resampled_arclength.empty() &&
      lanelet_length - resampled_arclength.back() < autoware::motion_utils::overlap_threshold) {
      resampled_arclength.back() = lanelet_length;
    } else {
      resampled_arclength.push_back(lanelet_length);
    }
    centerline_poses =
      autoware::motion_utils::resamplePoseVector(centerline_poses, resampled_arclength);
    if (centerline_poses.size() < 4) {
      RCLCPP_ERROR(
        rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
        "Failed to resample centerline");
      return std::nullopt;
    }
  }

  // Build trajectory from centerline poses
  auto centerline =
    autoware::experimental::trajectory::Trajectory<geometry_msgs::msg::Pose>::Builder{}.build(
      centerline_poses);
  if (!centerline) {
    return std::nullopt;
  }
  centerline->align_orientation_with_trajectory_direction();

  // Find intervals where driving direction is close to terminal yaw by angle_threshold_deg
  const auto terminal_yaw = tf2::getYaw(centerline->compute(centerline->length()).orientation);
  const auto intervals = autoware::experimental::trajectory::find_intervals(
    *centerline, [terminal_yaw, angle_threshold_deg](const geometry_msgs::msg::Pose & point) {
      const auto yaw = tf2::getYaw(point.orientation);
      return std::abs(autoware_utils::normalize_radian(yaw - terminal_yaw)) <
             autoware_utils::deg2rad(angle_threshold_deg);
    });
  if (intervals.empty()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
      "Failed to find intervals with driving direction close to terminal yaw");
    return std::nullopt;
  }

  // Return first point where driving direction difference is below threshold as required end point
  return lanelet::utils::conversion::toLaneletPoint(
    centerline->compute(intervals.front().start).position);
}
}  // namespace utils
}  // namespace autoware::path_generator
