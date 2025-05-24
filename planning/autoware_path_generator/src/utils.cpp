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

template <typename const_iterator>
std::vector<geometry_msgs::msg::Point> to_geometry_msgs_points(
  const const_iterator begin, const const_iterator end)
{
  std::vector<geometry_msgs::msg::Point> geometry_msgs_points{};
  geometry_msgs_points.reserve(std::distance(begin, end));
  std::transform(begin, end, std::back_inserter(geometry_msgs_points), [](const auto & point) {
    return lanelet::utils::conversion::toGeomMsgPt(point);
  });
  return geometry_msgs_points;
}

lanelet::BasicPoints3d to_lanelet_points(
  const std::vector<geometry_msgs::msg::Point> & geometry_msgs_points)
{
  lanelet::BasicPoints3d lanelet_points{};
  lanelet_points.reserve(geometry_msgs_points.size());
  std::transform(
    geometry_msgs_points.begin(), geometry_msgs_points.end(), std::back_inserter(lanelet_points),
    [](const auto & point) { return lanelet::utils::conversion::toLaneletPoint(point); });
  return lanelet_points;
}
}  // namespace

std::optional<lanelet::ConstLanelets> get_lanelets_within_route_up_to(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data, const double distance)
{
  if (!exists(planner_data.route_lanelets, lanelet)) {
    return std::nullopt;
  }

  lanelet::ConstLanelets lanelets{};
  auto current_lanelet = lanelet;
  auto length = 0.;

  while (rclcpp::ok() && length < distance) {
    const auto prev_lanelet = get_previous_lanelet_within_route(current_lanelet, planner_data);
    if (!prev_lanelet) {
      break;
    }

    lanelets.push_back(*prev_lanelet);
    current_lanelet = *prev_lanelet;
    length += lanelet::utils::getLaneletLength2d(*prev_lanelet);
  }

  std::reverse(lanelets.begin(), lanelets.end());
  return lanelets;
}

std::optional<lanelet::ConstLanelets> get_lanelets_within_route_after(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data, const double distance)
{
  if (!exists(planner_data.route_lanelets, lanelet)) {
    return std::nullopt;
  }

  lanelet::ConstLanelets lanelets{};
  auto current_lanelet = lanelet;
  auto length = 0.;

  while (rclcpp::ok() && length < distance) {
    const auto next_lanelet = get_next_lanelet_within_route(current_lanelet, planner_data);
    if (!next_lanelet) {
      break;
    }

    lanelets.push_back(*next_lanelet);
    current_lanelet = *next_lanelet;
    length += lanelet::utils::getLaneletLength2d(*next_lanelet);
  }

  return lanelets;
}

std::optional<lanelet::ConstLanelet> get_previous_lanelet_within_route(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data)
{
  if (exists(planner_data.start_lanelets, lanelet)) {
    return std::nullopt;
  }

  const auto prev_lanelets = planner_data.routing_graph_ptr->previous(lanelet);
  if (prev_lanelets.empty()) {
    return std::nullopt;
  }

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
    return std::nullopt;
  }

  const auto next_lanelets = planner_data.routing_graph_ptr->following(lanelet);
  if (
    next_lanelets.empty() ||
    next_lanelets.front().id() == planner_data.preferred_lanelets.front().id()) {
    return std::nullopt;
  }

  const auto next_lanelet_itr = std::find_if(
    next_lanelets.cbegin(), next_lanelets.cend(),
    [&](const lanelet::ConstLanelet & l) { return exists(planner_data.route_lanelets, l); });
  if (next_lanelet_itr == next_lanelets.cend()) {
    return std::nullopt;
  }
  return *next_lanelet_itr;
}

std::vector<WaypointGroup> get_waypoint_groups(
  const lanelet::LaneletSequence & lanelet_sequence, const lanelet::LaneletMap & lanelet_map,
  const double group_separation_threshold, const double interval_margin_ratio)
{
  std::vector<WaypointGroup> waypoint_groups{};

  const auto get_interval_bound =
    [&](const lanelet::ConstPoint3d & point, const double lateral_distance_factor) {
      const auto arc_coordinates = lanelet::geometry::toArcCoordinates(
        lanelet_sequence.centerline2d(), lanelet::utils::to2D(point));
      return arc_coordinates.length + lateral_distance_factor * std::abs(arc_coordinates.distance);
    };

  for (const auto & lanelet : lanelet_sequence) {
    if (!lanelet.hasAttribute("waypoints")) {
      continue;
    }

    const auto waypoints_id = lanelet.attribute("waypoints").asId().value();
    const auto & waypoints = lanelet_map.lineStringLayer.get(waypoints_id);

    if (
      waypoint_groups.empty() || lanelet::geometry::distance2d(
                                   waypoint_groups.back().waypoints.back().point,
                                   waypoints.front()) > group_separation_threshold) {
      waypoint_groups.emplace_back().interval.start =
        get_interval_bound(waypoints.front(), -interval_margin_ratio);
    }
    waypoint_groups.back().interval.end =
      get_interval_bound(waypoints.back(), interval_margin_ratio);

    std::transform(
      waypoints.begin(), waypoints.end(), std::back_inserter(waypoint_groups.back().waypoints),
      [&](const lanelet::ConstPoint3d & waypoint) {
        return WaypointGroup::Waypoint{waypoint, lanelet.id()};
      });
  }

  return waypoint_groups;
}

std::optional<double> get_first_intersection_arc_length(
  const lanelet::LaneletSequence & lanelet_sequence, const double s_start, const double s_end,
  const double vehicle_length)
{
  if (lanelet_sequence.empty()) {
    return std::nullopt;
  }

  std::optional<double> s_intersection{std::nullopt};

  const auto s_start_on_bounds = get_arc_length_on_bounds(lanelet_sequence, s_start);
  const auto s_end_on_bounds = get_arc_length_on_bounds(lanelet_sequence, s_end);

  const auto cropped_centerline = lanelet::utils::to2D(to_lanelet_points(crop_line_string(
    to_geometry_msgs_points(
      lanelet_sequence.centerline2d().begin(), lanelet_sequence.centerline2d().end()),
    s_start, s_end)));
  const auto cropped_left_bound = lanelet::utils::to2D(to_lanelet_points(crop_line_string(
    to_geometry_msgs_points(
      lanelet_sequence.leftBound2d().begin(), lanelet_sequence.leftBound2d().end()),
    s_start_on_bounds.left, s_end_on_bounds.left)));
  const auto cropped_right_bound = lanelet::utils::to2D(to_lanelet_points(crop_line_string(
    to_geometry_msgs_points(
      lanelet_sequence.rightBound2d().begin(), lanelet_sequence.rightBound2d().end()),
    s_start_on_bounds.right, s_end_on_bounds.right)));

  if (cropped_centerline.empty() || cropped_left_bound.empty() || cropped_right_bound.empty()) {
    return std::nullopt;
  }

  const lanelet::BasicLineString2d start_edge{
    cropped_left_bound.front(), cropped_right_bound.front()};

  // self intersection
  {
    const auto s_left_bound = [&]() {
      auto s = get_first_self_intersection_arc_length(cropped_left_bound);
      if (s) {
        *s += s_start_on_bounds.left;
      }
      return s;
    }();
    const auto s_right_bound = [&]() {
      auto s = get_first_self_intersection_arc_length(cropped_right_bound);
      if (s) {
        *s += s_start_on_bounds.right;
      }
      return s;
    }();

    const auto [s_left, s_right] =
      get_arc_length_on_centerline(lanelet_sequence, s_left_bound, s_right_bound);

    if (s_left && s_right) {
      s_intersection = std::min(s_left, s_right);
    } else {
      s_intersection = s_left ? s_left : s_right;
    }
  }

  // intersection between left and right bounds
  {
    lanelet::BasicPoints2d intersections;
    boost::geometry::intersection(cropped_left_bound, cropped_right_bound, intersections);
    for (const auto & intersection : intersections) {
      const auto s_on_centerline = get_arc_length_on_centerline(
        lanelet_sequence,
        s_start_on_bounds.left +
          lanelet::geometry::toArcCoordinates(cropped_left_bound, intersection).length,
        s_start_on_bounds.right +
          lanelet::geometry::toArcCoordinates(cropped_right_bound, intersection).length);
      const auto s_mutual = [&]() {
        if (s_on_centerline.left && s_on_centerline.right) {
          return std::max(s_on_centerline.left, s_on_centerline.right);
        }
        return s_on_centerline.left ? s_on_centerline.left : s_on_centerline.right;
      }();
      if (s_intersection && s_mutual) {
        s_intersection = std::min(s_intersection, s_mutual);
      } else if (s_mutual) {
        s_intersection = s_mutual;
      }
    }
  }

  // intersection between start edge of drivable area and left / right bound
  {
    const auto get_start_edge_intersection_arc_length =
      [&](const lanelet::BasicLineString2d & bound) {
        std::optional<double> s_start_edge = std::nullopt;
        if (bound.size() <= 2) {
          return s_start_edge;
        }
        lanelet::BasicPoints2d start_edge_intersections;
        boost::geometry::intersection(start_edge, bound, start_edge_intersections);
        for (const auto & intersection : start_edge_intersections) {
          if (boost::geometry::equals(intersection, bound.front())) {
            continue;
          }
          const auto s = lanelet::geometry::toArcCoordinates(bound, intersection).length;
          s_start_edge = s_start_edge ? std::min(*s_start_edge, s) : s;
        }
        return s_start_edge;
      };
    const auto s_left_bound = [&]() {
      auto s = get_start_edge_intersection_arc_length(cropped_left_bound);
      if (s) {
        *s += s_start_on_bounds.left;
      }
      return s;
    }();
    const auto s_right_bound = [&]() {
      auto s = get_start_edge_intersection_arc_length(cropped_right_bound);
      if (s) {
        *s += s_start_on_bounds.right;
      }
      return s;
    }();

    const auto s_on_centerline =
      get_arc_length_on_centerline(lanelet_sequence, s_left_bound, s_right_bound);

    const auto s_start_edge = [&]() {
      if (s_on_centerline.left && s_on_centerline.right) {
        return std::min(s_on_centerline.left, s_on_centerline.right);
      }
      return s_on_centerline.left ? s_on_centerline.left : s_on_centerline.right;
    }();
    if (s_intersection && s_start_edge) {
      s_intersection = std::min(s_intersection, s_start_edge);
    } else if (s_start_edge) {
      s_intersection = s_start_edge;
    }
  }

  // intersection between start edge of drivable area and center line
  {
    std::optional<double> s_start_edge = std::nullopt;
    lanelet::BasicPoints2d start_edge_intersections;
    boost::geometry::intersection(start_edge, cropped_centerline, start_edge_intersections);
    for (const auto & intersection : start_edge_intersections) {
      auto s = lanelet::geometry::toArcCoordinates(cropped_centerline, intersection).length;
      // Ignore intersections near the beginning of the centerline.
      // It is impossible to make a turn shorter than the vehicle_length, so use it as a threshold.
      if (s < vehicle_length) continue;
      s += s_start;
      s_start_edge = s_start_edge ? std::min(*s_start_edge, s) : s;
    }
    if (s_intersection && s_start_edge) {
      s_intersection = std::min(s_intersection, s_start_edge);
    } else if (s_start_edge) {
      s_intersection = s_start_edge;
    }
  }

  return s_intersection;
}

std::optional<double> get_first_self_intersection_arc_length(
  const lanelet::BasicLineString2d & line_string)
{
  if (line_string.size() < 3) {
    return std::nullopt;
  }

  std::optional<size_t> first_self_intersection_index = std::nullopt;
  std::optional<double> intersection_arc_length_on_latter_segment = std::nullopt;
  double s = 0.;

  for (size_t i = 0; i < line_string.size() - 1; ++i) {
    if (
      first_self_intersection_index && i == first_self_intersection_index &&
      intersection_arc_length_on_latter_segment) {
      return s + *intersection_arc_length_on_latter_segment;
    }

    const auto current_segment = lanelet::BasicSegment2d{line_string.at(i), line_string.at(i + 1)};
    s += lanelet::geometry::length(current_segment);

    lanelet::BasicPoints2d self_intersections{};
    for (size_t j = i + 1; j < line_string.size() - 1; ++j) {
      const auto segment = lanelet::BasicSegment2d{line_string.at(j), line_string.at(j + 1)};
      if (
        segment.first == current_segment.second || segment.second == current_segment.first ||
        segment.first == current_segment.first) {
        continue;
      }
      boost::geometry::intersection(current_segment, segment, self_intersections);
      if (self_intersections.empty()) {
        continue;
      }
      first_self_intersection_index = j;
      intersection_arc_length_on_latter_segment =
        (self_intersections.front() - segment.first).norm();
      break;
    }
  }

  return std::nullopt;
}

double get_arc_length_on_path(
  const lanelet::LaneletSequence & lanelet_sequence, const std::vector<PathPointWithLaneId> & path,
  const double s_centerline)
{
  std::optional<lanelet::Id> target_lanelet_id = std::nullopt;
  std::optional<lanelet::BasicPoint2d> point_on_centerline = std::nullopt;

  if (lanelet_sequence.empty() || path.empty()) {
    RCLCPP_WARN(
      rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
      "Input lanelet sequence or path is empty, returning 0.");
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
    point_on_centerline =
      lanelet::geometry::interpolatedPointAtDistance(it->centerline2d(), s_centerline - s);
    break;
  }

  if (!target_lanelet_id || !point_on_centerline) {
    // lanelet_sequence is too short, thus we return input arc length as is.
    return s_centerline;
  }

  auto s_path = 0.;
  lanelet::BasicLineString2d target_path_segment;

  for (auto it = path.begin(); it != path.end(); ++it) {
    if (
      std::find(it->lane_ids.begin(), it->lane_ids.end(), *target_lanelet_id) ==
      it->lane_ids.end()) {
      if (target_path_segment.empty() && it != std::prev(path.end())) {
        s_path += autoware_utils::calc_distance2d(*it, *std::next(it));
        continue;
      }
      break;
    }
    target_path_segment.push_back(
      lanelet::utils::conversion::toLaneletPoint(it->point.pose.position).basicPoint2d());
  }

  s_path += lanelet::geometry::toArcCoordinates(target_path_segment, *point_on_centerline).length;

  return s_path;
}

PathRange<std::vector<geometry_msgs::msg::Point>> get_path_bounds(
  const lanelet::LaneletSequence & lanelet_sequence, const double s_start, const double s_end)
{
  if (lanelet_sequence.empty()) {
    return {};
  }

  const auto [s_left_start, s_right_start] = get_arc_length_on_bounds(lanelet_sequence, s_start);
  const auto [s_left_end, s_right_end] = get_arc_length_on_bounds(lanelet_sequence, s_end);

  return {
    crop_line_string(
      to_geometry_msgs_points(
        lanelet_sequence.leftBound().begin(), lanelet_sequence.leftBound().end()),
      s_left_start, s_left_end),
    crop_line_string(
      to_geometry_msgs_points(
        lanelet_sequence.rightBound().begin(), lanelet_sequence.rightBound().end()),
      s_right_start, s_right_end)};
}

std::vector<geometry_msgs::msg::Point> crop_line_string(
  const std::vector<geometry_msgs::msg::Point> & line_string, const double s_start,
  const double s_end)
{
  if (s_start < 0.) {
    RCLCPP_WARN(
      rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
      "Start of crop range is negative, returning input as is");
    return line_string;
  }

  if (s_start > s_end) {
    RCLCPP_WARN(
      rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
      "Start of crop range is larger than end, returning input as is");
    return line_string;
  }

  auto trajectory =
    autoware::experimental::trajectory::Trajectory<geometry_msgs::msg::Point>::Builder()
      .set_xy_interpolator<autoware::experimental::trajectory::interpolator::Linear>()
      .build(line_string);
  if (!trajectory) {
    return {};
  }

  trajectory->crop(s_start, s_end - s_start);
  return trajectory->restore();
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

PathRange<std::optional<double>> get_arc_length_on_centerline(
  const lanelet::LaneletSequence & lanelet_sequence, const std::optional<double> & s_left_bound,
  const std::optional<double> & s_right_bound)
{
  std::optional<double> s_left_centerline = std::nullopt;
  std::optional<double> s_right_centerline = std::nullopt;

  if (s_left_bound && *s_left_bound < 0.) {
    RCLCPP_WARN(
      rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
      "Input left arc length is negative, returning 0.");
    s_left_centerline = 0.;
  }
  if (s_right_bound && *s_right_bound < 0.) {
    RCLCPP_WARN(
      rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
      "Input right arc length is negative, returning 0.");
    s_right_centerline = 0.;
  }

  auto s = 0.;
  auto s_left = 0.;
  auto s_right = 0.;

  for (auto it = lanelet_sequence.begin(); it != lanelet_sequence.end(); ++it) {
    const auto is_left_done = !s_left_bound || s_left_centerline;
    const auto is_right_done = !s_right_bound || s_right_centerline;
    if (is_left_done && is_right_done) {
      break;
    }

    const double centerline_length = lanelet::utils::getLaneletLength2d(*it);
    const double left_bound_length = lanelet::geometry::length(it->leftBound2d());
    const double right_bound_length = lanelet::geometry::length(it->rightBound2d());

    if (!is_left_done && s_left + left_bound_length > s_left_bound) {
      s_left_centerline = s + lanelet::geometry::toArcCoordinates(
                                it->centerline2d(), lanelet::geometry::interpolatedPointAtDistance(
                                                      it->leftBound2d(), *s_left_bound - s_left))
                                .length;
    }
    if (!is_right_done && s_right + right_bound_length > s_right_bound) {
      s_right_centerline =
        s + lanelet::geometry::toArcCoordinates(
              it->centerline2d(), lanelet::geometry::interpolatedPointAtDistance(
                                    it->rightBound2d(), *s_right_bound - s_right))
              .length;
    }

    s += centerline_length;
    s_left += left_bound_length;
    s_right += right_bound_length;
  }

  return {
    s_left_centerline ? s_left_centerline : s_left_bound,
    s_right_centerline ? s_right_centerline : s_right_bound};
}

geometry_msgs::msg::Pose refine_goal(
  const geometry_msgs::msg::Pose & goal, const lanelet::ConstLanelet & goal_lanelet)
{
  const auto goal_point_on_lanelet = lanelet::utils::conversion::toLaneletPoint(goal.position);
  const double distance = boost::geometry::distance(
    goal_lanelet.polygon2d().basicPolygon(),
    lanelet::utils::to2D(goal_point_on_lanelet).basicPoint());

  // You are almost at the goal
  if (distance < std::numeric_limits<double>::epsilon()) {
    return goal;
  }

  // Get the closest segment to the goal
  const auto segment = lanelet::utils::getClosestSegment(
    lanelet::utils::to2D(goal_point_on_lanelet), goal_lanelet.centerline());

  // If the segment is empty, return the original goal.
  if (segment.empty()) {
    return goal;
  }

  geometry_msgs::msg::Pose refined_goal;
  {
    // find position
    const auto p1 = segment.front().basicPoint();
    const auto p2 = segment.back().basicPoint();
    const auto direction_vector = (p2 - p1).normalized();
    const auto p1_to_goal = goal_point_on_lanelet.basicPoint() - p1;
    const double s = direction_vector.dot(p1_to_goal);
    const auto refined_point = p1 + direction_vector * s;

    refined_goal.position.x = refined_point.x();
    refined_goal.position.y = refined_point.y();
    refined_goal.position.z = refined_point.z();

    // find orientation
    const double yaw = std::atan2(direction_vector.y(), direction_vector.x());
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, yaw);
    refined_goal.orientation = tf2::toMsg(tf_quat);
  }
  return refined_goal;
}

// To perform smooth goal connection, we need to prepare the point before the goal point.
// This function prepares the point before the goal point.
// See the link below for more details:
//   https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/#fixed_goal_planner
PathPointWithLaneId prepare_pre_goal(
  const geometry_msgs::msg::Pose & goal, const lanelet::ConstLanelets & lanes)
{
  PathPointWithLaneId pre_refined_goal{};

  // -1.0 is to prepare the point before the goal point. See the link above for more details.
  constexpr double goal_to_pre_goal_distance = -1.0;

  // First, calculate the pose of the pre_goal point
  pre_refined_goal.point.pose =
    autoware_utils::calc_offset_pose(goal, goal_to_pre_goal_distance, 0.0, 0.0);

  // Second, find and set the lane_id that the pre_goal point belongs to
  for (const auto & lane : lanes) {
    if (lanelet::utils::isInLanelet(pre_refined_goal.point.pose, lane)) {
      // Prevent from duplication
      if (exists(pre_refined_goal.lane_ids, lane.id())) {
        continue;
      }
      pre_refined_goal.lane_ids.push_back(lane.id());
    }
  }

  return pre_refined_goal;
}

// A function that assumes a circle with radius max_dist centered at the goal and returns
// the index of the point closest to the circumference of the circle and outside of it.
// If no such point is found, return the farthest point from the goal in the circle.
std::optional<size_t> find_index_out_of_goal_search_range(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Pose & goal, const int64_t goal_lane_id, const double max_dist)
{
  if (points.empty()) {
    return std::nullopt;
  }

  // find goal index
  size_t min_dist_index;
  {
    bool found = false;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < points.size(); ++i) {
      const auto & lane_ids = points.at(i).lane_ids;

      const double dist_to_goal = autoware_utils::calc_distance2d(points.at(i).point.pose, goal);
      const bool is_goal_lane_id_in_point = exists(lane_ids, goal_lane_id);
      if (dist_to_goal < max_dist && dist_to_goal < min_dist && is_goal_lane_id_in_point) {
        min_dist_index = i;
        min_dist = dist_to_goal;
        found = true;
      }
    }
    if (!found) {
      return std::nullopt;
    }
  }

  // Find index out of goal search range
  size_t min_dist_out_of_range_index = min_dist_index;
  for (int i = min_dist_index; 0 <= i; --i) {
    const double dist = autoware_utils::calc_distance2d(points.at(i).point, goal);
    min_dist_out_of_range_index = i;
    if (max_dist < dist) {
      break;
    }
  }

  return min_dist_out_of_range_index;
}

// Clean up points around the goal for smooth goal connection
// This function returns the cleaned up path. You need to add pre_goal and goal to the returned
// path. This is because we'll do spline interpolation between the tail of the returned path and the
// pre_goal later at this file.
//   https://github.com/autowarefoundation/autoware_universe/blob/908cb7ee5cca01c367f03caf6db4562a620504fb/planning/behavior_path_planner/autoware_behavior_path_planner/src/behavior_path_planner_node.cpp#L724-L725
std::optional<PathWithLaneId> get_path_up_to_just_before_pre_goal(
  const PathWithLaneId & input, const geometry_msgs::msg::Pose & goal,
  const lanelet::Id goal_lane_id, const double search_radius_range)
{
  // Find min_dist_out_of_circle_index whose distance to goal is longer than search_radius_range
  const auto min_dist_out_of_circle_index_opt =
    find_index_out_of_goal_search_range(input.points, goal, goal_lane_id, search_radius_range);

  // It seems we are almost at the goal as no point is found outside of the circle whose center is
  // the goal
  if (!min_dist_out_of_circle_index_opt) {
    return std::nullopt;
  }

  // It seems we have a point outside of the circle whose center is the goal
  const auto min_dist_out_of_circle_index = min_dist_out_of_circle_index_opt.value();

  // Fill all the points up to just before the point outside of the circle
  PathWithLaneId output;
  for (size_t i = 0; i <= min_dist_out_of_circle_index; ++i) {
    output.points.push_back(input.points.at(i));
  }

  return output;
}

// Function to refine the path for the goal
experimental::trajectory::Trajectory<PathPointWithLaneId> refine_path_for_goal(
  const experimental::trajectory::Trajectory<PathPointWithLaneId> & input,  //
  const geometry_msgs::msg::Pose & goal_pose,                               //
  const lanelet::Id goal_lane_id,                                           //
  const double refine_goal_search_radius_range)
{
  auto contain_goal_lane_id = [&](const PathPointWithLaneId & point) {
    const auto & lane_ids = point.lane_ids;
    const bool is_goal_lane_id_in_point =
      std::find(lane_ids.begin(), lane_ids.end(), goal_lane_id) != lane_ids.end();
    return is_goal_lane_id_in_point;
  };

  auto outside_circle = [&](const PathPointWithLaneId & point) {
    const double dist = autoware_utils::calc_distance2d(point.point.pose, goal_pose);
    return dist > refine_goal_search_radius_range;
  };

  auto closest_to_goal = autoware::experimental::trajectory::closest_with_constraint(
    input, goal_pose, contain_goal_lane_id);

  if (!closest_to_goal) {
    return input;
  }

  auto cropped_path = autoware::experimental::trajectory::crop(input, 0, *closest_to_goal);

  auto intervals =
    autoware::experimental::trajectory::find_intervals(cropped_path, outside_circle, 10);

  std::vector<PathPointWithLaneId> goal_connected_trajectory_points;

  if (!intervals.empty()) {
    auto cropped = autoware::experimental::trajectory::crop(cropped_path, 0, intervals.back().end);
    goal_connected_trajectory_points = cropped.restore(2);
  }

  auto goal = input.compute(autoware::experimental::trajectory::closest(input, goal_pose));

  goal.point.pose = goal_pose;

  goal.point.longitudinal_velocity_mps = 0.0;

  auto pre_goal_pose = autoware_utils_geometry::calc_offset_pose(goal_pose, -1.0, 0.0, 0.0);

  auto pre_goal = input.compute(autoware::experimental::trajectory::closest(input, pre_goal_pose));

  pre_goal.point.pose = pre_goal_pose;

  goal_connected_trajectory_points.emplace_back(pre_goal);
  goal_connected_trajectory_points.emplace_back(goal);

  if (goal_connected_trajectory_points.size() < 4) {  // 4 points are required for spline
                                                      // interpolation
    auto output = experimental::trajectory::Trajectory<PathPointWithLaneId>::Builder{}
                    .set_xy_interpolator<experimental::trajectory::interpolator::Linear>()
                    .build(goal_connected_trajectory_points);
    if (output) {
      return output.value();
    }
  } else {
    auto output = experimental::trajectory::Trajectory<PathPointWithLaneId>::Builder{}.build(
      goal_connected_trajectory_points);
    if (output) {
      return output.value();
    }
  }
  return input;
}

lanelet::ConstLanelets extract_lanelets_from_trajectory(
  const experimental::trajectory::Trajectory<PathPointWithLaneId> & trajectory,
  const PlannerData & planner_data)
{
  lanelet::ConstLanelets lanelets{};
  const auto lane_ids = trajectory.get_contained_lane_ids();
  const auto lane_ids_set = std::set(lane_ids.begin(), lane_ids.end());
  for (const auto & lane_id : lane_ids_set) {
    const auto lanelet = planner_data.lanelet_map_ptr->laneletLayer.get(lane_id);
    lanelets.push_back(lanelet);
  }
  return lanelets;
}

bool is_in_lanelets(const geometry_msgs::msg::Pose & pose, const lanelet::ConstLanelets & lanes)
{
  for (const auto & lane : lanes) {
    if (lanelet::utils::isInLanelet(pose, lane)) {
      return true;
    }
  }
  return false;
}

bool is_trajectory_inside_lanelets(
  const experimental::trajectory::Trajectory<PathPointWithLaneId> & refined_path,
  const lanelet::ConstLanelets & lanelets)
{
  for (double s = 0.0; s < refined_path.length(); s += 0.1) {
    const auto point = refined_path.compute(s);
    if (!is_in_lanelets(point.point.pose, lanelets)) {
      return false;
    }
  }
  return true;
}

std::optional<experimental::trajectory::Trajectory<PathPointWithLaneId>>
modify_path_for_smooth_goal_connection(
  const experimental::trajectory::Trajectory<PathPointWithLaneId> & trajectory,
  const PlannerData & planner_data, const double refine_goal_search_radius_range)
{
  if (planner_data.preferred_lanelets.empty()) {
    return std::nullopt;
  }
  const auto lanelets = extract_lanelets_from_trajectory(trajectory, planner_data);
  auto goal_pose =
    utils::refine_goal(planner_data.goal_pose, planner_data.preferred_lanelets.back());

  // This process is to fit the trajectory inside the lanelets. By reducing
  // refine_goal_search_radius_range, we can fit the trajectory inside lanelets even if the
  // trajectory has a high curvature.
  for (double s = refine_goal_search_radius_range; s > 0; s -= 0.1) {
    const auto refined_trajectory = refine_path_for_goal(
      trajectory, goal_pose, planner_data.preferred_lanelets.back().id(),
      refine_goal_search_radius_range);
    const bool is_inside = is_trajectory_inside_lanelets(refined_trajectory, lanelets);
    if (is_inside) {
      return refined_trajectory;
    }
  }
  return std::nullopt;
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

  std::vector<lanelet::Id> searched_lanelet_ids = {};
  std::optional<double> arc_length_from_vehicle_front_to_lanelet_start = std::nullopt;

  auto calc_arc_length =
    [&](const lanelet::ConstLanelet & lanelet, const lanelet::BasicPoint2d & point) -> double {
    return lanelet::geometry::toArcCoordinates(lanelet.centerline2d(), point).length;
  };

  for (const auto & point : path.points) {
    for (const auto & lane_id : point.lane_ids) {
      if (exists(searched_lanelet_ids, lane_id)) {
        continue;
      }
      searched_lanelet_ids.push_back(lane_id);

      const auto lanelet = planner_data.lanelet_map_ptr->laneletLayer.get(lane_id);
      if (!get_next_lanelet_within_route(lanelet, planner_data)) {
        continue;
      }

      if (
        !arc_length_from_vehicle_front_to_lanelet_start &&
        !lanelet::geometry::inside(lanelet, current_point)) {
        continue;
      }

      if (lanelet.hasAttribute("turn_direction")) {
        turn_signal.command =
          turn_signal_command_map.at(lanelet.attribute("turn_direction").value());

        if (arc_length_from_vehicle_front_to_lanelet_start) {  // ego is in front of lanelet
          if (
            *arc_length_from_vehicle_front_to_lanelet_start >
            lanelet.attributeOr("turn_signal_distance", base_search_distance)) {
            turn_signal.command = TurnIndicatorsCommand::NO_COMMAND;
          }
          return turn_signal;
        }

        // ego is inside lanelet
        const auto required_end_point_opt =
          get_turn_signal_required_end_point(lanelet, angle_threshold_deg);
        if (!required_end_point_opt) continue;
        if (
          calc_arc_length(lanelet, current_point) <=
          calc_arc_length(lanelet, required_end_point_opt.value())) {
          return turn_signal;
        }
      }

      const auto lanelet_length = lanelet::utils::getLaneletLength2d(lanelet);
      if (arc_length_from_vehicle_front_to_lanelet_start) {
        *arc_length_from_vehicle_front_to_lanelet_start += lanelet_length;
      } else {
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
  std::vector<geometry_msgs::msg::Pose> centerline_poses(lanelet.centerline().size());
  std::transform(
    lanelet.centerline().begin(), lanelet.centerline().end(), centerline_poses.begin(),
    [](const auto & point) {
      geometry_msgs::msg::Pose pose{};
      pose.position = lanelet::utils::conversion::toGeomMsgPt(point);
      return pose;
    });

  // NOTE: Trajectory does not support less than 4 points, so resample if less than 4.
  //       This implementation should be replaced in the future
  if (centerline_poses.size() < 4) {
    const auto lanelet_length = autoware::motion_utils::calcArcLength(centerline_poses);
    const auto resampling_interval = lanelet_length / 4.0;
    std::vector<double> resampled_arclength;
    for (double s = 0.0; s < lanelet_length; s += resampling_interval) {
      resampled_arclength.push_back(s);
    }
    if (lanelet_length - resampled_arclength.back() < autoware::motion_utils::overlap_threshold) {
      resampled_arclength.back() = lanelet_length;
    } else {
      resampled_arclength.push_back(lanelet_length);
    }
    centerline_poses =
      autoware::motion_utils::resamplePoseVector(centerline_poses, resampled_arclength);
    if (centerline_poses.size() < 4) return std::nullopt;
  }

  auto centerline =
    autoware::experimental::trajectory::Trajectory<geometry_msgs::msg::Pose>::Builder{}.build(
      centerline_poses);
  if (!centerline) return std::nullopt;
  centerline->align_orientation_with_trajectory_direction();

  const auto terminal_yaw = tf2::getYaw(centerline->compute(centerline->length()).orientation);
  const auto intervals = autoware::experimental::trajectory::find_intervals(
    centerline.value(),
    [terminal_yaw, angle_threshold_deg](const geometry_msgs::msg::Pose & point) {
      const auto yaw = tf2::getYaw(point.orientation);
      return std::fabs(autoware_utils::normalize_radian(yaw - terminal_yaw)) <
             autoware_utils::deg2rad(angle_threshold_deg);
    });
  if (intervals.empty()) return std::nullopt;

  return lanelet::utils::conversion::toLaneletPoint(
    centerline->compute(intervals.front().start).position);
}
}  // namespace utils
}  // namespace autoware::path_generator
