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
#include <autoware/trajectory/utils/reference_path.hpp>
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
}  // namespace

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

std::optional<PathRange<std::vector<geometry_msgs::msg::Point>>> get_path_bounds(
  const std::vector<PathPointWithLaneId> & path_points,
  const lanelet::LaneletSequence & lanelet_sequence,
  const lanelet::routing::RoutingGraphConstPtr routing_graph, const double backward_offset,
  const double forward_offset)
{
  if (lanelet_sequence.empty()) {
    return std::nullopt;
  }

  auto lanelets = lanelet_sequence.lanelets();

  const auto path_start_lanelet_it = std::find_if(
    lanelets.cbegin(), lanelets.cend(),
    [&](const lanelet::ConstLanelet & l) { return exists(path_points.front().lane_ids, l.id()); });
  if (path_start_lanelet_it == lanelets.cend()) {
    // If path start is not in lanelet sequence, extend sequence backward
    while (true) {
      const auto prev_lanelets = routing_graph->previous(lanelets.front());
      if (prev_lanelets.empty()) {
        RCLCPP_ERROR(
          rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
          "Path start is outside lanelet sequence and no previous lanelets found");
        return std::nullopt;
      }
      lanelets.insert(lanelets.begin(), prev_lanelets.front());
      if (exists(path_points.front().lane_ids, prev_lanelets.front().id())) {
        break;
      }
    }
  } else {
    // If path start is in lanelet sequence, remove all previous lanelets
    lanelets.erase(lanelets.begin(), path_start_lanelet_it);
  }

  const auto path_end_lanelet_it = std::find_if(
    lanelets.cbegin(), lanelets.cend(),
    [&](const lanelet::ConstLanelet & l) { return exists(path_points.back().lane_ids, l.id()); });
  if (path_end_lanelet_it == lanelets.cend()) {
    // If path end is not in lanelet sequence, extend sequence forward
    while (true) {
      const auto next_lanelets = routing_graph->following(lanelets.back());
      if (next_lanelets.empty()) {
        RCLCPP_ERROR(
          rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
          "Path end is outside lanelet sequence and no following lanelets found");
        return std::nullopt;
      }
      lanelets.insert(lanelets.end(), next_lanelets.front());
      if (exists(path_points.back().lane_ids, next_lanelets.front().id())) {
        break;
      }
    }
  } else {
    // If path end is in lanelet sequence, remove all following lanelets
    lanelets.erase(std::next(path_end_lanelet_it), lanelets.end());
  }

  // Get longitudinal position of start point of path on centerline
  const auto s_start = get_arc_length_on_centerline(
    lanelets,
    lanelet::utils::conversion::toLaneletPoint(path_points.front().point.pose.position)
      .basicPoint2d(),
    lanelets.front().id());
  if (!s_start) {
    RCLCPP_ERROR(
      rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
      "Failed to get arc length of path start");
    return std::nullopt;
  }

  // Get longitudinal position of end point of path on centerline
  const auto s_end = get_arc_length_on_centerline(
    lanelets,
    lanelet::utils::conversion::toLaneletPoint(path_points.back().point.pose.position)
      .basicPoint2d(),
    lanelets.back().id());
  if (!s_end) {
    RCLCPP_ERROR(
      rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
      "Failed to get arc length of path end");
    return std::nullopt;
  }

  // Extend lanelet sequence to include start and end points with offsets
  const auto [extended_lanelets, new_s_start, new_s_end] =
    autoware::experimental::trajectory::supplement_lanelet_sequence(
      routing_graph, lanelets, *s_start - backward_offset, *s_end + forward_offset);

  // Get offset start and end points on centerline of extended lanelet sequence
  const lanelet::LaneletSequence extended_lanelet_sequence(extended_lanelets);
  const auto offset_start_point = lanelet::geometry::interpolatedPointAtDistance(
    extended_lanelet_sequence.centerline2d(), new_s_start);
  const auto offset_end_point = lanelet::geometry::interpolatedPointAtDistance(
    extended_lanelet_sequence.centerline2d(), new_s_end);

  // Get longitudinal positions of start of path on bounds
  const auto ss_bound_start = get_arc_length_on_bounds(
    extended_lanelet_sequence, offset_start_point, extended_lanelets.front().id());
  if (!ss_bound_start) {
    RCLCPP_ERROR(
      rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
      "Failed to get arc length of bound start");
    return std::nullopt;
  }

  // Get longitudinal positions of end of path on bounds
  const auto ss_bound_end = get_arc_length_on_bounds(
    extended_lanelet_sequence, offset_end_point, extended_lanelets.back().id());
  if (!ss_bound_end) {
    RCLCPP_ERROR(
      rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
      "Failed to get arc length of bound end");
    return std::nullopt;
  }

  return PathRange<std::vector<geometry_msgs::msg::Point>>{
    crop_line_string(
      extended_lanelet_sequence.leftBound().basicLineString(), ss_bound_start->left,
      ss_bound_end->left),
    crop_line_string(
      extended_lanelet_sequence.rightBound().basicLineString(), ss_bound_start->right,
      ss_bound_end->right)};
}

std::vector<geometry_msgs::msg::Point> crop_line_string(
  const lanelet::BasicLineString3d & line_string, const double s_start, const double s_end)
{
  const auto geom_msgs_points = to_geometry_msgs_points(line_string.begin(), line_string.end());

  if (s_start < 0.) {
    RCLCPP_WARN(
      rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
      "Start of crop range is negative, returning input as is");
    return geom_msgs_points;
  }

  if (s_start > s_end) {
    RCLCPP_WARN(
      rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
      "Start of crop range is larger than end, returning input as is");
    return geom_msgs_points;
  }

  auto trajectory =
    autoware::experimental::trajectory::Trajectory<geometry_msgs::msg::Point>::Builder()
      .set_xy_interpolator<autoware::experimental::trajectory::interpolator::Linear>()
      .build(geom_msgs_points);
  if (!trajectory) {
    return {};
  }

  trajectory->crop(s_start, s_end - s_start);
  return trajectory->restore();
}

std::optional<double> get_arc_length_on_centerline(
  const lanelet::LaneletSequence & lanelet_sequence, const lanelet::BasicPoint2d & point,
  const lanelet::Id & lane_id)
{
  auto s_centerline = 0.;

  for (const auto & lanelet : lanelet_sequence) {
    if (lanelet.id() != lane_id || !lanelet::geometry::inside(lanelet, point)) {
      s_centerline += lanelet::geometry::length(lanelet.centerline2d());
      continue;
    }

    s_centerline += lanelet::geometry::toArcCoordinates(lanelet.centerline2d(), point).length;

    return s_centerline;
  }

  // The path point is outside lanelet_sequence
  return std::nullopt;
}

std::optional<PathRange<double>> get_arc_length_on_bounds(
  const lanelet::LaneletSequence & lanelet_sequence, const lanelet::BasicPoint2d & point,
  const lanelet::Id & lane_id)
{
  auto s_left = 0.;
  auto s_right = 0.;

  for (const auto & lanelet : lanelet_sequence) {
    if (lanelet.id() != lane_id || !lanelet::geometry::inside(lanelet, point)) {
      s_left += lanelet::geometry::length(lanelet.leftBound2d());
      s_right += lanelet::geometry::length(lanelet.rightBound2d());
      continue;
    }

    s_left += lanelet::geometry::toArcCoordinates(lanelet.leftBound2d(), point).length;
    s_right += lanelet::geometry::toArcCoordinates(lanelet.rightBound2d(), point).length;

    return PathRange<double>{s_left, s_right};
  }

  // The path point is outside lanelet_sequence
  return std::nullopt;
}

// Function to refine the path for the goal
experimental::trajectory::Trajectory<PathPointWithLaneId> refine_path_for_goal(
  const experimental::trajectory::Trajectory<PathPointWithLaneId> & input,
  const geometry_msgs::msg::Pose & goal_pose, const lanelet::Id goal_lane_id,
  const double search_radius_range, const double pre_goal_offset)
{
  auto contain_goal_lane_id = [&](const PathPointWithLaneId & point) {
    const auto & lane_ids = point.lane_ids;
    const bool is_goal_lane_id_in_point =
      std::find(lane_ids.begin(), lane_ids.end(), goal_lane_id) != lane_ids.end();
    return is_goal_lane_id_in_point;
  };

  auto outside_circle = [&](const PathPointWithLaneId & point) {
    const double dist = autoware_utils::calc_distance2d(point.point.pose, goal_pose);
    return dist > search_radius_range;
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
  } else if (cropped_path.length() > pre_goal_offset) {
    // If distance from start to goal is smaller than refine_goal_search_radius_range and start is
    // farther from goal than pre_goal, we just connect start, pre_goal, and goal.
    goal_connected_trajectory_points = {cropped_path.compute(0)};
  }

  auto goal = input.compute(autoware::experimental::trajectory::closest(input, goal_pose));

  goal.point.pose = goal_pose;

  goal.point.longitudinal_velocity_mps = 0.0;

  auto pre_goal_pose =
    autoware_utils_geometry::calc_offset_pose(goal_pose, -pre_goal_offset, 0.0, 0.0);

  auto pre_goal = input.compute(autoware::experimental::trajectory::closest(input, pre_goal_pose));

  pre_goal.point.pose = pre_goal_pose;

  goal_connected_trajectory_points.emplace_back(pre_goal);
  goal_connected_trajectory_points.emplace_back(goal);

  if (
    const auto output =
      autoware::experimental::trajectory::pretty_build(goal_connected_trajectory_points)) {
    return *output;
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
  const PlannerData & planner_data, const double search_radius_range, const double pre_goal_offset)
{
  if (planner_data.preferred_lanelets.empty()) {
    return std::nullopt;
  }
  const auto lanelets = extract_lanelets_from_trajectory(trajectory, planner_data);

  // This process is to fit the trajectory inside the lanelets. By reducing
  // refine_goal_search_radius_range, we can fit the trajectory inside lanelets even if the
  // trajectory has a high curvature.
  for (double s = search_radius_range; s > 0; s -= 0.1) {
    const auto refined_trajectory = refine_path_for_goal(
      trajectory, planner_data.goal_pose, planner_data.preferred_lanelets.back().id(),
      search_radius_range, pre_goal_offset);
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
