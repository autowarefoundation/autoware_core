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

#include "autoware/path_generator/node.hpp"

#include "autoware/path_generator/utils.hpp"

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
template <typename LaneletT, typename PointT>
double get_arc_length_along_centerline(const LaneletT & lanelet, const PointT & point)
{
  return lanelet::geometry::toArcCoordinates(lanelet.centerline2d(), lanelet::utils::to2D(point))
    .length;
}
}  // namespace

namespace autoware::path_generator
{
PathGenerator::PathGenerator(const rclcpp::NodeOptions & node_options)
: Node("path_generator", node_options)
{
  param_listener_ =
    std::make_shared<::path_generator::ParamListener>(this->get_node_parameters_interface());

  path_publisher_ = create_publisher<PathWithLaneId>("~/output/path", 1);

  turn_signal_publisher_ =
    create_publisher<TurnIndicatorsCommand>("~/output/turn_indicators_cmd", 1);

  hazard_signal_publisher_ = create_publisher<HazardLightsCommand>("~/output/hazard_lights_cmd", 1);

  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();

  const auto params = param_listener_->get_params();

  // Ensure that the refine_goal_search_radius_range and search_radius_decrement must be positive
  if (params.refine_goal_search_radius_range <= 0 || params.search_radius_decrement <= 0) {
    throw std::runtime_error(
      "refine_goal_search_radius_range and search_radius_decrement must be positive");
  }

  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(params.planning_hz).period(),
    std::bind(&PathGenerator::run, this));
}

void PathGenerator::run()
{
  const auto input_data = take_data();
  set_planner_data(input_data);
  if (!is_data_ready(input_data)) {
    return;
  }

  const auto param = param_listener_->get_params();
  const auto path = plan_path(input_data, param);
  if (!path) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "output path is invalid");
    return;
  }

  auto turn_signal = utils::get_turn_signal(
    *path, planner_data_, input_data.odometry_ptr->pose.pose,
    input_data.odometry_ptr->twist.twist.linear.x, param.turn_signal.search_distance,
    param.turn_signal.search_time, param.turn_signal.angle_threshold_deg,
    vehicle_info_.max_longitudinal_offset_m);
  turn_signal.stamp = now();
  turn_signal_publisher_->publish(turn_signal);

  HazardLightsCommand hazard_signal;
  hazard_signal.command = HazardLightsCommand::NO_COMMAND;
  hazard_signal.stamp = now();
  hazard_signal_publisher_->publish(hazard_signal);

  path_publisher_->publish(*path);
}

PathGenerator::InputData PathGenerator::take_data()
{
  InputData input_data;

  // route
  if (const auto msg = route_subscriber_.take_data()) {
    if (msg->segments.empty()) {
      RCLCPP_ERROR(get_logger(), "input route is empty, ignoring...");
    } else {
      input_data.route_ptr = msg;
    }
  }

  // map
  if (const auto msg = vector_map_subscriber_.take_data()) {
    input_data.lanelet_map_bin_ptr = msg;
  }

  // velocity
  if (const auto msg = odometry_subscriber_.take_data()) {
    input_data.odometry_ptr = msg;
  }

  return input_data;
}

void PathGenerator::set_planner_data(const InputData & input_data)
{
  if (input_data.lanelet_map_bin_ptr) {
    planner_data_.lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(
      *input_data.lanelet_map_bin_ptr, planner_data_.lanelet_map_ptr,
      &planner_data_.traffic_rules_ptr, &planner_data_.routing_graph_ptr);
  }

  if (input_data.route_ptr) {
    set_route(input_data.route_ptr);
  }
}

void PathGenerator::set_route(const LaneletRoute::ConstSharedPtr & route_ptr)
{
  planner_data_.route_frame_id = route_ptr->header.frame_id;
  planner_data_.goal_pose = route_ptr->goal_pose;

  planner_data_.route_lanelets.clear();
  planner_data_.preferred_lanelets.clear();
  planner_data_.start_lanelets.clear();
  planner_data_.goal_lanelets.clear();

  size_t primitives_num = 0;
  for (const auto & route_section : route_ptr->segments) {
    primitives_num += route_section.primitives.size();
  }
  planner_data_.route_lanelets.reserve(primitives_num);

  for (const auto & route_section : route_ptr->segments) {
    for (const auto & primitive : route_section.primitives) {
      const auto id = primitive.id;
      const auto & lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(id);
      planner_data_.route_lanelets.push_back(lanelet);
      if (id == route_section.preferred_primitive.id) {
        planner_data_.preferred_lanelets.push_back(lanelet);
      }
    }
  }

  const auto set_lanelets_from_segment =
    [&](
      const autoware_planning_msgs::msg::LaneletSegment & segment,
      lanelet::ConstLanelets & lanelets) {
      lanelets.reserve(segment.primitives.size());
      for (const auto & primitive : segment.primitives) {
        const auto & lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(primitive.id);
        lanelets.push_back(lanelet);
      }
    };
  set_lanelets_from_segment(route_ptr->segments.front(), planner_data_.start_lanelets);
  set_lanelets_from_segment(route_ptr->segments.back(), planner_data_.goal_lanelets);
}

bool PathGenerator::is_data_ready(const InputData & input_data)
{
  const auto notify_waiting = [this](const std::string & name) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for %s", name.c_str());
  };

  if (!planner_data_.lanelet_map_ptr) {
    notify_waiting("map");
    return false;
  }

  if (planner_data_.route_lanelets.empty()) {
    notify_waiting("route");
    return false;
  }

  if (!input_data.odometry_ptr) {
    notify_waiting("odometry");
    return false;
  }

  return true;
}

std::optional<PathWithLaneId> PathGenerator::plan_path(
  const InputData & input_data, const Params & params)
{
  const auto path = generate_path(input_data.odometry_ptr->pose.pose, params);

  if (!path) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "output path is invalid");
    return std::nullopt;
  }
  if (path->points.empty()) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "output path is empty");
    return std::nullopt;
  }

  return path;
}

std::optional<PathWithLaneId> PathGenerator::generate_path(
  const geometry_msgs::msg::Pose & current_pose, const Params & params)
{
  if (!update_current_lanelet(current_pose, params)) {
    return std::nullopt;
  }

  const auto s_on_current_lanelet =
    lanelet::utils::getArcCoordinates({*current_lanelet_}, current_pose).length;

  const auto backward_lanelets = utils::get_lanelets_within_route_up_to(
    *current_lanelet_, planner_data_, params.path_length.backward - s_on_current_lanelet);
  if (!backward_lanelets) {
    return std::nullopt;
  }

  const auto forward_lanelets = utils::get_lanelets_within_route_after(
    *current_lanelet_, planner_data_,
    params.path_length.forward -
      (lanelet::utils::getLaneletLength2d(*current_lanelet_) - s_on_current_lanelet));
  if (!forward_lanelets) {
    return std::nullopt;
  }

  lanelet::ConstLanelets lanelets(*backward_lanelets);
  lanelets.push_back(*current_lanelet_);
  lanelets.insert(lanelets.end(), forward_lanelets->begin(), forward_lanelets->end());

  const auto s = s_on_current_lanelet + lanelet::utils::getLaneletLength2d(*backward_lanelets);
  const auto s_start = std::max(0., s - params.path_length.backward);
  const auto s_end = [&]() {
    auto s_end = s + params.path_length.forward;

    if (!utils::get_next_lanelet_within_route(lanelets.back(), planner_data_)) {
      s_end = std::min(s_end, lanelet::utils::getLaneletLength2d(lanelets));
    }

    if (std::any_of(
          planner_data_.goal_lanelets.begin(), planner_data_.goal_lanelets.end(),
          [&](const auto & goal_lanelet) { return lanelets.back().id() == goal_lanelet.id(); })) {
      const auto goal_arc_coordinates =
        lanelet::utils::getArcCoordinates(lanelets, planner_data_.goal_pose);
      s_end = std::min(s_end, goal_arc_coordinates.length);
    }

    if (
      const auto s_intersection = utils::get_first_intersection_arc_length(
        lanelets, std::max(0., s_start - vehicle_info_.max_longitudinal_offset_m),
        s_end + vehicle_info_.max_longitudinal_offset_m, vehicle_info_.vehicle_length_m)) {
      s_end =
        std::min(s_end, std::max(0., *s_intersection - vehicle_info_.max_longitudinal_offset_m));
    }

    return s_end;
  }();

  return generate_path(lanelets, s_start, s_end, params);
}

std::optional<PathWithLaneId> PathGenerator::generate_path(
  const lanelet::LaneletSequence & lanelet_sequence, const double s_start, const double s_end,
  const Params & params) const
{
  std::vector<PathPointWithLaneId> path_points_with_lane_id{};

  const auto s_bound_start = s_start - vehicle_info_.max_longitudinal_offset_m;
  const auto s_bound_end = s_end + vehicle_info_.max_longitudinal_offset_m;

  const auto waypoint_groups = utils::get_waypoint_groups(
    lanelet_sequence, *planner_data_.lanelet_map_ptr, params.waypoint_group.separation_threshold,
    params.waypoint_group.interval_margin_ratio);

  auto extended_lanelets = lanelet_sequence.lanelets();
  auto s_offset = 0.;

  {
    auto extended_lanelets_length = lanelet::geometry::length2d(lanelet_sequence);
    while (extended_lanelets_length < s_bound_end) {
      const auto next_lanelets =
        planner_data_.routing_graph_ptr->following(extended_lanelets.back());
      if (next_lanelets.empty()) {
        break;
      }
      extended_lanelets.push_back(next_lanelets.front());
      extended_lanelets_length += lanelet::geometry::length2d(next_lanelets.front());
    }
  }

  while (s_offset + s_bound_start < 0.) {
    const auto prev_lanelets = planner_data_.routing_graph_ptr->previous(extended_lanelets.front());
    if (prev_lanelets.empty()) {
      break;
    }
    extended_lanelets.insert(extended_lanelets.begin(), prev_lanelets.front());
    s_offset += lanelet::geometry::length2d(prev_lanelets.front());
  }

  for (const auto & [waypoints, interval] : waypoint_groups) {
    if (s_offset + interval.start > 0.) {
      continue;
    }
    const auto prev_lanelet =
      utils::get_previous_lanelet_within_route(extended_lanelets.front(), planner_data_);
    if (!prev_lanelet) {
      break;
    }
    extended_lanelets.insert(extended_lanelets.begin(), *prev_lanelet);
    s_offset += lanelet::geometry::length2d(*prev_lanelet);
  }

  const auto add_path_point = [&](const auto & path_point, const lanelet::ConstLanelet & lanelet) {
    PathPointWithLaneId path_point_with_lane_id{};
    path_point_with_lane_id.lane_ids.push_back(lanelet.id());
    path_point_with_lane_id.point.pose.position =
      lanelet::utils::conversion::toGeomMsgPt(path_point);
    path_point_with_lane_id.point.longitudinal_velocity_mps =
      planner_data_.traffic_rules_ptr->speedLimit(lanelet).speedLimit.value();
    path_points_with_lane_id.push_back(std::move(path_point_with_lane_id));
  };

  const lanelet::LaneletSequence extended_lanelet_sequence(extended_lanelets);
  std::optional<size_t> overlapping_waypoint_group_index = std::nullopt;

  for (auto lanelet_it = extended_lanelet_sequence.begin();
       lanelet_it != extended_lanelet_sequence.end(); ++lanelet_it) {
    const auto & centerline = lanelet_it->centerline();
    auto s = get_arc_length_along_centerline(extended_lanelet_sequence, centerline.front());

    for (auto point_it = centerline.begin(); point_it != centerline.end(); ++point_it) {
      if (point_it != centerline.begin()) {
        s += lanelet::geometry::distance2d(*std::prev(point_it), *point_it);
      } else if (lanelet_it != extended_lanelet_sequence.begin()) {
        continue;
      }

      if (overlapping_waypoint_group_index) {
        const auto & [waypoints, interval] = waypoint_groups[*overlapping_waypoint_group_index];
        if (s >= interval.start + s_offset && s <= interval.end + s_offset) {
          continue;
        }
        overlapping_waypoint_group_index = std::nullopt;
      }

      for (size_t i = 0; i < waypoint_groups.size(); ++i) {
        const auto & [waypoints, interval] = waypoint_groups[i];
        if (s < interval.start + s_offset || s > interval.end + s_offset) {
          continue;
        }
        for (const auto & waypoint : waypoints) {
          const auto s_waypoint =
            get_arc_length_along_centerline(extended_lanelet_sequence, waypoint);
          for (auto waypoint_lanelet_it = extended_lanelet_sequence.begin();
               waypoint_lanelet_it != extended_lanelet_sequence.end(); ++waypoint_lanelet_it) {
            if (
              s_waypoint > get_arc_length_along_centerline(
                             extended_lanelet_sequence, waypoint_lanelet_it->centerline().back())) {
              continue;
            }
            add_path_point(waypoint, *waypoint_lanelet_it);
            break;
          }
        }
        overlapping_waypoint_group_index = i;
        break;
      }
      if (overlapping_waypoint_group_index) {
        continue;
      }

      add_path_point(*point_it, *lanelet_it);
      if (
        point_it == std::prev(centerline.end()) &&
        lanelet_it != std::prev(extended_lanelet_sequence.end())) {
        if (
          lanelet_it != extended_lanelet_sequence.begin() ||
          lanelet_it->id() == lanelet_sequence.begin()->id()) {
          path_points_with_lane_id.back().lane_ids.push_back(std::next(lanelet_it)->id());
        } else {
          path_points_with_lane_id.back().lane_ids = {std::next(lanelet_it)->id()};
        }
      }
    }
  }

  auto trajectory = Trajectory::Builder().build(path_points_with_lane_id);
  if (!trajectory) {
    return std::nullopt;
  }

  // Attach orientation for all the points
  trajectory->align_orientation_with_trajectory_direction();

  double start =
    s_offset + s_start -
    get_arc_length_along_centerline(
      extended_lanelet_sequence, lanelet::utils::conversion::toLaneletPoint(
                                   path_points_with_lane_id.front().point.pose.position));

  const double length = std::max(0.1, s_end - s_start);

  // Refine the trajectory by cropping
  trajectory->crop(0, start + length);

  // Compose the polished path

  PathWithLaneId finalized_path_with_lane_id{};

  finalized_path_with_lane_id.points = trajectory->restore();

  // Check if the goal point is in the search range
  // Note: We only see if the goal is approaching the tail of the path.
  const auto distance_to_goal = autoware_utils::calc_distance2d(
    trajectory->compute(trajectory->length()), planner_data_.goal_pose);

  if (distance_to_goal < params.refine_goal_search_radius_range) {
    auto refined_path = utils::modify_path_for_smooth_goal_connection(
      *trajectory, planner_data_, params.refine_goal_search_radius_range);

    if (refined_path) {
      refined_path->align_orientation_with_trajectory_direction();
      *trajectory = *refined_path;
    }
  }

  if (!(trajectory->length() - start < 0)) {
    trajectory->crop(start, trajectory->length() - start);
  }
  finalized_path_with_lane_id.points = trajectory->restore();

  if (finalized_path_with_lane_id.points.empty()) {
    return std::nullopt;
  }

  // Set header which is needed to engage
  finalized_path_with_lane_id.header.frame_id = planner_data_.route_frame_id;
  finalized_path_with_lane_id.header.stamp = now();

  const auto [left_bound, right_bound] = utils::get_path_bounds(
    extended_lanelet_sequence, std::max(0., s_offset + s_bound_start),
    std::max(0., s_offset + s_bound_end));
  finalized_path_with_lane_id.left_bound = left_bound;
  finalized_path_with_lane_id.right_bound = right_bound;

  return finalized_path_with_lane_id;
}

bool PathGenerator::update_current_lanelet(
  const geometry_msgs::msg::Pose & current_pose, const Params & params)
{
  if (!current_lanelet_) {
    lanelet::ConstLanelet current_lanelet;
    if (lanelet::utils::query::getClosestLanelet(
          planner_data_.route_lanelets, current_pose, &current_lanelet)) {
      current_lanelet_ = current_lanelet;
      return true;
    }
    return false;
  }

  lanelet::ConstLanelets candidates;
  if (
    const auto previous_lanelet =
      utils::get_previous_lanelet_within_route(*current_lanelet_, planner_data_)) {
    candidates.push_back(*previous_lanelet);
  }
  candidates.push_back(*current_lanelet_);
  if (
    const auto next_lanelet =
      utils::get_next_lanelet_within_route(*current_lanelet_, planner_data_)) {
    candidates.push_back(*next_lanelet);
  }

  if (lanelet::utils::query::getClosestLaneletWithConstrains(
        candidates, current_pose, &*current_lanelet_, params.ego_nearest_dist_threshold,
        params.ego_nearest_yaw_threshold)) {
    return true;
  }

  if (lanelet::utils::query::getClosestLanelet(
        planner_data_.route_lanelets, current_pose, &*current_lanelet_)) {
    return true;
  }

  return false;
}
}  // namespace autoware::path_generator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::path_generator::PathGenerator)
