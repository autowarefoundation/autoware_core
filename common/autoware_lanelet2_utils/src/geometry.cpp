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

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <range/v3/all.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LaneletSequence.h>
#include <lanelet2_core/primitives/Point.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <vector>

namespace autoware::experimental::lanelet2_utils
{
lanelet::ConstPoint3d extrapolate_point(
  const lanelet::ConstPoint3d & first, const lanelet::ConstPoint3d & second, const double distance)
{
  const double segment_length = lanelet::geometry::distance3d(first, second);
  if (segment_length == 0.0) {
    return first;
  }

  return lanelet::ConstPoint3d{
    lanelet::InvalId,
    second.basicPoint() + (second.basicPoint() - first.basicPoint()).normalized() * distance};
}

std::optional<lanelet::ConstPoint3d> interpolate_point(
  const lanelet::ConstPoint3d & first, const lanelet::ConstPoint3d & second, const double distance)
{
  const double segment_length = lanelet::geometry::distance3d(first, second);

  if (segment_length == 0.0 || distance < 0.0 || distance > segment_length) {
    return std::nullopt;
  }

  return lanelet::ConstPoint3d{
    lanelet::InvalId,
    first.basicPoint() + (second.basicPoint() - first.basicPoint()).normalized() * distance};
}

template <typename Line>
std::optional<lanelet::ConstPoint3d> interpolate_linestring(
  const Line & linestring, double distance)
{
  if (linestring.size() < 2) {
    return std::nullopt;
  }

  const double total_length = lanelet::geometry::length(linestring);
  if (distance < 0.0 || distance > total_length) {
    return std::nullopt;
  }

  double accumulated = 0.0;
  for (std::size_t i = 0; i + 1 < linestring.size(); ++i) {
    const auto & p1 = linestring[i];
    const auto & p2 = linestring[i + 1];
    const double seg_len = lanelet::geometry::distance3d(p1, p2);
    if (accumulated + seg_len >= distance) {
      return interpolate_point(p1, p2, distance - accumulated);
    }
    accumulated += seg_len;
  }

  return std::nullopt;
}

std::optional<lanelet::ConstPoint3d> interpolate_lanelet(
  const lanelet::ConstLanelet & lanelet, const double distance)
{
  const auto linestring = lanelet.centerline();
  return interpolate_linestring(linestring, distance);
}

std::optional<lanelet::ConstPoint3d> interpolate_lanelet_sequence(
  const lanelet::ConstLanelets & lanelet_sequence, double distance)
{
  const auto merged_sequence = lanelet::LaneletSequence(lanelet_sequence);
  const auto centerline = merged_sequence.centerline();
  return interpolate_linestring(centerline, distance);
  ;
}

std::optional<lanelet::CompoundLineString3d> concatenate_center_line(
  const lanelet::ConstLanelets & lanelets)
{
  if (lanelets.empty()) {
    return std::nullopt;
  }
  const auto merged_sequence = lanelet::LaneletSequence(lanelets);
  return merged_sequence.centerline();
}

std::optional<lanelet::ConstLineString3d> get_linestring_from_arc_length(
  const lanelet::ConstLineString3d & linestring, const double s1, const double s2)
{
  lanelet::ConstPoints3d points;
  double accumulated_length = 0;
  size_t start_index = linestring.size();
  if (linestring.size() < 2) {
    return std::nullopt;
  }

  const double total_length = lanelet::geometry::length(linestring);
  if (s1 < 0.0 || s2 > total_length || s1 >= s2) {
    return std::nullopt;
  }
  const size_t last_linestring_idx = linestring.size() - 1;

  for (size_t i = 0; i < last_linestring_idx; i++) {
    const auto & p1 = linestring[i];
    const auto & p2 = linestring[i + 1];
    const double length = lanelet::geometry::distance3d(p1, p2);

    if (accumulated_length + length > s1) {
      start_index = i;
      break;
    }
    accumulated_length += length;
  }

  if (start_index < last_linestring_idx) {
    const auto & p1 = linestring[start_index];
    const auto & p2 = linestring[start_index + 1];
    const double residue = s1 - accumulated_length;

    const auto start_point = interpolate_point(p1, p2, residue);
    if (!start_point.has_value()) return std::nullopt;
    points.emplace_back(start_point.value());
  }

  size_t end_index = linestring.size();
  for (size_t i = start_index; i < last_linestring_idx; i++) {
    const auto & p1 = linestring[i];
    const auto & p2 = linestring[i + 1];
    const double length = lanelet::geometry::distance3d(p1, p2);
    if (accumulated_length + length > s2) {
      end_index = i;
      break;
    }
    accumulated_length += length;
  }

  for (size_t i = start_index + 1; i < end_index; i++) {
    const auto p = lanelet::Point3d(linestring[i]);
    points.emplace_back(p);
  }
  if (end_index < last_linestring_idx) {
    const auto & p1 = linestring[end_index];
    const auto & p2 = linestring[end_index + 1];
    const double residue = s2 - accumulated_length;
    const auto end_point = interpolate_point(p1, p2, residue);
    points.emplace_back(linestring[end_index]);

    if (!end_point.has_value()) return std::nullopt;

    points.emplace_back(lanelet::InvalId, end_point.value());
  }
  return create_safe_linestring(points);
}

std::optional<geometry_msgs::msg::Pose> get_pose_from_2d_arc_length(
  const lanelet::ConstLanelets & lanelet_sequence, const double s)
{
  double accumulated_distance2d = 0.0;

  for (const auto & llt : lanelet_sequence) {
    const auto & centerline = llt.centerline();
    for (auto it = centerline.begin(); std::next(it) != centerline.end(); ++it) {
      const auto & pt = *it;
      const auto & next_pt = *std::next(it);
      const double distance2d = lanelet::geometry::distance3d(pt, next_pt);

      if (accumulated_distance2d + distance2d > s) {
        double rem = s - accumulated_distance2d;
        auto const_pt = interpolate_point(pt, next_pt, rem);
        if (!const_pt.has_value()) {
          return std::nullopt;
        }
        auto P = const_pt.value().basicPoint();

        double half_yaw = std::atan2(next_pt.y() - pt.y(), next_pt.x() - pt.x()) * 0.5;

        geometry_msgs::msg::Pose pose;
        pose.position.x = P.x();
        pose.position.y = P.y();
        pose.position.z = P.z();
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = std::sin(half_yaw);
        pose.orientation.w = std::cos(half_yaw);

        return pose;
      }

      accumulated_distance2d += distance2d;
    }
  }
  return std::nullopt;
}

lanelet::ConstLineString3d get_closest_segment(
  const lanelet::ConstLineString3d & linestring, const lanelet::BasicPoint3d & search_pt)
{
  lanelet::ConstLineString3d closest_segment;
  double min_distance = std::numeric_limits<double>::max();

  for (const auto & [prev_pt, current_pt] :
       ranges::views::zip(linestring, linestring | ranges::views::drop(1))) {
    lanelet::Point3d prev_pt_3d(lanelet::InvalId, prev_pt.x(), prev_pt.y(), prev_pt.z());
    lanelet::Point3d current_pt_3d(
      lanelet::InvalId, current_pt.x(), current_pt.y(), current_pt.z());

    lanelet::ConstLineString3d current_segment(lanelet::InvalId, {prev_pt_3d, current_pt_3d});
    double distance = lanelet::geometry::distance3d(current_segment.basicLineString(), search_pt);
    if (distance < min_distance) {
      closest_segment = current_segment;
      min_distance = distance;
    }
  }
  return closest_segment;
}

double get_lanelet_angle(
  const lanelet::ConstLanelet & lanelet, const lanelet::BasicPoint3d & search_pt)
{
  lanelet::ConstLineString3d segment = get_closest_segment(lanelet.centerline(), search_pt);
  return std::atan2(
    segment.back().y() - segment.front().y(), segment.back().x() - segment.front().x());
}

geometry_msgs::msg::Pose get_closest_center_pose(
  const lanelet::ConstLanelet & lanelet, const lanelet::BasicPoint3d & search_pt)
{
  lanelet::ConstLineString3d segment = get_closest_segment(lanelet.centerline(), search_pt);
  if (segment.empty()) {
    geometry_msgs::msg::Pose closest_pose;
    closest_pose.position = to_ros(lanelet.centerline().front(), search_pt.z());
    closest_pose.orientation.x = 0.0;
    closest_pose.orientation.y = 0.0;
    closest_pose.orientation.z = 0.0;
    closest_pose.orientation.w = 1.0;
    return closest_pose;
  }

  const Eigen::Vector2d direction(
    (segment.back().basicPoint2d() - segment.front().basicPoint2d()).normalized());
  const Eigen::Vector2d xf(segment.front().basicPoint2d());
  const Eigen::Vector2d x(search_pt.x(), search_pt.y());
  const Eigen::Vector2d p = xf + (x - xf).dot(direction) * direction;

  geometry_msgs::msg::Pose closest_pose;
  closest_pose.position = to_ros(p, search_pt.z());

  const double lane_yaw = get_lanelet_angle(lanelet, search_pt);
  closest_pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(lane_yaw);

  return closest_pose;
}

}  // namespace autoware::experimental::lanelet2_utils
