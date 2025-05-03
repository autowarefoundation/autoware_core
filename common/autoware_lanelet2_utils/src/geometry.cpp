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

#include <autoware/lanelet2_utils/geometry.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>

#include <algorithm>
#include <iostream>
#include <vector>

namespace autoware::lanelet2_utils
{
double compute_linestring_length3d(const lanelet::ConstLineString3d & linestring)
{
  double total_length = 0.0;
  for (size_t i = 0; i + 1 < linestring.size(); ++i) {
    const auto & p1 = linestring[i].basicPoint();
    const auto & p2 = linestring[i + 1].basicPoint();

    double dx = p2.x() - p1.x();
    double dy = p2.y() - p1.y();
    double dz = p2.z() - p1.z();

    total_length += std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  return total_length;
}

std::optional<lanelet::ConstPoint3d> extrapolate_point(
  const lanelet::ConstPoint3d & first, const lanelet::ConstPoint3d & second, const double distance)
{
  double dx = second.x() - first.x();
  double dy = second.y() - first.y();
  double dz = second.z() - first.z();

  double segment_length = std::hypot(dx, dy, dz);
  if (segment_length == 0.0) {
    return std::nullopt;
  }

  double normalized = distance / segment_length;
  double new_x, new_y, new_z;
  new_x = second.x() + normalized * dx;
  new_y = second.y() + normalized * dy;
  new_z = second.z() + normalized * dz;
  return lanelet::ConstPoint3d(lanelet::InvalId, new_x, new_y, new_z);
}

std::optional<lanelet::ConstPoint3d> interpolate_point(
  const lanelet::ConstPoint3d & first, const lanelet::ConstPoint3d & second, const double distance,
  const bool from_first)
{
  double dx = second.x() - first.x();
  double dy = second.y() - first.y();
  double dz = second.z() - first.z();

  double segment_length = std::hypot(dx, dy, dz);

  if (segment_length == 0.0 || distance < 0.0 || distance > segment_length) {
    return std::nullopt;
  }

  double normalized = distance / segment_length;
  double new_x, new_y, new_z;

  if (from_first) {
    new_x = first.x() + normalized * dx;
    new_y = first.y() + normalized * dy;
    new_z = first.z() + normalized * dz;
  } else {
    new_x = second.x() - normalized * dx;
    new_y = second.y() - normalized * dy;
    new_z = second.z() - normalized * dz;
  }

  return lanelet::ConstPoint3d(lanelet::InvalId, new_x, new_y, new_z);
}

std::optional<lanelet::ConstPoint3d> interpolate_lanelet(
  const lanelet::ConstLanelet & lanelet, const double distance)
{
  std::vector<lanelet::Point3d> points;
  const auto linestring = lanelet.centerline();

  if (linestring.size() < 2) {
    return std::nullopt;
  }

  const double total_length = compute_linestring_length3d(linestring);
  if (distance < 0.0 || distance > total_length) {
    return std::nullopt;
  }

  double accumulated_length = 0.0;

  for (std::size_t idx = 0; idx < linestring.size() - 1; ++idx) {
    const auto & p1 = linestring[idx];
    const auto & p2 = linestring[idx + 1];
    double segment_length = std::hypot(p2.x() - p1.x(), p2.y() - p1.y(), p2.z() - p1.z());
    if (accumulated_length + segment_length >= distance) {
      double residue = distance - accumulated_length;
      return interpolate_point(linestring[idx], linestring[idx + 1], residue);
    }
    accumulated_length += segment_length;
  }
  return std::nullopt;
}

std::optional<lanelet::ConstPoint3d> interpolate_lanelet_sequence(
  const lanelet::ConstLanelets & lanelet_sequence, double distance)
{
  for (const auto & llt : lanelet_sequence) {
    auto interpolated_pt = interpolate_lanelet(llt, distance);
    if (interpolated_pt.has_value()) {
      return interpolated_pt;
    }
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLineString3d> concatenate_center_line(
  const lanelet::ConstLanelets & lanelets)
{
  if (lanelets.empty()) {
    return std::nullopt;
  }

  std::vector<lanelet::Point3d> pts;
  pts.reserve(lanelets.size() * 10);

  for (const auto & llt : lanelets) {
    const auto & center_line = llt.centerline();
    for (const auto & cl_point : center_line) {
      if (pts.empty() || pts.back().basicPoint() != cl_point.basicPoint()) {
        pts.emplace_back(cl_point);
      }
    }
  }

  return lanelet::ConstLineString3d{lanelet::InvalId, pts};
}

std::optional<lanelet::LineString3d> get_linestring_from_arc_length(
  const lanelet::ConstLineString3d & linestring, const double s1, const double s2)
{
  lanelet::Points3d points;
  double accumulated_length = 0;
  size_t start_index = linestring.size();
  if (linestring.size() < 2) {
    return std::nullopt;
  }

  const double total_length = compute_linestring_length3d(linestring);
  if (s1 < 0.0 || s2 > total_length || s1 >= s2) {
    return std::nullopt;
  }

  for (size_t i = 0; i < linestring.size() - 1; i++) {
    const auto & p1 = linestring[i];
    const auto & p2 = linestring[i + 1];
    const double length = std::hypot(p2.x() - p1.x(), p2.y() - p1.y(), p2.z() - p1.z());

    if (accumulated_length + length > s1) {
      start_index = i;
      break;
    }
    accumulated_length += length;
  }

  if (start_index < linestring.size() - 1) {
    const auto & p1 = linestring[start_index];
    const auto & p2 = linestring[start_index + 1];
    const double residue = s1 - accumulated_length;

    const auto start_point = interpolate_point(p1, p2, residue);
    if (!start_point.has_value()) return std::nullopt;
    points.emplace_back(start_point.value());
  }

  size_t end_index = linestring.size();
  for (size_t i = start_index; i < linestring.size() - 1; i++) {
    const auto & p1 = linestring[i];
    const auto & p2 = linestring[i + 1];
    const double length = std::hypot(p2.x() - p1.x(), p2.y() - p1.y(), p2.z() - p1.z());
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
  if (end_index < linestring.size() - 1) {
    const auto & p1 = linestring[end_index];
    const auto & p2 = linestring[end_index + 1];
    const double residue = s2 - accumulated_length;
    const auto end_point = interpolate_point(p1, p2, residue);
    points.emplace_back(lanelet::Point3d(linestring[end_index]));

    if (!end_point.has_value()) return std::nullopt;

    points.emplace_back(lanelet::Point3d(lanelet::InvalId, end_point.value()));
  }
  return lanelet::LineString3d{lanelet::InvalId, points};
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
      double distance2d = std::hypot(next_pt.x() - pt.x(), next_pt.y() - pt.y());

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
        geometry_msgs::msg::Quaternion q;
        q.x = 0.0;
        q.y = 0.0;
        q.z = std::sin(half_yaw);
        q.w = std::cos(half_yaw);
        pose.orientation = q;
        return pose;
      }

      accumulated_distance2d += distance2d;
    }
  }
  return std::nullopt;
}

}  // namespace autoware::lanelet2_utils
