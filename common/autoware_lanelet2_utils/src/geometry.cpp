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

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>

#include <algorithm>
#include <iostream>

namespace autoware::lanelet2_utils
{
lanelet::ConstPoint3d extrapolate_point(
  const lanelet::ConstPoint3d & first, const lanelet::ConstPoint3d & second, const double distance,
  const bool is_forward)
{
  double dx = second.x() - first.x();
  double dy = second.y() - first.y();
  double dz = second.z() - first.z();

  double segment_length = std::hypot(dx, dy, dz);
  if (segment_length == 0.0) {
    return is_forward ? first : second;
  }

  double normalized = distance / segment_length;
  double new_x, new_y, new_z;
  if (!is_forward) {
    new_x = first.x() - normalized * dx;
    new_y = first.y() - normalized * dy;
    new_z = first.z() - normalized * dz;
  } else {
    new_x = second.x() + normalized * dx;
    new_y = second.y() + normalized * dy;
    new_z = second.z() + normalized * dz;
  }
  return lanelet::ConstPoint3d(lanelet::InvalId, new_x, new_y, new_z);
}

lanelet::ConstPoint3d interpolate_point(
  const lanelet::ConstPoint3d & first, const lanelet::ConstPoint3d & second, const double distance,
  const bool from_first)
{
  double dx = second.x() - first.x();
  double dy = second.y() - first.y();
  double dz = second.z() - first.z();

  double segment_length = std::hypot(dx, dy, dz);

  if (segment_length == 0.0 || distance < 0.0 || distance > segment_length) {
    return from_first ? first : second;
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

lanelet::ConstLineString3d interpolate_linestring(
  const lanelet::ConstLineString3d & linestring, const double distance, const bool from_first)
{
  std::vector<lanelet::Point3d> points;

  if (linestring.size() < 2) {
    return lanelet::ConstLineString3d(lanelet::InvalId, points);
  }

  std::size_t segment_idx = 0;
  double accumulated_length = 0.0;

  if (from_first) {
    for (std::size_t i = 0; i < linestring.size() - 1; ++i) {
      const auto & p1 = linestring[i];
      const auto & p2 = linestring[i + 1];
      double segment_length = std::hypot(p2.x() - p1.x(), p2.y() - p1.y(), p2.z() - p1.z());
      if (accumulated_length + segment_length >= distance) {
        segment_idx = i;
        break;
      }
      accumulated_length += segment_length;
    }

    double residue = distance - accumulated_length;
    lanelet::ConstPoint3d interp_pt =
      interpolate_point(linestring[segment_idx], linestring[segment_idx + 1], residue, true);

    for (std::size_t i = 0; i <= segment_idx; i++) {
      points.push_back(lanelet::Point3d(linestring[i]));
    }
    points.push_back(lanelet::Point3d(interp_pt));
    for (std::size_t i = segment_idx + 1; i < linestring.size(); i++) {
      points.push_back(lanelet::Point3d(linestring[i]));
    }

  } else {
    accumulated_length = 0.0;
    for (std::size_t i = linestring.size() - 1; i > 0; --i) {
      const auto & p2 = linestring[i];
      const auto & p1 = linestring[i - 1];
      double segment_length = std::hypot(p2.x() - p1.x(), p2.y() - p1.y(), p2.z() - p1.z());
      if (accumulated_length + segment_length >= distance) {
        segment_idx = i - 1;
        break;
      }
      accumulated_length += segment_length;
    }
    double residue = distance - accumulated_length;
    lanelet::ConstPoint3d interp_pt =
      interpolate_point(linestring[segment_idx], linestring[segment_idx + 1], residue, false);

    for (std::size_t i = 0; i <= segment_idx; i++) {
      points.push_back(lanelet::Point3d(linestring[i]));
    }
    points.push_back(lanelet::Point3d(interp_pt));
    for (std::size_t i = segment_idx + 1; i < linestring.size(); i++) {
      points.push_back(lanelet::Point3d(linestring[i]));
    }
  }

  return lanelet::ConstLineString3d(lanelet::InvalId, points);
}

lanelet::ConstLineString3d extrapolate_linestring(
  const lanelet::ConstLineString3d & linestring, const double distance, const bool is_forward)
{
  if (linestring.size() < 2) {
    return linestring;
  }
  std::vector<lanelet::Point3d> new_points;

  if (is_forward) {
    const auto & p1 = linestring[0];
    const auto & p2 = linestring[1];
    lanelet::ConstPoint3d extra_point = extrapolate_point(p1, p2, distance, false);

    new_points.push_back(lanelet::Point3d(extra_point));
    for (const auto & p : linestring) {
      new_points.push_back(lanelet::Point3d(p));
    }
  } else {
    const auto & p1 = linestring[linestring.size() - 2];
    const auto & p2 = linestring.back();
    lanelet::ConstPoint3d extra_point = extrapolate_point(p1, p2, distance, true);

    for (const auto & p : linestring) {
      new_points.push_back(lanelet::Point3d(p));
    }
    new_points.push_back(lanelet::Point3d(extra_point));
  }

  return lanelet::ConstLineString3d(lanelet::InvalId, new_points);
}

}  // namespace autoware::lanelet2_utils
