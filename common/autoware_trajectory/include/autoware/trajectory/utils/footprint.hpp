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

#ifndef AUTOWARE__TRAJECTORY__UTILS__FOOTPRINT_HPP_
#define AUTOWARE__TRAJECTORY__UTILS__FOOTPRINT_HPP_

#include "autoware/trajectory/point.hpp"
#include "autoware/trajectory/pose.hpp"
#include "autoware/trajectory/threshold.hpp"

#include <autoware_utils_geometry/boost_polygon_utils.hpp>

#include <boost/geometry/algorithms/correct.hpp>

#include <vector>

namespace
{
/** @brief convert LinearRing2d to Polygon2d
 *  @param footprint LinearRing2d
 *  @return Polygon2d
 */
autoware_utils_geometry::Polygon2d convert_linear_ring_to_polygon(
  autoware_utils_geometry::LinearRing2d footprint)
{
  autoware_utils_geometry::Polygon2d footprint_polygon;
  for (auto point : footprint) {
    footprint_polygon.outer().push_back(point);
  }

  boost::geometry::correct(footprint_polygon);
  return footprint_polygon;
}
}  // namespace

namespace autoware::experimental::trajectory
{

/** @brief create long polygon along trajectory from start_s to end_s with designated width
 *  @param trajectory trajectory
 *  @param start_s start arc-length
 *  @param end_s end arc_length
 *  @param width width of the polygon (vehicle_info.width/2)
 *  @return Polygon2d (one)
 */
template <class TrajectoryPointType>
autoware_utils_geometry::Polygon2d build_path_polygon(
  const Trajectory<TrajectoryPointType> & trajectory, const double start_s, const double end_s,
  const double width)
{
  autoware_utils_geometry::Polygon2d long_polygon;
  for (auto target_s = start_s; target_s <= end_s; target_s += k_points_minimum_dist_threshold) {
    const auto p = trajectory.compute(target_s);
    const auto yaw = autoware_utils_geometry::get_rpy(p).z;
    const double x = p.position.x + width * std::sin(yaw);
    const double y = p.position.y - width * std::cos(yaw);
    long_polygon.outer().push_back(autoware_utils_geometry::Point2d(x, y));
  }
  for (auto target_s = end_s; target_s >= start_s; target_s -= k_points_minimum_dist_threshold) {
    const auto p = trajectory.compute(target_s);
    const auto yaw = autoware_utils_geometry::get_rpy(p).z;
    const double x = p.position.x - width * std::sin(yaw);
    const double y = p.position.y + width * std::cos(yaw);
    long_polygon.outer().push_back(autoware_utils_geometry::Point2d(x, y));
  }

  boost::geometry::correct(long_polygon);
  return long_polygon;
}

/** @brief create footprints trace along trajectory from start_s to end_s with size of base
 * footprint (LinearRing2d)
 *  @param trajectory trajectory
 *  @param start_s start arc-length
 *  @param end_s end arc_length
 *  @param base_footprint base footprint (LinearRing2d)
 *  @return vector of Polygon2d (several footprints - footprint trace)
 */
template <class TrajectoryPointType>
std::vector<autoware_utils_geometry::Polygon2d> build_path_footprints(
  const Trajectory<TrajectoryPointType> & trajectory, const double start_s, const double end_s,
  const autoware_utils_geometry::LinearRing2d & base_footprint)
{
  std::vector<autoware_utils_geometry::Polygon2d> footprints;
  double interval = 1.0;
  auto footprint_size = (end_s - start_s) / interval;
  footprints.reserve(footprint_size);
  for (auto target_s = start_s; target_s <= end_s; target_s += interval) {
    const auto p = trajectory.compute(target_s);
    auto fp = autoware_utils_geometry::transform_vector(
      base_footprint, autoware_utils_geometry::pose2transform(p));
    footprints.push_back(convert_linear_ring_to_polygon(fp));
  }
  return footprints;
}

}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__UTILS__FOOTPRINT_HPP_
