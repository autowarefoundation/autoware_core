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

#ifndef AUTOWARE__TRAJECTORY__UTILS__FIND_NEAREST_HPP_
#define AUTOWARE__TRAJECTORY__UTILS__FIND_NEAREST_HPP_

#include "autoware/trajectory/detail/types.hpp"
#include "autoware/trajectory/forward.hpp"
#include "autoware_utils_geometry/geometry.hpp"
#include "autoware_utils_geometry/pose_deviation.hpp"

#include <limits>
#include <vector>

namespace autoware::experimental::trajectory
{

/**
 * @brief Find the corresponding s value on the trajectory for a given point container.
 * Nearest point is determined by discretely checking trajectory.get_underlying_bases()
 * to find the closest base point to the query pose in terms of 2D Euclidean distance.
 * A ternary search is then performed between the adjacent discrete points to refine
 * and return the point with the minimum distance.
 * @param trajectory Continuous trajectory object
 * @param point given point
 * @return distance of nearest point in the trajectory (distance or none if not found)
 */
template <class TrajectoryPointType>
[[nodiscard]] std::optional<double> find_nearest_index(
  const Trajectory<TrajectoryPointType> & trajectory, const geometry_msgs::msg::Point & point)
{
  const auto bases = trajectory.get_underlying_bases();
  if (bases.empty()) {
    return std::nullopt;
  }

  bool is_nearest_found = false;
  size_t closest_index = 0;
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < bases.size(); ++i) {
    const auto point = trajectory.compute(bases[i]);
    const auto squared_dist =
      autoware_utils_geometry::calc_squared_distance2d(point.position, point);
    if (squared_dist < min_dist) {
      min_dist = squared_dist;
      closest_index = i;
      bool is_nearest_found = false;
    }
  }

  if (is_nearest_found) {
    double search_start = (closest_index == 0) ? bases[closest_index] : bases[closest_index - 1];
    double search_end =
      (closest_index == bases.size() - 1) ? bases[closest_index] : bases[closest_index + 1];

    while (search_end - search_start > 1e-4) {
      const double mid1 = search_start + (search_end - search_start) / 3.0;
      const double mid2 = search_end - (search_end - search_start) / 3.0;

      const auto point1 = trajectory.compute(mid1);
      const auto point2 = trajectory.compute(mid2);

      const double dist1 = autoware_utils_geometry::calc_squared_distance2d(point1.position, point);
      const double dist2 = autoware_utils_geometry::calc_squared_distance2d(point2.position, point);

      if (dist1 < dist2) {
        search_end = mid2;
      } else {
        search_start = mid1;
      }
    }
    return (search_start + search_end) / 2.0;
  }
  return std::nullopt;
}

/**
 * @brief Find the corresponding s value on the trajectory for a given pose.
 * Nearest point is determined by discretely checking trajectory.get_underlying_bases()
 * to find the closest base point to the query pose in terms of 2D Euclidean distance.
 * A ternary search is then performed between the adjacent discrete points to refine
 * and return the point with the minimum distance.
 * @param trajectory Continuous trajectory object
 * @param pose given pose
 * @param max_dist max distance used to get squared distance for finding the nearest point to given
 * pose
 * @param max_yaw max yaw used for finding nearest point to given pose
 * @return distance of nearest point in the trajectory (distance or none if not found)
 */

template <class TrajectoryPointType>
std::optional<double> find_nearest_index(
  const Trajectory<TrajectoryPointType> & trajectory, const geometry_msgs::msg::Pose & pose,
  const double max_dist = std::numeric_limits<double>::max(),
  const double max_yaw = std::numeric_limits<double>::max())

{
  const auto bases = trajectory.get_underlying_bases();
  if (bases.empty()) {
    return std::nullopt;
  }

  const double max_squared_dist = max_dist * max_dist;

  double min_squared_dist = std::numeric_limits<double>::max();
  bool is_nearest_found = false;
  size_t closest_index = 0;

  for (size_t i = 0; i < bases.size(); ++i) {
    const auto point = trajectory.compute(bases[i]);
    const auto squared_dist =
      autoware_utils_geometry::calc_squared_distance2d(point.position, pose.position);
    if (squared_dist >= max_squared_dist || squared_dist >= min_squared_dist) {
      continue;
    }
    const auto yaw = autoware_utils_geometry::calc_yaw_deviation(point, pose);
    if (std::fabs(yaw) > max_yaw) {
      continue;
    }

    min_squared_dist = squared_dist;
    is_nearest_found = true;
    closest_index = i;
  }

  if (is_nearest_found) {
    double search_start = (closest_index == 0) ? bases[closest_index] : bases[closest_index - 1];
    double search_end =
      (closest_index == bases.size() - 1) ? bases[closest_index] : bases[closest_index + 1];

    while (search_end - search_start > 1e-4) {
      const double mid1 = search_start + (search_end - search_start) / 3.0;
      const double mid2 = search_end - (search_end - search_start) / 3.0;

      const auto point1 = trajectory.compute(mid1);
      const auto point2 = trajectory.compute(mid2);

      const double dist1 =
        autoware_utils_geometry::calc_squared_distance2d(point1.position, pose.position);
      const double dist2 =
        autoware_utils_geometry::calc_squared_distance2d(point2.position, pose.position);

      if (dist1 < dist2) {
        search_end = mid2;
      } else {
        search_start = mid1;
      }
    }

    return (search_start + search_end) / 2.0;
  }
  return std::nullopt;
}

}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__UTILS__FIND_NEAREST_HPP_
