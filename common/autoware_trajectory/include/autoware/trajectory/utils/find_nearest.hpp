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
#include "autoware/trajectory/threshold.hpp"
#include "autoware_utils_geometry/geometry.hpp"
#include "autoware_utils_geometry/pose_deviation.hpp"

#include <limits>
#include <vector>

namespace autoware::experimental::trajectory
{

/**
 * @brief Find the corresponding s value on the trajectory for a given point container.
 * @details Nearest point is determined by performing ternary search between the front
 * and back of baseline points to refine, and return the point with the minimum distance.
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

  double search_start = bases.front();
  double search_end = bases.back();

  double min_dist = std::numeric_limits<double>::infinity();
  double min_s = (search_start + search_end) * 0.5;

  while (search_end - search_start > k_points_minimum_dist_threshold) {
    const double mid1 = search_start + (search_end - search_start) / 3.0;
    const double mid2 = search_end - (search_end - search_start) / 3.0;

    const auto pose1 = trajectory.compute(mid1);
    const auto pose2 = trajectory.compute(mid2);

    const double dist1 = autoware_utils_geometry::calc_squared_distance2d(pose1.position, point);
    const double dist2 = autoware_utils_geometry::calc_squared_distance2d(pose2.position, point);

    if (dist1 < min_dist) {
      min_dist = dist1;
      min_s = mid1;
    }
    if (dist2 < min_dist) {
      min_dist = dist2;
      min_s = mid2;
    }

    if (dist1 < dist2) {
      search_end = mid2;
    } else {
      search_start = mid1;
    }
  }

  if (min_dist == std::numeric_limits<double>::infinity()) {
    return std::nullopt;
  }
  return min_s;
}

/**
 * @brief Find the corresponding s value on the trajectory for a given pose.
 * @details Nearest point is determined by performing ternary search between the front
 * and back of baseline points to refine, and return the point with the minimum distance.
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
  double search_start = bases.front();
  double search_end = bases.back();

  double min_dist = std::numeric_limits<double>::infinity();
  double min_s = (search_start + search_end) * 0.5;

  while (search_end - search_start > k_points_minimum_dist_threshold) {
    const double mid1 = search_start + (search_end - search_start) / 3.0;
    const double mid2 = search_end - (search_end - search_start) / 3.0;

    const auto pose1 = trajectory.compute(mid1);
    double squared_dist1 =
      autoware_utils_geometry::calc_squared_distance2d(pose1.position, pose.position);
    const double yaw_dev1 = autoware_utils_geometry::calc_yaw_deviation(pose1, pose);

    if (squared_dist1 <= max_squared_dist && std::fabs(yaw_dev1) <= max_yaw) {
      if (squared_dist1 < min_dist) {
        min_dist = squared_dist1;
        min_s = mid1;
      }
    } else {
      squared_dist1 = std::numeric_limits<double>::infinity();
    }

    const auto pose2 = trajectory.compute(mid2);
    double squared_dist2 =
      autoware_utils_geometry::calc_squared_distance2d(pose2.position, pose.position);
    const double yaw_dev2 = autoware_utils_geometry::calc_yaw_deviation(pose2, pose);

    if (squared_dist2 <= max_squared_dist && std::fabs(yaw_dev2) <= max_yaw) {
      if (squared_dist2 < min_dist) {
        min_dist = squared_dist2;
        min_s = mid2;
      }
    } else {
      squared_dist2 = std::numeric_limits<double>::infinity();
    }

    if (squared_dist1 < squared_dist2) {
      search_end = mid2;
    } else {
      search_start = mid1;
    }
  }

  if (min_dist == std::numeric_limits<double>::infinity()) {
    return std::nullopt;
  }
  return min_s;
}

}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__UTILS__FIND_NEAREST_HPP_
