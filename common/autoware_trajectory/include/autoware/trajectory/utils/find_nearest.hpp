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

#include "autoware_utils_geometry/geometry.hpp"
#include "autoware_utils_geometry/pose_deviation.hpp"

#include <autoware_utils_system/backtrace.hpp>
#include <rclcpp/rclcpp.hpp>

#include <execinfo.h>

#include <limits>
#include <vector>

namespace autoware::experimental::trajectory
{
inline void print_backtrace()
{
  constexpr size_t max_frames = 100;
  void * addrlist[max_frames + 1];

  int addrlen = backtrace(addrlist, sizeof(addrlist) / sizeof(void *));

  if (addrlen == 0) {
    return;
  }

  char ** symbol_list = backtrace_symbols(addrlist, addrlen);

  std::stringstream ss;
  ss << "\n   @   ********** back trace **********" << std::endl;
  for (int i = 1; i < addrlen; i++) {
    ss << "   @   " << symbol_list[i] << std::endl;
  }
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("autoware_utils"), ss.str());

  free(symbol_list);
}

/**
 * @brief validate if points container is empty or not
 * @param points points of trajectory, path, ...
 */
template <class T>
void validate_non_empty(const T & points)
{
  if (points.empty()) {
    print_backtrace();  // TODO(mraditya01): is it ok to just import (from
                        // autoware_utils_system/backtrace.hpp)?
    throw std::invalid_argument("[autoware_motion_utils] validate_non_empty(): Points is empty.");
  }
}

/**
 * @brief find nearest point index through points container for a given point.
 * Finding nearest point is determined by looping through the points container,
 * and calculating the 2D squared distance between each point in the container and the given point.
 * The index of the point with minimum distance and yaw deviation comparing to the given point will
 * be returned.
 * @param points points of trajectory, path, ...
 * @param point given point
 * @return index of nearest point
 */
template <class T>
[[nodiscard]] size_t find_nearest_index(const T & points, const geometry_msgs::msg::Point & point)
{
  validate_non_empty(points);

  double min_dist = std::numeric_limits<double>::max();
  size_t min_idx = 0;

  for (size_t i = 0; i < points.size(); ++i) {
    const auto dist = autoware_utils_geometry::calc_squared_distance2d(points.at(i), point);
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }
  return min_idx;
}

/**
 * @brief find nearest point index through points container for a given pose.
 * Finding nearest point is determined by looping through the points container,
 * and finding the nearest point to the given pose in terms of squared 2D distance and yaw
 * deviation. The index of the point with minimum distance and yaw deviation comparing to the given
 * pose will be returned.
 * @param points points of trajectory, path, ...
 * @param pose given pose
 * @param max_dist max distance used to get squared distance for finding the nearest point to given
 * pose
 * @param max_yaw max yaw used for finding nearest point to given pose
 * @return index of nearest point (index or none if not found)
 */
template <class T>
std::optional<size_t> find_nearest_index(
  const T & points, const geometry_msgs::msg::Pose & pose,
  const double max_dist = std::numeric_limits<double>::max(),
  const double max_yaw = std::numeric_limits<double>::max())
{
  try {
    validate_non_empty(points);
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(rclcpp::get_logger("trajectory_utils"), "%s", e.what());
    return {};
  }

  const double max_squared_dist = max_dist * max_dist;

  double min_squared_dist = std::numeric_limits<double>::max();
  bool is_nearest_found = false;
  size_t min_idx = 0;

  for (size_t i = 0; i < points.size(); ++i) {
    const auto squared_dist = autoware_utils_geometry::calc_squared_distance2d(points.at(i), pose);
    if (squared_dist > max_squared_dist || squared_dist >= min_squared_dist) {
      continue;
    }

    const auto yaw = autoware_utils_geometry::calc_yaw_deviation(
      autoware_utils_geometry::get_pose(points.at(i)), pose);
    if (std::fabs(yaw) > max_yaw) {
      continue;
    }

    min_squared_dist = squared_dist;
    min_idx = i;
    is_nearest_found = true;
  }

  if (is_nearest_found) {
    return min_idx;
  }
  return std::nullopt;
}

}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__UTILS__FIND_NEAREST_HPP_
