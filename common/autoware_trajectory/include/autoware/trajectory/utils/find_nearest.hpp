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
#include "autoware/trajectory/path_point.hpp"
#include "autoware/trajectory/path_point_with_lane_id.hpp"
#include "autoware/trajectory/point.hpp"
#include "autoware/trajectory/pose.hpp"
#include "autoware/trajectory/threshold.hpp"
#include "autoware/trajectory/trajectory_point.hpp"
#include "autoware_utils_geometry/geometry.hpp"
#include "autoware_utils_geometry/pose_deviation.hpp"

#include <limits>
#include <vector>

namespace autoware::experimental::trajectory
{
namespace detail
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
[[nodiscard]] double find_precise_index(
  const Trajectory<TrajectoryPointType> & trajectory, const geometry_msgs::msg::Point & point,
  const size_t & min_idx)
{
  const auto bases = trajectory.get_underlying_bases();

  double search_start = (min_idx == 0) ? bases[0ul] : bases[min_idx - 1];
  double search_end = (min_idx == bases.size() - 1) ? bases[bases.size() - 1] : bases[min_idx + 1];

  double min_dist = std::numeric_limits<double>::infinity();
  double min_s = (search_start + search_end) * 0.5;

  while (search_end - search_start > k_points_minimum_dist_threshold) {
    const double mid1 = search_start + (search_end - search_start) / 3.0;
    const double mid2 = search_end - (search_end - search_start) / 3.0;

    const auto pose1 = trajectory.compute(mid1);
    const auto pose2 = trajectory.compute(mid2);

    const double dist1 = autoware_utils_geometry::calc_squared_distance2d(pose1, point);
    const double dist2 = autoware_utils_geometry::calc_squared_distance2d(pose2, point);

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

  return min_s;
}

/**
 * @brief Find the corresponding s value on the trajectory for a given pose.
 * @details Nearest point is determined by performing ternary search between the front
 * and back of baseline points to refine, and return the point with the minimum distance.
 * @param trajectory Continuous trajectory object
 * @param pose given pose
 * @param min_idx the closest base point's index for scoping range of Ternary Search.
 * @param dist_threshold max distance used to get squared distance for finding the nearest point to
 * given pose
 * @param yaw_threshold max yaw used for finding nearest point to given pose
 * @return distance of nearest point in the trajectory (distance or none if not found)
 */
template <class TrajectoryPointType>
std::optional<double> find_precise_index(
  const Trajectory<TrajectoryPointType> & trajectory, const geometry_msgs::msg::Pose & pose,
  const size_t & min_idx, const double dist_threshold = std::numeric_limits<double>::max(),
  const double yaw_threshold = std::numeric_limits<double>::max())
{
  const auto bases = trajectory.get_underlying_bases();

  double squared_dist_threshold = dist_threshold * dist_threshold;

  double search_start = (min_idx == 0) ? bases[0ul] : bases[min_idx - 1];
  double search_end = (min_idx == bases.size() - 1) ? bases[bases.size() - 1] : bases[min_idx + 1];

  double min_dist = std::numeric_limits<double>::infinity();
  double min_s = (search_start + search_end) * 0.5;

  while (search_end - search_start > k_points_minimum_dist_threshold) {
    const double mid1 = search_start + (search_end - search_start) / 3.0;
    const double mid2 = search_end - (search_end - search_start) / 3.0;

    const auto pose1 = trajectory.compute(mid1);
    double squared_dist1 = autoware_utils_geometry::calc_squared_distance2d(pose1, pose.position);
    const double yaw_dev1 =
      autoware_utils_geometry::calc_yaw_deviation(autoware_utils_geometry::get_pose(pose1), pose);

    if (squared_dist1 <= squared_dist_threshold && std::fabs(yaw_dev1) <= yaw_threshold) {
      if (squared_dist1 < min_dist) {
        min_dist = squared_dist1;
        min_s = mid1;
      }
    } else {
      squared_dist1 = std::numeric_limits<double>::infinity();
    }

    const auto pose2 = trajectory.compute(mid2);
    double squared_dist2 = autoware_utils_geometry::calc_squared_distance2d(pose2, pose.position);
    const double yaw_dev2 =
      autoware_utils_geometry::calc_yaw_deviation(autoware_utils_geometry::get_pose(pose2), pose);

    if (squared_dist2 <= squared_dist_threshold && std::fabs(yaw_dev2) <= yaw_threshold) {
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
}  // namespace detail

/**
 * @brief Find the s value on the trajectory for a given point container.
 * @details Nearest index is considered from brute force search for all underlying bases of
 * trajectory, then use pass to find_precise_index to return s value
 * @param trajectory Continuous trajectory object
 * @param point given point
 * @return index of nearest point in the trajectory
 */
template <class TrajectoryPointType>
[[nodiscard]] double find_nearest_index(
  const Trajectory<TrajectoryPointType> & trajectory, const geometry_msgs::msg::Point & point)
{
  const auto bases = trajectory.get_underlying_bases();

  double min_dist = std::numeric_limits<double>::max();
  size_t min_idx = 0;

  for (size_t i = 0; i < bases.size(); ++i) {
    auto pnt = trajectory.compute(bases[i]);
    const auto dist = autoware_utils_geometry::calc_squared_distance2d(pnt, point);
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }
  return detail::find_precise_index(trajectory, point, min_idx);
}

/**
 * @brief Find the corresponding s value on the trajectory for a given pose.
 * @details Find the nearest index from Trajectory underlying base (the nearest point index in the
 * first area that constraints are satisfied.), then perform find_precise_index to provide s value.
 * @param trajectory Continuous trajectory object
 * @param pose given pose
 * @param dist_threshold max distance used to get squared distance for finding the nearest point to
 * given pose
 * @param yaw_threshold max yaw used for finding nearest point to given pose
 * @return s of nearest point on the trajectory (s or none if not found any base in constraint)
 */
template <class TrajectoryPointType>
[[nodiscard]] std::optional<double> find_first_nearest_index(
  const Trajectory<TrajectoryPointType> & trajectory, const geometry_msgs::msg::Pose & pose,
  const double dist_threshold = std::numeric_limits<double>::max(),
  const double yaw_threshold = std::numeric_limits<double>::max())
{
  const auto bases = trajectory.get_underlying_bases();
  std::optional<size_t> base_nearest_index;
  {
    const double squared_dist_threshold = dist_threshold * dist_threshold;
    double min_squared_dist = std::numeric_limits<double>::max();
    size_t min_idx = 0;
    bool is_within_constraints = false;
    for (size_t i = 0; i < bases.size(); ++i) {
      const auto point = trajectory.compute(bases[i]);
      const auto squared_dist =
        autoware_utils_geometry::calc_squared_distance2d(point, pose.position);
      const auto yaw_dev =
        autoware_utils_geometry::calc_yaw_deviation(autoware_utils_geometry::get_pose(point), pose);

      if (squared_dist_threshold < squared_dist || yaw_threshold < std::abs(yaw_dev)) {
        if (is_within_constraints) {
          break;
        }
        continue;
      }

      if (min_squared_dist <= squared_dist) {
        continue;
      }

      min_squared_dist = squared_dist;
      min_idx = i;
      is_within_constraints = true;
    }

    // nearest index is found
    if (is_within_constraints) {
      base_nearest_index = min_idx;
    }
  }

  if (base_nearest_index.has_value()) {
    return detail::find_precise_index(
      trajectory, pose, *base_nearest_index, dist_threshold, yaw_threshold);
  } else {
    return std::nullopt;
  }
}

// Extern template for Point =====================================================================
extern template double find_nearest_index<autoware_planning_msgs::msg::PathPoint>(
  const Trajectory<autoware_planning_msgs::msg::PathPoint> & trajectory,
  const geometry_msgs::msg::Point & point);

extern template double
find_nearest_index<autoware_internal_planning_msgs::msg::PathPointWithLaneId>(
  const Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & trajectory,
  const geometry_msgs::msg::Point & point);

extern template double find_nearest_index<geometry_msgs::msg::Pose>(
  const Trajectory<geometry_msgs::msg::Pose> & trajectory, const geometry_msgs::msg::Point & point);

extern template double find_nearest_index<autoware_planning_msgs::msg::TrajectoryPoint>(
  const Trajectory<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const geometry_msgs::msg::Point & point);

// Extern template for Pose =====================================================================
extern template std::optional<double>
find_first_nearest_index<autoware_planning_msgs::msg::PathPoint>(
  const Trajectory<autoware_planning_msgs::msg::PathPoint> & trajectory,
  const geometry_msgs::msg::Pose & pose, const double max_dist, const double max_yaw);

extern template std::optional<double>
find_first_nearest_index<autoware_internal_planning_msgs::msg::PathPointWithLaneId>(
  const Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & trajectory,
  const geometry_msgs::msg::Pose & pose, const double max_dist, const double max_yaw);

extern template std::optional<double> find_first_nearest_index<geometry_msgs::msg::Pose>(
  const Trajectory<geometry_msgs::msg::Pose> & trajectory, const geometry_msgs::msg::Pose & pose,
  const double max_dist, const double max_yaw);

extern template std::optional<double>
find_first_nearest_index<autoware_planning_msgs::msg::TrajectoryPoint>(
  const Trajectory<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const geometry_msgs::msg::Pose & pose, const double max_dist, const double max_yaw);

}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__UTILS__FIND_NEAREST_HPP_
