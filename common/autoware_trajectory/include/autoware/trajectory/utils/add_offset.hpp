// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__TRAJECTORY__UTILS__ADD_OFFSET_HPP_
#define AUTOWARE__TRAJECTORY__UTILS__ADD_OFFSET_HPP_

#include "autoware/trajectory/detail/types.hpp"
#include "autoware/trajectory/forward.hpp"
#include "autoware/trajectory/temporal_trajectory.hpp"

#include <autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_planning_msgs/msg/path_point.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <tf2/utils.h>

#include <cmath>
#include <type_traits>
#include <utility>
#include <vector>

namespace autoware::experimental::trajectory
{

namespace detail
{

// Extract yaw from orientation quaternion for types that have pose
inline double get_yaw_from_point_type(const geometry_msgs::msg::Pose & pose)
{
  return tf2::getYaw(pose.orientation);
}

inline double get_yaw_from_point_type(const autoware_planning_msgs::msg::PathPoint & point)
{
  return tf2::getYaw(point.pose.orientation);
}

inline double get_yaw_from_point_type(const autoware_planning_msgs::msg::TrajectoryPoint & point)
{
  return tf2::getYaw(point.pose.orientation);
}

inline double get_yaw_from_point_type(
  const autoware_internal_planning_msgs::msg::PathPointWithLaneId & point)
{
  return tf2::getYaw(point.point.pose.orientation);
}

}  // namespace detail

/**
 * @brief Compute a trajectory offset from the base_link by a fixed vehicle-frame offset.
 * @details
 * Given a trajectory whose points represent the base_link pose, this function computes a new
 * trajectory where each point is translated by `(offset_x, offset_y)` expressed in the **local
 * vehicle frame** (i.e., forward is +x, left is +y). The orientation of each pose is preserved.
 *
 * For point types with orientation (Pose, PathPoint, TrajectoryPoint, PathPointWithLaneId), the
 * pose's orientation is used to determine the heading. For `geometry_msgs::msg::Point`, the
 * trajectory's azimuth (tangent direction) is used instead.
 *
 * This is useful for obtaining the path of the vehicle front, rear, or side edges from a
 * base_link-centered trajectory.
 *
 * @tparam PointType The type of points in the trajectory (e.g., TrajectoryPoint, PathPoint).
 * @param reference_trajectory The input trajectory (base_link centered).
 * @param offset_x Forward offset in the vehicle frame [m].
 * @param offset_y Lateral offset in the vehicle frame [m].
 * @return A new trajectory with the offset applied.
 */
template <typename PointType>
trajectory::Trajectory<PointType> add_offset(
  const trajectory::Trajectory<PointType> & reference_trajectory, const double offset_x,
  const double offset_y)
{
  const auto underlying_bases = reference_trajectory.get_underlying_bases();

  std::vector<PointType> offset_points;
  offset_points.reserve(underlying_bases.size());

  for (const auto s : underlying_bases) {
    auto point = reference_trajectory.compute(s);

    // Use pose orientation for types with orientation, or trajectory azimuth for Point
    double yaw = 0.0;
    if constexpr (std::is_same_v<PointType, geometry_msgs::msg::Point>) {
      // For Point type, use the trajectory's azimuth (tangent direction)
      yaw = reference_trajectory.azimuth(s);
    } else {
      // For Pose, PathPoint, TrajectoryPoint, PathPointWithLaneId, use the pose's orientation
      yaw = detail::get_yaw_from_point_type(point);
    }

    // Rotate the vehicle-frame offset into the global frame
    const double global_offset_x = std::cos(yaw) * offset_x - std::sin(yaw) * offset_y;
    const double global_offset_y = std::sin(yaw) * offset_x + std::cos(yaw) * offset_y;

    detail::to_point(point).x += global_offset_x;
    detail::to_point(point).y += global_offset_y;

    offset_points.emplace_back(std::move(point));
  }

  auto offset_trajectory = reference_trajectory;
  [[maybe_unused]] const auto result = offset_trajectory.build(offset_points);
  // build() should succeed because we preserved the same number and ordering of bases
  return offset_trajectory;
}

/**
 * @brief Compute a TemporalTrajectory offset from the base_link by a fixed vehicle-frame offset.
 * @details
 * Given a TemporalTrajectory whose points represent the base_link pose, this function computes a
 * new TemporalTrajectory where each point is translated by `(offset_x, offset_y)` expressed in the
 * **local vehicle frame** (i.e., forward is +x, left is +y). The orientation and time mapping of
 * each pose is preserved.
 *
 * This is useful for obtaining the time-parameterized path of the vehicle front, rear, or side
 * edges from a base_link-centered trajectory.
 *
 * @param reference_trajectory The input TemporalTrajectory (base_link centered).
 * @param offset_x Forward offset in the vehicle frame [m].
 * @param offset_y Lateral offset in the vehicle frame [m].
 * @return A new TemporalTrajectory with the offset applied.
 */
inline TemporalTrajectory add_offset(
  const TemporalTrajectory & reference_trajectory, const double offset_x, const double offset_y)
{
  const auto & spatial_traj = reference_trajectory.spatial_trajectory();
  const auto underlying_bases = spatial_traj.get_underlying_bases();

  std::vector<TemporalTrajectory::PointType> offset_points;
  offset_points.reserve(underlying_bases.size());

  for (const auto s : underlying_bases) {
    auto point = spatial_traj.compute(s);

    // Use pose orientation from the trajectory point
    const double yaw = detail::get_yaw_from_point_type(point);

    // Rotate the vehicle-frame offset into the global frame
    const double global_offset_x = std::cos(yaw) * offset_x - std::sin(yaw) * offset_y;
    const double global_offset_y = std::sin(yaw) * offset_x + std::cos(yaw) * offset_y;

    detail::to_point(point).x += global_offset_x;
    detail::to_point(point).y += global_offset_y;

    offset_points.emplace_back(point);
  }

  auto offset_trajectory =
    TemporalTrajectory::Builder{}.build(offset_points).value_or(TemporalTrajectory{});  // NOLINT

  // Copy time offset from original
  offset_trajectory.set_time_offset(reference_trajectory.time_offset());

  return offset_trajectory;
}

}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__UTILS__ADD_OFFSET_HPP_
