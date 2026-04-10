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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>

#include <cassert>
#include <cmath>
#include <type_traits>
#include <utility>
#include <vector>

namespace autoware::experimental::trajectory
{

namespace detail
{

inline tf2::Quaternion get_orientation_from_point_type(const geometry_msgs::msg::Pose & pose)
{
  return tf2::Quaternion(
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
}

inline tf2::Quaternion get_orientation_from_point_type(
  const autoware_planning_msgs::msg::PathPoint & point)
{
  return get_orientation_from_point_type(point.pose);
}

inline tf2::Quaternion get_orientation_from_point_type(
  const autoware_planning_msgs::msg::TrajectoryPoint & point)
{
  return get_orientation_from_point_type(point.pose);
}

inline tf2::Quaternion get_orientation_from_point_type(
  const autoware_internal_planning_msgs::msg::PathPointWithLaneId & point)
{
  return get_orientation_from_point_type(point.point.pose);
}

}  // namespace detail

/**
 * @brief Compute a trajectory offset from the base_link by a fixed vehicle-frame offset.
 * @details
 * Given a trajectory whose points represent the base_link pose, this function computes a new
 * trajectory where each point is translated by `(offset_x, offset_y, offset_z)` expressed in the
 * **local vehicle frame** (i.e., forward is +x, left is +y, up is +z). The orientation of each
 * pose is preserved.
 *
 * The vehicle-frame offset is rotated into the global frame using the point's full orientation
 * quaternion. This means roll, pitch, and yaw are all respected when available.
 *
 * For point types with orientation (Pose, PathPoint, TrajectoryPoint, PathPointWithLaneId), the
 * pose's full orientation is used. For `geometry_msgs::msg::Point`, the trajectory's azimuth
 * (tangent direction) and elevation are used to synthesize an orientation with zero roll.
 *
 * This is useful for obtaining the path of the vehicle front, rear, or side edges from a
 * base_link-centered trajectory.
 *
 * @tparam PointType The type of points in the trajectory (e.g., TrajectoryPoint, PathPoint).
 * @param reference_trajectory The input trajectory (base_link centered).
 * @param offset_x Forward offset in the vehicle frame [m].
 * @param offset_y Lateral offset in the vehicle frame [m].
 * @param offset_z Vertical offset in the vehicle frame [m].
 * @return A new trajectory with the offset applied.
 */
template <typename PointType>
trajectory::Trajectory<PointType> add_offset(
  const trajectory::Trajectory<PointType> & reference_trajectory, const double offset_x,
  const double offset_y, const double offset_z = 0.0)
{
  const auto underlying_bases = reference_trajectory.get_underlying_bases();

  std::vector<PointType> offset_points;
  offset_points.reserve(underlying_bases.size());

  for (const auto s : underlying_bases) {
    auto point = reference_trajectory.compute(s);

    // Use pose orientation for types with orientation, or synthesize one from the tangent for
    // Point trajectories.
    tf2::Quaternion orientation;
    if constexpr (std::is_same_v<PointType, geometry_msgs::msg::Point>) {
      orientation.setRPY(0.0, reference_trajectory.elevation(s), reference_trajectory.azimuth(s));
    } else {
      orientation = detail::get_orientation_from_point_type(point);
    }

    orientation.normalize();
    const auto global_offset =
      tf2::quatRotate(orientation, tf2::Vector3(offset_x, offset_y, offset_z));

    detail::to_point(point).x += global_offset.x();
    detail::to_point(point).y += global_offset.y();
    detail::to_point(point).z += global_offset.z();

    offset_points.emplace_back(std::move(point));
  }

  auto offset_trajectory = reference_trajectory;
  const auto result = offset_trajectory.build(offset_points);
  assert(
    result.has_value() &&
    "add_offset: failed to build trajectory with offset points");  // The build should never fail
                                                                   // since the offset points are
                                                                   // generated from a valid
                                                                   // trajectory.
  return offset_trajectory;
}

/**
 * @brief Compute a TemporalTrajectory offset from the base_link by a fixed vehicle-frame offset.
 * @details
 * Given a TemporalTrajectory whose points represent the base_link pose, this function computes a
 * new TemporalTrajectory where each point is translated by `(offset_x, offset_y, offset_z)`
 * expressed in the **local vehicle frame** (i.e., forward is +x, left is +y, up is +z). The
 * orientation and time mapping of each pose is preserved.
 *
 * The vehicle-frame offset is rotated into the global frame using the point's full orientation
 * quaternion. This means roll, pitch, and yaw are all respected.
 *
 * This is useful for obtaining the time-parameterized path of the vehicle front, rear, or side
 * edges from a base_link-centered trajectory.
 *
 * @param reference_trajectory The input TemporalTrajectory (base_link centered).
 * @param offset_x Forward offset in the vehicle frame [m].
 * @param offset_y Lateral offset in the vehicle frame [m].
 * @param offset_z Vertical offset in the vehicle frame [m].
 * @return A new TemporalTrajectory with the offset applied.
 */
inline TemporalTrajectory add_offset(
  const TemporalTrajectory & reference_trajectory, const double offset_x, const double offset_y,
  const double offset_z = 0.0)
{
  const auto underlying_time_bases = reference_trajectory.get_underlying_time_bases();

  std::vector<TemporalTrajectory::PointType> offset_points;
  offset_points.reserve(underlying_time_bases.size());

  for (const auto t : underlying_time_bases) {
    // Use compute_from_distance to get the full point including time_from_start
    auto point = reference_trajectory.compute_from_time(t);

    // Use pose orientation from the trajectory point
    auto orientation = detail::get_orientation_from_point_type(point);
    orientation.normalize();

    // Rotate the vehicle-frame offset into the global frame using quaternion
    const auto global_offset =
      tf2::quatRotate(orientation, tf2::Vector3(offset_x, offset_y, offset_z));

    detail::to_point(point).x += global_offset.x();
    detail::to_point(point).y += global_offset.y();
    detail::to_point(point).z += global_offset.z();

    offset_points.emplace_back(point);
  }

  auto offset_trajectory = reference_trajectory;
  const auto result = offset_trajectory.build(offset_points);
  assert(
    result.has_value() &&
    "add_offset: failed to build TemporalTrajectory with offset points");  // The build should
                                                                           // never fail
                                                                           // since the
                                                                           // offset points are
                                                                           // generated from a
                                                                           // valid trajectory.
  return offset_trajectory;
}

}  // namespace autoware::experimental::trajectory

#endif  // AUTOWARE__TRAJECTORY__UTILS__ADD_OFFSET_HPP_
