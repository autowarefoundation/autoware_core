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

#ifndef AUTOWARE__LANELET2_UTILS__GEOMETRY_HPP_
#define AUTOWARE__LANELET2_UTILS__GEOMETRY_HPP_

#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/LineString.h>

#include <optional>

namespace autoware::experimental::lanelet2_utils
{

/**
 * @brief extrapolates a point beyond a segment defined by two points.
 *        To extrapolate from first, revert the arguments
 * @param [in] first first endpoint of the segment.
 * @param [in] second second endpoint of the segment.
 * @param [in] distance distance to extrapolate from second toward outward.
 * @return lanelet::ConstPoint3d The extrapolated point.
 */
lanelet::ConstPoint3d extrapolate_point(
  const lanelet::ConstPoint3d & first, const lanelet::ConstPoint3d & second, const double distance);

/**
 * @brief linearly interpolates a point along a segment.
 * @param [in] first first endpoint of the segment.
 * @param [in] second second endpoint of the segment.
 * @param [in] distance desired distance from the reference endpoint along the segment.
 * @return lanelet::ConstPoint3d The interpolated point.
 */

std::optional<lanelet::ConstPoint3d> interpolate_point(
  const lanelet::ConstPoint3d & first, const lanelet::ConstPoint3d & second, const double distance);

/**
 * @brief find an interpolated point from a lanelet at a given distance.
 * @param [in] lanelet input lanelet.
 * @param [in] distance desired distance.
 * @return lanelet::ConstPoint3d; the interpolated point within lanelet.centerline().
 */
std::optional<lanelet::ConstPoint3d> interpolate_lanelet(
  const lanelet::ConstLanelet & lanelet, const double distance);

/**
 * @brief find an interpolated point from a lanelet at a given distance.
 * @param [in] lanelet input lanelet.
 * @param [in] distance desired distance.
 * @param [in] from_first the distance is measured from the beginning (true) or from the end
 * (false).
 * @return lanelet::ConstPoint3d; the first interpolated point within sequence of
 * lanelet.centerline().
 * @note return as soon as you hit a segment in any lanelet whose cumulative 2D‑length
 * exceeds distance.
 */
std::optional<lanelet::ConstPoint3d> interpolate_lanelet_sequence(
  const lanelet::ConstLanelets & lanelet_sequence, double distance);

/**
 * @brief concatenate all center line of inputted lanelet sequence.
 * @param [in] lanelet input lanelet.
 * @return lanelet.centerline of all lanelet as lanelet::CompoundLineString3d.
 */
std::optional<lanelet::CompoundLineString3d> concatenate_center_line(
  const lanelet::ConstLanelets & lanelets);

/**
 * @brief extract a sub-linestring between two arc-length positions along an input linestring.
 * @param[in] linestring the original ConstLineString3d.
 * @param[in] s1 the start distance (arc length from the beginning of the linestring).
 * @param[in] s2 the end distance (arc length from the beginning of the linestring).
 * @return new LineString3d containing the interpolated start point and end point, with original
 * point strictly between s1 and s2.
 */
std::optional<lanelet::LineString3d> get_linestring_from_arc_length(
  const lanelet::ConstLineString3d & linestring, const double s1, const double s2);

/**
 * @brief compute the 2D pose (position and heading) at a given arc-length along a sequence of
 * lanelets.
 * @param[in] lanelet_sequence sequence of ConstLanelets whose centerlines define the path.
 * @param[in] s arc-length distance (from the start of the sequence).
 * @return optional Pose message, returns std::nullopt if the sequence is empty or if s is outside
 * the total path length.
 */
std::optional<geometry_msgs::msg::Pose> get_pose_from_2d_arc_length(
  const lanelet::ConstLanelets & lanelet_sequence, const double s);

}  // namespace autoware::experimental::lanelet2_utils

#endif  // AUTOWARE__LANELET2_UTILS__GEOMETRY_HPP_
