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

#include <lanelet2_core/Forward.h>

#include <optional>

namespace autoware::lanelet2_utils
{

/**
 * @brief extrapolates a point beyond a segment defined by two points.
 * @param [in] first first endpoint of the segment.
 * @param [in] second second endpoint of the segment.
 * @param [in] distance distance to extrapolate.
 * @param [in] is_forward direction of extrapolation.
 * @return lanelet::ConstPoint3d The extrapolated point.
 */
lanelet::ConstPoint3d extrapolate_point(
  const lanelet::ConstPoint3d & first, const lanelet::ConstPoint3d & second, const double distance,
  const bool is_forward);

/**
 * @brief linearly interpolates a point along a segment.
 * @param [in] first first endpoint of the segment.
 * @param [in] second second endpoint of the segment.
 * @param [in] distance desired distance from the reference endpoint along the segment.
 * @param [in] from_first measure from the first point (if true) or from the second (if false).
 * @return lanelet::ConstPoint3d The interpolated point.
 */

lanelet::ConstPoint3d interpolate_point(
  const lanelet::ConstPoint3d & first, const lanelet::ConstPoint3d & second, const double distance,
  const bool from_first);

/**
 * @brief inserts an interpolated point into a linestring at a given distance.
 * @param [in] linestring constant linestring.
 * @param [in] distance desired distance.
 * @param [in] from_first the distance is measured from the beginning (true) or from the end
 * (false).
 * @return lanelet::ConstLineString3d A new linestring with the interpolated point inserted.
 */
lanelet::ConstLineString3d interpolate_linestring(
  const lanelet::ConstLineString3d & linestring, const double distance, const bool from_first);

/**
 * @brief extrapolate linestring by distance
 * @param [in] linestring input linestring
 * @param [in] distance distance to extrapolate
 * @param [in] is_forward flag if the extrapolation is forward or backward
 * @return lanelet::ConstLineString3d A new linestring with the extrapolated point inserted.
 */
lanelet::ConstLineString3d extrapolate_linestring(
  const lanelet::ConstLineString3d & linestring, const double distance, const bool is_forward);

}  // namespace autoware::lanelet2_utils

#endif  // AUTOWARE__LANELET2_UTILS__GEOMETRY_HPP_
