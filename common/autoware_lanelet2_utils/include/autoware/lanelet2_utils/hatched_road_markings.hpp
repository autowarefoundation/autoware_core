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

#ifndef AUTOWARE__LANELET2_UTILS__HATCHED_ROAD_MARKINGS_HPP_
#define AUTOWARE__LANELET2_UTILS__HATCHED_ROAD_MARKINGS_HPP_

#include <lanelet2_core/Forward.h>

#include <vector>

namespace autoware::experimental::lanelet2_utils
{

struct AdjacentHatchedRoadMarkings
{
  const lanelet::ConstPolygons3d left;
  const lanelet::ConstPolygons3d right;
};

/**
 * @brief Return "hatched_road_markings" polygons that touch the bounds of the given lanelets.
 *
 * Polygons are grouped by the side (left/right) on which they are found and duplicates are
 * removed.
 *
 * @param lanelets    Lanelets to inspect.
 * @param lanelet_map Complete lanelet map that owns the polygon layer.
 * @return Left/right adjacent hatched road marking polygons.
 */
AdjacentHatchedRoadMarkings get_adjacent_hatched_road_markings(
  const lanelet::ConstLanelets & lanelets, const lanelet::LaneletMapConstPtr lanelet_map);

}  // namespace autoware::experimental::lanelet2_utils

#endif  // AUTOWARE__LANELET2_UTILS__HATCHED_ROAD_MARKINGS_HPP_
