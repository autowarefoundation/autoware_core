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

#include <autoware/lanelet2_utils/hatched_road_markings.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <cstring>
#include <unordered_set>
#include <utility>
namespace autoware::experimental::lanelet2_utils
{
namespace
{
// Utility to collect hatched road marking polygons connected to the given line string points.
template <typename LineStringT>
void collect_adjacent_polygons(
  const LineStringT & line, const lanelet::LaneletMapConstPtr & map,
  std::unordered_set<lanelet::Id> & visited_ids, lanelet::ConstPolygons3d & out_polygons)
{
  for (const auto & pt : line) {
    const auto usages = map->polygonLayer.findUsages(pt);
    for (const auto & poly : usages) {
      const auto type_attr = poly.attributeOr(lanelet::AttributeName::Type, "none");
      if (std::strcmp(type_attr, "hatched_road_markings") != 0) {
        continue;
      }
      // Deduplicate polygons using their unique id
      if (visited_ids.insert(poly.id()).second) {
        out_polygons.emplace_back(poly);
      }
    }
  }
}
}  // namespace

AdjacentHatchedRoadMarkings get_adjacent_hatched_road_markings(
  const lanelet::ConstLanelets & lanelets, const lanelet::LaneletMapConstPtr lanelet_map)
{
  // These sets keep track of polygons already registered to avoid duplicates.
  std::unordered_set<lanelet::Id> visited_left_ids;
  std::unordered_set<lanelet::Id> visited_right_ids;

  lanelet::ConstPolygons3d left_polygons;
  lanelet::ConstPolygons3d right_polygons;

  for (const auto & llt : lanelets) {
    // Collect polygons adjacent to the left bound of the current lanelet.
    collect_adjacent_polygons(llt.leftBound3d(), lanelet_map, visited_left_ids, left_polygons);
    // Collect polygons adjacent to the right bound of the current lanelet.
    collect_adjacent_polygons(llt.rightBound3d(), lanelet_map, visited_right_ids, right_polygons);
  }

  return {left_polygons, right_polygons};
}
}  // namespace autoware::experimental::lanelet2_utils
