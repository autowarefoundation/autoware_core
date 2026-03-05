// Copyright 2024 The Autoware Contributors
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

#ifndef LANELET2_MAP_LOADER__LANELET2_MAP_LOADER_UTILS_HPP_
#define LANELET2_MAP_LOADER__LANELET2_MAP_LOADER_UTILS_HPP_

#include <lanelet2_core/LaneletMap.h>

#include <string>
#include <vector>

namespace autoware::map_loader::utils
{

/// @brief Check whether a path refers to an existing OSM file (not a directory).
/// @param path Filesystem path to check.
/// @return True if the path exists, is not a directory, and has a `.osm` or `.OSM` extension.
bool is_osm_file(const std::string & path);

/// @brief Collect lanelet2 map file paths from a file or directory.
///
/// If @p lanelet2_map_path points to a single `.osm` file the result contains only that path.
/// If it points to a directory, all `.osm` files directly inside are returned in sorted order.
/// Returns an empty vector and logs an error if the path does not exist or is neither an OSM
/// file nor a directory.
///
/// @param lanelet2_map_path Path to a single `.osm` file or a directory containing `.osm` files.
/// @return Sorted list of absolute paths to lanelet2 map files.
std::vector<std::string> get_lanelet2_paths(const std::string & lanelet2_map_path);

/// @brief Merge all map primitives from @p merge_source into @p merge_target.
///
/// Lanelets, areas, regulatory elements, line strings, polygons, and points are all copied.
/// Points shared between maps (same ID) are deduplicated: the target's existing point object
/// is reused so that successor/predecessor relationships remain consistent.
///
/// @note The source map must not be destroyed after merging because lanelets hold shared
///       ownership of their primitives through the source map.
/// @param merge_target Destination map; primitives are added to this map.
/// @param merge_source Source map; its primitives are read and added to @p merge_target.
void merge_lanelet2_maps(lanelet::LaneletMap & merge_target, lanelet::LaneletMap & merge_source);

}  // namespace autoware::map_loader::utils

#endif  // LANELET2_MAP_LOADER__LANELET2_MAP_LOADER_UTILS_HPP_
