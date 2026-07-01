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

#ifndef LANELET2_MAP_LOADER__LANELET2_MAP_CELL_METADATA_HPP_
#define LANELET2_MAP_LOADER__LANELET2_MAP_CELL_METADATA_HPP_

#include <lanelet2_core/LaneletMap.h>

#include <map>
#include <optional>
#include <string>

namespace autoware::map_loader
{

/// @brief Bounding-box metadata for a single lanelet2 map cell (one OSM file).
///
/// The bounding box covers all points in the loaded map, making it accurate without
/// an external metadata file or a fixed grid resolution.
/// @p id equals the absolute file path and serves as the cell identifier in
/// `GetSelectedLanelet2Map` requests.
struct Lanelet2FileMetaData
{
  std::string id;  ///< Unique cell identifier (== absolute file path).
  double min_x;
  double min_y;
  double max_x;
  double max_y;
};

}  // namespace autoware::map_loader

namespace autoware::map_loader::utils
{

/// @brief Compute the axis-aligned bounding box of all points in @p map.
/// If the map has no points all coordinates are set to 0.
Lanelet2FileMetaData compute_cell_metadata(
  const std::string & cell_id, const lanelet::LaneletMap & map);

/// @brief Load per-cell bounding-box metadata from @p yaml_path.
///
/// The expected YAML format is:
/// @code
///   x_resolution: 100.0
///   y_resolution: 100.0
///   1.osm: [58700.0, 42500.0]   # [min_x, min_y] of that cell
///   2.osm: [58800.0, 42500.0]
///   ...
/// @endcode
///
/// Cell bounding boxes are `[min_x, min_y, min_x + x_resolution, min_y + y_resolution]`.
/// Relative filename keys are resolved against the parent directory of @p yaml_path.
///
/// @return Populated metadata dict on success, or `std::nullopt` if the file does not exist or
///         cannot be parsed.
std::optional<std::map<std::string, Lanelet2FileMetaData>> load_cell_metadata_from_yaml(
  const std::string & yaml_path);

}  // namespace autoware::map_loader::utils

#endif  // LANELET2_MAP_LOADER__LANELET2_MAP_CELL_METADATA_HPP_
