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

#include "lanelet2_map_loader_utils.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Area.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <lanelet2_core/primitives/RegulatoryElement.h>

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

namespace autoware::map_loader::utils
{

bool is_osm_file(const std::string & path)
{
  if (std::filesystem::is_directory(path)) {
    return false;
  }
  const std::string ext = std::filesystem::path(path).extension();
  return ext == ".osm" || ext == ".OSM";
}

std::vector<std::string> get_lanelet2_paths(const std::string & lanelet2_map_path)
{
  static const auto logger = rclcpp::get_logger("map_loader");
  std::vector<std::string> lanelet2_paths;

  if (!std::filesystem::exists(lanelet2_map_path)) {
    RCLCPP_ERROR_STREAM(logger, "No such file or directory: " << lanelet2_map_path);
    return lanelet2_paths;
  }

  if (is_osm_file(lanelet2_map_path)) {
    lanelet2_paths.push_back(lanelet2_map_path);
  } else if (std::filesystem::is_directory(lanelet2_map_path)) {
    for (const auto & entry : std::filesystem::directory_iterator(lanelet2_map_path)) {
      if (is_osm_file(entry.path().string())) {
        lanelet2_paths.push_back(entry.path().string());
      }
    }
    std::sort(lanelet2_paths.begin(), lanelet2_paths.end());
  } else {
    RCLCPP_ERROR_STREAM(logger, "Not a valid .osm file or directory: " << lanelet2_map_path);
  }

  return lanelet2_paths;
}

void merge_lanelet2_maps(lanelet::LaneletMap & merge_target, lanelet::LaneletMap & merge_source)
{
  for (lanelet::Lanelet & lanelet : merge_source.laneletLayer) {
    merge_target.add(lanelet);
  }
  for (lanelet::Area & area : merge_source.areaLayer) {
    merge_target.add(area);
  }
  for (lanelet::RegulatoryElementPtr & regulatory_element : merge_source.regulatoryElementLayer) {
    merge_target.add(regulatory_element);
  }
  for (lanelet::LineString3d & line_string : merge_source.lineStringLayer) {
    // Special handling for line strings to avoid duplicate points, which would break
    // successor/predecessor relationship calculations
    for (lanelet::Point3d & point : line_string) {
      if (merge_target.pointLayer.find(point.id()) != merge_target.pointLayer.end()) {
        point = merge_target.pointLayer.get(point.id());
      }
    }
    merge_target.add(line_string);
  }
  for (lanelet::Polygon3d & polygon : merge_source.polygonLayer) {
    merge_target.add(polygon);
  }
  for (lanelet::Point3d & point : merge_source.pointLayer) {
    if (merge_target.pointLayer.find(point.id()) == merge_target.pointLayer.end()) {
      merge_target.add(point);
    }
  }
}

}  // namespace autoware::map_loader::utils
