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

#include "lanelet2_map_cell_metadata.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <limits>
#include <map>
#include <optional>
#include <string>

namespace autoware::map_loader::utils
{

Lanelet2FileMetaData compute_cell_metadata(
  const std::string & cell_id, const lanelet::LaneletMap & map)
{
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();

  for (const lanelet::ConstPoint3d & pt : map.pointLayer) {
    if (pt.x() < min_x) min_x = pt.x();
    if (pt.y() < min_y) min_y = pt.y();
    if (pt.x() > max_x) max_x = pt.x();
    if (pt.y() > max_y) max_y = pt.y();
  }

  if (map.pointLayer.empty()) {
    min_x = max_x = min_y = max_y = 0.0;
  }

  return {cell_id, min_x, min_y, max_x, max_y};
}

std::optional<std::map<std::string, Lanelet2FileMetaData>> load_cell_metadata_from_yaml(
  const std::string & yaml_path)
{
  if (!std::filesystem::exists(yaml_path)) {
    return std::nullopt;
  }

  const std::filesystem::path map_dir = std::filesystem::path(yaml_path).parent_path();

  const YAML::Node node = YAML::LoadFile(yaml_path);
  const double x_res = node["x_resolution"].as<double>();
  const double y_res = node["y_resolution"].as<double>();

  std::map<std::string, Lanelet2FileMetaData> result;
  for (const auto & kv : node) {
    const std::string key = kv.first.as<std::string>();
    if (key == "x_resolution" || key == "y_resolution") {
      continue;
    }
    const double min_x = kv.second[0].as<double>();
    const double min_y = kv.second[1].as<double>();
    const std::string abs_path = (map_dir / key).string();
    result[abs_path] = {abs_path, min_x, min_y, min_x + x_res, min_y + y_res};
  }

  return result;
}

}  // namespace autoware::map_loader::utils
