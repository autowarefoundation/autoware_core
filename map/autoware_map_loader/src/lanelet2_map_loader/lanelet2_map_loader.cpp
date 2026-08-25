// Copyright 2021 TierIV
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

/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Simon Thompson, Ryohsuke Mitsudome
 *
 */

#include "lanelet2_map_loader.hpp"

#include "utils/lanelet2_local_projector.hpp"
#include "utils/lanelet2_map_loader_utils.hpp"

#include <autoware/geography_utils/lanelet2_projector.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_lanelet2_extension/io/autoware_osm_parser.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_lanelet2_extension/version.hpp>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::map_loader
{

Lanelet2MapLoader::Lanelet2MapLoader(const Lanelet2MapLoaderParams & params) : params_(params)
{
}

Lanelet2MapLoaderResult Lanelet2MapLoader::execute(
  const autoware_map_msgs::msg::MapProjectorInfo & projector_info, const rclcpp::Time & now) const
{
  Lanelet2MapLoaderResult result;

  const std::vector<std::string> lanelet2_paths =
    utils::get_lanelet2_paths(params_.lanelet2_map_path);
  if (lanelet2_paths.empty()) {
    throw std::runtime_error("No lanelet2 map files found from " + params_.lanelet2_map_path);
  }

  std::vector<lanelet::LaneletMapPtr> maps;
  for (const auto & path : lanelet2_paths) {
    auto map_tmp = load_map(path, projector_info, result.warnings);
    if (!map_tmp) {
      throw std::runtime_error("Failed to load lanelet2_map: " + path);
    }
    maps.push_back(map_tmp);
  }

  const auto yaml_metadata =
    utils::load_cell_metadata_from_yaml(params_.lanelet2_map_metadata_path);
  if (yaml_metadata) {
    result.cell_metadata_dict = *yaml_metadata;
    result.infos.push_back("Loaded cell metadata from " + params_.lanelet2_map_metadata_path + ".");
  } else if (lanelet2_paths.size() == 1) {
    result.cell_metadata_dict[lanelet2_paths[0]] =
      utils::compute_cell_metadata(lanelet2_paths[0], *maps[0]);
  } else {
    throw std::runtime_error(
      "Lanelet2 metadata file not found: " + params_.lanelet2_map_metadata_path);
  }

  auto map = std::make_shared<lanelet::LaneletMap>();
  for (auto & loaded_map : maps) {
    utils::merge_lanelet2_maps(*map, *loaded_map);
  }

  const auto & lanelet2_filename = lanelet2_paths.front();
  std::string format_version{};
  std::string map_version{};
  lanelet::io_handlers::AutowareOsmParser::parseVersions(
    lanelet2_filename, &format_version, &map_version);

  if (format_version == "null" || format_version.empty() || !isdigit(format_version[0])) {
    std::string warn = lanelet2_filename +
                       " has no format_version(null) or non semver-style format_version(" +
                       format_version + ") information";
    result.warnings.push_back(warn);
    if (!params_.allow_unsupported_version) {
      throw std::invalid_argument(
        "allow_unsupported_version is false, so stop loading lanelet map\n" + warn);
    }
  } else if (const auto map_major_ver_opt = lanelet::io_handlers::parseMajorVersion(format_version);
             map_major_ver_opt.has_value()) {
    const auto map_major_ver = map_major_ver_opt.value();
    if (map_major_ver > static_cast<uint64_t>(lanelet::autoware::version)) {
      std::string warn = "format_version(" + std::to_string(map_major_ver) +
                         ") of the provided map(" + lanelet2_filename +
                         ") is larger than the supported version(" +
                         std::to_string(static_cast<uint64_t>(lanelet::autoware::version)) + ")";
      result.warnings.push_back(warn);
      if (!params_.allow_unsupported_version) {
        throw std::invalid_argument(
          "allow_unsupported_version is false, so stop loading lanelet map\n" + warn);
      }
    }
  }

  result.infos.push_back("Loaded map format_version: " + format_version);

  if (params_.use_waypoints) {
    lanelet::utils::overwriteLaneletsCenterlineWithWaypoints(
      map, params_.center_line_resolution, false);
  } else {
    lanelet::utils::overwriteLaneletsCenterline(map, params_.center_line_resolution, false);
  }

  result.map_bin_msg = create_map_bin_msg(map, lanelet2_filename);
  result.map_bin_msg.header.stamp = now;
  result.map_bin_msg.header.frame_id = "map";

  return result;
}

lanelet::LaneletMapPtr Lanelet2MapLoader::load_map(
  const std::string & lanelet2_filename,
  const autoware_map_msgs::msg::MapProjectorInfo & projector_info,
  std::vector<std::string> & warnings)
{
  lanelet::ErrorMessages errors{};
  if (projector_info.projector_type != autoware_map_msgs::msg::MapProjectorInfo::LOCAL) {
    std::unique_ptr<lanelet::Projector> projector =
      autoware::geography_utils::get_lanelet2_projector(projector_info);
    lanelet::LaneletMapPtr map = lanelet::load(lanelet2_filename, *projector, &errors);
    if (errors.empty()) return map;
  } else {
    const autoware::map_loader::LocalProjector projector;
    lanelet::LaneletMapPtr map = lanelet::load(lanelet2_filename, projector, &errors);
    if (!errors.empty()) {
      for (const auto & error : errors) warnings.push_back(error);
    }
    for (lanelet::Point3d point : map->pointLayer) {
      if (point.hasAttribute("local_x")) point.x() = point.attribute("local_x").asDouble().value();
      if (point.hasAttribute("local_y")) point.y() = point.attribute("local_y").asDouble().value();
    }
    for (lanelet::Lanelet lanelet : map->laneletLayer) {
      auto left = lanelet.leftBound();
      auto right = lanelet.rightBound();
      std::tie(left, right) = lanelet::geometry::align(left, right);
      lanelet.setLeftBound(left);
      lanelet.setRightBound(right);
    }
    return map;
  }
  throw MapLoadException(errors);
}

autoware_map_msgs::msg::LaneletMapBin Lanelet2MapLoader::create_map_bin_msg(
  const lanelet::LaneletMapPtr map, const std::string & lanelet2_filename)
{
  std::string format_version{};
  std::string map_version{};
  lanelet::io_handlers::AutowareOsmParser::parseVersions(
    lanelet2_filename, &format_version, &map_version);
  auto map_bin_msg = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  map_bin_msg.version_map_format = format_version;
  map_bin_msg.version_map = map_version;
  return map_bin_msg;
}

}  // namespace autoware::map_loader
