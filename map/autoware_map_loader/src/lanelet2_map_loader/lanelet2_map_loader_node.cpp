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

#include "autoware/map_loader/lanelet2_map_loader_node.hpp"

#include "lanelet2_local_projector.hpp"
#include "lanelet2_map_cell_metadata.hpp"
#include "lanelet2_map_loader_utils.hpp"
#include "lanelet2_selected_map_loader_module.hpp"

#include <autoware/geography_utils/lanelet2_projector.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_lanelet2_extension/io/autoware_osm_parser.hpp>
#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>
#include <autoware_lanelet2_extension/projection/transverse_mercator_projector.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::map_loader
{

using autoware_map_msgs::msg::LaneletMapBin;
using autoware_map_msgs::msg::MapProjectorInfo;

Lanelet2MapLoaderNode::Lanelet2MapLoaderNode(const rclcpp::NodeOptions & options)
: Node("lanelet2_map_loader", options)
{
  // subscription
  sub_map_projector_info_ = this->create_subscription<MapProjectorInfo::Message>(
    MapProjectorInfo::name, autoware::component_interface_specs::get_qos<MapProjectorInfo>(),
    [this](const MapProjectorInfo::Message::ConstSharedPtr msg) { on_map_projector_info(msg); });

  declare_parameter<bool>("allow_unsupported_version");
  declare_parameter<std::string>("lanelet2_map_path");
  declare_parameter<double>("center_line_resolution");
  declare_parameter<bool>("use_waypoints");
  declare_parameter<bool>("enable_selected_map_loading");
  declare_parameter<std::string>("metadata_file_path");
}

// Defined here so the compiler sees the full type of Lanelet2SelectedMapLoaderModule
// when generating the unique_ptr destructor.
Lanelet2MapLoaderNode::~Lanelet2MapLoaderNode() = default;

void Lanelet2MapLoaderNode::on_map_projector_info(
  const MapProjectorInfo::Message::ConstSharedPtr msg)
{
  const auto allow_unsupported_version = get_parameter("allow_unsupported_version").as_bool();
  const auto lanelet2_map_path = get_parameter("lanelet2_map_path").as_string();
  const auto center_line_resolution = get_parameter("center_line_resolution").as_double();
  const auto use_waypoints = get_parameter("use_waypoints").as_bool();
  const auto enable_selected_map_loading =
    get_parameter("enable_selected_map_loading").as_bool();
  const auto metadata_file_path = get_parameter("metadata_file_path").as_string();

  // get lanelet2 file paths (handles both a single .osm file and a directory)
  const std::vector<std::string> lanelet2_paths = utils::get_lanelet2_paths(lanelet2_map_path);
  if (lanelet2_paths.empty()) {
    RCLCPP_ERROR(get_logger(), "No lanelet2 map files found from %s", lanelet2_map_path.c_str());
    return;
  }

  // load maps from files
  // Note: we cannot destroy loaded maps even after merging because lanelets expire when the map
  // object is destructed
  std::vector<lanelet::LaneletMapPtr> maps;
  for (const auto & path : lanelet2_paths) {
    auto map_tmp = load_map(path, *msg);
    if (!map_tmp) {
      RCLCPP_ERROR(get_logger(), "Failed to load lanelet2_map: %s", path.c_str());
      return;
    }
    maps.push_back(map_tmp);
  }

  if (enable_selected_map_loading) {
    std::map<std::string, Lanelet2FileMetaData> cell_metadata_dict;

    const auto yaml_metadata = utils::load_cell_metadata_from_yaml(metadata_file_path);
    if (yaml_metadata) {
      cell_metadata_dict = *yaml_metadata;
      RCLCPP_INFO(get_logger(), "Loaded cell metadata from %s.", metadata_file_path.c_str());
    } else {
      for (size_t i = 0; i < lanelet2_paths.size(); ++i) {
        cell_metadata_dict[lanelet2_paths[i]] =
          utils::compute_cell_metadata(lanelet2_paths[i], *maps[i]);
      }
    }

    selected_map_loader_module_ = std::make_unique<Lanelet2SelectedMapLoaderModule>(
      this, std::move(cell_metadata_dict), *msg, center_line_resolution, use_waypoints);

    RCLCPP_INFO(get_logger(), "Selected lanelet2 map loading is enabled.");
  }

  // merge all loaded maps into a new empty map
  auto map = std::make_shared<lanelet::LaneletMap>();
  for (auto & loaded_map : maps) {
    utils::merge_lanelet2_maps(*map, *loaded_map);
  }

  // use the first file path for format/version metadata
  const auto & lanelet2_filename = lanelet2_paths.front();

  std::string format_version{"null"}, map_version{""};
  lanelet::io_handlers::AutowareOsmParser::parseVersions(
    lanelet2_filename, &format_version, &map_version);
  if (format_version == "null" || format_version.empty() || !isdigit(format_version[0])) {
    RCLCPP_WARN(
      get_logger(),
      "%s has no format_version(null) or non semver-style format_version(%s) information",
      lanelet2_filename.c_str(), format_version.c_str());
    if (!allow_unsupported_version) {
      throw std::invalid_argument(
        "allow_unsupported_version is false, so stop loading lanelet map");
    }
  } else if (const auto map_major_ver_opt = lanelet::io_handlers::parseMajorVersion(format_version);
             map_major_ver_opt.has_value()) {
    const auto map_major_ver = map_major_ver_opt.value();
    if (map_major_ver > static_cast<uint64_t>(lanelet::autoware::version)) {
      RCLCPP_WARN(
        get_logger(),
        "format_version(%ld) of the provided map(%s) is larger than the supported version(%ld)",
        map_major_ver, lanelet2_filename.c_str(),
        static_cast<uint64_t>(lanelet::autoware::version));
      if (!allow_unsupported_version) {
        throw std::invalid_argument(
          "allow_unsupported_version is false, so stop loading lanelet map");
      }
    }
  }
  RCLCPP_INFO(get_logger(), "Loaded map format_version: %s", format_version.c_str());

  // overwrite centerline
  if (use_waypoints) {
    lanelet::utils::overwriteLaneletsCenterlineWithWaypoints(map, center_line_resolution, false);
  } else {
    lanelet::utils::overwriteLaneletsCenterline(map, center_line_resolution, false);
  }

  // create map bin msg
  const auto map_bin_msg = create_map_bin_msg(map, lanelet2_filename, now());

  // create publisher and publish
  pub_map_bin_ =
    create_publisher<VectorMap::Message>(VectorMap::name, rclcpp::QoS{1}.transient_local());
  pub_map_bin_->publish(map_bin_msg);
  RCLCPP_INFO(get_logger(), "Succeeded to load lanelet2_map. Map is published.");
}

lanelet::LaneletMapPtr Lanelet2MapLoaderNode::load_map(
  const std::string & lanelet2_filename,
  const autoware_map_msgs::msg::MapProjectorInfo & projector_info)
{
  lanelet::ErrorMessages errors{};
  if (projector_info.projector_type != autoware_map_msgs::msg::MapProjectorInfo::LOCAL) {
    std::unique_ptr<lanelet::Projector> projector =
      autoware::geography_utils::get_lanelet2_projector(projector_info);
    lanelet::LaneletMapPtr map = lanelet::load(lanelet2_filename, *projector, &errors);
    if (errors.empty()) {
      return map;
    }
  } else {
    const autoware::map_loader::LocalProjector projector;
    lanelet::LaneletMapPtr map = lanelet::load(lanelet2_filename, projector, &errors);

    if (!errors.empty()) {
      for (const auto & error : errors) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_loader"), error);
      }
    }

    // overwrite local_x, local_y
    for (lanelet::Point3d point : map->pointLayer) {
      if (point.hasAttribute("local_x")) {
        point.x() = point.attribute("local_x").asDouble().value();
      }
      if (point.hasAttribute("local_y")) {
        point.y() = point.attribute("local_y").asDouble().value();
      }
    }

    // realign lanelet borders using updated points
    for (lanelet::Lanelet lanelet : map->laneletLayer) {
      auto left = lanelet.leftBound();
      auto right = lanelet.rightBound();
      std::tie(left, right) = lanelet::geometry::align(left, right);
      lanelet.setLeftBound(left);
      lanelet.setRightBound(right);
    }

    return map;
  }

  for (const auto & error : errors) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_loader"), error);
  }
  return nullptr;
}

LaneletMapBin Lanelet2MapLoaderNode::create_map_bin_msg(
  const lanelet::LaneletMapPtr map, const std::string & lanelet2_filename, const rclcpp::Time & now)
{
  std::string format_version{};
  std::string map_version{};
  lanelet::io_handlers::AutowareOsmParser::parseVersions(
    lanelet2_filename, &format_version, &map_version);

  LaneletMapBin map_bin_msg = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  map_bin_msg.header.stamp = now;
  map_bin_msg.header.frame_id = "map";
  map_bin_msg.version_map_format = format_version;
  map_bin_msg.version_map = map_version;

  return map_bin_msg;
}
}  // namespace autoware::map_loader

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::map_loader::Lanelet2MapLoaderNode)
