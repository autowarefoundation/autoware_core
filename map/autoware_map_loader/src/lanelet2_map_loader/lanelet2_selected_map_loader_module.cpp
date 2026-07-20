// Copyright 2026 The Autoware Contributors
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

#include "lanelet2_selected_map_loader_module.hpp"

#include "lanelet2_map_loader.hpp"
#include "utils/lanelet2_map_loader_utils.hpp"

#include <autoware/map_loader/lanelet2_map_loader_node.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::map_loader
{

autoware_map_msgs::msg::LaneletMapMetaData Lanelet2SelectedMapLoaderModule::build_metadata_msg()
  const
{
  autoware_map_msgs::msg::LaneletMapMetaData msg;

  for (const auto & [path, meta] : all_cell_metadata_dict_) {
    autoware_map_msgs::msg::LaneletMapCellMetaData cell;
    cell.cell_id = meta.id;
    cell.min_x = meta.min_x;
    cell.min_y = meta.min_y;
    cell.max_x = meta.max_x;
    cell.max_y = meta.max_y;
    msg.metadata_list.push_back(cell);
  }

  return msg;
}

Lanelet2SelectedMapLoaderModule::Lanelet2SelectedMapLoaderModule(
  std::map<std::string, Lanelet2FileMetaData> cell_metadata_dict,
  const autoware_map_msgs::msg::MapProjectorInfo & projector_info,
  const double center_line_resolution, const bool use_waypoints)
: all_cell_metadata_dict_(std::move(cell_metadata_dict)),
  projector_info_(projector_info),
  center_line_resolution_(center_line_resolution),
  use_waypoints_(use_waypoints)
{
}

autoware_map_msgs::msg::LaneletMapBin Lanelet2SelectedMapLoaderModule::execute(
  const std::vector<std::string> & cell_ids) const
{
  // Resolve requested cell IDs to file paths.
  std::vector<std::string> paths;
  for (const auto & cell_id : cell_ids) {
    const auto it = all_cell_metadata_dict_.find(cell_id);
    if (it == all_cell_metadata_dict_.end()) {
      continue;
    }
    if (!std::filesystem::exists(cell_id)) {
      continue;
    }
    paths.push_back(cell_id);
  }

  if (paths.empty()) {
    return autoware_map_msgs::msg::LaneletMapBin();
  }

  std::vector<lanelet::LaneletMapPtr> maps;
  maps.reserve(paths.size());
  for (const auto & path : paths) {
    std::vector<std::string> warnings;

    try {
      auto map = Lanelet2MapLoader::load_map(path, projector_info_, warnings);
      if (map) {
        maps.push_back(std::move(map));
      }
    } catch (const MapLoadException & e) {
      // Here I catch exp and still continue so a single bad cell won't crash entire node
      continue;
    }
  }

  auto merged = std::make_shared<lanelet::LaneletMap>();
  for (auto & m : maps) {
    utils::merge_lanelet2_maps(*merged, *m);
  }

  if (use_waypoints_) {
    lanelet::utils::overwriteLaneletsCenterlineWithWaypoints(
      merged, center_line_resolution_, false);
  } else {
    lanelet::utils::overwriteLaneletsCenterline(merged, center_line_resolution_, false);
  }

  // Pure logic only.
  // Headers (stamp and frame_id) are now node's responsibility.
  return Lanelet2MapLoader::create_map_bin_msg(merged, paths.front());
}

}  // namespace autoware::map_loader
