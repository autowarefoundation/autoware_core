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

#include "lanelet2_selected_map_loader_module.hpp"

#include "lanelet2_map_loader_utils.hpp"

#include <autoware/map_loader/lanelet2_map_loader_node.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::map_loader
{

namespace
{

autoware_map_msgs::msg::LaneletMapMetaData build_metadata_msg(
  const std::map<std::string, Lanelet2FileMetaData> & cell_metadata_dict,
  const rclcpp::Time & stamp)
{
  autoware_map_msgs::msg::LaneletMapMetaData msg;
  msg.header.frame_id = "map";
  msg.header.stamp = stamp;

  for (const auto & [path, meta] : cell_metadata_dict) {
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

}  // namespace

Lanelet2SelectedMapLoaderModule::Lanelet2SelectedMapLoaderModule(
  rclcpp::Node * node, std::map<std::string, Lanelet2FileMetaData> cell_metadata_dict,
  const autoware_map_msgs::msg::MapProjectorInfo & projector_info,
  const double center_line_resolution, const bool use_waypoints)
: logger_(node->get_logger()),
  clock_(node->get_clock()),
  all_cell_metadata_dict_(std::move(cell_metadata_dict)),
  projector_info_(projector_info),
  center_line_resolution_(center_line_resolution),
  use_waypoints_(use_waypoints)
{
  pub_metadata_ = node->create_publisher<autoware_map_msgs::msg::LaneletMapMetaData>(
    "output/lanelet2_map_metadata", rclcpp::QoS{1}.transient_local());
  pub_metadata_->publish(build_metadata_msg(all_cell_metadata_dict_, node->now()));

  service_ = node->create_service<GetSelectedLanelet2Map>(
    "service/get_selected_lanelet2_map",
    std::bind(
      &Lanelet2SelectedMapLoaderModule::on_service_get_selected_lanelet2_map, this,
      std::placeholders::_1, std::placeholders::_2));
}

bool Lanelet2SelectedMapLoaderModule::on_service_get_selected_lanelet2_map(
  GetSelectedLanelet2Map::Request::SharedPtr req,
  GetSelectedLanelet2Map::Response::SharedPtr res) const
{
  // Resolve requested cell IDs to file paths.
  std::vector<std::string> paths;
  for (const auto & cell_id : req->cell_ids) {
    const auto it = all_cell_metadata_dict_.find(cell_id);
    if (it == all_cell_metadata_dict_.end()) {
      RCLCPP_WARN(logger_, "Requested cell ID not found in metadata: %s", cell_id.c_str());
      continue;
    }
    if (!std::filesystem::exists(cell_id)) {
      RCLCPP_WARN(logger_, "Map file no longer exists on disk: %s", cell_id.c_str());
      continue;
    }
    paths.push_back(cell_id);
  }

  if (paths.empty()) {
    RCLCPP_ERROR(logger_, "No valid cell IDs in GetSelectedLanelet2Map request.");
    return false;
  }

  // Load each cell from disk.
  // All loaded maps must remain alive until after serialisation because
  // lanelet primitives hold shared ownership of their data through the source map.
  std::vector<lanelet::LaneletMapPtr> maps;
  maps.reserve(paths.size());
  for (const auto & path : paths) {
    auto map = Lanelet2MapLoaderNode::load_map(path, projector_info_);
    if (!map) {
      RCLCPP_ERROR(logger_, "Failed to load map cell: %s", path.c_str());
      return false;
    }
    maps.push_back(std::move(map));
  }

  // Merge all cells into one map.
  auto merged = std::make_shared<lanelet::LaneletMap>();
  for (auto & m : maps) {
    utils::merge_lanelet2_maps(*merged, *m);
  }

  // Apply centerline overwriting consistent with the whole-map load path.
  if (use_waypoints_) {
    lanelet::utils::overwriteLaneletsCenterlineWithWaypoints(
      merged, center_line_resolution_, false);
  } else {
    lanelet::utils::overwriteLaneletsCenterline(merged, center_line_resolution_, false);
  }

  res->lanelet2_cells =
    Lanelet2MapLoaderNode::create_map_bin_msg(merged, paths.front(), clock_->now());
  res->header.frame_id = "map";
  return true;
}

}  // namespace autoware::map_loader
