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

#ifndef LANELET2_MAP_LOADER__LANELET2_SELECTED_MAP_LOADER_MODULE_HPP_
#define LANELET2_MAP_LOADER__LANELET2_SELECTED_MAP_LOADER_MODULE_HPP_

#include "utils/lanelet2_map_cell_metadata.hpp"

#include <autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp>
#include <autoware/agnocast_wrapper/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_meta_data.hpp>
#include <autoware_map_msgs/msg/map_projector_info.hpp>
#include <autoware_map_msgs/srv/get_selected_lanelet2_map.hpp>

#include <map>
#include <string>
#include <vector>

namespace autoware::map_loader
{

/// @brief Provides on-demand loading of lanelet2 map cells via a ROS 2 service.
///
/// On construction the module publishes a `LaneletMapMetaData` message with
/// transient-local durability so any subscriber — including late-joiners — can
/// discover all available cells and their bounding boxes.
///
/// The `service/get_selected_lanelet2_map` service accepts a list of cell IDs,
/// loads the corresponding OSM files from disk, merges them into a single map,
/// applies centerline overwriting, and returns the serialised `LaneletMapBin`.
class Lanelet2SelectedMapLoaderModule
{
public:
  /// @param cell_metadata_dict    Pre-computed bounding boxes keyed by file path.
  /// @param projector_info        Projector used when reloading cells from disk.
  /// @param center_line_resolution  Passed to `overwriteLaneletsCenterline`.
  /// @param use_waypoints         If true, use waypoint-based centerline overwrite.
  Lanelet2SelectedMapLoaderModule(
    std::map<std::string, Lanelet2FileMetaData> cell_metadata_dict,
    const autoware_map_msgs::msg::MapProjectorInfo & projector_info, double center_line_resolution,
    bool use_waypoints);

  autoware_map_msgs::msg::LaneletMapMetaData build_metadata_msg() const;

  autoware_map_msgs::msg::LaneletMapBin execute(
    const std::vector<std::string> & cell_ids, std::vector<std::string> & warnings) const;

private:
  std::map<std::string, Lanelet2FileMetaData> all_cell_metadata_dict_;
  autoware_map_msgs::msg::MapProjectorInfo projector_info_;
  double center_line_resolution_;
  bool use_waypoints_;
};

}  // namespace autoware::map_loader

#endif  // LANELET2_MAP_LOADER__LANELET2_SELECTED_MAP_LOADER_MODULE_HPP_
