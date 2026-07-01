// Copyright 2022 The Autoware Contributors
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

#ifndef POINTCLOUD_MAP_LOADER__SELECTED_MAP_LOADER_HPP_
#define POINTCLOUD_MAP_LOADER__SELECTED_MAP_LOADER_HPP_

#include "utils.hpp"

#include <autoware_map_msgs/msg/point_cloud_map_meta_data.hpp>

#include <map>
#include <string>
#include <vector>

namespace autoware::map_loader
{
/// @brief Planning result for selected-map requests.
struct SelectedMapLoadPlan
{
  /// @brief IDs found in the metadata dictionary and to be loaded.
  std::vector<std::string> map_ids_to_load;
  /// @brief Requested IDs not found in the metadata dictionary.
  std::vector<std::string> missing_ids;
};

/// @brief Build metadata message published for map cell bounds.
/// @param pcd_file_metadata_dict Metadata dictionary keyed by map ID.
/// @return PointCloudMapMetaData message for all known map cells.
autoware_map_msgs::msg::PointCloudMapMetaData create_metadata(
  const std::map<std::string, PCDFileMetadata> & pcd_file_metadata_dict);

/// @brief Create a load plan from requested selected IDs and known metadata.
/// @param request_ids Requested map IDs.
/// @param pcd_file_metadata_dict Metadata dictionary keyed by map ID.
/// @return Plan containing IDs to load and missing IDs.
SelectedMapLoadPlan create_selected_map_load_plan(
  const std::vector<std::string> & request_ids,
  const std::map<std::string, PCDFileMetadata> & pcd_file_metadata_dict);
}  // namespace autoware::map_loader

#endif  // POINTCLOUD_MAP_LOADER__SELECTED_MAP_LOADER_HPP_
