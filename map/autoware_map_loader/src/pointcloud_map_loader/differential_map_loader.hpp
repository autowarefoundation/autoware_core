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

#ifndef POINTCLOUD_MAP_LOADER__DIFFERENTIAL_MAP_LOADER_HPP_
#define POINTCLOUD_MAP_LOADER__DIFFERENTIAL_MAP_LOADER_HPP_

#include "utils.hpp"

#include <autoware_map_msgs/msg/area_info.hpp>

#include <map>
#include <string>
#include <vector>

namespace autoware::map_loader
{
/// @brief Result of differential map planning for a request.
struct DifferentialMapLoadPlan
{
  /// @brief Map IDs that should be newly loaded for the requested area.
  std::vector<std::string> map_ids_to_load;
  /// @brief Cached map IDs that are no longer needed and should be removed.
  std::vector<std::string> ids_to_remove;
};

/// @brief Build a differential load plan from requested area and currently cached IDs.
/// @param area_info Requested load area.
/// @param cached_ids IDs already cached by the requester.
/// @param pcd_file_metadata_dict Metadata dictionary keyed by map ID.
/// @return Differential load plan with IDs to load and remove.
DifferentialMapLoadPlan create_differential_map_load_plan(
  const autoware_map_msgs::msg::AreaInfo & area_info, const std::vector<std::string> & cached_ids,
  const std::map<std::string, PCDFileMetadata> & pcd_file_metadata_dict);
}  // namespace autoware::map_loader

#endif  // POINTCLOUD_MAP_LOADER__DIFFERENTIAL_MAP_LOADER_HPP_
