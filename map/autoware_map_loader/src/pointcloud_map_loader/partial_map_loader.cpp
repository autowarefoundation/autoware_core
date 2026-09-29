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

#include "partial_map_loader.hpp"

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace autoware::map_loader
{
PartialMapLoaderModule::PartialMapLoaderModule(
  std::map<std::string, PCDFileMetadata> pcd_file_metadata_dict,
  PointcloudLoaderLogFunction on_error)
: all_pcd_file_metadata_dict_(std::move(pcd_file_metadata_dict)), on_error_(std::move(on_error))
{
}

namespace
{
/// @brief Collect map IDs that intersect the requested area.
/// @param area Requested load area.
/// @param pcd_file_metadata_dict Metadata dictionary keyed by map ID.
/// @return Map IDs that should be loaded for the request.
std::vector<std::string> collect_partial_map_ids(
  const autoware_map_msgs::msg::AreaInfo & area,
  const std::map<std::string, PCDFileMetadata> & pcd_file_metadata_dict)
{
  std::vector<std::string> map_ids_to_load;
  for (const auto & ele : pcd_file_metadata_dict) {
    const std::string & map_id = ele.first;
    const PCDFileMetadata & metadata = ele.second;

    if (!is_grid_within_queried_area(area, metadata)) {
      continue;
    }
    map_ids_to_load.push_back(map_id);
  }

  return map_ids_to_load;
}
}  // namespace

bool PartialMapLoaderModule::create_response(
  GetPartialPointCloudMap::Request::SharedPtr req,
  GetPartialPointCloudMap::Response::SharedPtr res) const
{
  const auto map_ids_to_load = collect_partial_map_ids(req->area, all_pcd_file_metadata_dict_);
  for (const auto & map_id : map_ids_to_load) {
    const auto metadata_it = all_pcd_file_metadata_dict_.find(map_id);
    if (metadata_it == all_pcd_file_metadata_dict_.end()) {
      continue;
    }

    const auto & path = metadata_it->first;
    const auto & metadata = metadata_it->second;
    res->new_pointcloud_with_ids.push_back(
      load_point_cloud_map_cell_with_id(path, map_id, metadata, on_error_));
  }

  res->header.frame_id = "map";
  return true;
}
}  // namespace autoware::map_loader
