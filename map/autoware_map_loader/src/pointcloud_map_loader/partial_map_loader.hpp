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

#ifndef POINTCLOUD_MAP_LOADER__PARTIAL_MAP_LOADER_HPP_
#define POINTCLOUD_MAP_LOADER__PARTIAL_MAP_LOADER_HPP_

#include "utils.hpp"

#include <autoware_map_msgs/msg/area_info.hpp>
#include <autoware_map_msgs/srv/get_partial_point_cloud_map.hpp>

#include <map>
#include <string>
#include <vector>

namespace autoware::map_loader
{
/// @brief Collect map IDs that intersect the requested area.
/// @param area Requested load area.
/// @param pcd_file_metadata_dict Metadata dictionary keyed by map ID.
/// @return Map IDs that should be loaded for the request.
std::vector<std::string> collect_partial_map_ids(
  const autoware_map_msgs::msg::AreaInfo & area,
  const std::map<std::string, PCDFileMetadata> & pcd_file_metadata_dict);

class PartialMapLoaderModule
{
  using GetPartialPointCloudMap = autoware_map_msgs::srv::GetPartialPointCloudMap;

public:
  explicit PartialMapLoaderModule(std::map<std::string, PCDFileMetadata> pcd_file_metadata_dict);

  [[nodiscard]] bool create_response(
    GetPartialPointCloudMap::Request::SharedPtr req,
    GetPartialPointCloudMap::Response::SharedPtr res) const;

private:
  std::map<std::string, PCDFileMetadata> all_pcd_file_metadata_dict_;
};
}  // namespace autoware::map_loader

#endif  // POINTCLOUD_MAP_LOADER__PARTIAL_MAP_LOADER_HPP_
