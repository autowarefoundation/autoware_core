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

#include "partial_map_loader_module.hpp"

#include <map>
#include <string>
#include <utility>

namespace autoware::map_loader
{
PartialMapLoaderModule::PartialMapLoaderModule(
  autoware::agnocast_wrapper::Node * node,
  std::map<std::string, PCDFileMetadata> pcd_file_metadata_dict)
: logger_(node->get_logger()), all_pcd_file_metadata_dict_(std::move(pcd_file_metadata_dict))
{
  get_partial_pcd_maps_service_ = node->create_service<GetPartialPointCloudMap>(
    "service/get_partial_pcd_map",
    std::bind(
      &PartialMapLoaderModule::on_service_get_partial_point_cloud_map, this, std::placeholders::_1,
      std::placeholders::_2));
}

void PartialMapLoaderModule::partial_area_load(
  const autoware_map_msgs::msg::AreaInfo & area,
  const AUTOWARE_SERVICE_RESPONSE_PTR(GetPartialPointCloudMap) & response) const
{
  // iterate over all the available pcd map grids
  for (const auto & ele : all_pcd_file_metadata_dict_) {
    std::string path = ele.first;
    PCDFileMetadata metadata = ele.second;

    // assume that the map ID = map path (for now)
    const std::string & map_id = path;

    // skip if the pcd file is not within the queried area
    if (!is_grid_within_queried_area(area, metadata)) continue;

    response->new_pointcloud_with_ids.push_back(
      load_point_cloud_map_cell_with_id(logger_, path, map_id, metadata));
  }
}

bool PartialMapLoaderModule::on_service_get_partial_point_cloud_map(
  AUTOWARE_SERVICE_REQUEST_PTR(GetPartialPointCloudMap) req,
  AUTOWARE_SERVICE_RESPONSE_PTR(GetPartialPointCloudMap) res) const
{
  auto area = req->area;
  partial_area_load(area, res);
  res->header.frame_id = "map";
  return true;
}
}  // namespace autoware::map_loader
