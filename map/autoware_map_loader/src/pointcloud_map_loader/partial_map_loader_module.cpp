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

#include "partial_map_loader.hpp"

#include <map>
#include <string>
#include <utility>

namespace autoware::map_loader
{
PartialMapLoaderModule::PartialMapLoaderModule(
  std::map<std::string, PCDFileMetadata> pcd_file_metadata_dict, rclcpp::Logger logger)
: logger_(std::move(logger)), all_pcd_file_metadata_dict_(std::move(pcd_file_metadata_dict))
{}

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
      load_point_cloud_map_cell_with_id(logger_, path, map_id, metadata));
  }

  res->header.frame_id = "map";
  return true;
}

bool PartialMapLoaderModule::on_service_get_partial_point_cloud_map(
  GetPartialPointCloudMap::Request::SharedPtr req,
  GetPartialPointCloudMap::Response::SharedPtr res) const
{
  return create_response(req, res);
}
}  // namespace autoware::map_loader
