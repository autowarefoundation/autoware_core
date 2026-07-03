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

#include "differential_map_loader_module.hpp"

#include "differential_map_loader.hpp"

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace autoware::map_loader
{
DifferentialMapLoaderModule::DifferentialMapLoaderModule(
  std::map<std::string, PCDFileMetadata> pcd_file_metadata_dict)
: all_pcd_file_metadata_dict_(std::move(pcd_file_metadata_dict))
{}

bool DifferentialMapLoaderModule::create_response(
  GetDifferentialPointCloudMap::Request::SharedPtr req,
  GetDifferentialPointCloudMap::Response::SharedPtr res) const
{
  const auto plan =
    create_differential_map_load_plan(req->area, req->cached_ids, all_pcd_file_metadata_dict_);

  for (const auto & map_id : plan.map_ids_to_load) {
    const auto metadata_it = all_pcd_file_metadata_dict_.find(map_id);
    if (metadata_it == all_pcd_file_metadata_dict_.end()) {
      continue;
    }

    const auto & path = metadata_it->first;
    const auto & metadata = metadata_it->second;
    const auto loaded_cell = load_point_cloud_map_cell_with_id(path, map_id, metadata);
    if (!loaded_cell) {
      return false;
    }
    res->new_pointcloud_with_ids.push_back(loaded_cell.value());
  }

  res->ids_to_remove = plan.ids_to_remove;
  res->header.frame_id = "map";
  return true;
}

bool DifferentialMapLoaderModule::on_service_get_differential_point_cloud_map(
  GetDifferentialPointCloudMap::Request::SharedPtr req,
  GetDifferentialPointCloudMap::Response::SharedPtr res) const
{
  return create_response(req, res);
}
}  // namespace autoware::map_loader
