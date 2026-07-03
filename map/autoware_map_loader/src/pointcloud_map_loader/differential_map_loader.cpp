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

#include "differential_map_loader.hpp"

#include <algorithm>
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace autoware::map_loader
{
DifferentialMapLoaderModule::DifferentialMapLoaderModule(
  std::map<std::string, PCDFileMetadata> pcd_file_metadata_dict)
: all_pcd_file_metadata_dict_(std::move(pcd_file_metadata_dict))
{
}

DifferentialMapLoaderModule::DifferentialMapLoaderModule(
  rclcpp::Node * node, std::map<std::string, PCDFileMetadata> pcd_file_metadata_dict)
: DifferentialMapLoaderModule(std::move(pcd_file_metadata_dict))
{
  get_differential_pcd_maps_service_ = node->create_service<GetDifferentialPointCloudMap>(
    "service/get_differential_pcd_map",
    std::bind(
      &DifferentialMapLoaderModule::on_service_get_differential_point_cloud_map, this,
      std::placeholders::_1, std::placeholders::_2));
}

DifferentialMapLoadPlan create_differential_map_load_plan(
  const autoware_map_msgs::msg::AreaInfo & area_info, const std::vector<std::string> & cached_ids,
  const std::map<std::string, PCDFileMetadata> & pcd_file_metadata_dict)
{
  DifferentialMapLoadPlan plan;

  std::vector<bool> should_remove(static_cast<int>(cached_ids.size()), true);
  for (const auto & ele : pcd_file_metadata_dict) {
    const std::string & path = ele.first;
    const PCDFileMetadata & metadata = ele.second;

    // Assume that the map ID = map path (for now).
    const std::string & map_id = path;

    if (!is_grid_within_queried_area(area_info, metadata)) {
      continue;
    }

    const auto id_in_cached_list = std::find(cached_ids.begin(), cached_ids.end(), map_id);
    if (id_in_cached_list != cached_ids.end()) {
      const auto index = static_cast<int>(id_in_cached_list - cached_ids.begin());
      should_remove[index] = false;
    } else {
      plan.map_ids_to_load.push_back(map_id);
    }
  }

  for (size_t i = 0; i < cached_ids.size(); ++i) {
    if (should_remove[i]) {
      plan.ids_to_remove.push_back(cached_ids[i]);
    }
  }

  return plan;
}

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
