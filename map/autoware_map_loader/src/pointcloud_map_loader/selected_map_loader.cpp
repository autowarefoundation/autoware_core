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

#include "selected_map_loader.hpp"

#include <rclcpp/clock.hpp>

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace autoware::map_loader
{
SelectedMapLoaderModule::SelectedMapLoaderModule(
  std::map<std::string, PCDFileMetadata> pcd_file_metadata_dict,
  PointcloudLoaderLogFunction on_error)
: all_pcd_file_metadata_dict_(std::move(pcd_file_metadata_dict)), on_error_(std::move(on_error))
{
}

autoware_map_msgs::msg::PointCloudMapMetaData create_metadata(
  const std::map<std::string, PCDFileMetadata> & pcd_file_metadata_dict)
{
  autoware_map_msgs::msg::PointCloudMapMetaData metadata_msg;
  metadata_msg.header.frame_id = "map";
  metadata_msg.header.stamp = rclcpp::Clock().now();

  for (const auto & ele : pcd_file_metadata_dict) {
    const PCDFileMetadata & metadata = ele.second;

    // Assume that the map ID = map path (for now).
    const std::string & map_id = ele.first;

    autoware_map_msgs::msg::PointCloudMapCellMetaDataWithID cell_metadata_with_id;
    cell_metadata_with_id.cell_id = map_id;
    cell_metadata_with_id.metadata.min_x = metadata.min.x;
    cell_metadata_with_id.metadata.min_y = metadata.min.y;
    cell_metadata_with_id.metadata.max_x = metadata.max.x;
    cell_metadata_with_id.metadata.max_y = metadata.max.y;

    metadata_msg.metadata_list.push_back(cell_metadata_with_id);
  }

  return metadata_msg;
}

SelectedMapLoadPlan create_selected_map_load_plan(
  const std::vector<std::string> & request_ids,
  const std::map<std::string, PCDFileMetadata> & pcd_file_metadata_dict)
{
  SelectedMapLoadPlan plan;

  for (const auto & request_id : request_ids) {
    const auto selected_map_it = pcd_file_metadata_dict.find(request_id);
    if (selected_map_it == pcd_file_metadata_dict.end()) {
      plan.missing_ids.push_back(request_id);
      continue;
    }

    const std::string & map_id = selected_map_it->first;
    plan.map_ids_to_load.push_back(map_id);
  }

  return plan;
}

bool SelectedMapLoaderModule::create_response(
  GetSelectedPointCloudMap::Request::SharedPtr req,
  GetSelectedPointCloudMap::Response::SharedPtr res) const
{
  const auto load_plan = create_selected_map_load_plan(req->cell_ids, all_pcd_file_metadata_dict_);

  if (on_error_) {
    for (const auto & missing_id : load_plan.missing_ids) {
      on_error_("ID not found: " + missing_id);
    }
  }

  for (const auto & map_id : load_plan.map_ids_to_load) {
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
