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

#include "selected_map_loader_module.hpp"

#include <map>
#include <string>
#include <utility>

namespace autoware::map_loader
{
SelectedMapLoaderModule::SelectedMapLoaderModule(
  rclcpp::Node * node, std::map<std::string, PCDFileMetadata> pcd_file_metadata_dict)
: logger_(node->get_logger()), all_pcd_file_metadata_dict_(std::move(pcd_file_metadata_dict))
{
  get_selected_pcd_maps_service_ = node->create_service<GetSelectedPointCloudMap>(
    "service/get_selected_pcd_map",
    std::bind(
      &SelectedMapLoaderModule::on_service_get_selected_point_cloud_map, this,
      std::placeholders::_1, std::placeholders::_2));

  // publish the map metadata
  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();
  pub_metadata_ = node->create_publisher<autoware_map_msgs::msg::PointCloudMapMetaData>(
    "output/pointcloud_map_metadata", durable_qos);
  pub_metadata_->publish(create_metadata(all_pcd_file_metadata_dict_));
}

bool SelectedMapLoaderModule::on_service_get_selected_point_cloud_map(
  GetSelectedPointCloudMap::Request::SharedPtr req,
  GetSelectedPointCloudMap::Response::SharedPtr res) const
{
  const auto load_plan = create_selected_map_load_plan(req->cell_ids, all_pcd_file_metadata_dict_);

  for (const auto & missing_id : load_plan.missing_ids) {
    RCLCPP_WARN(logger_, "ID %s not found", missing_id.c_str());
  }

  for (const auto & map_id : load_plan.map_ids_to_load) {
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
}  // namespace autoware::map_loader
