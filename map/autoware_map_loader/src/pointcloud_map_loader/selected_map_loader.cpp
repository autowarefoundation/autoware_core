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
#include <vector>

namespace autoware::map_loader
{
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
}  // namespace autoware::map_loader
