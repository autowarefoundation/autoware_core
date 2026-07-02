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

#include "pointcloud_map_loader_node.hpp"

#include "pointcloud_map_loader_node_core.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::map_loader
{
PointCloudMapLoaderNode::PointCloudMapLoaderNode(const rclcpp::NodeOptions & options)
: Node("pointcloud_map_loader", options)
{
  const auto pcd_paths = resolve_pcd_paths(
    declare_parameter<std::vector<std::string>>("pcd_paths_or_directory"),
    [this](const std::string & msg) { RCLCPP_ERROR_STREAM(get_logger(), msg); });
  std::string pcd_metadata_path = declare_parameter<std::string>("pcd_metadata_path");
  bool enable_whole_load = declare_parameter<bool>("enable_whole_load");
  bool enable_downsample_whole_load = declare_parameter<bool>("enable_downsampled_whole_load");
  bool enable_partial_load = declare_parameter<bool>("enable_partial_load");
  bool enable_selected_load = declare_parameter<bool>("enable_selected_load");

  if (enable_whole_load) {
    std::string publisher_name = "output/pointcloud_map";
    pcd_map_loader_ =
      std::make_unique<PointcloudMapLoaderModule>(this, pcd_paths, publisher_name, false);
  }

  if (enable_downsample_whole_load) {
    std::string publisher_name = "output/debug/downsampled_pointcloud_map";
    downsampled_pcd_map_loader_ =
      std::make_unique<PointcloudMapLoaderModule>(this, pcd_paths, publisher_name, true);
  }

  // Parse the metadata file and get the map of (absolute pcd path, pcd file metadata)
  auto pcd_metadata_dict = build_pcd_metadata_dict(pcd_metadata_path, pcd_paths);

  if (enable_partial_load) {
    partial_map_loader_ = std::make_unique<PartialMapLoaderModule>(this, pcd_metadata_dict);
  }

  differential_map_loader_ = std::make_unique<DifferentialMapLoaderModule>(this, pcd_metadata_dict);

  if (enable_selected_load) {
    selected_map_loader_ = std::make_unique<SelectedMapLoaderModule>(this, pcd_metadata_dict);
  }
}
}  // namespace autoware::map_loader

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::map_loader::PointCloudMapLoaderNode)
