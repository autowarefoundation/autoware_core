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

#ifndef POINTCLOUD_MAP_LOADER__POINTCLOUD_MAP_LOADER_HPP_
#define POINTCLOUD_MAP_LOADER__POINTCLOUD_MAP_LOADER_HPP_

#include "utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <boost/optional.hpp>

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::map_loader
{
/// @brief Progress callback invoked periodically while loading multiple PCD files.
using PointcloudLoaderProgressFunction = std::function<void(
  size_t processed_pcd_file_num, size_t pcd_path_size, const std::string & path)>;

/// @brief Downsample a pointcloud message with a voxel-grid filter.
/// @param msg_input Input pointcloud message.
/// @param leaf_size Voxel leaf size in meters.
/// @return Downsampled pointcloud message.
sensor_msgs::msg::PointCloud2 downsample_pointcloud(
  const sensor_msgs::msg::PointCloud2 & msg_input, float leaf_size);

/// @brief Load and merge multiple PCD files into a single pointcloud message.
/// @param pcd_paths Absolute paths to source PCD files.
/// @param leaf_size Optional downsample leaf size. If not set, downsampling is skipped.
/// @param on_progress Callback invoked during loading progress.
/// @param on_error Callback invoked when a PCD load failure occurs.
/// @return Merged pointcloud message. Failed files are reported through @on_error and skipped.
sensor_msgs::msg::PointCloud2 load_pointcloud_map(
  const std::vector<std::string> & pcd_paths, boost::optional<float> leaf_size,
  const PointcloudLoaderProgressFunction & on_progress = PointcloudLoaderProgressFunction{},
  const PointcloudLoaderLogFunction & on_error = PointcloudLoaderLogFunction{});

/// @brief Resolve input paths to concrete PCD file paths.
/// @param pcd_paths_or_directory Input entries, each being a PCD file path or directory.
/// @param error_log Callback for invalid-path warnings.
/// @return Resolved PCD file paths.
std::vector<std::string> resolve_pcd_paths(
  const std::vector<std::string> & pcd_paths_or_directory,
  const PointcloudLoaderLogFunction & error_log);

/// @brief Build metadata dictionary keyed by absolute PCD path.
/// @param pcd_metadata_path Path to metadata YAML file.
/// @param pcd_paths Resolved PCD file paths.
/// @return Metadata dictionary used by map loader modules.
/// @throws std::runtime_error on missing segments, missing metadata file, or PCD load failure.
std::map<std::string, PCDFileMetadata> build_pcd_metadata_dict(
  const std::string & pcd_metadata_path, const std::vector<std::string> & pcd_paths);

class PointcloudMapLoaderModule
{
public:
  PointcloudMapLoaderModule() = default;

  static sensor_msgs::msg::PointCloud2 create_map_message(
    const std::vector<std::string> & pcd_paths, boost::optional<float> leaf_size,
    const PointcloudLoaderProgressFunction & on_progress = PointcloudLoaderProgressFunction{},
    const PointcloudLoaderLogFunction & on_error = PointcloudLoaderLogFunction{});
};
}  // namespace autoware::map_loader

#endif  // POINTCLOUD_MAP_LOADER__POINTCLOUD_MAP_LOADER_HPP_
