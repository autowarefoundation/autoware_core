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

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <boost/optional.hpp>
#include <tl/expected.hpp>

#include <functional>
#include <map>
#include <string>
#include <vector>

namespace autoware::map_loader
{
/// @brief Logging callback type used by pointcloud map loading core logic.
using PointcloudLoaderLogFunction = std::function<void(const std::string &)>;

struct LoadPointcloudMapSuccess
{
  sensor_msgs::msg::PointCloud2 loaded_pcd;
  std::vector<std::string> debug_messages;
};

struct LoadPointcloudMapError
{
  std::string error_message;
  std::vector<std::string> debug_messages;
};

using LoadPointcloudMapResult = tl::expected<LoadPointcloudMapSuccess, LoadPointcloudMapError>;

/// @brief Downsample a pointcloud message with a voxel-grid filter.
/// @param msg_input Input pointcloud message.
/// @param leaf_size Voxel leaf size in meters.
/// @return Downsampled pointcloud message.
sensor_msgs::msg::PointCloud2 downsample_pointcloud(
  const sensor_msgs::msg::PointCloud2 & msg_input, float leaf_size);

/// @brief Load and merge multiple PCD files into a single pointcloud message.
/// @param pcd_paths Absolute paths to source PCD files.
/// @param leaf_size Optional downsample leaf size. If not set, downsampling is skipped.
/// @return Merged pointcloud message and collected debug logs on success,
///         or an error message on failure.
LoadPointcloudMapResult load_pointcloud_map(
  const std::vector<std::string> & pcd_paths, boost::optional<float> leaf_size);

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
}  // namespace autoware::map_loader

#endif  // POINTCLOUD_MAP_LOADER__POINTCLOUD_MAP_LOADER_HPP_
