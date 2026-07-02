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

#ifndef POINTCLOUD_MAP_LOADER__POINTCLOUD_MAP_LOADER_NODE_CORE_HPP_
#define POINTCLOUD_MAP_LOADER__POINTCLOUD_MAP_LOADER_NODE_CORE_HPP_

#include "utils.hpp"

#include <functional>
#include <map>
#include <string>
#include <vector>

namespace autoware::map_loader
{
/// @brief Logging callback type used by node core logic.
using PointcloudMapLoaderNodeLogFunction = std::function<void(const std::string &)>;

/// @brief Resolve input paths to concrete PCD file paths.
/// @param pcd_paths_or_directory Input entries, each being a PCD file path or directory.
/// @param error_log Callback for invalid-path warnings.
/// @return Resolved PCD file paths.
std::vector<std::string> resolve_pcd_paths(
  const std::vector<std::string> & pcd_paths_or_directory,
  const PointcloudMapLoaderNodeLogFunction & error_log);

/// @brief Build metadata dictionary keyed by absolute PCD path.
/// @param pcd_metadata_path Path to metadata YAML file.
/// @param pcd_paths Resolved PCD file paths.
/// @return Metadata dictionary used by map loader modules.
/// @throws std::runtime_error on missing segments, missing metadata file, or PCD load failure.
std::map<std::string, PCDFileMetadata> build_pcd_metadata_dict(
  const std::string & pcd_metadata_path, const std::vector<std::string> & pcd_paths);
}  // namespace autoware::map_loader

#endif  // POINTCLOUD_MAP_LOADER__POINTCLOUD_MAP_LOADER_NODE_CORE_HPP_
