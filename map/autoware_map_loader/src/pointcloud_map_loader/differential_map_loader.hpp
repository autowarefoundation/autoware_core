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

#ifndef POINTCLOUD_MAP_LOADER__DIFFERENTIAL_MAP_LOADER_HPP_
#define POINTCLOUD_MAP_LOADER__DIFFERENTIAL_MAP_LOADER_HPP_

#include "utils.hpp"

#include <autoware_map_msgs/msg/area_info.hpp>
#include <autoware_map_msgs/srv/get_differential_point_cloud_map.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::map_loader
{
class DifferentialMapLoaderModule
{
  using GetDifferentialPointCloudMap = autoware_map_msgs::srv::GetDifferentialPointCloudMap;

public:
  explicit DifferentialMapLoaderModule(
    std::map<std::string, PCDFileMetadata> pcd_file_metadata_dict,
    PointcloudLoaderLogFunction on_error = PointcloudLoaderLogFunction{});

  [[nodiscard]] bool create_response(
    GetDifferentialPointCloudMap::Request::SharedPtr req,
    GetDifferentialPointCloudMap::Response::SharedPtr res) const;

private:
  std::map<std::string, PCDFileMetadata> all_pcd_file_metadata_dict_;
  PointcloudLoaderLogFunction on_error_;
};
}  // namespace autoware::map_loader

#endif  // POINTCLOUD_MAP_LOADER__DIFFERENTIAL_MAP_LOADER_HPP_
