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

#include "pointcloud_map_loader_module.hpp"

#include "pointcloud_map_loader.hpp"

#include <string>
#include <vector>

namespace autoware::map_loader
{
LoadPointcloudMapResult PointcloudMapLoaderModule::create_map_message(
  const std::vector<std::string> & pcd_paths, boost::optional<float> leaf_size) const
{
  return load_pointcloud_map(pcd_paths, leaf_size);
}
}  // namespace autoware::map_loader
