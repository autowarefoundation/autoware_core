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

#include "partial_map_loader.hpp"

#include <map>
#include <string>
#include <vector>

namespace autoware::map_loader
{
std::vector<std::string> collect_partial_map_ids(
  const autoware_map_msgs::msg::AreaInfo & area,
  const std::map<std::string, PCDFileMetadata> & pcd_file_metadata_dict)
{
  std::vector<std::string> map_ids_to_load;
  for (const auto & ele : pcd_file_metadata_dict) {
    const std::string & map_id = ele.first;
    const PCDFileMetadata & metadata = ele.second;

    if (!is_grid_within_queried_area(area, metadata)) {
      continue;
    }
    map_ids_to_load.push_back(map_id);
  }

  return map_ids_to_load;
}
}  // namespace autoware::map_loader
