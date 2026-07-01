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

#include "differential_map_loader.hpp"

#include <algorithm>
#include <map>
#include <string>
#include <vector>

namespace autoware::map_loader
{
DifferentialMapLoadPlan create_differential_map_load_plan(
  const autoware_map_msgs::msg::AreaInfo & area_info, const std::vector<std::string> & cached_ids,
  const std::map<std::string, PCDFileMetadata> & pcd_file_metadata_dict)
{
  DifferentialMapLoadPlan plan;

  std::vector<bool> should_remove(static_cast<int>(cached_ids.size()), true);
  for (const auto & ele : pcd_file_metadata_dict) {
    const std::string & path = ele.first;
    const PCDFileMetadata & metadata = ele.second;

    // Assume that the map ID = map path (for now).
    const std::string & map_id = path;

    if (!is_grid_within_queried_area(area_info, metadata)) {
      continue;
    }

    const auto id_in_cached_list = std::find(cached_ids.begin(), cached_ids.end(), map_id);
    if (id_in_cached_list != cached_ids.end()) {
      const auto index = static_cast<int>(id_in_cached_list - cached_ids.begin());
      should_remove[index] = false;
    } else {
      plan.map_ids_to_load.push_back(map_id);
    }
  }

  for (size_t i = 0; i < cached_ids.size(); ++i) {
    if (should_remove[i]) {
      plan.ids_to_remove.push_back(cached_ids[i]);
    }
  }

  return plan;
}
}  // namespace autoware::map_loader
