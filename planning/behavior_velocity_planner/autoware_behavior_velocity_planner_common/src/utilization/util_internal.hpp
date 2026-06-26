// Copyright 2026 TIER IV, Inc.
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

#ifndef UTILIZATION__UTIL_INTERNAL_HPP_
#define UTILIZATION__UTIL_INTERNAL_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner::planning_utils
{

/**
 * @brief Format regulatory-element, lanelet and line ids into a human-readable id tag.
 *
 * Each non-empty id group is rendered as "[<prefix>: id0, id1, ...]" and the groups are
 * concatenated in the order reg / lane / line. Empty groups are skipped entirely.
 *
 * This is an internal helper shared by the stable and experimental SceneModuleInterface
 * translation units; it is intentionally not part of the installed public API.
 *
 * @param regulatory_element_ids ids rendered under the "Reg" prefix
 * @param lanelet_ids ids rendered under the "Lane" prefix
 * @param line_ids ids rendered under the "Line" prefix
 * @return concatenated id tag (empty string when all groups are empty)
 */
std::string formatIds(
  const std::vector<int64_t> & regulatory_element_ids, const std::vector<int64_t> & lanelet_ids,
  const std::vector<int64_t> & line_ids);

}  // namespace autoware::behavior_velocity_planner::planning_utils

#endif  // UTILIZATION__UTIL_INTERNAL_HPP_
