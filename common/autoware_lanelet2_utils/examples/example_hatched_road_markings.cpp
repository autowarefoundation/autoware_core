// Copyright 2025 TIER IV, Inc.
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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/hatched_road_markings.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <filesystem>
#include <iostream>
#include <string>
#include <unordered_set>

namespace fs = std::filesystem;

static lanelet::LaneletMapConstPtr set_up_lanelet_map_ptr()
{
  const auto sample_map_dir =
    fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
    "sample_map";
  const auto map_path = sample_map_dir / "vm_06_01/lanelet2_map.osm";

  lanelet::LaneletMapConstPtr lanelet_map_ptr_ =
    autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(map_path.string());

  return lanelet_map_ptr_;
}

int main()
{
  auto lanelet_map_ptr_ = set_up_lanelet_map_ptr();
  // Collect several lanelets known to be adjacent to hatched road markings
  lanelet::ConstLanelets lanelets;
  for (auto id : {47, 45, 46}) {
    lanelets.emplace_back(lanelet_map_ptr_->laneletLayer.get(id));
  }

  const auto adjacent = autoware::experimental::lanelet2_utils::get_adjacent_hatched_road_markings(
    lanelets, lanelet_map_ptr_);

  // Collect unique polygon IDs from the result.
  std::unordered_set<lanelet::Id> left_ids;
  std::unordered_set<lanelet::Id> right_ids;

  for (const auto & poly : adjacent.left) {
    left_ids.insert(poly.id());
  }
  for (const auto & poly : adjacent.right) {
    right_ids.insert(poly.id());
  }

  std::cout << "Left ID size is: " << left_ids.size() << std::endl;
  std::cout << "Left ID is: " << std::endl;
  for (const auto & left : left_ids) {
    std::cout << left << std::endl;
  }

  std::cout << "Right ID size is: " << right_ids.size() << std::endl;
  std::cout << "Right ID is: " << std::endl;
  for (const auto & right : right_ids) {
    std::cout << right << std::endl;
  }
  return 0;
}
