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
#include <autoware/lanelet2_utils/kind.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <filesystem>
#include <iostream>
#include <string>

namespace fs = std::filesystem;

static lanelet::LaneletMapConstPtr set_up_lanelet_map_ptr()
{
  const auto sample_map_dir =
    fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
    "sample_map";
  const auto road_shoulder_highway_map_path =
    sample_map_dir / "vm_01_15-16/highway/lanelet2_map.osm";

  lanelet::LaneletMapConstPtr lanelet_map_ptr_ =
    autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
      road_shoulder_highway_map_path.string());

  return lanelet_map_ptr_;
}

int main()
{
  auto lanelet_map_ptr_ = set_up_lanelet_map_ptr();
  const auto ll = lanelet_map_ptr_->laneletLayer.get(46);
  bool check_road_lane = autoware::experimental::lanelet2_utils::is_road_lane(ll);
  bool check_shoulder_lane = autoware::experimental::lanelet2_utils::is_shoulder_lane(ll);
  bool check_bicycle_lane = autoware::experimental::lanelet2_utils::is_bicycle_lane(ll);

  if (check_road_lane) {
    std::cout << "This lanelet is road lane." << std::endl;
  } else {
    std::cout << "This lanelet is NOT road lane." << std::endl;
  }

  if (check_shoulder_lane) {
    std::cout << "This lanelet is shoulder lane." << std::endl;
  } else {
    std::cout << "This lanelet is NOT shoulder lane." << std::endl;
  }

  if (check_bicycle_lane) {
    std::cout << "This lanelet is bicycle lane." << std::endl;
  } else {
    std::cout << "This lanelet is NOT bicycle lane." << std::endl;
  }

  return 0;
}
