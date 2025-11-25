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
#include <autoware/lanelet2_utils/map_handler.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <filesystem>
#include <iostream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

namespace autoware::experimental
{
std::optional<lanelet2_utils::MapHandler> load_map_handler(
  const std::string & map_path = "vm_03/left_hand/lanelet2_map.osm")
{
  const auto sample_map_dir =
    fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
    "sample_map";
  const auto intersection_crossing_map_path = sample_map_dir / map_path;

  // load map
  const lanelet::LaneletMapConstPtr lanelet_map_ptr_ =
    autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
      intersection_crossing_map_path.string());

  // convert to laneletMapBin
  auto map_msg_ = lanelet2_utils::to_autoware_map_msgs(lanelet_map_ptr_);

  std::optional<lanelet2_utils::MapHandler> map_handler_opt_;
  map_handler_opt_.emplace(lanelet2_utils::MapHandler::create(map_msg_).value());
  return map_handler_opt_;
}

void map_handler_main()
{
  auto map_handler = load_map_handler().value();
  auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  // for left_lanelet
  {
    const auto opt = map_handler.left_lanelet(
      lanelet_map_ptr->laneletLayer.get(2257), false, lanelet2_utils::ExtraVRU::RoadOnly);
    std::cout << (opt.has_value() ? "There is left lane with Road Only"
                                  : "There is no left lane with Road Only")
              << std::endl;
  }

  {
    const auto lane = map_handler.left_lanelet(
      lanelet_map_ptr->laneletLayer.get(2257), false, lanelet2_utils::ExtraVRU::Shoulder);
    std::cout << "Shoulder lane id is " << (*lane).id() << std::endl;
  }

  {
    const auto opt = map_handler.left_lanelet(
      lanelet_map_ptr->laneletLayer.get(2257), false, lanelet2_utils::ExtraVRU::BicycleLane);
    std::cout << (opt.has_value() ? "There is left lane that is Bicycle Lane"
                                  : "There is no left lane that is Bicycle Lane")
              << std::endl;
  }

  {
    const auto opt = map_handler.left_lanelet(
      lanelet_map_ptr->laneletLayer.get(2257), false,
      lanelet2_utils::ExtraVRU::ShoulderAndBicycleLane);
    std::cout << (opt.has_value() ? "There is left lane that is Shoulder and Bicycle Lane"
                                  : "There is no left lane that is Shoulder and Bicycle Lane")
              << std::endl;
    auto lane = *opt;
    std::cout << "Shoulder and Bicycle lane id is " << lane.id() << std::endl;
  }

  // for right_lanelet
  {
    const auto opt = map_handler.right_lanelet(
      lanelet_map_ptr->laneletLayer.get(2245), false, lanelet2_utils::ExtraVRU::RoadOnly);
    auto lane = *opt;
    std::cout << "Right Road only lane id is " << lane.id() << std::endl;
  }

  // for leftmost_lanelet
  {
    const auto opt = map_handler.leftmost_lanelet(
      lanelet_map_ptr->laneletLayer.get(2288), false, lanelet2_utils::ExtraVRU::RoadOnly);
    auto lane = *opt;
    std::cout << "Leftmost Road only lane id is " << lane.id() << std::endl;
  }

  // for rightmost_lanelet
  {
    const auto opt = map_handler.rightmost_lanelet(
      lanelet_map_ptr->laneletLayer.get(2286), false, lanelet2_utils::ExtraVRU::RoadOnly);
    auto lane = *opt;
    std::cout << "Rightmost Road only lane id is " << lane.id() << std::endl;
  }

  // for left_lanelets
  {
    const auto lefts = map_handler.left_lanelets(lanelet_map_ptr->laneletLayer.get(2288));
    std::cout << "Left lanelets size is " << lefts.size() << std::endl;
    std::cout << "That has id" << std::endl;
    for (size_t i = 0ul; i < lefts.size(); ++i) {
      std::cout << lefts[i].id() << std::endl;
    }
  }

  // for right_lanelets with opposite
  {
    const auto rights = map_handler.right_lanelets(lanelet_map_ptr->laneletLayer.get(2286), true);
    std::cout << "Right lanelets size is " << rights.size() << std::endl;
    std::cout << "That has id" << std::endl;
    for (size_t i = 0ul; i < rights.size(); ++i) {
      std::cout << rights[i].id() << std::endl;
    }
  }

  // for get_shoulder_lanelet_sequence
  {
    auto map_handler002 = load_map_handler("vm_01_15-16/highway/lanelet2_map.osm").value();
    auto lanelet_map_ptr002 = map_handler002.lanelet_map_ptr();
    const auto seq =
      map_handler002.get_shoulder_lanelet_sequence(lanelet_map_ptr002->laneletLayer.get(48));
    std::cout << "lanelet sequence size is " << seq.size() << std::endl;
    std::cout << "That has id" << std::endl;
    for (size_t i = 0ul; i < seq.size(); ++i) {
      std::cout << seq.at(i).id() << std::endl;
    }
  }
}
}  // namespace autoware::experimental

int main()
{
  autoware::experimental::map_handler_main();
  return 0;
}
