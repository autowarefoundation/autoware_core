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
#include <autoware/lanelet2_utils/intersection.hpp>

#include <gtest/gtest.h>
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
  const auto intersection_crossing_map_path = sample_map_dir / "vm_03/left_hand/lanelet2_map.osm";

  lanelet::LaneletMapConstPtr lanelet_map_ptr_ =
    autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
      intersection_crossing_map_path.string());

  return lanelet_map_ptr_;
}

namespace autoware::experimental
{
void intersection_main()
{
  auto lanelet_map_ptr_ = set_up_lanelet_map_ptr();
  {
    bool check = lanelet2_utils::is_intersection_lanelet(lanelet_map_ptr_->laneletLayer.get(2274));
    std::cout << (check ? "This lanelet is intersection!" : "This lanelet is not intersection.")
              << std::endl;
  }
  {
    bool check = lanelet2_utils::is_straight_direction(lanelet_map_ptr_->laneletLayer.get(2278));
    std::cout << (check ? "This lanelet is straight direction!"
                        : "This lanelet is not straight direction.")
              << std::endl;
  }
  {
    bool check = lanelet2_utils::is_left_direction(lanelet_map_ptr_->laneletLayer.get(2274));
    std::cout << (check ? "This lanelet is left direction!" : "This lanelet is not left direction.")
              << std::endl;
  }
  {
    bool check = lanelet2_utils::is_right_direction(lanelet_map_ptr_->laneletLayer.get(2277));
    std::cout << (check ? "This lanelet is right direction!"
                        : "This lanelet is not right direction.")
              << std::endl;
  }

  // get_turn_direction
  {
    {
      // not intersection
      auto opt = lanelet2_utils::get_turn_direction(lanelet_map_ptr_->laneletLayer.get(2257));
      bool check = opt.has_value();
      std::cout << (check ? "This lanelet has turn_direction attribute!"
                          : "This lanelet has no turn_direction attribute.")
                << std::endl;
    }
    // straight
    {
      const auto opt = lanelet2_utils::get_turn_direction(lanelet_map_ptr_->laneletLayer.get(2278));
      if (opt.has_value()) {
        auto direction = *opt;
        std::cout << "Straight: " << direction << std::endl;
      }
    }

    // left
    {
      const auto opt = lanelet2_utils::get_turn_direction(lanelet_map_ptr_->laneletLayer.get(2274));
      if (opt.has_value()) {
        auto direction = *opt;
        std::cout << "Left: " << direction << std::endl;
      }
    }

    // right
    {
      const auto opt = lanelet2_utils::get_turn_direction(lanelet_map_ptr_->laneletLayer.get(2277));
      if (opt.has_value()) {
        auto direction = *opt;
        std::cout << "Right: " << direction << std::endl;
      }
    }
  }
}
}  // namespace autoware::experimental

int main()
{
  autoware::experimental::intersection_main();
  return 0;
}
