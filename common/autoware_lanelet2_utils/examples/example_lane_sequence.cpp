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
#include <autoware/lanelet2_utils/lane_sequence.hpp>
#include <range/v3/all.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>

#include <filesystem>
#include <iostream>
#include <string>
#include <tuple>

namespace fs = std::filesystem;

static std::tuple<lanelet::LaneletMapConstPtr, lanelet::routing::RoutingGraphConstPtr>
set_up_lanelet_map_ptr()
{
  // setup path
  const auto sample_map_dir =
    fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
    "sample_map";
  const auto intersection_crossing_map_path = sample_map_dir / "vm_03/left_hand/lanelet2_map.osm";

  // load map
  lanelet::LaneletMapConstPtr lanelet_map_ptr_ =
    autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
      intersection_crossing_map_path.string());

  lanelet::routing::RoutingGraphConstPtr routing_graph_ptr_ =
    autoware::experimental::lanelet2_utils::instantiate_routing_graph_and_traffic_rules(
      lanelet_map_ptr_)
      .first;

  return std::make_tuple(lanelet_map_ptr_, routing_graph_ptr_);
}

namespace autoware::experimental
{
void lane_sequence_main()
{
  lanelet::LaneletMapConstPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphConstPtr routing_graph_ptr_;
  std::tie(lanelet_map_ptr_, routing_graph_ptr_) = set_up_lanelet_map_ptr();

  auto lane1 = lanelet_map_ptr_->laneletLayer.get(2245);  // lane1 -> lane2 is connected
  auto lane2 = lanelet_map_ptr_->laneletLayer.get(2261);  // lane2 -> lane_other is not connected
  auto lane_other = lanelet_map_ptr_->laneletLayer.get(2312);
  {
    const auto seq = lanelet2_utils::LaneSequence(lane1);
    std::cout << "LaneSequence size is: " << seq.as_lanelets().size() << std::endl;
  }
  {
    const auto opt =
      lanelet2_utils::LaneSequence::create({lane1, lane2, lane_other}, routing_graph_ptr_);
    std::cout << (opt.has_value()
                    ? "opt has value"
                    : "opt doesn't have value because lane_other is not connecting to lane2")
              << std::endl;
  }
  {
    const auto opt = lanelet2_utils::LaneSequence::create({lane1, lane2}, routing_graph_ptr_);
    const auto seq = *opt;
    std::cout << "Created LaneSequence size is: " << seq.as_lanelets().size() << std::endl;
  }
}
}  // namespace autoware::experimental

int main()
{
  autoware::experimental::lane_sequence_main();
  return 0;
}
