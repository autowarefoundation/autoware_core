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
#include <autoware/lanelet2_utils/topology.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <filesystem>
#include <iostream>
#include <string>
#include <tuple>
#include <vector>

namespace fs = std::filesystem;

static std::tuple<lanelet::LaneletMapConstPtr, lanelet::routing::RoutingGraphConstPtr>
set_up_lanelet_map_ptr(const bool left = true)
{
  const auto sample_map_dir =
    fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
    "sample_map";
  const auto intersection_crossing_map_path =
    left ? sample_map_dir / "vm_03/left_hand/lanelet2_map.osm"
         : sample_map_dir / "vm_03/right_hand/lanelet2_map.osm";

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
void opposite_lanelet()
{
  // left opposite lanelet
  {
    lanelet::LaneletMapConstPtr lanelet_map_ptr_;
    lanelet::routing::RoutingGraphConstPtr routing_graph_ptr_;
    std::tie(lanelet_map_ptr_, routing_graph_ptr_) = set_up_lanelet_map_ptr(false);
    const auto opt = lanelet2_utils::left_opposite_lanelet(
      lanelet_map_ptr_->laneletLayer.get(2311), lanelet_map_ptr_);
    const auto lane = *opt;
    std::cout << "The left opposite lanelet id is : " << lane.id() << std::endl;
  }

  // right opposite lanelet
  {
    lanelet::LaneletMapConstPtr lanelet_map_ptr_;
    lanelet::routing::RoutingGraphConstPtr routing_graph_ptr_;
    std::tie(lanelet_map_ptr_, routing_graph_ptr_) = set_up_lanelet_map_ptr();
    const auto opt = lanelet2_utils::right_opposite_lanelet(
      lanelet_map_ptr_->laneletLayer.get(2288), lanelet_map_ptr_);
    const auto lane = *opt;
    std::cout << "The right opposite lanelet id is : " << lane.id() << std::endl;
  }
}
void related_lanelets()
{
  lanelet::LaneletMapConstPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphConstPtr routing_graph_ptr_;
  std::tie(lanelet_map_ptr_, routing_graph_ptr_) = set_up_lanelet_map_ptr();
  // following lanelets
  {
    const auto following = lanelet2_utils::following_lanelets(
      lanelet_map_ptr_->laneletLayer.get(2244), routing_graph_ptr_);
    std::cout << "The following lanelets id are :" << std::endl;
    for (const auto & ll : following) {
      std::cout << ll.id() << std::endl;
    }
  }
  // previous lanelets
  {
    const auto previous = lanelet2_utils::previous_lanelets(
      lanelet_map_ptr_->laneletLayer.get(2249), routing_graph_ptr_);
    std::cout << "The previous lanelets id are :" << std::endl;
    for (const auto & ll : previous) {
      std::cout << ll.id() << std::endl;
    }
  }
  // sibling lanelets
  {
    const auto siblings = lanelet2_utils::sibling_lanelets(
      lanelet_map_ptr_->laneletLayer.get(2273), routing_graph_ptr_);
    std::cout << "The sibling lanelets id are :" << std::endl;
    for (const auto & ll : siblings) {
      std::cout << ll.id() << std::endl;
    }
  }
  // conflicting lanelets
  {
    const auto conflicting_lanelets = lanelet2_utils::get_conflicting_lanelets(
      lanelet_map_ptr_->laneletLayer.get(2270), routing_graph_ptr_);
    std::cout << "The conflicting lanelets id are :" << std::endl;
    for (const auto & ll : conflicting_lanelets) {
      std::cout << ll.id() << std::endl;
    }
  }
  // from ids
  {
    const auto lanelets =
      lanelet2_utils::from_ids(lanelet_map_ptr_, std::vector<lanelet::Id>({2296, 2286, 2270}));
    std::cout << "Get ConstLanelets of size: " << lanelets.size() << std::endl;
  }
}

void neighbor_lanelet()
{
  lanelet::LaneletMapConstPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphConstPtr routing_graph_ptr_;
  std::tie(lanelet_map_ptr_, routing_graph_ptr_) = set_up_lanelet_map_ptr();

  // left_lanelet
  {
    const auto lane =
      lanelet2_utils::left_lanelet(lanelet_map_ptr_->laneletLayer.get(2246), routing_graph_ptr_);
    if (lane.has_value()) {
      std::cout << "Left lanelet Id is " << lane.value().id() << std::endl;
    }
  }

  // right_lanelet
  {
    const auto lane =
      lanelet2_utils::right_lanelet(lanelet_map_ptr_->laneletLayer.get(2246), routing_graph_ptr_);
    if (lane.has_value()) {
      std::cout << "Right lanelet Id is " << lane.value().id() << std::endl;
    }
  }

  // left_lanelets
  {
    const auto lanes =
      lanelet2_utils::left_lanelets(lanelet_map_ptr_->laneletLayer.get(2246), routing_graph_ptr_);
    if (lanes.has_value()) {
      std::cout << "There are " << lanes.value().size() << " left lanes." << std::endl;
      std::cout << "Those Ids are" << std::endl;
      for (auto lane : lanes.value()) {
        std::cout << lane.id() << std::endl;
      }
    }
  }

  // right_lanelets
  {
    const auto lanes =
      lanelet2_utils::right_lanelets(lanelet_map_ptr_->laneletLayer.get(2246), routing_graph_ptr_);
    if (lanes.has_value()) {
      std::cout << "There are " << lanes.value().size() << " right lanes." << std::endl;
      std::cout << "Those Ids are" << std::endl;
      for (auto lane : lanes.value()) {
        std::cout << lane.id() << std::endl;
      }
    }
  }

  // all_neighbor_lanelets
  {
    const auto lanes = lanelet2_utils::all_neighbor_lanelets(
      lanelet_map_ptr_->laneletLayer.get(2246), routing_graph_ptr_);
    std::cout << "There are " << lanes.size() << " neighbor lanelets." << std::endl;
    std::cout << "Those Ids are" << std::endl;
    for (auto lane : lanes) {
      std::cout << lane.id() << std::endl;
    }
  }

  // lane_changeable_neighbors
  {
    const auto lanes = lanelet2_utils::lane_changeable_neighbors(
      lanelet_map_ptr_->laneletLayer.get(2254), routing_graph_ptr_);
    std::cout << "There are " << lanes.size() << " lane changeable neighbor lanelets." << std::endl;
    std::cout << "Those Ids are" << std::endl;
    for (auto lane : lanes) {
      std::cout << lane.id() << std::endl;
    }
  }
}

void following_and_previous_lanelet_sequences()
{
  lanelet::LaneletMapConstPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphConstPtr routing_graph_ptr_;
  std::tie(lanelet_map_ptr_, routing_graph_ptr_) = set_up_lanelet_map_ptr();

  // succeeding_lanelet_sequences
  {
    double length = 50;
    const auto succeeding_lanelet_sequences = lanelet2_utils::get_succeeding_lanelet_sequences(
      lanelet_map_ptr_->laneletLayer.get(2244), routing_graph_ptr_, length);
    std::cout << "There are " << succeeding_lanelet_sequences.size()
              << " first set of succeeding lanelets." << std::endl;

    // check the first lanelet sequence
    std::cout << "The first lanelet sequence has " << succeeding_lanelet_sequences.front().size()
              << " lanelet(s) within range." << std::endl;
    std::cout << "Id(s) are " << succeeding_lanelet_sequences.front()[0].id() << ", "
              << succeeding_lanelet_sequences.front()[1].id() << "." << std::endl;

    // check the last lanelet sequence
    std::cout << "The last lanelet sequence has " << succeeding_lanelet_sequences.back().size()
              << " lanelet(s) within range." << std::endl;
    std::cout << "Id(s) is " << succeeding_lanelet_sequences.back()[0].id() << "." << std::endl;
  }

  // preceding_lanelet_sequences
  // closest->furthest order
  // 2265, 2244, 2301, 2239
  // 2270, *2286*, 2296
  // 2283
  {
    lanelet::ConstLanelets exclude_lanelets;
    // second lanelet in the second lanelet sequence
    exclude_lanelets.push_back(lanelet_map_ptr_->laneletLayer.get(2286));

    double length = 50;

    const auto preceding_lanelet_sequences = lanelet2_utils::get_preceding_lanelet_sequences(
      lanelet_map_ptr_->laneletLayer.get(2249), routing_graph_ptr_, length, exclude_lanelets);
    std::cout << "There are " << preceding_lanelet_sequences.size()
              << " first set of preceding lanelets." << std::endl;

    // check the first lanelet sequence
    std::cout << "The first lanelet sequence has " << preceding_lanelet_sequences.front().size()
              << " lanelet(s) within range." << std::endl;
    std::cout << "Id(s) are " << preceding_lanelet_sequences.front()[0].id() << ", "
              << preceding_lanelet_sequences.front()[1].id() << "." << std::endl;

    // check the second lanelet sequence
    std::cout << "The second lanelet sequence has " << preceding_lanelet_sequences[1].size()
              << " lanelet(s) within range." << std::endl;
    std::cout << "Id(s) is " << preceding_lanelet_sequences[1][0].id() << "." << std::endl;

    // check the last lanelet sequence
    std::cout << "The last lanelet sequence has " << preceding_lanelet_sequences.back().size()
              << " lanelet(s) within range." << std::endl;
    std::cout << "Id(s) is " << preceding_lanelet_sequences.back()[0].id() << "." << std::endl;
  }
}
}  // namespace autoware::experimental

int main()
{
  autoware::experimental::opposite_lanelet();
  autoware::experimental::related_lanelets();
  autoware::experimental::neighbor_lanelet();
  autoware::experimental::following_and_previous_lanelet_sequences();

  return 0;
}
