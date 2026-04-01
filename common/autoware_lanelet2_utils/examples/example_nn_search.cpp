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
#include <autoware/lanelet2_utils/nn_search.hpp>
#include <range/v3/all.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <filesystem>
#include <iostream>
#include <string>
#include <tuple>

namespace fs = std::filesystem;

static lanelet::LaneletMapConstPtr load_lanelet_map()
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

static std::tuple<
  lanelet::ConstLanelets, std::optional<autoware::experimental::lanelet2_utils::LaneletRTree>>
load_lanelets(const lanelet::LaneletMapConstPtr & lanelet_map_ptr_)
{
  lanelet::ConstLanelets all_lanelets_ = lanelet_map_ptr_->laneletLayer | ranges::to<std::vector>();
  std::optional<autoware::experimental::lanelet2_utils::LaneletRTree> rtree_{all_lanelets_};
  return std::make_tuple(all_lanelets_, rtree_);
}

namespace autoware::experimental
{
void nn_search_main()
{
  auto lanelet_map_ptr_ = load_lanelet_map();

  lanelet::ConstLanelets all_lanelets_;
  std::optional<lanelet2_utils::LaneletRTree> rtree_;
  std::tie(all_lanelets_, rtree_) = load_lanelets(lanelet_map_ptr_);

  // Value from test_nn_search_001.yaml
  geometry_msgs::msg::Pose P0;
  P0.position.x = 120.14562274983756;
  P0.position.y = 214.41436149252192;
  P0.position.z = 100.0;
  P0.orientation.x = 0.0;
  P0.orientation.y = 0.0;
  P0.orientation.z = -0.6933854908702647;
  P0.orientation.w = 0.7205668331602573;

  // get_closest_lanelet (without rtree)
  {
    auto opt = lanelet2_utils::get_closest_lanelet(all_lanelets_, P0);
    auto closest = *opt;
    std::cout << "Closest Lanelet id is: " << closest.id() << std::endl;
  }
  // get_closest_lanelet (using rtree)
  {
    auto opt = rtree_->get_closest_lanelet(P0);
    auto closest = *opt;
    std::cout << "Closest Lanelet id is: " << closest.id() << std::endl;
  }

  static constexpr double ego_nearest_dist_threshold = 3.0;
  static constexpr double ego_nearest_yaw_threshold = 1.046;

  // get_closest_lanelet_within_constraint (without rtree)
  {
    auto opt = lanelet2_utils::get_closest_lanelet_within_constraint(
      all_lanelets_, P0, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
    auto closest = *opt;
    std::cout << "Closest Lanelet id is: " << closest.id() << std::endl;
  }
  // get_closest_lanelet_within_constraint (using rtree)
  {
    auto opt = rtree_->get_closest_lanelet_within_constraint(
      P0, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
    auto closest = *opt;
    std::cout << "Closest Lanelet id is: " << closest.id() << std::endl;
  }

  // get_road_lanelets_at
  {
    geometry_msgs::msg::Point p;
    p.x = 137.53617965826874;
    p.y = 189.8355248506006;
    const auto road_lanelets = lanelet2_utils::get_road_lanelets_at(lanelet_map_ptr_, p.x, p.y);
    std::cout << "There are " << road_lanelets.size() << " road lanelet(s) in this LaneletMap."
              << std::endl;
  }

  // get_shoulder_lanelets_at
  {
    geometry_msgs::msg::Point p;
    p.x = 101.11975409926032;
    p.y = 143.75759863307977;
    const auto shoulder_lanelets =
      lanelet2_utils::get_shoulder_lanelets_at(lanelet_map_ptr_, p.x, p.y);
    std::cout << "There are " << shoulder_lanelets.size()
              << " shoulder lanelet(s) in this LaneletMap." << std::endl;
  }
}
}  // namespace autoware::experimental

int main()
{
  autoware::experimental::nn_search_main();
  return 0;
}
