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
#include <autoware/lanelet2_utils/route_manager.hpp>

#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <filesystem>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

static autoware_planning_msgs::msg::LaneletRoute create_route_msg(
  const std::vector<std::pair<std::vector<lanelet::Id>, lanelet::Id>> & route_ids)
{
  using autoware_planning_msgs::msg::LaneletPrimitive;
  using autoware_planning_msgs::msg::LaneletSegment;
  autoware_planning_msgs::msg::LaneletRoute route_msg;
  for (const auto & route_id : route_ids) {
    const auto & [ids, preferred_id] = route_id;
    LaneletSegment segment;
    segment.preferred_primitive.id = preferred_id;
    for (const auto & id : ids) {
      auto primitive =
        autoware_planning_msgs::build<LaneletPrimitive>().id(id).primitive_type("road");
      segment.primitives.push_back(primitive);
    }
    route_msg.segments.push_back(segment);
  }
  return route_msg;
}

namespace autoware::experimental
{
autoware_map_msgs::msg::LaneletMapBin load_map_msg()
{
  const auto sample_map_dir =
    fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
    "sample_map";
  const auto intersection_crossing_map_path = sample_map_dir / "vm_03/left_hand/lanelet2_map.osm";

  // load map
  const lanelet::LaneletMapConstPtr lanelet_map_ptr_ =
    lanelet2_utils::load_mgrs_coordinate_map(intersection_crossing_map_path.string());
  // convert to msg
  auto map_msg = lanelet2_utils::to_autoware_map_msgs(lanelet_map_ptr_);
  return map_msg;
}

void route_manager_main()
{
  autoware_planning_msgs::msg::LaneletRoute route_msg_ = create_route_msg(
    {{{2239, 2240, 2241, 2242}, 2240},
     {{2301, 2302, 2300, 2299}, 2302},
     {{2244, 2245, 2246, 2247}, 2245},
     {{2265, 2261, 2262, 2263}, 2261}});
  autoware_map_msgs::msg::LaneletMapBin map_msg_ = load_map_msg();

  geometry_msgs::msg::Pose initial_pose_;
  initial_pose_.position.x = 122.81823934652175;
  initial_pose_.position.y = 264.7931657588342;
  initial_pose_.position.z = 100.0;

  initial_pose_.orientation.w = 0.7216753233664642;
  initial_pose_.orientation.x = 0.0;
  initial_pose_.orientation.y = 0.0;
  initial_pose_.orientation.z = -0.6922317008371615;

  // create
  {
    const auto route_manager_opt =
      lanelet2_utils::RouteManager::create(map_msg_, route_msg_, initial_pose_);
    std::cout << (route_manager_opt.has_value() ? "Route Manager created"
                                                : "Route manager create failed.")
              << std::endl;

    // get current_lanelet
    const auto & route_manager = route_manager_opt.value();
    const auto initial_lanelet = route_manager.current_lanelet();
    std::cout << "Current lanelet (initial) id is " << initial_lanelet.id() << std::endl;
    // or directly get from opt
    const auto initial_lanelet_from_opt = route_manager_opt->current_lanelet();
    std::cout << "Current lanelet (initial) id from opt is " << initial_lanelet_from_opt.id()
              << std::endl;
  }

  // update lanelet
  {
    auto route_manager_opt =
      lanelet2_utils::RouteManager::create(map_msg_, route_msg_, initial_pose_);

    geometry_msgs::msg::Pose P1;
    P1.position.x = 123.00734123632856;
    P1.position.y = 244.8744333658504;
    P1.position.z = 100.0;

    P1.orientation.w = 0.724127942325602;
    P1.orientation.x = 0.0;
    P1.orientation.y = 0.0;
    P1.orientation.z = -0.689665660406033;
    static constexpr double ego_nearest_dist_threshold = 3.0;
    static constexpr double ego_nearest_yaw_threshold = 1.046;
    {
      route_manager_opt =
        std::move(route_manager_opt.value())
          .update_current_pose(P1, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);

      const auto & route_manager = route_manager_opt.value();
      const auto & current_lanelet = route_manager.current_lanelet();
      std::cout << "Current lanelet (moved) id is " << current_lanelet.id() << std::endl;
    }

    geometry_msgs::msg::Pose P5;
    P5.position.x = 121.24239026479836;
    P5.position.y = 224.26232737690827;
    P5.position.z = 100.0;

    P5.orientation.w = 0.7307612086507389;
    P5.orientation.x = 0.0;
    P5.orientation.y = 0.0;
    P5.orientation.z = -0.6826331781647528;

    {
      route_manager_opt = std::move(route_manager_opt.value()).commit_lane_change_success(P5);
      const auto & route_manager = route_manager_opt.value();
      const auto & lane_changed_lanelet = route_manager.current_lanelet();
      std::cout << "Current lanelet (lane changed) id is " << lane_changed_lanelet.id()
                << std::endl;
    }
  }

  // reset route_manager_opt
  const auto route_manager_opt =
    lanelet2_utils::RouteManager::create(map_msg_, route_msg_, initial_pose_);
  const auto & route_manager = route_manager_opt.value();

  // get lanelet_sequence
  {
    {
      const auto seq = route_manager.get_lanelet_sequence_on_route(0.0, 15.0);
      std::cout << "Size of the lanelet sequence is " << seq.as_lanelets().size() << std::endl;
      std::cout << "Including id " << std::endl;
      for (auto i = 0ul; i < seq.as_lanelets().size(); ++i) {
        std::cout << seq.as_lanelets().at(i).id() << std::endl;
      }
    }
    {
      const auto seq = route_manager.get_lanelet_sequence_outward_route(0.0, 15.0);
      std::cout << "Size of the outward lanelet sequence is " << seq.as_lanelets().size()
                << std::endl;
      std::cout << "Including id " << std::endl;
      for (auto i = 0ul; i < seq.as_lanelets().size(); ++i) {
        std::cout << seq.as_lanelets().at(i).id() << std::endl;
      }
    }
  }
  {
    {
      const auto lane_opt = route_manager.get_closest_preferred_route_lanelet(initial_pose_);
      auto lane = *lane_opt;
      std::cout << "Closest preferred route lanelet id is " << lane.id() << std::endl;
    }

    {
      static constexpr double ego_nearest_dist_threshold = 3.0;
      static constexpr double ego_nearest_yaw_threshold = 1.046;
      const auto lane_opt = route_manager.get_closest_route_lanelet_within_constraints(
        initial_pose_, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
      auto lane = *lane_opt;
      std::cout << "Closest route lanelet (with constraint) id is " << lane.id() << std::endl;
    }
  }
}
}  // namespace autoware::experimental

int main()
{
  autoware::experimental::route_manager_main();
  return 0;
}
