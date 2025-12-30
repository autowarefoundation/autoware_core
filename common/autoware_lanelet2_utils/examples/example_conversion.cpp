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

#include "autoware/lanelet2_utils/conversion.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>

#include <filesystem>
#include <iostream>
#include <string>
#include <typeinfo>
#include <vector>

namespace fs = std::filesystem;

namespace autoware::experimental
{
void load_mgrs()
{
  // setup path
  const auto sample_map_dir =
    fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
    "sample_map";
  const auto intersection_crossing_map_path = sample_map_dir / "vm_03/left_hand/lanelet2_map.osm";

  // load map
  lanelet::LaneletMapConstPtr lanelet_map_ptr_ =
    lanelet2_utils::load_mgrs_coordinate_map(intersection_crossing_map_path.string(), 5.0);

  std::cout << "LaneletMapConstPtr Loaded!" << std::endl;

  // load routing graph and traffic rules
  lanelet::routing::RoutingGraphConstPtr routing_graph_{nullptr};
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_{nullptr};
  std::tie(routing_graph_, traffic_rules_) =
    lanelet2_utils::instantiate_routing_graph_and_traffic_rules(lanelet_map_ptr_);

  // Or get only routing graph or traffic rules
  routing_graph_ =
    lanelet2_utils::instantiate_routing_graph_and_traffic_rules(lanelet_map_ptr_).first;

  traffic_rules_ =
    lanelet2_utils::instantiate_routing_graph_and_traffic_rules(lanelet_map_ptr_).second;

  std::cout << "RoutingGraph and TrafficRules Loaded!" << std::endl;

  // convert to LaneletMapBin
  autoware_map_msgs::msg::LaneletMapBin map_msg_ =
    lanelet2_utils::to_autoware_map_msgs(lanelet_map_ptr_);
  std::cout << "Convert LaneletMapConstPtr to LaneletMapBin!" << std::endl;

  // convert to LaneletMapConstPtr
  lanelet::LaneletMapConstPtr lanelet_map_ptr_converted =
    lanelet2_utils::from_autoware_map_msgs(map_msg_);
  std::cout << "Convert LaneletMapBin to LaneletMapConstPtr !" << std::endl;
}

void msg_conversion()
{
  // BasicPoint3d -> Point
  {
    auto basicpoint3d = lanelet::BasicPoint3d(1.0, 2.0, 3.0);

    auto point_from_basicpoint3d = lanelet2_utils::to_ros(basicpoint3d);
    std::cout << "Convert from lanelet::BasicPoint3d to geometry_msgs::msg::Point." << std::endl;
    std::cout << "For avoid unused variable error, x: " << point_from_basicpoint3d.x << std::endl;
  }

  // ConstPoint3d <-> Point
  {
    auto constpoint3d = lanelet::ConstPoint3d(lanelet::Point3d(lanelet::InvalId, 1.0, 2.0, 3.0));

    auto point_from_constpoint3d = lanelet2_utils::to_ros(constpoint3d);
    std::cout << "Convert from lanelet::ConstPoint3d to geometry_msgs::msg::Point." << std::endl;

    auto converted_constpoint3d_from_point = lanelet2_utils::from_ros(point_from_constpoint3d);
    std::cout << "Convert from geometry_msgs::msg::Point to lanelet::ConstPoint3d." << std::endl;
  }

  // Pose -> ConstPoint3d
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = 1.0;
    pose.position.y = 2.0;
    pose.position.z = 3.0;

    auto const_pt = lanelet2_utils::from_ros(pose);
    std::cout << "Convert from geometry_msgs::msg::Pose to lanelet::ConstPoint3d." << std::endl;
  }

  // BasicPoint2d -> Point
  {
    auto basicpoint2d = lanelet::BasicPoint2d(1.0, 2.0);

    auto point_from_basicpoint2d = lanelet2_utils::to_ros(basicpoint2d, 3.0);
    std::cout << "Convert from lanelet::BasicPoint2d to geometry_msgs::msg::Point." << std::endl;
    std::cout << "For avoid unused variable error, x: " << point_from_basicpoint2d.x << std::endl;
  }

  // ConstPoint2d <-> Point
  {
    auto constpoint2d = lanelet::ConstPoint2d(lanelet::Point2d(lanelet::InvalId, 1.0, 2.0));

    auto point_from_constpoint2d = lanelet2_utils::to_ros(constpoint2d, 3.0);
    std::cout << "Convert from lanelet::ConstPoint2d to geometry_msgs::msg::Point." << std::endl;

    auto constpoint3d_from_point = lanelet2_utils::from_ros(point_from_constpoint2d);
    std::cout << "Convert from geometry_msgs::msg::Point to lanelet::ConstPoint3d." << std::endl;

    auto constpoint2d_from_point = lanelet::utils::to2D(constpoint3d_from_point);
    std::cout << "Convert from  lanelet::ConstPoint3d to lanelet::ConstPoint2d." << std::endl;
  }
}

void create_safe_object()
{
  // construct BasicLinestring3d from BasicPoints3d.
  {
    auto p1 = lanelet::BasicPoint3d(1.0, 1.0, 1.0);
    auto p2 = lanelet::BasicPoint3d(2.0, 2.0, 2.0);
    auto p3 = lanelet::BasicPoint3d(3.0, 3.0, 3.0);
    std::vector<lanelet::BasicPoint3d> vector_points = {p1, p2, p3};
    auto opt = autoware::experimental::lanelet2_utils::create_safe_linestring(vector_points);
    auto basic_linestring = *opt;
  }

  // construct ConstLinestring3d from ConstPoints3d.
  {
    auto p1 = lanelet::ConstPoint3d(lanelet::Point3d(lanelet::InvalId, 1.0, 1.0, 1.0));
    auto p2 = lanelet::ConstPoint3d(lanelet::Point3d(lanelet::InvalId, 2.0, 2.0, 2.0));
    auto p3 = lanelet::ConstPoint3d(lanelet::Point3d(lanelet::InvalId, 3.0, 3.0, 3.0));
    std::vector<lanelet::ConstPoint3d> vector_points = {p1, p2, p3};
    auto opt = autoware::experimental::lanelet2_utils::create_safe_linestring(vector_points);
    auto const_linestring = *opt;
    std::cout << "Construct ConstLinestring3d from ConstPoints3d." << std::endl;
  }

  // construct ConstLanelet using two BasicPoints3d
  {
    auto p1 = lanelet::BasicPoint3d(1.0, 1.0, 1.0);
    auto p2 = lanelet::BasicPoint3d(2.0, 2.0, 2.0);
    auto p3 = lanelet::BasicPoint3d(3.0, 3.0, 3.0);
    auto p4 = lanelet::BasicPoint3d(4.0, 4.0, 4.0);
    std::vector<lanelet::BasicPoint3d> left_points = {p1, p2};
    std::vector<lanelet::BasicPoint3d> right_points = {p3, p4};

    const auto opt =
      autoware::experimental::lanelet2_utils::create_safe_lanelet(left_points, right_points);
    auto lanelet = *opt;
    std::cout << "Construct ConstLanelet from BasicPoints3d." << std::endl;
  }

  // construct ConstLanelet using two ConstPoints3d
  {
    // BasicPoint3d
    auto p1 = lanelet::BasicPoint3d(1.0, 1.0, 1.0);
    auto p2 = lanelet::BasicPoint3d(2.0, 2.0, 2.0);
    auto p3 = lanelet::BasicPoint3d(3.0, 3.0, 3.0);
    auto p4 = lanelet::BasicPoint3d(4.0, 4.0, 4.0);
    std::vector<lanelet::BasicPoint3d> left_points = {p1, p2};
    std::vector<lanelet::BasicPoint3d> right_points = {p3, p4};
    const auto opt =
      autoware::experimental::lanelet2_utils::create_safe_lanelet(left_points, right_points);
    auto lanelet = *opt;
    std::cout << "Construct ConstLanelet from ConstPoints3d." << std::endl;
  }
}
}  // namespace autoware::experimental

int main()
{
  autoware::experimental::load_mgrs();
  autoware::experimental::msg_conversion();
  autoware::experimental::create_safe_object();
  return 0;
}
