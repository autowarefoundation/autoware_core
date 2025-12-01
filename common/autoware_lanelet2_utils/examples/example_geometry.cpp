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
#include "autoware/lanelet2_utils/geometry.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

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

void extrapolation()
{
  lanelet::ConstPoint3d p1(1, 0.0, 0.0, 0.0);
  lanelet::ConstPoint3d p2(2, 10.0, 0.0, 0.0);
  double distance = 5.0;

  auto extrapolated_point = lanelet2_utils::extrapolate_point(p1, p2, distance);

  std::cout << "Extrapolated Point" << std::endl;
  std::cout << "x: " << extrapolated_point.x() << std::endl;
  std::cout << "y: " << extrapolated_point.y() << std::endl;
  std::cout << "z: " << extrapolated_point.z() << std::endl;
}

void interpolation()
{
  // Setup lanelet_map_ptr
  auto lanelet_map_ptr_ = set_up_lanelet_map_ptr();
  // Interpolate point
  {
    lanelet::ConstPoint3d p1(1, 1.0, 2.0, 3.0);
    lanelet::ConstPoint3d p2(2, 4.0, 5.0, 6.0);
    double half_of_segment_length = std::hypot(4.0 - 1.0, 5.0 - 2.0, 6.0 - 3.0) / 2;

    auto opt = lanelet2_utils::interpolate_point(p1, p2, half_of_segment_length);
    auto interpolated_pt = *opt;
    std::cout << "Interpolated Point (Half length)" << std::endl;
    std::cout << "x: " << interpolated_pt.x() << std::endl;
    std::cout << "y: " << interpolated_pt.y() << std::endl;
    std::cout << "z: " << interpolated_pt.z() << std::endl;
  }
  // Interpolate lanelet
  {
    const auto ll = lanelet_map_ptr_->laneletLayer.get(2287);
    auto opt_pt = lanelet2_utils::interpolate_lanelet(ll, 3.0);
    auto interpolated_pt = *opt_pt;
    std::cout << "Interpolated Point (From Lanelet)" << std::endl;
    std::cout << "x: " << interpolated_pt.x() << std::endl;
    std::cout << "y: " << interpolated_pt.y() << std::endl;
    std::cout << "z: " << interpolated_pt.z() << std::endl;
  }
  // Interpolate lanelet sequence
  {
    lanelet::ConstLanelets lanelets;
    lanelets.reserve(3);
    for (const auto & id : {2287, 2288, 2289}) {
      lanelets.push_back(lanelet_map_ptr_->laneletLayer.get(id));
    }
    auto opt_pt = lanelet2_utils::interpolate_lanelet_sequence(lanelets, 3.0);
    auto interpolated_pt = *opt_pt;
    std::cout << "Interpolated Point (From LaneletSequence)" << std::endl;
    std::cout << "x: " << interpolated_pt.x() << std::endl;
    std::cout << "y: " << interpolated_pt.y() << std::endl;
    std::cout << "z: " << interpolated_pt.z() << std::endl;
  }
}
void concatenation()
{
  // Setup lanelet_map_ptr
  auto lanelet_map_ptr_ = set_up_lanelet_map_ptr();

  lanelet::ConstLanelets lanelets;
  lanelets.reserve(3);
  for (auto id : {2287, 2288, 2289}) {
    lanelets.push_back(lanelet_map_ptr_->laneletLayer.get(id));
  }

  auto opt_ls = lanelet2_utils::concatenate_center_line(lanelets);

  const auto & ls = *opt_ls;

  std::cout << "Front of Concatenated centerline" << std::endl;
  std::cout << "x: " << ls.front().x() << std::endl;
  std::cout << "y: " << ls.front().y() << std::endl;
  std::cout << "z: " << ls.front().z() << std::endl;

  std::cout << "Back of Concatenated centerline" << std::endl;
  std::cout << "x: " << ls.back().x() << std::endl;
  std::cout << "y: " << ls.back().y() << std::endl;
  std::cout << "z: " << ls.back().z() << std::endl;

  bool no_duplicate_point = (std::adjacent_find(ls.begin(), ls.end()) == ls.end());
  if (no_duplicate_point) {
    std::cout << "There is no duplicate adjacent point." << std::endl;
  } else {
    std::cout << "There is duplicate adjacent point." << std::endl;
  }
}

void get_from_arc_length()
{
  // get_linestring_from_arc_length
  {
    std::vector<lanelet::Point3d> pts = {
      lanelet::Point3d{lanelet::ConstPoint3d(1, 0.0, 0.0, 0.0)},
      lanelet::Point3d{lanelet::ConstPoint3d(1, 1.0, 0.0, 0.0)},
      lanelet::Point3d{lanelet::ConstPoint3d(1, 1.7, 0.0, 0.0)},
      lanelet::Point3d{lanelet::ConstPoint3d(1, 2.0, 0.0, 0.0)}};
    lanelet::ConstLineString3d line{lanelet::InvalId, pts};
    auto opt =
      autoware::experimental::lanelet2_utils::get_linestring_from_arc_length(line, 0.5, 1.5);
    const auto & out = *opt;

    std::cout << "Get linestring from 0.5 to 1.5" << std::endl;
    std::cout << "The first point of linestring" << std::endl;
    std::cout << "x: " << out[0].x() << std::endl;
    std::cout << "The last point of linestring" << std::endl;
    std::cout << "x: " << out[2].x() << std::endl;
  }

  // get_pose_from_arc_length
  {
    auto lanelet_map_ptr_ = set_up_lanelet_map_ptr();
    lanelet::ConstLanelets lanelets;
    for (auto id : {2287, 2288, 2289}) {
      lanelets.push_back(lanelet_map_ptr_->laneletLayer.get(id));
    }
    auto opt_pose =
      autoware::experimental::lanelet2_utils::get_pose_from_2d_arc_length(lanelets, 3.0);
    const auto & p = *opt_pose;
    std::cout << "Pose from arc-length" << std::endl;
    std::cout << "x: " << p.position.x << std::endl;
    std::cout << "y: " << p.position.y << std::endl;
    std::cout << "z: " << p.position.z << std::endl;
    std::cout << "orientation x: " << p.orientation.x << std::endl;
    std::cout << "orientation y: " << p.orientation.y << std::endl;
    std::cout << "orientation z: " << p.orientation.z << std::endl;
    std::cout << "orientation w: " << p.orientation.w << std::endl;
  }
}

void closest_segment()
{
  std::vector<lanelet::Point3d> pts = {
    lanelet::Point3d{lanelet::ConstPoint3d(1, 0.0, 0.0, 0.0)},
    lanelet::Point3d{lanelet::ConstPoint3d(1, 1.0, 0.0, 0.0)},
    lanelet::Point3d{lanelet::ConstPoint3d(1, 2.0, 0.0, 0.0)}};
  lanelet::ConstLineString3d line{lanelet::InvalId, pts};

  lanelet::BasicPoint3d query(1.5, 0.0, 0.0);
  auto out = autoware::experimental::lanelet2_utils::get_closest_segment(line, query);

  lanelet::BasicPoint3d start_point = out.front().basicPoint();
  lanelet::BasicPoint3d end_point = out.back().basicPoint();

  std::cout << "Closest Segment" << std::endl;
  std::cout << "start point: " << start_point.x() << std::endl;
  std::cout << "end point: " << end_point.x() << std::endl;
}

void lanelet_angle()
{
  auto lanelet_map_ptr_ = set_up_lanelet_map_ptr();
  const auto ll = lanelet_map_ptr_->laneletLayer.get(2258);
  lanelet::BasicPoint3d p(106.71, 149.3, 100);
  auto out = autoware::experimental::lanelet2_utils::get_lanelet_angle(ll, p);
  std::cout << "Lanelet angle: " << out << std::endl;
}

void closest_center_pose()
{
  using autoware::experimental::lanelet2_utils::create_safe_lanelet;
  auto p1 = lanelet::BasicPoint3d(0.0, 0.0, 0.0);
  auto p2 = lanelet::BasicPoint3d(0.0, 3.0, 0.0);
  auto p3 = lanelet::BasicPoint3d(2.0, 0.0, 0.0);
  auto p4 = lanelet::BasicPoint3d(2.0, 3.0, 0.0);

  std::vector<lanelet::BasicPoint3d> left_points = {p1, p2};
  std::vector<lanelet::BasicPoint3d> right_points = {p3, p4};
  auto ll = create_safe_lanelet(left_points, right_points);

  auto search_pt = lanelet::BasicPoint3d(1.2, 1.0, 0.0);
  auto p = autoware::experimental::lanelet2_utils::get_closest_center_pose(*ll, search_pt);
  std::cout << "Pose from closest center line" << std::endl;
  std::cout << "x: " << p.position.x << std::endl;
  std::cout << "y: " << p.position.y << std::endl;
  std::cout << "z: " << p.position.z << std::endl;
  std::cout << "orientation x: " << p.orientation.x << std::endl;
  std::cout << "orientation y: " << p.orientation.y << std::endl;
  std::cout << "orientation z: " << p.orientation.z << std::endl;
  std::cout << "orientation w: " << p.orientation.w << std::endl;
}
}  // namespace autoware::experimental

int main()
{
  autoware::experimental::extrapolation();
  autoware::experimental::interpolation();
  autoware::experimental::concatenation();
  autoware::experimental::get_from_arc_length();
  autoware::experimental::closest_segment();
  autoware::experimental::lanelet_angle();
  autoware::experimental::closest_center_pose();
  return 0;
}
