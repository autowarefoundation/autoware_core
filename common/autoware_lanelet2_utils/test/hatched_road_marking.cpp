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

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <filesystem>
#include <string>
#include <unordered_set>

namespace fs = std::filesystem;

namespace autoware::experimental
{
TEST(HatchedRoadMarking, AdjacentPolygons)
{
  // Locate sample map distributed with this package
  const auto sample_map_dir =
    fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
    "sample_map";
  const auto map_path = sample_map_dir / "hatched_road_marking" / "lanelet2_map.osm";

  const auto lanelet_map_ptr = lanelet2_utils::load_mgrs_coordinate_map(map_path.string());
  ASSERT_TRUE(static_cast<bool>(lanelet_map_ptr));

  // Collect several lanelets known to be adjacent to hatched road markings
  lanelet::ConstLanelets lanelets;
  lanelets.emplace_back(lanelet_map_ptr->laneletLayer.get(47));  // relation id 47 in OSM
  lanelets.emplace_back(lanelet_map_ptr->laneletLayer.get(45));  // relation id 45
  lanelets.emplace_back(lanelet_map_ptr->laneletLayer.get(46));  // relation id 46

  const auto adjacent =
    lanelet2_utils::get_adjacent_hatched_road_markings(lanelets, lanelet_map_ptr);

  // Collect unique polygon IDs from the result.
  std::unordered_set<lanelet::Id> left_ids;
  std::unordered_set<lanelet::Id> right_ids;

  for (const auto & poly : adjacent.left) {
    left_ids.insert(poly.id());
  }
  for (const auto & poly : adjacent.right) {
    right_ids.insert(poly.id());
  }

  // Expected IDs from the sample map.
  EXPECT_EQ(left_ids.count(43), 1U);
  EXPECT_EQ(left_ids.count(44), 1U);
  EXPECT_EQ(left_ids.size(), 2U);

  EXPECT_EQ(right_ids.count(42), 1U);
  EXPECT_EQ(right_ids.size(), 1U);
}
}  // namespace autoware::experimental

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
