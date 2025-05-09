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

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>

#include <filesystem>
#include <string>

namespace fs = std::filesystem;

namespace autoware::experimental
{
class TestWithRoadShoulderHighwayMap : public ::testing::Test
{
protected:
  lanelet::LaneletMapConstPtr lanelet_map_ptr_{nullptr};

  void SetUp() override
  {
    const auto sample_map_dir =
      fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
      "sample_map";
    const auto road_shoulder_highway_map_path = sample_map_dir / "road_shoulder" / "highway.osm";

    lanelet_map_ptr_ =
      lanelet2_utils::load_mgrs_coordinate_map(road_shoulder_highway_map_path.string());
  }
};

class TestWithIntersectionCrossingMap : public ::testing::Test
{
protected:
  lanelet::LaneletMapConstPtr lanelet_map_ptr_{nullptr};

  void SetUp() override
  {
    const auto sample_map_dir =
      fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
      "sample_map";
    const auto road_shoulder_highway_map_path = sample_map_dir / "intersection" / "crossing.osm";

    lanelet_map_ptr_ =
      lanelet2_utils::load_mgrs_coordinate_map(road_shoulder_highway_map_path.string());
  }
};

TEST_F(TestWithRoadShoulderHighwayMap, LoadCheck)
{
  const auto point = lanelet_map_ptr_->pointLayer.get(1);
  EXPECT_NEAR(point.x(), 100.0, 0.05);
  EXPECT_NEAR(point.y(), 100.0, 0.05);
}

TEST_F(TestWithRoadShoulderHighwayMap, is_road_lane)
{
  const auto ll = lanelet_map_ptr_->laneletLayer.get(46);
  EXPECT_EQ(lanelet2_utils::is_road_lane(ll), true);
  EXPECT_EQ(lanelet2_utils::is_shoulder_lane(ll), false);
  EXPECT_EQ(lanelet2_utils::is_bicycle_lane(ll), false);
}

TEST_F(TestWithRoadShoulderHighwayMap, is_shoulder_lane)
{
  const auto ll = lanelet_map_ptr_->laneletLayer.get(47);
  EXPECT_EQ(lanelet2_utils::is_road_lane(ll), false);
  EXPECT_EQ(lanelet2_utils::is_shoulder_lane(ll), true);
  EXPECT_EQ(lanelet2_utils::is_bicycle_lane(ll), false);
}

TEST_F(TestWithIntersectionCrossingMap, is_shoulder_lane)
{
  const auto ll = lanelet_map_ptr_->laneletLayer.get(2303);
  EXPECT_EQ(lanelet2_utils::is_road_lane(ll), false);
  EXPECT_EQ(lanelet2_utils::is_shoulder_lane(ll), false);
  EXPECT_EQ(lanelet2_utils::is_bicycle_lane(ll), true);
}
}  // namespace autoware::experimental

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
