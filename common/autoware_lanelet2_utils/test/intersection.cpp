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
#include <string>

namespace fs = std::filesystem;

namespace autoware::experimental
{
class TestWithIntersectionCrossingMap : public ::testing::Test
{
protected:
  lanelet::LaneletMapConstPtr lanelet_map_ptr_{nullptr};

  void SetUp() override
  {
    const auto sample_map_dir =
      fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
      "sample_map";
    const auto intersection_crossing_map_path = sample_map_dir / "intersection" / "crossing.osm";

    lanelet_map_ptr_ =
      lanelet2_utils::load_mgrs_coordinate_map(intersection_crossing_map_path.string());
  }
};

TEST_F(TestWithIntersectionCrossingMap, is_intersection_lanelet_false)
{
  EXPECT_EQ(
    lanelet2_utils::is_intersection_lanelet(lanelet_map_ptr_->laneletLayer.get(2257)), false);
}

TEST_F(TestWithIntersectionCrossingMap, is_intersection_lanelet_true)
{
  EXPECT_EQ(
    lanelet2_utils::is_intersection_lanelet(lanelet_map_ptr_->laneletLayer.get(2274)), true);
}

TEST_F(TestWithIntersectionCrossingMap, is_straight_direction_false)
{
  EXPECT_EQ(lanelet2_utils::is_straight_direction(lanelet_map_ptr_->laneletLayer.get(2274)), false);
}

TEST_F(TestWithIntersectionCrossingMap, is_straight_direction_true)
{
  EXPECT_EQ(lanelet2_utils::is_straight_direction(lanelet_map_ptr_->laneletLayer.get(2278)), true);
}

TEST_F(TestWithIntersectionCrossingMap, is_left_direction_false)
{
  EXPECT_EQ(lanelet2_utils::is_left_direction(lanelet_map_ptr_->laneletLayer.get(2278)), false);
}

TEST_F(TestWithIntersectionCrossingMap, is_left_direction_true)
{
  EXPECT_EQ(lanelet2_utils::is_left_direction(lanelet_map_ptr_->laneletLayer.get(2274)), true);
}

TEST_F(TestWithIntersectionCrossingMap, is_right_direction_false)
{
  EXPECT_EQ(lanelet2_utils::is_right_direction(lanelet_map_ptr_->laneletLayer.get(2274)), false);
}

TEST_F(TestWithIntersectionCrossingMap, is_right_direction_true)
{
  EXPECT_EQ(lanelet2_utils::is_right_direction(lanelet_map_ptr_->laneletLayer.get(2277)), true);
}

TEST_F(TestWithIntersectionCrossingMap, get_turn_direction)
{
  // not intersection
  EXPECT_EQ(
    lanelet2_utils::get_turn_direction(lanelet_map_ptr_->laneletLayer.get(2257)).has_value(),
    false);

  // straight
  {
    const auto lane = lanelet2_utils::get_turn_direction(lanelet_map_ptr_->laneletLayer.get(2278));
    EXPECT_EQ(lane.has_value() && lane.value() == lanelet2_utils::TurnDirection::Straight, true);
  }

  // left
  {
    const auto lane = lanelet2_utils::get_turn_direction(lanelet_map_ptr_->laneletLayer.get(2274));
    EXPECT_EQ(lane.has_value() && lane.value() == lanelet2_utils::TurnDirection::Left, true);
  }

  // right
  {
    const auto lane = lanelet2_utils::get_turn_direction(lanelet_map_ptr_->laneletLayer.get(2277));
    EXPECT_EQ(lane.has_value() && lane.value() == lanelet2_utils::TurnDirection::Right, true);
  }
}

}  // namespace autoware::experimental

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
