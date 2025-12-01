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

#include "test_case.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/map_handler.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>

#include <filesystem>
#include <string>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

namespace autoware::experimental
{

class TestWithIntersectionCrossingMap : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto sample_map_dir =
      fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
      "sample_map";
    const auto intersection_crossing_map_path = sample_map_dir / "vm_03/left_hand/lanelet2_map.osm";

    map_msg_ = lanelet2_utils::to_autoware_map_msgs(
      lanelet2_utils::load_mgrs_coordinate_map(intersection_crossing_map_path.string()));

    map_handler_opt_.emplace(lanelet2_utils::MapHandler::create(map_msg_).value());
  }

  autoware_map_msgs::msg::LaneletMapBin map_msg_;
  std::optional<lanelet2_utils::MapHandler> map_handler_opt_;
};

TEST_F(TestWithIntersectionCrossingMap, LoadCheck)
{
  ASSERT_TRUE(map_handler_opt_.has_value());
  const auto & map_handler = map_handler_opt_.value();

  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  const auto point = lanelet_map_ptr->pointLayer.get(1791);
  EXPECT_NEAR(point.x(), 100.0, 0.05);
  EXPECT_NEAR(point.y(), 100.0, 0.05);
}

TEST_F(TestWithIntersectionCrossingMap, left_lanelet_of_road_with_shoulder_on_left)
{
  const auto & map_handler = map_handler_opt_.value();
  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  {
    const auto lane = map_handler.left_lanelet(
      lanelet_map_ptr->laneletLayer.get(2257), false, lanelet2_utils::ExtraVRU::RoadOnly);
    EXPECT_EQ(lane.has_value(), false);
  }

  {
    const auto lane = map_handler.left_lanelet(
      lanelet_map_ptr->laneletLayer.get(2257), false, lanelet2_utils::ExtraVRU::Shoulder);
    EXPECT_EQ(lane.has_value(), true);
    EXPECT_EQ(lane.value().id(), 2309);
  }

  {
    const auto lane = map_handler.left_lanelet(
      lanelet_map_ptr->laneletLayer.get(2257), false, lanelet2_utils::ExtraVRU::BicycleLane);
    EXPECT_EQ(lane.has_value(), false);
  }

  {
    const auto lane = map_handler.left_lanelet(
      lanelet_map_ptr->laneletLayer.get(2257), false,
      lanelet2_utils::ExtraVRU::ShoulderAndBicycleLane);
    EXPECT_EQ(lane.has_value(), true);
    EXPECT_EQ(lane.value().id(), 2309);
  }
}

TEST_F(TestWithIntersectionCrossingMap, left_lanelet_of_road_with_bicycle_lane_on_left)
{
  const auto & map_handler = map_handler_opt_.value();
  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  {
    const auto lane = map_handler.left_lanelet(
      lanelet_map_ptr->laneletLayer.get(2286), false, lanelet2_utils::ExtraVRU::RoadOnly);
    EXPECT_EQ(lane.has_value(), false);
  }

  {
    const auto lane = map_handler.left_lanelet(
      lanelet_map_ptr->laneletLayer.get(2286), false, lanelet2_utils::ExtraVRU::Shoulder);
    EXPECT_EQ(lane.has_value(), false);
  }

  {
    const auto lane = map_handler.left_lanelet(
      lanelet_map_ptr->laneletLayer.get(2286), false, lanelet2_utils::ExtraVRU::BicycleLane);
    EXPECT_EQ(lane.has_value(), true);
    EXPECT_EQ(lane.value().id(), 2303);
  }

  {
    const auto lane = map_handler.left_lanelet(
      lanelet_map_ptr->laneletLayer.get(2286), false,
      lanelet2_utils::ExtraVRU::ShoulderAndBicycleLane);
    EXPECT_EQ(lane.has_value(), true);
    EXPECT_EQ(lane.value().id(), 2303);
  }
}

TEST_F(TestWithIntersectionCrossingMap, left_lanelet_without_lc_permission)
{
  const auto & map_handler = map_handler_opt_.value();
  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  const auto lane = map_handler.left_lanelet(
    lanelet_map_ptr->laneletLayer.get(2246), false, lanelet2_utils::ExtraVRU::RoadOnly);
  EXPECT_EQ(lane.value().id(), 2245);
}

TEST_F(TestWithIntersectionCrossingMap, left_lanelet_with_take_sibling_with_sibling)
{
  const auto & map_handler = map_handler_opt_.value();
  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  const auto lane = map_handler.left_lanelet(
    lanelet_map_ptr->laneletLayer.get(2243), true, lanelet2_utils::ExtraVRU::RoadOnly);
  EXPECT_EQ(lane.value().id(), 2299);
}

TEST_F(TestWithIntersectionCrossingMap, right_lanelet_with_take_sibling_with_sibling)
{
  const auto & map_handler = map_handler_opt_.value();
  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  const auto lane = map_handler.right_lanelet(
    lanelet_map_ptr->laneletLayer.get(2299), true, lanelet2_utils::ExtraVRU::RoadOnly);
  EXPECT_EQ(lane.value().id(), 2243);
}

TEST_F(TestWithIntersectionCrossingMap, left_lanelet_with_lc_permission)
{
  const auto & map_handler = map_handler_opt_.value();
  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  const auto lane = map_handler.left_lanelet(
    lanelet_map_ptr->laneletLayer.get(2245), false, lanelet2_utils::ExtraVRU::RoadOnly);
  EXPECT_EQ(lane.value().id(), 2244);
}

TEST_F(TestWithIntersectionCrossingMap, right_lanelet_without_lc_permission)
{
  const auto & map_handler = map_handler_opt_.value();
  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  const auto lane = map_handler.right_lanelet(
    lanelet_map_ptr->laneletLayer.get(2245), false, lanelet2_utils::ExtraVRU::RoadOnly);
  EXPECT_EQ(lane.value().id(), 2246);
}

TEST_F(TestWithIntersectionCrossingMap, right_lanelet_with_lc_permission)
{
  const auto & map_handler = map_handler_opt_.value();
  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  const auto lane = map_handler.right_lanelet(
    lanelet_map_ptr->laneletLayer.get(2244), false, lanelet2_utils::ExtraVRU::RoadOnly);
  EXPECT_EQ(lane.value().id(), 2245);
}

TEST_F(TestWithIntersectionCrossingMap, leftmost_lanelet_valid)
{
  const auto & map_handler = map_handler_opt_.value();
  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  const auto lane = map_handler.leftmost_lanelet(
    lanelet_map_ptr->laneletLayer.get(2288), false, lanelet2_utils::ExtraVRU::RoadOnly);
  EXPECT_EQ(lane.value().id(), 2286);
}

TEST_F(TestWithIntersectionCrossingMap, leftmost_lanelet_null)
{
  const auto & map_handler = map_handler_opt_.value();
  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  const auto lane = map_handler.leftmost_lanelet(
    lanelet_map_ptr->laneletLayer.get(2286), false, lanelet2_utils::ExtraVRU::RoadOnly);
  EXPECT_EQ(lane.has_value(), false);
}

TEST_F(TestWithIntersectionCrossingMap, rightmost_lanelet_valid)
{
  const auto & map_handler = map_handler_opt_.value();
  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  const auto lane = map_handler.rightmost_lanelet(
    lanelet_map_ptr->laneletLayer.get(2286), false, lanelet2_utils::ExtraVRU::RoadOnly);
  EXPECT_EQ(lane.value().id(), 2288);
}

TEST_F(TestWithIntersectionCrossingMap, rightmost_lanelet_null)
{
  const auto & map_handler = map_handler_opt_.value();
  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  const auto lane = map_handler.rightmost_lanelet(
    lanelet_map_ptr->laneletLayer.get(2288), false, lanelet2_utils::ExtraVRU::RoadOnly);
  EXPECT_EQ(lane.has_value(), false);
}

TEST_F(TestWithIntersectionCrossingMap, left_lanelets_without_opposite)
{
  const auto & map_handler = map_handler_opt_.value();
  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  const auto lefts = map_handler.left_lanelets(lanelet_map_ptr->laneletLayer.get(2288));
  EXPECT_EQ(lefts.size(), 2);
  EXPECT_EQ(lefts[0].id(), 2287);
  EXPECT_EQ(lefts[1].id(), 2286);
}

TEST_F(TestWithIntersectionCrossingMap, left_lanelets_without_opposite_empty)
{
  const auto & map_handler = map_handler_opt_.value();
  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  const auto lefts = map_handler.left_lanelets(lanelet_map_ptr->laneletLayer.get(2286));
  EXPECT_EQ(lefts.size(), 0);
}

TEST_F(TestWithIntersectionCrossingMap, right_lanelets_without_opposite)
{
  const auto & map_handler = map_handler_opt_.value();
  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  const auto lefts = map_handler.right_lanelets(lanelet_map_ptr->laneletLayer.get(2286));
  EXPECT_EQ(lefts.size(), 2);
  EXPECT_EQ(lefts[0].id(), 2287);
  EXPECT_EQ(lefts[1].id(), 2288);
}

TEST_F(TestWithIntersectionCrossingMap, right_lanelets_without_opposite_empty)
{
  const auto & map_handler = map_handler_opt_.value();
  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  const auto lefts = map_handler.right_lanelets(lanelet_map_ptr->laneletLayer.get(2288));
  EXPECT_EQ(lefts.size(), 0);
}

TEST_F(TestWithIntersectionCrossingMap, right_lanelets_with_opposite)
{
  const auto & map_handler = map_handler_opt_.value();
  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  const auto rights = map_handler.right_lanelets(lanelet_map_ptr->laneletLayer.get(2286), true);
  EXPECT_EQ(rights.size(), 4);
  EXPECT_EQ(rights[0].id(), 2287);
  EXPECT_EQ(rights[1].id(), 2288);
  EXPECT_EQ(rights[2].id(), 2311);
  EXPECT_EQ(rights[3].id(), 2312);
}

TEST_F(TestWithIntersectionCrossingMap, right_lanelets_with_opposite_without_actual_opposites)
{
  const auto & map_handler = map_handler_opt_.value();
  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  const auto rights = map_handler.right_lanelets(lanelet_map_ptr->laneletLayer.get(2259), true);
  EXPECT_EQ(rights.size(), 1);
  EXPECT_EQ(rights[0].id(), 2260);
}

TEST_F(TestWithIntersectionCrossingMap, right_lanelet_of_shoulder)
{
  const auto & map_handler = map_handler_opt_.value();
  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  {
    const auto lane = map_handler.right_lanelet(
      lanelet_map_ptr->laneletLayer.get(2309), false, lanelet2_utils::ExtraVRU::RoadOnly);
    EXPECT_EQ(lane.has_value(), true);
    EXPECT_EQ(lane.value().id(), 2257);
  }
}

class TestWithIntersectionCrossingInverseMap : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto sample_map_dir =
      fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
      "sample_map";
    const auto intersection_crossing_map_path =
      sample_map_dir / "vm_03/right_hand/lanelet2_map.osm";

    map_msg_ = lanelet2_utils::to_autoware_map_msgs(
      lanelet2_utils::load_mgrs_coordinate_map(intersection_crossing_map_path.string()));
    map_handler_opt_.emplace(lanelet2_utils::MapHandler::create(map_msg_).value());
  }

  autoware_map_msgs::msg::LaneletMapBin map_msg_;
  std::optional<lanelet2_utils::MapHandler> map_handler_opt_;
};

TEST_F(TestWithIntersectionCrossingInverseMap, left_lanelets_with_opposite)
{
  const auto & map_handler = map_handler_opt_.value();
  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  const auto lefts = map_handler.left_lanelets(lanelet_map_ptr->laneletLayer.get(2312), true);
  EXPECT_EQ(lefts.size(), 4);
  EXPECT_EQ(lefts[0].id(), 2311);
  EXPECT_EQ(lefts[1].id(), 2288);
  EXPECT_EQ(lefts[2].id(), 2287);
  EXPECT_EQ(lefts[3].id(), 2286);
}

TEST_F(TestWithIntersectionCrossingInverseMap, left_lanelets_with_opposite_without_actual_opposites)
{
  const auto & map_handler = map_handler_opt_.value();
  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  const auto lefts = map_handler.left_lanelets(lanelet_map_ptr->laneletLayer.get(2251), true);
  EXPECT_EQ(lefts.size(), 1);
  EXPECT_EQ(lefts[0].id(), 2252);
}

TEST_F(TestWithIntersectionCrossingInverseMap, left_lanelet_of_shoulder)
{
  const auto & map_handler = map_handler_opt_.value();
  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  {
    const auto lane = map_handler.left_lanelet(
      lanelet_map_ptr->laneletLayer.get(2309), false, lanelet2_utils::ExtraVRU::RoadOnly);
    EXPECT_EQ(lane.has_value(), true);
    EXPECT_EQ(lane->id(), 2257);
  }
}

TEST_F(TestWithIntersectionCrossingInverseMap, right_lanelet_of_road_with_shoulder_on_right)
{
  const auto & map_handler = map_handler_opt_.value();
  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  {
    const auto lane = map_handler.right_lanelet(
      lanelet_map_ptr->laneletLayer.get(2257), false, lanelet2_utils::ExtraVRU::RoadOnly);
    EXPECT_EQ(lane.has_value(), false);
  }

  {
    const auto lane = map_handler.right_lanelet(
      lanelet_map_ptr->laneletLayer.get(2257), false, lanelet2_utils::ExtraVRU::Shoulder);
    EXPECT_EQ(lane.has_value(), true);
    EXPECT_EQ(lane->id(), 2309);
  }

  {
    const auto lane = map_handler.right_lanelet(
      lanelet_map_ptr->laneletLayer.get(2257), false, lanelet2_utils::ExtraVRU::BicycleLane);
    EXPECT_EQ(lane.has_value(), false);
  }

  {
    const auto lane = map_handler.right_lanelet(
      lanelet_map_ptr->laneletLayer.get(2257), false,
      lanelet2_utils::ExtraVRU::ShoulderAndBicycleLane);
    EXPECT_EQ(lane.has_value(), true);
    EXPECT_EQ(lane->id(), 2309);
  }
}

TEST_F(TestWithIntersectionCrossingInverseMap, right_lanelet_of_road_with_bicycle_lane_on_right)
{
  const auto & map_handler = map_handler_opt_.value();
  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  {
    const auto lane = map_handler.right_lanelet(
      lanelet_map_ptr->laneletLayer.get(2286), false, lanelet2_utils::ExtraVRU::RoadOnly);
    EXPECT_EQ(lane.has_value(), false);
  }

  {
    const auto lane = map_handler.right_lanelet(
      lanelet_map_ptr->laneletLayer.get(2286), false, lanelet2_utils::ExtraVRU::Shoulder);
    EXPECT_EQ(lane.has_value(), false);
  }

  {
    const auto lane = map_handler.right_lanelet(
      lanelet_map_ptr->laneletLayer.get(2286), false, lanelet2_utils::ExtraVRU::BicycleLane);
    EXPECT_EQ(lane.has_value(), true);
    EXPECT_EQ(lane->id(), 2303);
  }

  {
    const auto lane = map_handler.right_lanelet(
      lanelet_map_ptr->laneletLayer.get(2286), false,
      lanelet2_utils::ExtraVRU::ShoulderAndBicycleLane);
    EXPECT_EQ(lane.has_value(), true);
    EXPECT_EQ(lane->id(), 2303);
  }
}

class TestWithShoulderHighwayMap : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto sample_map_dir =
      fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
      "sample_map";
    const auto intersection_crossing_map_path =
      sample_map_dir / "vm_01_15-16/highway/lanelet2_map.osm";

    map_msg_ = lanelet2_utils::to_autoware_map_msgs(
      lanelet2_utils::load_mgrs_coordinate_map(intersection_crossing_map_path.string()));
    map_handler_opt_.emplace(lanelet2_utils::MapHandler::create(map_msg_).value());
  }

  autoware_map_msgs::msg::LaneletMapBin map_msg_;
  std::optional<lanelet2_utils::MapHandler> map_handler_opt_;
};

TEST_F(TestWithShoulderHighwayMap, shoulder_sequence)
{
  const auto & map_handler = map_handler_opt_.value();
  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  {
    const auto seq =
      map_handler.get_shoulder_lanelet_sequence(lanelet_map_ptr->laneletLayer.get(48));
    EXPECT_EQ(seq.size(), 2);
    EXPECT_EQ(seq.at(0).id(), 48);
    EXPECT_EQ(seq.at(1).id(), 49);
  }

  {
    const auto seq =
      map_handler.get_shoulder_lanelet_sequence(lanelet_map_ptr->laneletLayer.get(49));
    EXPECT_EQ(seq.size(), 2);
    EXPECT_EQ(seq.at(0).id(), 48);
    EXPECT_EQ(seq.at(1).id(), 49);
  }
}

class TestWithShoulderLoopMap : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto sample_map_dir =
      fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
      "sample_map";
    const auto intersection_crossing_map_path =
      sample_map_dir / "vm_01_15-16/loop/lanelet2_map.osm";

    map_msg_ = lanelet2_utils::to_autoware_map_msgs(
      lanelet2_utils::load_mgrs_coordinate_map(intersection_crossing_map_path.string()));
    map_handler_opt_.emplace(lanelet2_utils::MapHandler::create(map_msg_).value());
  }

  autoware_map_msgs::msg::LaneletMapBin map_msg_;
  std::optional<lanelet2_utils::MapHandler> map_handler_opt_;
};

TEST_F(TestWithShoulderLoopMap, shoulder_sequence)
{
  const auto & map_handler = map_handler_opt_.value();
  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  {
    const auto seq =
      map_handler.get_shoulder_lanelet_sequence(lanelet_map_ptr->laneletLayer.get(357));
    EXPECT_EQ(seq.size(), 16);
    EXPECT_EQ(seq.front().id(), 358);
    EXPECT_EQ(seq.back().id(), 357);
  }

  {
    const auto seq =
      map_handler.get_shoulder_lanelet_sequence(lanelet_map_ptr->laneletLayer.get(359));
    EXPECT_EQ(seq.size(), 16);
    EXPECT_EQ(seq.front().id(), 350);
    EXPECT_EQ(seq.back().id(), 359);
  }
}

}  // namespace autoware::experimental

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
