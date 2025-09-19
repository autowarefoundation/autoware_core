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
#include <autoware/lanelet2_utils/route_manager.hpp>
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
    const auto intersection_crossing_map_path = sample_map_dir / "intersection" / "crossing.osm";

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

TEST_F(TestWithIntersectionCrossingMap, shoulder_lane_is_inaccessible_on_routing_graph)
{
  const auto & map_handler = map_handler_opt_.value();
  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  const auto lane = map_handler.left_lanelet(
    lanelet_map_ptr->laneletLayer.get(2257), false, lanelet2_utils::ExtraVRU::RoadOnly);
  EXPECT_EQ(lane.has_value(), false);
}

TEST_F(TestWithIntersectionCrossingMap, bicycle_lane_is_inaccessible_on_routing_graph)
{
  const auto & map_handler = map_handler_opt_.value();
  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  const auto lane = map_handler.left_lanelet(
    lanelet_map_ptr->laneletLayer.get(2286), false, lanelet2_utils::ExtraVRU::RoadOnly);
  EXPECT_EQ(lane.has_value(), false);
}

TEST_F(TestWithIntersectionCrossingMap, left_lanelet_without_lc_permission)
{
  const auto & map_handler = map_handler_opt_.value();
  const auto lanelet_map_ptr = map_handler.lanelet_map_ptr();

  const auto lane = map_handler.left_lanelet(
    lanelet_map_ptr->laneletLayer.get(2246), false, lanelet2_utils::ExtraVRU::RoadOnly);
  EXPECT_EQ(lane.value().id(), 2245);
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

class TestWithIntersectionCrossingInverseMap : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto sample_map_dir =
      fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
      "sample_map";
    const auto intersection_crossing_map_path =
      sample_map_dir / "intersection" / "crossing_inverse.osm";

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

}  // namespace autoware::experimental

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
