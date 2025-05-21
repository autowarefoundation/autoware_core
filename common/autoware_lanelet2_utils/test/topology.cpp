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
#include <range/v3/all.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>

#include <filesystem>
#include <set>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace autoware::experimental
{

class TestWithIntersectionCrossingMap : public ::testing::Test
{
protected:
  lanelet::LaneletMapConstPtr lanelet_map_ptr_{nullptr};
  lanelet::routing::RoutingGraphConstPtr routing_graph_ptr_{nullptr};

  void SetUp() override
  {
    const auto sample_map_dir =
      fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
      "sample_map";
    const auto intersection_crossing_map_path = sample_map_dir / "intersection" / "crossing.osm";

    lanelet_map_ptr_ =
      lanelet2_utils::load_mgrs_coordinate_map(intersection_crossing_map_path.string());
    routing_graph_ptr_ =
      lanelet2_utils::instantiate_routing_graph_and_traffic_rules(lanelet_map_ptr_).first;
  }
};

TEST_F(TestWithIntersectionCrossingMap, LoadCheck)
{
  const auto point = lanelet_map_ptr_->pointLayer.get(1791);
  EXPECT_NEAR(point.x(), 100.0, 0.05);
  EXPECT_NEAR(point.y(), 100.0, 0.05);
}

TEST_F(TestWithIntersectionCrossingMap, shoulder_lane_is_inaccessible_on_routing_graph)
{
  const auto lane =
    lanelet2_utils::left_lanelet(lanelet_map_ptr_->laneletLayer.get(2257), routing_graph_ptr_);
  EXPECT_EQ(lane.has_value(), false);
}

TEST_F(TestWithIntersectionCrossingMap, bicycle_lane_is_inaccessible_on_routing_graph)
{
  const auto lane =
    lanelet2_utils::left_lanelet(lanelet_map_ptr_->laneletLayer.get(2286), routing_graph_ptr_);
  EXPECT_EQ(lane.has_value(), false);
}

TEST_F(TestWithIntersectionCrossingMap, left_lanelet_without_lc_permission)
{
  const auto lane =
    lanelet2_utils::left_lanelet(lanelet_map_ptr_->laneletLayer.get(2246), routing_graph_ptr_);
  EXPECT_EQ(lane.value().id(), 2245);
}

TEST_F(TestWithIntersectionCrossingMap, left_lanelet_with_lc_permission)
{
  const auto lane =
    lanelet2_utils::left_lanelet(lanelet_map_ptr_->laneletLayer.get(2245), routing_graph_ptr_);
  EXPECT_EQ(lane.value().id(), 2244);
}

TEST_F(TestWithIntersectionCrossingMap, right_lanelet_without_lc_permission)
{
  const auto lane =
    lanelet2_utils::right_lanelet(lanelet_map_ptr_->laneletLayer.get(2245), routing_graph_ptr_);
  EXPECT_EQ(lane.value().id(), 2246);
}

TEST_F(TestWithIntersectionCrossingMap, right_lanelet_with_lc_permission)
{
  const auto lane =
    lanelet2_utils::right_lanelet(lanelet_map_ptr_->laneletLayer.get(2244), routing_graph_ptr_);
  EXPECT_EQ(lane.value().id(), 2245);
}

TEST_F(TestWithIntersectionCrossingMap, right_opposite_lanelet_valid)
{
  const auto lane = lanelet2_utils::right_opposite_lanelet(
    lanelet_map_ptr_->laneletLayer.get(2288), lanelet_map_ptr_);
  EXPECT_EQ(lane.value().id(), 2311);
}

TEST_F(TestWithIntersectionCrossingMap, right_opposite_lanelet_null)
{
  const auto lane = lanelet2_utils::right_opposite_lanelet(
    lanelet_map_ptr_->laneletLayer.get(2260), lanelet_map_ptr_);
  EXPECT_EQ(lane.has_value(), false);
}

TEST_F(TestWithIntersectionCrossingMap, leftmost_lanelet_valid)
{
  const auto lane =
    lanelet2_utils::leftmost_lanelet(lanelet_map_ptr_->laneletLayer.get(2288), routing_graph_ptr_);
  EXPECT_EQ(lane.value().id(), 2286);
}

TEST_F(TestWithIntersectionCrossingMap, leftmost_lanelet_null)
{
  const auto lane =
    lanelet2_utils::leftmost_lanelet(lanelet_map_ptr_->laneletLayer.get(2286), routing_graph_ptr_);
  EXPECT_EQ(lane.has_value(), false);
}

TEST_F(TestWithIntersectionCrossingMap, rightmost_lanelet_valid)
{
  const auto lane =
    lanelet2_utils::rightmost_lanelet(lanelet_map_ptr_->laneletLayer.get(2286), routing_graph_ptr_);
  EXPECT_EQ(lane.value().id(), 2288);
}

TEST_F(TestWithIntersectionCrossingMap, rightmost_lanelet_null)
{
  const auto lane =
    lanelet2_utils::rightmost_lanelet(lanelet_map_ptr_->laneletLayer.get(2288), routing_graph_ptr_);
  EXPECT_EQ(lane.has_value(), false);
}

TEST_F(TestWithIntersectionCrossingMap, left_lanelets_without_opposite)
{
  const auto lefts = lanelet2_utils::left_lanelets(
    lanelet_map_ptr_->laneletLayer.get(2288), lanelet_map_ptr_, routing_graph_ptr_);
  EXPECT_EQ(lefts.size(), 2);
  EXPECT_EQ(lefts[0].id(), 2287);
  EXPECT_EQ(lefts[1].id(), 2286);
}

TEST_F(TestWithIntersectionCrossingMap, left_lanelets_without_opposite_empty)
{
  const auto lefts = lanelet2_utils::left_lanelets(
    lanelet_map_ptr_->laneletLayer.get(2286), lanelet_map_ptr_, routing_graph_ptr_);
  EXPECT_EQ(lefts.size(), 0);
}

TEST_F(TestWithIntersectionCrossingMap, right_lanelets_without_opposite)
{
  const auto lefts = lanelet2_utils::right_lanelets(
    lanelet_map_ptr_->laneletLayer.get(2286), lanelet_map_ptr_, routing_graph_ptr_);
  EXPECT_EQ(lefts.size(), 2);
  EXPECT_EQ(lefts[0].id(), 2287);
  EXPECT_EQ(lefts[1].id(), 2288);
}

TEST_F(TestWithIntersectionCrossingMap, right_lanelets_without_opposite_empty)
{
  const auto lefts = lanelet2_utils::right_lanelets(
    lanelet_map_ptr_->laneletLayer.get(2288), lanelet_map_ptr_, routing_graph_ptr_);
  EXPECT_EQ(lefts.size(), 0);
}

TEST_F(TestWithIntersectionCrossingMap, right_lanelets_with_opposite)
{
  const auto rights = lanelet2_utils::right_lanelets(
    lanelet_map_ptr_->laneletLayer.get(2286), lanelet_map_ptr_, routing_graph_ptr_,
    true /* include opposite */);
  EXPECT_EQ(rights.size(), 4);
  EXPECT_EQ(rights[0].id(), 2287);
  EXPECT_EQ(rights[1].id(), 2288);
  EXPECT_EQ(rights[2].id(), 2311);
  EXPECT_EQ(rights[3].id(), 2312);
}

TEST_F(TestWithIntersectionCrossingMap, right_lanelets_with_opposite_without_actual_opposites)
{
  const auto rights = lanelet2_utils::right_lanelets(
    lanelet_map_ptr_->laneletLayer.get(2259), lanelet_map_ptr_, routing_graph_ptr_,
    true /* include opposite */);
  EXPECT_EQ(rights.size(), 1);
  EXPECT_EQ(rights[0].id(), 2260);
}

TEST_F(TestWithIntersectionCrossingMap, following_lanelets)
{
  const auto following = lanelet2_utils::following_lanelets(
    lanelet_map_ptr_->laneletLayer.get(2244), routing_graph_ptr_);
  EXPECT_EQ(following.size(), 2);
  const auto ids = following | ranges::views::transform([](const auto & l) { return l.id(); }) |
                   ranges::to<std::set>();
  EXPECT_EQ(ids.find(2271) != ids.end(), true);
  EXPECT_EQ(ids.find(2265) != ids.end(), true);
}

TEST_F(TestWithIntersectionCrossingMap, previous_lanelets)
{
  const auto previous =
    lanelet2_utils::previous_lanelets(lanelet_map_ptr_->laneletLayer.get(2249), routing_graph_ptr_);
  EXPECT_EQ(previous.size(), 3);
  const auto ids = previous | ranges::views::transform([](const auto & l) { return l.id(); }) |
                   ranges::to<std::set>();
  EXPECT_EQ(ids.find(2283) != ids.end(), true);
  EXPECT_EQ(ids.find(2265) != ids.end(), true);
  EXPECT_EQ(ids.find(2270) != ids.end(), true);
}

TEST_F(TestWithIntersectionCrossingMap, sibling_lanelets)
{
  const auto siblings =
    lanelet2_utils::sibling_lanelets(lanelet_map_ptr_->laneletLayer.get(2273), routing_graph_ptr_);
  const auto ids = siblings | ranges::views::transform([](const auto & l) { return l.id(); }) |
                   ranges::to<std::set>();
  EXPECT_EQ(ids.find(2273) != ids.end(), false);
  EXPECT_EQ(ids.find(2280) != ids.end(), true);
  EXPECT_EQ(ids.find(2281) != ids.end(), true);
}

TEST_F(TestWithIntersectionCrossingMap, from_ids)
{
  const auto lanelets =
    lanelet2_utils::from_ids(lanelet_map_ptr_, std::vector<lanelet::Id>({2296, 2286, 2270}));
  EXPECT_EQ(lanelets.size(), 3);
  EXPECT_EQ(lanelets[0].id(), 2296);
  EXPECT_EQ(lanelets[1].id(), 2286);
  EXPECT_EQ(lanelets[2].id(), 2270);
}

class TestWithIntersectionCrossingInverseMap : public ::testing::Test
{
protected:
  lanelet::LaneletMapConstPtr lanelet_map_ptr_{nullptr};
  lanelet::routing::RoutingGraphConstPtr routing_graph_ptr_{nullptr};

  void SetUp() override
  {
    const auto sample_map_dir =
      fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
      "sample_map";
    const auto intersection_crossing_map_path =
      sample_map_dir / "intersection" / "crossing_inverse.osm";

    lanelet_map_ptr_ =
      lanelet2_utils::load_mgrs_coordinate_map(intersection_crossing_map_path.string());
    routing_graph_ptr_ =
      lanelet2_utils::instantiate_routing_graph_and_traffic_rules(lanelet_map_ptr_).first;
  }
};

TEST_F(TestWithIntersectionCrossingInverseMap, left_opposite_lanelet_valid)
{
  const auto lane = lanelet2_utils::left_opposite_lanelet(
    lanelet_map_ptr_->laneletLayer.get(2311), lanelet_map_ptr_);
  EXPECT_EQ(lane.value().id(), 2288);
}

TEST_F(TestWithIntersectionCrossingInverseMap, left_opposite_lanelet_null)
{
  const auto lane = lanelet2_utils::left_opposite_lanelet(
    lanelet_map_ptr_->laneletLayer.get(2252), lanelet_map_ptr_);
  EXPECT_EQ(lane.has_value(), false);
}

TEST_F(TestWithIntersectionCrossingInverseMap, left_lanelets_with_opposite)
{
  const auto lefts = lanelet2_utils::left_lanelets(
    lanelet_map_ptr_->laneletLayer.get(2312), lanelet_map_ptr_, routing_graph_ptr_, true);
  EXPECT_EQ(lefts.size(), 4);
  EXPECT_EQ(lefts[0].id(), 2311);
  EXPECT_EQ(lefts[1].id(), 2288);
  EXPECT_EQ(lefts[2].id(), 2287);
  EXPECT_EQ(lefts[3].id(), 2286);
}

TEST_F(TestWithIntersectionCrossingInverseMap, left_lanelets_with_opposite_without_actual_opposites)
{
  const auto lefts = lanelet2_utils::left_lanelets(
    lanelet_map_ptr_->laneletLayer.get(2251), lanelet_map_ptr_, routing_graph_ptr_, true);
  EXPECT_EQ(lefts.size(), 1);
  EXPECT_EQ(lefts[0].id(), 2252);
}

}  // namespace autoware::experimental

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
