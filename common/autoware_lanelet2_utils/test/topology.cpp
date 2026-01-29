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
#include <limits>
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
    const auto intersection_crossing_map_path = sample_map_dir / "vm_03/left_hand/lanelet2_map.osm";

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

TEST_F(TestWithIntersectionCrossingMap, left_lanelets)
{
  const auto lanes =
    lanelet2_utils::left_lanelets(lanelet_map_ptr_->laneletLayer.get(2246), routing_graph_ptr_);
  ASSERT_TRUE(lanes.has_value());
  ASSERT_EQ(lanes.value().size(), 2);
  EXPECT_EQ(lanes.value()[0].id(), 2245);
  EXPECT_EQ(lanes.value()[1].id(), 2244);
}

TEST_F(TestWithIntersectionCrossingMap, left_lanelets_not_exist)
{
  const auto lanes =
    lanelet2_utils::left_lanelets(lanelet_map_ptr_->laneletLayer.get(2244), routing_graph_ptr_);
  ASSERT_FALSE(lanes.has_value());
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

TEST_F(TestWithIntersectionCrossingMap, right_lanelets)
{
  const auto lanes =
    lanelet2_utils::right_lanelets(lanelet_map_ptr_->laneletLayer.get(2246), routing_graph_ptr_);
  ASSERT_TRUE(lanes.has_value());
  ASSERT_EQ(lanes.value().size(), 2);
  EXPECT_EQ(lanes.value()[0].id(), 2247);
  EXPECT_EQ(lanes.value()[1].id(), 2248);
}

TEST_F(TestWithIntersectionCrossingMap, right_lanelets_not_exist)
{
  const auto lanes =
    lanelet2_utils::right_lanelets(lanelet_map_ptr_->laneletLayer.get(2248), routing_graph_ptr_);
  ASSERT_FALSE(lanes.has_value());
}

TEST_F(TestWithIntersectionCrossingMap, all_neighbor_lanelets)
{
  const auto lanes = lanelet2_utils::all_neighbor_lanelets(
    lanelet_map_ptr_->laneletLayer.get(2246), routing_graph_ptr_);
  ASSERT_EQ(lanes.size(), 5);
  // Left
  EXPECT_EQ(lanes[0].id(), 2244);
  EXPECT_EQ(lanes[1].id(), 2245);
  // Itself
  EXPECT_EQ(lanes[2].id(), 2246);
  // Right
  EXPECT_EQ(lanes[3].id(), 2247);
  EXPECT_EQ(lanes[4].id(), 2248);

  // check that the first lane is leftmost
  {
    auto leftmost_opt = lanelet2_utils::left_lanelet(
      lanelet_map_ptr_->laneletLayer.get(lanes.front().id()), routing_graph_ptr_);
    EXPECT_FALSE(leftmost_opt.has_value());
  }
  // check that the last lane is leftmost
  {
    auto rightmost_opt = lanelet2_utils::right_lanelet(
      lanelet_map_ptr_->laneletLayer.get(lanes.back().id()), routing_graph_ptr_);
    EXPECT_FALSE(rightmost_opt.has_value());
  }
}

TEST_F(TestWithIntersectionCrossingMap, lane_changeable_neighbors_with_lc_permission)
{
  const auto lanes = lanelet2_utils::lane_changeable_neighbors(
    lanelet_map_ptr_->laneletLayer.get(2254), routing_graph_ptr_);
  ASSERT_EQ(lanes.size(), 4);
  std::vector<int64_t> expected_id = {2253, 2254, 2255, 2256};
  for (auto i = 0ul; i < lanes.size(); ++i) {
    EXPECT_EQ(lanes[i].id(), expected_id[i]);
  }
}

TEST_F(TestWithIntersectionCrossingMap, lane_changeable_neighbors_without_lc_permission)
{
  const auto lanes = lanelet2_utils::lane_changeable_neighbors(
    lanelet_map_ptr_->laneletLayer.get(2258), routing_graph_ptr_);
  ASSERT_EQ(lanes.size(), 1);
  EXPECT_EQ(lanes.front().id(), 2258);
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
      sample_map_dir / "vm_03/right_hand/lanelet2_map.osm";

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

TEST_F(TestWithIntersectionCrossingMap, empty_conflicting_lanelet)
{
  const auto conflicting_lanelets = lanelet2_utils::get_conflicting_lanelets(
    lanelet_map_ptr_->laneletLayer.get(2312), routing_graph_ptr_);

  ASSERT_EQ(conflicting_lanelets.size(), 0) << "Conflicting lanelets should be empty";
}

TEST_F(TestWithIntersectionCrossingMap, ordinary_conflicting_lanelet)
{
  const auto conflicting_lanelets = lanelet2_utils::get_conflicting_lanelets(
    lanelet_map_ptr_->laneletLayer.get(2270), routing_graph_ptr_);

  ASSERT_EQ(conflicting_lanelets.size(), 3) << "Size of the conflicting_lanelet doesn't match";

  const auto conflicting_ids = conflicting_lanelets |
                               ranges::views::transform([](const auto & l) { return l.id(); }) |
                               ranges::to<std::set>();
  EXPECT_EQ(conflicting_ids.find(2265) != conflicting_ids.end(), true) << "id 2265 doesn't exist";
  EXPECT_EQ(conflicting_ids.find(2283) != conflicting_ids.end(), true) << "id 2283 doesn't exist";
  EXPECT_EQ(conflicting_ids.find(2340) != conflicting_ids.end(), true) << "id 2340 doesn't exist";
  EXPECT_EQ(conflicting_ids.find(2271) != conflicting_ids.end(), false) << "id 2271 exists (wrong)";
}

TEST_F(TestWithIntersectionCrossingMap, empty_succeeding_lanelet_sequences)
{
  const auto succeeding_lanelet_sequences = lanelet2_utils::get_succeeding_lanelet_sequences(
    lanelet_map_ptr_->laneletLayer.get(2274), routing_graph_ptr_,
    std::numeric_limits<double>::max());
  EXPECT_EQ(succeeding_lanelet_sequences.size(), 0);
}

TEST_F(TestWithIntersectionCrossingMap, ordinary_succeeding_lanelet_sequences)
{
  const auto succeeding_lanelet_sequences = lanelet2_utils::get_succeeding_lanelet_sequences(
    lanelet_map_ptr_->laneletLayer.get(2244), routing_graph_ptr_,
    std::numeric_limits<double>::max());
  EXPECT_EQ(succeeding_lanelet_sequences.size(), 2);
  // 2265, 2249
  // check the first lanelet sequence
  ASSERT_EQ(succeeding_lanelet_sequences.front().size(), 2);
  EXPECT_EQ(succeeding_lanelet_sequences.front().front().id(), 2265);
  EXPECT_EQ(succeeding_lanelet_sequences.front().back().id(), 2249);

  // 2271
  // check the last lanelet sequence
  ASSERT_EQ(succeeding_lanelet_sequences.back().size(), 1);
  EXPECT_EQ(succeeding_lanelet_sequences.back().front().id(), 2271);
}

TEST_F(TestWithIntersectionCrossingMap, ordinary_succeeding_lanelet_sequences_with_length)
{
  // with length = 0 (only the next lanelet)
  {
    const auto succeeding_lanelet_sequences = lanelet2_utils::get_succeeding_lanelet_sequences(
      lanelet_map_ptr_->laneletLayer.get(2244), routing_graph_ptr_, 49);
    EXPECT_EQ(succeeding_lanelet_sequences.size(), 2);
    // 2265, 2249
    // check the first lanelet sequence
    ASSERT_EQ(succeeding_lanelet_sequences.front().size(), 1);
    EXPECT_EQ(succeeding_lanelet_sequences.front().front().id(), 2265);

    // 2271
    // check the last lanelet sequence
    ASSERT_EQ(succeeding_lanelet_sequences.back().size(), 1);
    EXPECT_EQ(succeeding_lanelet_sequences.back().front().id(), 2271);
  }

  // centerline of lanelet 2265 is (126.3859, 208.2609), (126.6293, 158.9185) -> distance 49.343
  // slightly **less** than first following lanelet
  {
    const auto succeeding_lanelet_sequences = lanelet2_utils::get_succeeding_lanelet_sequences(
      lanelet_map_ptr_->laneletLayer.get(2244), routing_graph_ptr_, 49);
    EXPECT_EQ(succeeding_lanelet_sequences.size(), 2);
    // check the first lanelet sequence
    ASSERT_EQ(succeeding_lanelet_sequences.front().size(), 1);
    EXPECT_EQ(succeeding_lanelet_sequences.front().front().id(), 2265);

    // check the last lanelet sequence
    ASSERT_EQ(succeeding_lanelet_sequences.back().size(), 1);
    EXPECT_EQ(succeeding_lanelet_sequences.back().front().id(), 2271);
  }

  // slightly **more** than first following lanelet
  {
    const auto succeeding_lanelet_sequences = lanelet2_utils::get_succeeding_lanelet_sequences(
      lanelet_map_ptr_->laneletLayer.get(2244), routing_graph_ptr_, 50);
    EXPECT_EQ(succeeding_lanelet_sequences.size(), 2);
    // check the first lanelet sequence
    ASSERT_EQ(succeeding_lanelet_sequences.front().size(), 2);
    EXPECT_EQ(succeeding_lanelet_sequences.front().front().id(), 2265);
    EXPECT_EQ(succeeding_lanelet_sequences.front().back().id(), 2249);

    // check the last lanelet sequence
    ASSERT_EQ(succeeding_lanelet_sequences.back().size(), 1);
    EXPECT_EQ(succeeding_lanelet_sequences.back().front().id(), 2271);
  }
}

TEST_F(TestWithIntersectionCrossingMap, empty_preceding_lanelet_sequences)
{
  const auto preceding_lanelet_sequences = lanelet2_utils::get_preceding_lanelet_sequences(
    lanelet_map_ptr_->laneletLayer.get(2240), routing_graph_ptr_,
    std::numeric_limits<double>::max());
  EXPECT_EQ(preceding_lanelet_sequences.size(), 0);
}

TEST_F(TestWithIntersectionCrossingMap, ordinary_preceding_lanelet_sequences)
{
  const auto preceding_lanelet_sequences = lanelet2_utils::get_preceding_lanelet_sequences(
    lanelet_map_ptr_->laneletLayer.get(2249), routing_graph_ptr_,
    std::numeric_limits<double>::max());
  ASSERT_EQ(preceding_lanelet_sequences.size(), 3);
  // closest->furthest order
  // 2265, 2244, 2301, 2239
  // 2270, 2286, 2296
  // 2283
  // check the first lanelet sequence
  ASSERT_EQ(preceding_lanelet_sequences.front().size(), 4);
  EXPECT_EQ(preceding_lanelet_sequences.front()[0].id(), 2239);
  EXPECT_EQ(preceding_lanelet_sequences.front()[1].id(), 2301);
  EXPECT_EQ(preceding_lanelet_sequences.front()[2].id(), 2244);
  EXPECT_EQ(preceding_lanelet_sequences.front()[3].id(), 2265);

  // check the second lanelet sequence
  ASSERT_EQ(preceding_lanelet_sequences[1].size(), 3);
  EXPECT_EQ(preceding_lanelet_sequences[1][0].id(), 2296);
  EXPECT_EQ(preceding_lanelet_sequences[1][1].id(), 2286);
  EXPECT_EQ(preceding_lanelet_sequences[1][2].id(), 2270);

  // check the last lanelet sequence
  ASSERT_EQ(preceding_lanelet_sequences.back().size(), 1);
  EXPECT_EQ(preceding_lanelet_sequences.back()[0].id(), 2283);
}

TEST_F(TestWithIntersectionCrossingMap, ordinary_preceding_lanelet_sequences_with_length)
{
  // with length = 0 (only the previous lanelet)
  {
    const auto preceding_lanelet_sequences = lanelet2_utils::get_preceding_lanelet_sequences(
      lanelet_map_ptr_->laneletLayer.get(2249), routing_graph_ptr_, 0);
    ASSERT_EQ(preceding_lanelet_sequences.size(), 3);
    // closest->furthest order
    // 2265, 2244, 2301, 2239
    // 2270, 2286, 2296
    // 2283
    // check the first lanelet sequence
    ASSERT_EQ(preceding_lanelet_sequences.front().size(), 1);
    EXPECT_EQ(preceding_lanelet_sequences.front()[0].id(), 2265);

    // check the second lanelet sequence
    ASSERT_EQ(preceding_lanelet_sequences[1].size(), 1);
    EXPECT_EQ(preceding_lanelet_sequences[1][0].id(), 2270);

    // check the last lanelet sequence
    ASSERT_EQ(preceding_lanelet_sequences.back().size(), 1);
    EXPECT_EQ(preceding_lanelet_sequences.back()[0].id(), 2283);
  }

  // slightly **less** than lanelet 2265
  // centerline of lanelet 2265 is (126.3859, 208.2609), (126.6293, 158.9185) -> distance 49.343
  {
    const auto preceding_lanelet_sequences = lanelet2_utils::get_preceding_lanelet_sequences(
      lanelet_map_ptr_->laneletLayer.get(2249), routing_graph_ptr_, 49);
    ASSERT_EQ(preceding_lanelet_sequences.size(), 3);
    // closest->furthest order
    // 2265, 2244, 2301, 2239
    // 2270, 2286, 2296
    // 2283
    // check the first lanelet sequence
    ASSERT_EQ(preceding_lanelet_sequences.front().size(), 1);
    EXPECT_EQ(preceding_lanelet_sequences.front()[0].id(), 2265);

    // check the second lanelet sequence
    ASSERT_EQ(preceding_lanelet_sequences[1].size(), 2);
    EXPECT_EQ(preceding_lanelet_sequences[1][0].id(), 2286);
    EXPECT_EQ(preceding_lanelet_sequences[1][1].id(), 2270);

    // check the last lanelet sequence
    ASSERT_EQ(preceding_lanelet_sequences.back().size(), 1);
    EXPECT_EQ(preceding_lanelet_sequences.back()[0].id(), 2283);
  }

  // slightly **more** than lanelet 2265
  {
    const auto preceding_lanelet_sequences = lanelet2_utils::get_preceding_lanelet_sequences(
      lanelet_map_ptr_->laneletLayer.get(2249), routing_graph_ptr_, 50);
    ASSERT_EQ(preceding_lanelet_sequences.size(), 3);
    // closest->furthest order
    // 2265, 2244, 2301, 2239
    // 2270, 2286, 2296
    // 2283
    // check the first lanelet sequence
    ASSERT_EQ(preceding_lanelet_sequences.front().size(), 2);
    EXPECT_EQ(preceding_lanelet_sequences.front()[0].id(), 2244);
    EXPECT_EQ(preceding_lanelet_sequences.front()[1].id(), 2265);

    // check the second lanelet sequence
    ASSERT_EQ(preceding_lanelet_sequences[1].size(), 2);
    EXPECT_EQ(preceding_lanelet_sequences[1][0].id(), 2286);
    EXPECT_EQ(preceding_lanelet_sequences[1][1].id(), 2270);

    // check the last lanelet sequence
    ASSERT_EQ(preceding_lanelet_sequences.back().size(), 1);
    EXPECT_EQ(preceding_lanelet_sequences.back()[0].id(), 2283);
  }
}

TEST_F(TestWithIntersectionCrossingMap, ordinary_preceding_lanelet_sequences_exclude_lanelets)
{
  lanelet::ConstLanelets exclude_lanelets;
  // lanelet in the first lanelet sequence
  exclude_lanelets.push_back(lanelet_map_ptr_->laneletLayer.get(2301));
  // initial lanelet in the second lanelet sequence
  exclude_lanelets.push_back(lanelet_map_ptr_->laneletLayer.get(2270));

  const auto preceding_lanelet_sequences = lanelet2_utils::get_preceding_lanelet_sequences(
    lanelet_map_ptr_->laneletLayer.get(2249), routing_graph_ptr_,
    std::numeric_limits<double>::max(), exclude_lanelets);
  ASSERT_EQ(preceding_lanelet_sequences.size(), 2);
  // closest->furthest order
  // 1 - 2265, 2244, *2301*, 2239
  // 2 - *2270*, 2286, 2296
  // 3 - 2283 (become second after excluding)
  // check the first lanelet sequence
  ASSERT_EQ(preceding_lanelet_sequences.front().size(), 2);
  EXPECT_EQ(preceding_lanelet_sequences.front()[0].id(), 2244);
  EXPECT_EQ(preceding_lanelet_sequences.front()[1].id(), 2265);

  // check the last lanelet sequence
  ASSERT_EQ(preceding_lanelet_sequences.back().size(), 1);
  EXPECT_EQ(preceding_lanelet_sequences.back()[0].id(), 2283);
}
}  // namespace autoware::experimental

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
