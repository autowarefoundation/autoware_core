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
#include <autoware/lanelet2_utils/lane_sequence.hpp>
#include <range/v3/all.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>

#include <filesystem>
#include <string>
namespace fs = std::filesystem;

namespace autoware::experimental
{
class TestLaneSequence : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto test_case_path = std::filesystem::path(ament_index_cpp::get_package_share_directory(
                                  "autoware_lanelet2_utils")) /
                                "test_data" / "test_nn_search_001.yaml";
    const auto test_case_data = autoware::test_utils::load_test_case(test_case_path.string());

    lanelet_map_ = lanelet2_utils::load_mgrs_coordinate_map(test_case_data.map_abs_path);
    routing_graph_ =
      lanelet2_utils::instantiate_routing_graph_and_traffic_rules(lanelet_map_).first;

    lane1 = lanelet_map_->laneletLayer.get(2245);  // lane1 -> lane2 is connected
    lane2 = lanelet_map_->laneletLayer.get(2261);  // lane2 -> lane_other is not connected
    lane_other = lanelet_map_->laneletLayer.get(2312);
  };

  lanelet::LaneletMapConstPtr lanelet_map_;
  lanelet::routing::RoutingGraphConstPtr routing_graph_;

  lanelet::ConstLanelet lane1{};
  lanelet::ConstLanelet lane2{};
  lanelet::ConstLanelet lane_other{};
};

TEST_F(TestLaneSequence, test_ctor)
{
  const auto seq = lanelet2_utils::LaneSequence(lane1);
  ASSERT_EQ(seq.as_lanelets().size(), 1);
}

TEST_F(TestLaneSequence, test_ctor_from_non_sequence)
{
  const auto seq = lanelet2_utils::LaneSequence::create({lane1, lane2, lane_other}, routing_graph_);
  ASSERT_FALSE(seq.has_value());
}

}  // namespace autoware::experimental

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
