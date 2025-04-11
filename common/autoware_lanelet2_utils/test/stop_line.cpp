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

#include "autoware_lanelet2_utils/stop_line.hpp"

#include "map_loader.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace autoware
{
class TestWithIntersectionCrossingMap : public ::testing::Test
{
protected:
  lanelet::LaneletMapPtr lanelet_map_intersection_ptr_{nullptr};
  lanelet::LaneletMapPtr lanelet_map_regulatory_elements_ptr_{nullptr};

  void SetUp() override
  {
    const auto sample_map_dir =
      fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
      "sample_map";
    const auto intersection_crossing_map_path = sample_map_dir / "intersection" / "crossing.osm";

    lanelet_map_intersection_ptr_ =
      load_mgrs_coordinate_map<lanelet::LaneletMapPtr>(intersection_crossing_map_path.string());

    const auto regulatory_elements_map_path =
      sample_map_dir / "intersection" / "regulatory_elements.osm";

    lanelet_map_regulatory_elements_ptr_ =
      load_mgrs_coordinate_map<lanelet::LaneletMapPtr>(regulatory_elements_map_path.string());
  }
};
// test 1: get_stop_line_from_map retrieves the stop line for a specific lane (ID 1993) from the
// map, then verifies that the stop line exists and has the expected ID.
TEST_F(TestWithIntersectionCrossingMap, GetStopLineFromMap)
{
  lanelet::Id valid_lane_id = 2273;
  auto stopline_opt = lanelet2_utils::get_stop_line_from_road_marking(
    valid_lane_id, lanelet_map_intersection_ptr_, "", false);
  std::cout << "Stop line ID: " << stopline_opt->id() << std::endl;
  ASSERT_TRUE(stopline_opt.has_value()) << "No stop line found on lane " << valid_lane_id;
  ASSERT_EQ(stopline_opt->id(), 1993) << "Wrong id";
}

// test 2: get_stop_line_from_stop_sign
TEST_F(TestWithIntersectionCrossingMap, GetStopLinesFromStopSign)
{
  const lanelet::ConstLanelets lanelets(
    lanelet_map_regulatory_elements_ptr_->laneletLayer.begin(),
    lanelet_map_regulatory_elements_ptr_->laneletLayer.end());

  std::string stop_sign_id = "stop_sign";
  auto stoplines = lanelet2_utils::get_stop_lines_from_stop_sign(lanelets, stop_sign_id);

  ASSERT_FALSE(stoplines.empty()) << "No stop lines were found for stop sign type: "
                                  << stop_sign_id;

  std::set<lanelet::Id> unique_ids;
  for (const auto & sl : stoplines) {
    unique_ids.insert(sl.id());
  }
  EXPECT_EQ(stoplines.size(), unique_ids.size()) << "Duplicate stop lines found in the output.";

  lanelet::Id expected_stop_line_id = 206;
  bool found_expected = false;
  for (const auto & sl : stoplines) {
    if (sl.id() == expected_stop_line_id) {
      found_expected = true;
      break;
    }
  }
  EXPECT_TRUE(found_expected) << "Expected stop line with id " << expected_stop_line_id
                              << " was not found.";
}

// test 3: get_stop_lines_from_no_stopping_area
TEST_F(TestWithIntersectionCrossingMap, GetStopLinesFromNoStoppingArea)
{
  const lanelet::ConstLanelets lanelets(
    lanelet_map_regulatory_elements_ptr_->laneletLayer.begin(),
    lanelet_map_regulatory_elements_ptr_->laneletLayer.end());

  auto stoplines = lanelet2_utils::get_stop_lines_from_no_stopping_area(lanelets);

  ASSERT_FALSE(stoplines.empty()) << "No stop lines were found for NoStoppingArea.";

  std::set<lanelet::Id> unique_ids;
  for (const auto & sl : stoplines) {
    unique_ids.insert(sl.id());
  }
  EXPECT_EQ(stoplines.size(), unique_ids.size())
    << "Duplicate stop lines found in NoStoppingArea output.";

  lanelet::Id expected_stop_line_id = 208;
  bool found_expected = false;
  for (const auto & sl : stoplines) {
    if (sl.id() == expected_stop_line_id) {
      found_expected = true;
      break;
    }
  }
  EXPECT_TRUE(found_expected) << "Expected stop line with id " << expected_stop_line_id
                              << " was not found.";
}

// test 4: get_stop_lines_from_detection_area
TEST_F(TestWithIntersectionCrossingMap, GetStopLinesFromDetectionArea)
{
  const lanelet::ConstLanelets lanelets(
    lanelet_map_regulatory_elements_ptr_->laneletLayer.begin(),
    lanelet_map_regulatory_elements_ptr_->laneletLayer.end());

  auto stoplines = lanelet2_utils::get_stop_lines_from_detection_area(lanelets);

  ASSERT_FALSE(stoplines.empty()) << "No stop lines were found for DetectionArea.";

  std::set<lanelet::Id> unique_ids;
  for (const auto & sl : stoplines) {
    unique_ids.insert(sl.id());
  }
  EXPECT_EQ(stoplines.size(), unique_ids.size())
    << "Duplicate stop lines found in DetectionArea output.";

  lanelet::Id expected_stop_line_id = 209;
  bool found_expected = false;
  for (const auto & sl : stoplines) {
    if (sl.id() == expected_stop_line_id) {
      found_expected = true;
      break;
    }
  }
  EXPECT_TRUE(found_expected) << "Expected stop line with id " << expected_stop_line_id
                              << " was not found.";
}
}  // namespace autoware

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
