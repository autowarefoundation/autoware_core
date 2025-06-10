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
#include <autoware/lanelet2_utils/stop_line.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace autoware::experimental
{
class TestGetStopLineFromMap : public ::testing::Test
{
protected:
  lanelet::LaneletMapConstPtr lanelet_map_ptr_{nullptr};

  void SetUp() override
  {
    const auto sample_map_dir =
      fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
      "sample_map";
    const auto regulatory_elements_map_path =
      sample_map_dir / "intersection" / "regulatory_elements.osm";

    lanelet_map_ptr_ =
      lanelet2_utils::load_mgrs_coordinate_map(regulatory_elements_map_path.string());
  }
};

// Test 1: deprecated crosswalk stop line retrieval.
TEST_F(TestGetStopLineFromMap, GetStopLineFromDeprecatedCrosswalk)
{
  const lanelet::ConstLanelets lanelets(
    lanelet_map_ptr_->laneletLayer.begin(), lanelet_map_ptr_->laneletLayer.end());
  std::optional<lanelet::ConstLineString3d> stopline_opt;
  for (const auto & llt : lanelet_map_ptr_->laneletLayer) {
    stopline_opt = experimental::lanelet2_utils::get_stop_line_from_deprecated_crosswalk(llt);
    if (stopline_opt) break;
  }

  ASSERT_TRUE(stopline_opt.has_value()) << "No stop line found for RoadMarking without target_id.";
  std::cout << "Stop line ID: " << stopline_opt->id() << std::endl;
  ASSERT_EQ(stopline_opt->id(), 198) << "Unexpected stop line ID for deprecated crosswalk.";
}

// Test 2: deprecated crosswalk Empty container no stop line
TEST_F(TestGetStopLineFromMap, EmptyLaneletContainer_DeprecatedCrosswalk)
{
  lanelet::ConstLanelet empty_lanelet;
  auto stopline_opt =
    experimental::lanelet2_utils::get_stop_line_from_deprecated_crosswalk(empty_lanelet);
  EXPECT_FALSE(stopline_opt.has_value())
    << "Stop line found in empty lanelet container for deprecated crosswalk.";
}

// Test 3: get_stop_lines_from_no_stopping_area from map.
TEST_F(TestGetStopLineFromMap, GetStopLineFromNoStoppingArea_NoTarget)
{
  const lanelet::ConstLanelets lanelets(
    lanelet_map_ptr_->laneletLayer.begin(), lanelet_map_ptr_->laneletLayer.end());
  std::optional<lanelet::ConstLineString3d> stopline_opt;
  for (const auto & llt : lanelet_map_ptr_->laneletLayer) {
    stopline_opt = experimental::lanelet2_utils::get_stop_lines_from_no_stopping_area(llt);
    if (stopline_opt) break;
  }
  ASSERT_TRUE(stopline_opt.has_value())
    << "No stop line found for NoStoppingArea without target_id.";
  lanelet::Id expected_stop_line_id = 208;
  EXPECT_EQ(stopline_opt->id(), expected_stop_line_id)
    << "Expected stop line ID (" << expected_stop_line_id << ") not found in NoStoppingArea.";
}

// Test 4: get_stop_lines_from_no_stopping_area empty lanelet container no stop line
TEST_F(TestGetStopLineFromMap, EmptyLaneletContainer_NoStoppingArea)
{
  lanelet::ConstLanelet empty_lanelet;
  auto stopline_opt =
    experimental::lanelet2_utils::get_stop_lines_from_no_stopping_area(empty_lanelet);
  EXPECT_FALSE(stopline_opt.has_value())
    << "Stop line found in empty lanelet container for NoStoppingArea.";
}

// Test 5: get_stop_lines_from_detection_area from map.
TEST_F(TestGetStopLineFromMap, GetStopLineFromDetectionArea_NoTarget)
{
  const lanelet::ConstLanelets lanelets(
    lanelet_map_ptr_->laneletLayer.begin(), lanelet_map_ptr_->laneletLayer.end());

  std::optional<lanelet::ConstLineString3d> stopline_opt;
  for (const auto & llt : lanelet_map_ptr_->laneletLayer) {
    stopline_opt = experimental::lanelet2_utils::get_stop_lines_from_detection_area(llt);
    if (stopline_opt) break;
  }
  ASSERT_TRUE(stopline_opt.has_value())
    << "No stop line found for DetectionArea without target_id.";
  lanelet::Id expected_stop_line_id = 209;
  EXPECT_EQ(stopline_opt->id(), expected_stop_line_id)
    << "Expected stop line ID (" << expected_stop_line_id << ") not found in DetectionArea.";
}

// Test 6: get_stop_lines_from_detection_area empty lanelet container stop line
TEST_F(TestGetStopLineFromMap, EmptyLaneletContainer_DetectionArea)
{
  lanelet::ConstLanelet empty_lanelet;
  auto stopline_opt =
    experimental::lanelet2_utils::get_stop_lines_from_detection_area(empty_lanelet);
  EXPECT_FALSE(stopline_opt.has_value())
    << "Stop line found in empty lanelet container for DetectionArea.";
}

// Test 7: get_stop_line_from_intersection_marking from map.
TEST_F(TestGetStopLineFromMap, GetStopLineFromIntersectionMarking_NoTarget)
{
  const lanelet::ConstLanelets lanelets(
    lanelet_map_ptr_->laneletLayer.begin(), lanelet_map_ptr_->laneletLayer.end());

  std::optional<lanelet::ConstLineString3d> stopline_opt;
  for (const auto & llt : lanelet_map_ptr_->laneletLayer) {
    stopline_opt = experimental::lanelet2_utils::get_stop_line_from_intersection_marking(llt);
    if (stopline_opt) break;
  }

  ASSERT_TRUE(stopline_opt.has_value())
    << "No stop line found for IntersectionMarking without target_id.";
  lanelet::Id expected_stop_line_id = 198;
  EXPECT_EQ(stopline_opt->id(), expected_stop_line_id)
    << "Expected stop line ID (" << expected_stop_line_id << ") not found in IntersectionMarking.";
}

// Test 8: get_stop_lines_from_stop_sign from map.
TEST_F(TestGetStopLineFromMap, GetStopLineFromStopSign_NoTarget)
{
  const lanelet::ConstLanelets lanelets(
    lanelet_map_ptr_->laneletLayer.begin(), lanelet_map_ptr_->laneletLayer.end());

  std::optional<lanelet::ConstLineString3d> stopline_opt;
  for (const auto & llt : lanelet_map_ptr_->laneletLayer) {
    stopline_opt = experimental::lanelet2_utils::get_stop_lines_from_stop_sign(llt);
    if (stopline_opt) break;
  }

  ASSERT_TRUE(stopline_opt.has_value())
    << "No stop line found for get_stop_lines_from_stop_sign without target_id.";
  lanelet::Id expected_stop_line_id = 206;
  EXPECT_EQ(stopline_opt->id(), expected_stop_line_id)
    << "Expected stop line ID (" << expected_stop_line_id
    << ") not found for get_stop_lines_from_stop_sign without target_id.";
}
}  // namespace autoware::experimental

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
