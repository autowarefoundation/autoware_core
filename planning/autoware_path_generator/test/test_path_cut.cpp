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

#include "utils_test.hpp"

#include <lanelet2_core/geometry/Lanelet.h>

namespace autoware::path_generator
{
struct GetFirstIntersectionArcLengthTestParam
{
  std::string description;
  std::vector<lanelet::Id> lane_ids;
  double s_start;
  double s_end;
  std::optional<double> expected_s_intersection;
};

std::ostream & operator<<(std::ostream & os, const GetFirstIntersectionArcLengthTestParam & p)
{
  return os << p.description;
}

struct GetFirstIntersectionArcLengthTest
: public UtilsTest,
  public ::testing::WithParamInterface<GetFirstIntersectionArcLengthTestParam>
{
  void SetUp() override
  {
    UtilsTest::SetUp();
    set_map("autoware_test_utils", "overlap/lanelet2_map.osm");
  }
};

TEST_F(UtilsTest, getPathBound)
{
  set_map("autoware_test_utils", "2km_test.osm");
  constexpr auto epsilon = 1e-1;

  {  // lanelet sequence is empty
    const auto result = utils::get_path_bounds({}, {}, {}, {}, {});

    ASSERT_FALSE(result.has_value());
  }

  {  // normal case
    PathPointWithLaneId start, end;
    start.point.pose.position.x = -999.0;
    start.point.pose.position.y = 1.75;
    start.lane_ids = {4417};
    end.point.pose.position.x = -976.0;
    end.point.pose.position.y = 1.75;
    end.lane_ids = {4417};
    const auto result = utils::get_path_bounds(
      {start, end}, get_lanelets_from_ids({4417}), planner_data_.routing_graph_ptr, 0.0, 0.0);

    ASSERT_TRUE(result.has_value());
    ASSERT_GE(result->left.size(), 2);
    ASSERT_NEAR(result->left.front().x, -999.0, epsilon);
    ASSERT_NEAR(result->left.front().y, 3.5, epsilon);
    ASSERT_NEAR(result->left.back().x, -976.0, epsilon);
    ASSERT_NEAR(result->left.back().y, 3.5, epsilon);
    ASSERT_GE(result->right.size(), 2);
    ASSERT_NEAR(result->right.front().x, -999.0, epsilon);
    ASSERT_NEAR(result->right.front().y, 0, epsilon);
    ASSERT_NEAR(result->right.back().x, -976.0, epsilon);
    ASSERT_NEAR(result->right.back().y, 0, epsilon);
  }

  {  // normal case with multiple lanelets
    PathPointWithLaneId start, end;
    start.point.pose.position.x = -974.0;
    start.point.pose.position.y = 1.75;
    start.lane_ids = {4429};
    end.point.pose.position.x = -926.0;
    end.point.pose.position.y = 1.75;
    end.lane_ids = {4434};
    const auto result = utils::get_path_bounds(
      {start, end}, get_lanelets_from_ids({4429, 4434}), planner_data_.routing_graph_ptr, 0.0, 0.0);

    ASSERT_TRUE(result.has_value());
    ASSERT_GE(result->left.size(), 2);
    ASSERT_NEAR(result->left.front().x, -974.0, epsilon);
    ASSERT_NEAR(result->left.front().y, 3.5, epsilon);
    ASSERT_NEAR(result->left.back().x, -926.0, epsilon);
    ASSERT_NEAR(result->left.back().y, 3.5, epsilon);
    ASSERT_GE(result->right.size(), 2);
    ASSERT_NEAR(result->right.front().x, -974.0, epsilon);
    ASSERT_NEAR(result->right.front().y, 0.0, epsilon);
    ASSERT_NEAR(result->right.back().x, -926.0, epsilon);
    ASSERT_NEAR(result->right.back().y, 0.0, epsilon);
  }

  {  // normal case with offsets
    PathPointWithLaneId start, end;
    start.point.pose.position.x = -974.5;
    start.point.pose.position.y = 1.75;
    start.lane_ids = {4429};
    end.point.pose.position.x = -950.5;
    end.point.pose.position.y = 1.75;
    end.lane_ids = {4429};
    const auto result = utils::get_path_bounds(
      {start, end}, get_lanelets_from_ids({4429}), planner_data_.routing_graph_ptr, 1.0, 1.0);

    ASSERT_TRUE(result.has_value());
    ASSERT_GE(result->left.size(), 2);
    ASSERT_NEAR(result->left.front().x, -975.5, epsilon);
    ASSERT_NEAR(result->left.front().y, 3.5, epsilon);
    ASSERT_NEAR(result->left.back().x, -949.5, epsilon);
    ASSERT_NEAR(result->left.back().y, 3.5, epsilon);
    ASSERT_GE(result->right.size(), 2);
    ASSERT_NEAR(result->right.front().x, -975.5, epsilon);
    ASSERT_NEAR(result->right.front().y, 0, epsilon);
    ASSERT_NEAR(result->right.back().x, -949.5, epsilon);
    ASSERT_NEAR(result->right.back().y, 0, epsilon);
  }
}

TEST_F(UtilsTest, getLaneletSequenceCoveringPath)
{
  set_map("autoware_test_utils", "2km_test.osm");
  constexpr auto epsilon = 1e-1;

  {  // lanelet sequence is empty
    const auto result = utils::get_lanelet_sequence_covering_path({}, {PathPointWithLaneId{}}, {});

    ASSERT_FALSE(result.has_value());
  }

  {  // path points are empty
    const auto result =
      utils::get_lanelet_sequence_covering_path(get_lanelets_from_ids({4417}), {}, {});

    ASSERT_FALSE(result.has_value());
  }

  {  // normal case
    PathPointWithLaneId start, end;
    start.point.pose.position.x = -999.0;
    start.point.pose.position.y = 1.75;
    start.lane_ids = {4417};
    end.point.pose.position.x = -976.0;
    end.point.pose.position.y = 1.75;
    end.lane_ids = {4417};
    const auto result = utils::get_lanelet_sequence_covering_path(
      get_lanelets_from_ids({4417}), {start, end}, planner_data_.routing_graph_ptr);

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->lanelet_sequence.size(), 1);
    ASSERT_EQ(result->lanelet_sequence[0].id(), 4417);
    ASSERT_NEAR(result->s_start, 1.0, epsilon);
    ASSERT_NEAR(result->s_end, 24.0, epsilon);
  }

  {  // normal case with missing lanelet sequence
    PathPointWithLaneId start, end;
    start.point.pose.position.x = -974.0;
    start.point.pose.position.y = 1.75;
    start.lane_ids = {4429};
    end.point.pose.position.x = -926.0;
    end.point.pose.position.y = 1.75;
    end.lane_ids = {4434};
    const auto result = utils::get_lanelet_sequence_covering_path(
      get_lanelets_from_ids({4429}), {start, end}, planner_data_.routing_graph_ptr);

    ASSERT_TRUE(result.has_value());
    ASSERT_GE(result->lanelet_sequence.size(), 2);
    ASSERT_EQ(result->lanelet_sequence[0].id(), 4429);
    ASSERT_EQ(result->lanelet_sequence[1].id(), 4434);
    ASSERT_NEAR(result->s_start, 1.0, epsilon);
    ASSERT_NEAR(result->s_end, 49.0, epsilon);
  }

  {  // normal case with redundant lanelet sequence
    PathPointWithLaneId start, end;
    start.point.pose.position.x = -974.5;
    start.point.pose.position.y = 1.75;
    start.lane_ids = {4429};
    end.point.pose.position.x = -950.5;
    end.point.pose.position.y = 1.75;
    end.lane_ids = {4429};
    const auto result = utils::get_lanelet_sequence_covering_path(
      get_lanelets_from_ids({4417, 4429, 4434}), {start, end}, planner_data_.routing_graph_ptr);

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->lanelet_sequence.size(), 1);
    ASSERT_EQ(result->lanelet_sequence[0].id(), 4429);
    ASSERT_NEAR(result->s_start, 0.5, epsilon);
    ASSERT_NEAR(result->s_end, 24.5, epsilon);
  }
}

TEST_F(UtilsTest, cropLineString)
{
  constexpr auto epsilon = 1e-1;

  {  // line string is empty
    const auto result = utils::crop_line_string({}, {}, {});

    ASSERT_TRUE(result.empty());
  }

  {  // line string has only 1 point
    const auto result = utils::crop_line_string({{}}, {}, {});

    ASSERT_TRUE(result.empty());
  }

  {  // normal case
    const auto result = utils::crop_line_string(
      {lanelet::BasicPoint3d{0.0, 0.0, 0.0}, lanelet::BasicPoint3d{3.0, 0.0, 0.0}}, 1.0, 2.0);

    ASSERT_GE(result.size(), 2);
    ASSERT_NEAR(result.front().x, 1.0, epsilon);
    ASSERT_NEAR(result.front().y, 0.0, epsilon);
    ASSERT_NEAR(result.back().x, 2.0, epsilon);
    ASSERT_NEAR(result.back().y, 0.0, epsilon);
  }

  {  // start of crop range is negative
    const auto result = utils::crop_line_string(
      {lanelet::BasicPoint3d{0.0, 0.0, 0.0}, lanelet::BasicPoint3d{3.0, 0.0, 0.0}}, -1.0, 2.0);

    ASSERT_GE(result.size(), 2);
    ASSERT_NEAR(result.front().x, 0.0, epsilon);
    ASSERT_NEAR(result.front().y, 0.0, epsilon);
    ASSERT_NEAR(result.back().x, 3.0, epsilon);
    ASSERT_NEAR(result.back().y, 0.0, epsilon);
  }

  {  // end of crop range exceeds line string length
    const auto result = utils::crop_line_string(
      {lanelet::BasicPoint3d{0.0, 0.0, 0.0}, lanelet::BasicPoint3d{3.0, 0.0, 0.0}}, 1.0, 4.0);

    ASSERT_GE(result.size(), 2);
    ASSERT_NEAR(result.front().x, 1.0, epsilon);
    ASSERT_NEAR(result.front().y, 0.0, epsilon);
    ASSERT_NEAR(result.back().x, 3.0, epsilon);
    ASSERT_NEAR(result.back().y, 0.0, epsilon);
  }

  {  // start of crop range is larger than end
    const auto result = utils::crop_line_string(
      {lanelet::BasicPoint3d{0.0, 0.0, 0.0}, lanelet::BasicPoint3d{3.0, 0.0, 0.0}}, 2.0, 1.0);

    ASSERT_GE(result.size(), 2);
    ASSERT_NEAR(result.front().x, 0.0, epsilon);
    ASSERT_NEAR(result.front().y, 0.0, epsilon);
    ASSERT_NEAR(result.back().x, 3.0, epsilon);
    ASSERT_NEAR(result.back().y, 0.0, epsilon);
  }
}

TEST_F(UtilsTest, GetArcLengthOnBounds)
{
  const auto epsilon = 1e-1;

  {  // lanelet sequence is empty
    const auto result = utils::get_arc_length_on_bounds({}, {}, {});

    ASSERT_FALSE(result.has_value());
  }

  {  // normal case
    const auto result =
      utils::get_arc_length_on_bounds(get_lanelets_from_ids({50}), {3765.5, 73746.0}, 50);

    ASSERT_TRUE(result.has_value());
    ASSERT_NEAR(result->left, 10.672, epsilon);
    ASSERT_NEAR(result->right, 8.203, epsilon);
  }

  {  // normal case with multiple lanelets
    const auto result =
      utils::get_arc_length_on_bounds(get_lanelets_from_ids({50, 122}), {3758.0, 73754.0}, 122);

    ASSERT_TRUE(result.has_value());
    ASSERT_NEAR(result->left, 22.944, epsilon);
    ASSERT_NEAR(result->right, 18.266, epsilon);
  }

  {  // path point is outside lanelet sequence
    const auto result =
      utils::get_arc_length_on_bounds(get_lanelets_from_ids({50}), {3777.0, 73748.5}, 50);

    ASSERT_FALSE(result.has_value());
  }
}
}  // namespace autoware::path_generator
