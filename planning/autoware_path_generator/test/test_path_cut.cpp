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

#include <autoware/trajectory/interpolator/linear.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

namespace autoware::path_generator
{
namespace
{
using Trajectory = experimental::trajectory::Trajectory<PathPointWithLaneId>;

Trajectory create_path(const std::vector<std::pair<lanelet::Ids, lanelet::BasicPoint2d>> & points)
{
  std::vector<PathPointWithLaneId> path_points;
  path_points.reserve(points.size());
  for (const auto & [lane_ids, point] : points) {
    PathPointWithLaneId path_point;
    path_point.point.pose.position.x = point.x();
    path_point.point.pose.position.y = point.y();
    path_point.lane_ids = lane_ids;
    path_points.push_back(path_point);
  }
  return *Trajectory::Builder{}
            .set_xy_interpolator<autoware::experimental::trajectory::interpolator::Linear>()
            .build(path_points);
}
}  // namespace

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

TEST_P(GetFirstIntersectionArcLengthTest, getFirstIntersectionArcLength)
{
  const auto & p = GetParam();

  const auto result = utils::get_first_intersection_arc_length(
    get_lanelets_from_ids(p.lane_ids), p.s_start, p.s_end, vehicle_info_.vehicle_length_m);

  ASSERT_EQ(result.has_value(), p.expected_s_intersection.has_value());

  constexpr auto epsilon = 1e-1;
  if (p.expected_s_intersection.has_value()) {
    ASSERT_NEAR(*result, *p.expected_s_intersection, epsilon);
  }
}

INSTANTIATE_TEST_SUITE_P(
  , GetFirstIntersectionArcLengthTest,
  ::testing::Values(
    GetFirstIntersectionArcLengthTestParam{
      "UTurnWithGap", {601, 602, 600}, 0.0, std::numeric_limits<double>::max(), std::nullopt},
    GetFirstIntersectionArcLengthTestParam{
      "UTurnWithFullCrossing",
      {615, 616, 604, 605, 603, 618, 617},
      0.0,
      std::numeric_limits<double>::max(),
      194.477},
    GetFirstIntersectionArcLengthTestParam{
      "UTurnWithHalfCrossing",
      {619, 621, 607, 608, 606, 622, 620},
      0.0,
      std::numeric_limits<double>::max(),
      195.507},
    GetFirstIntersectionArcLengthTestParam{
      "Overpass", {609, 610, 612, 611, 613}, 0.0, std::numeric_limits<double>::max(), 311.068},
    GetFirstIntersectionArcLengthTestParam{
      "OverpassWithStartEdgeIntersection",
      {609, 610, 612, 611, 613},
      36.0,
      std::numeric_limits<double>::max(),
      325.376},
    GetFirstIntersectionArcLengthTestParam{
      "OverpassWithIntersectionBehind",
      {609, 610, 612, 611, 613},
      76.0,
      std::numeric_limits<double>::max(),
      std::nullopt},
    GetFirstIntersectionArcLengthTestParam{
      "OverpassWithIntersectionAhead", {609, 610, 612, 611, 613}, 0.0, 16.0, std::nullopt}),
  ::testing::PrintToStringParamName{});

TEST_F(UtilsTest, getFirstSelfIntersectionArcLength)
{
  {  // line string is empty
    const auto result = utils::get_first_self_intersection_arc_length(lanelet::BasicLineString2d{});

    ASSERT_FALSE(result);
  }
}

TEST_F(UtilsTest, getFirstMutualIntersectionArcLength)
{
  const lanelet::LaneletSequence dummy_lanelet_sequence(get_lanelets_from_ids({122}));
  const auto dummy_left_bound = dummy_lanelet_sequence.leftBound2d().basicLineString();
  const auto dummy_right_bound = dummy_lanelet_sequence.rightBound2d().basicLineString();

  {  // lanelet sequence is empty
    const auto result =
      utils::get_first_mutual_intersection_arc_length({}, dummy_left_bound, dummy_right_bound, {});

    ASSERT_FALSE(result);
  }

  {  // left bound is empty
    const auto result = utils::get_first_mutual_intersection_arc_length(
      dummy_lanelet_sequence, {}, dummy_right_bound, {});

    ASSERT_FALSE(result);
  }

  {  // right bound is empty
    const auto result = utils::get_first_mutual_intersection_arc_length(
      dummy_lanelet_sequence, dummy_left_bound, {}, {});

    ASSERT_FALSE(result);
  }
}

TEST_F(UtilsTest, getFirstStartEdgeBoundIntersectionArcLength)
{
  constexpr auto to_geometry_msgs_points = [](const auto & line_string) {
    std::vector<geometry_msgs::msg::Point> geometry_msgs_points{};
    geometry_msgs_points.reserve(line_string.size());
    std::transform(
      line_string.begin(), line_string.end(), std::back_inserter(geometry_msgs_points),
      [](const auto & point) { return lanelet::utils::conversion::toGeomMsgPt(point); });
    return geometry_msgs_points;
  };

  const lanelet::LaneletSequence dummy_lanelet_sequence(get_lanelets_from_ids({122}));
  const auto dummy_left_bound =
    *autoware::experimental::trajectory::Trajectory<geometry_msgs::msg::Point>::Builder{}.build(
      to_geometry_msgs_points(dummy_lanelet_sequence.leftBound()));
  const auto dummy_right_bound =
    *autoware::experimental::trajectory::Trajectory<geometry_msgs::msg::Point>::Builder{}.build(
      to_geometry_msgs_points(dummy_lanelet_sequence.rightBound()));

  {  // lanelet sequence is empty
    const auto result = utils::get_first_start_edge_bound_intersection_arc_length(
      {}, {{}, {}}, dummy_left_bound, dummy_right_bound, {}, {});

    ASSERT_FALSE(result);
  }

  {  // start edge is empty
    const auto result = utils::get_first_start_edge_bound_intersection_arc_length(
      dummy_lanelet_sequence, {}, dummy_left_bound, dummy_right_bound, {}, {});

    ASSERT_FALSE(result);
  }
}

TEST_F(UtilsTest, getFirstStartEdgeCenterlineIntersectionArcLength)
{
  const lanelet::LaneletSequence dummy_lanelet_sequence(get_lanelets_from_ids({122}));

  {  // lanelet sequence is empty
    const auto result =
      utils::get_first_start_edge_centerline_intersection_arc_length({}, {{}, {}}, {}, {}, {});

    ASSERT_FALSE(result);
  }

  {  // start edge is empty
    const auto result = utils::get_first_start_edge_centerline_intersection_arc_length(
      dummy_lanelet_sequence, {}, 0.0, std::numeric_limits<double>::max(), {});

    ASSERT_FALSE(result);
  }
}

TEST_F(UtilsTest, GetArcLengthOnPath)
{
  constexpr auto epsilon = 1e-1;

  const auto path =
    create_path({{{55, 122}, {3757.5609, 73751.8479}}, {{122}, {3752.1707, 73762.1772}}});

  {  // lanelet sequence is empty
    const auto result = utils::get_arc_length_on_path({}, path, {});

    ASSERT_NEAR(result, 0.0, epsilon);
  }

  {  // normal case
    const auto result = utils::get_arc_length_on_path(get_lanelets_from_ids({122}), path, 10.0);

    ASSERT_NEAR(result, 10.0, epsilon);
  }

  {  // input arc length is negative
    const auto result = utils::get_arc_length_on_path(get_lanelets_from_ids({122}), path, -10.0);

    ASSERT_NEAR(result, 0.0, epsilon);
  }

  {  // input arc length exceeds lanelet length
    const auto result = utils::get_arc_length_on_path(get_lanelets_from_ids({122}), path, 100.0);

    ASSERT_NEAR(result, 100.0, epsilon);
  }
}

TEST_F(UtilsTest, getPathBound)
{
  set_map("autoware_test_utils", "2km_test.osm");
  constexpr auto epsilon = 1e-1;

  {  // lanelet sequence is empty
    const auto [left, right] = utils::get_path_bounds(get_lanelets_from_ids({}), {}, {});

    ASSERT_TRUE(left.empty());
    ASSERT_TRUE(right.empty());
  }

  {  // normal case
    const auto [left, right] = utils::get_path_bounds(get_lanelets_from_ids({4417}), 1.0, 24.0);

    ASSERT_GE(left.size(), 2);
    ASSERT_NEAR(left.front().x, -999.0, epsilon);
    ASSERT_NEAR(left.front().y, 3.5, epsilon);
    ASSERT_NEAR(left.back().x, -976.0, epsilon);
    ASSERT_NEAR(left.back().y, 3.5, epsilon);
    ASSERT_GE(right.size(), 2);
    ASSERT_NEAR(right.front().x, -999.0, epsilon);
    ASSERT_NEAR(right.front().y, 0, epsilon);
    ASSERT_NEAR(right.back().x, -976.0, epsilon);
    ASSERT_NEAR(right.back().y, 0, epsilon);
  }

  {  // normal case with multiple lanelets
    const auto [left, right] =
      utils::get_path_bounds(get_lanelets_from_ids({4429, 4434}), 1.0, 49.0);

    ASSERT_GE(left.size(), 2);
    ASSERT_NEAR(left.front().x, -974.0, epsilon);
    ASSERT_NEAR(left.front().y, 3.5, epsilon);
    ASSERT_NEAR(left.back().x, -926.0, epsilon);
    ASSERT_NEAR(left.back().y, 3.5, epsilon);
    ASSERT_GE(right.size(), 2);
    ASSERT_NEAR(right.front().x, -974.0, epsilon);
    ASSERT_NEAR(right.front().y, 0.0, epsilon);
    ASSERT_NEAR(right.back().x, -926.0, epsilon);
    ASSERT_NEAR(right.back().y, 0.0, epsilon);
  }

  {  // start of bound is negative
    const auto [left, right] = utils::get_path_bounds(get_lanelets_from_ids({4417}), -1.0, 24.0);

    ASSERT_GE(left.size(), 2);
    ASSERT_NEAR(left.front().x, -1000.0, epsilon);
    ASSERT_NEAR(left.front().y, 3.5, epsilon);
    ASSERT_NEAR(left.back().x, -976.0, epsilon);
    ASSERT_NEAR(left.back().y, 3.5, epsilon);
    ASSERT_GE(right.size(), 2);
    ASSERT_NEAR(right.front().x, -1000.0, epsilon);
    ASSERT_NEAR(right.front().y, 0, epsilon);
    ASSERT_NEAR(right.back().x, -976.0, epsilon);
    ASSERT_NEAR(right.back().y, 0, epsilon);
  }

  {  // end of bound exceeds lanelet length
    const auto [left, right] = utils::get_path_bounds(get_lanelets_from_ids({4417}), 1.0, 26.0);

    ASSERT_GE(left.size(), 2);
    ASSERT_NEAR(left.front().x, -999.0, epsilon);
    ASSERT_NEAR(left.front().y, 3.5, epsilon);
    ASSERT_NEAR(left.back().x, -975.0, epsilon);
    ASSERT_NEAR(left.back().y, 3.5, epsilon);
    ASSERT_GE(right.size(), 2);
    ASSERT_NEAR(right.front().x, -999.0, epsilon);
    ASSERT_NEAR(right.front().y, 0, epsilon);
    ASSERT_NEAR(right.back().x, -975.0, epsilon);
    ASSERT_NEAR(right.back().y, 0, epsilon);
  }

  {  // start of bound is larger than end
    const auto [left, right] =
      utils::get_path_bounds(get_lanelets_from_ids({4429, 4434}), 30.0, 20.0);

    ASSERT_TRUE(left.empty());
    ASSERT_TRUE(right.empty());
  }
}

TEST_F(UtilsTest, buildCroppedTrajectory)
{
  constexpr auto epsilon = 1e-1;

  {  // line string is empty
    const auto result = utils::build_cropped_trajectory({}, {}, {});

    ASSERT_FALSE(result);
  }

  {  // line string has only 1 point
    const auto result = utils::build_cropped_trajectory({geometry_msgs::msg::Point{}}, {}, {});

    ASSERT_FALSE(result);
  }

  {  // normal case
    const auto result = utils::build_cropped_trajectory(
      {lanelet::utils::conversion::toGeomMsgPt(lanelet::BasicPoint3d{0.0, 0.0, 0.0}),
       lanelet::utils::conversion::toGeomMsgPt(lanelet::BasicPoint3d{3.0, 0.0, 0.0})},
      1.0, 2.0);

    ASSERT_TRUE(result);

    const auto start = result->compute(0);
    ASSERT_NEAR(start.x, 1.0, epsilon);
    ASSERT_NEAR(start.y, 0.0, epsilon);
    const auto end = result->compute(result->length());
    ASSERT_NEAR(end.x, 2.0, epsilon);
    ASSERT_NEAR(end.y, 0.0, epsilon);
  }

  {  // start of crop range is negative
    const auto result = utils::build_cropped_trajectory(
      {lanelet::utils::conversion::toGeomMsgPt(lanelet::BasicPoint3d{0.0, 0.0, 0.0}),
       lanelet::utils::conversion::toGeomMsgPt(lanelet::BasicPoint3d{3.0, 0.0, 0.0})},
      -1.0, 2.0);

    ASSERT_FALSE(result);
  }

  {  // end of crop range exceeds line string length
    const auto result = utils::build_cropped_trajectory(
      {lanelet::utils::conversion::toGeomMsgPt(lanelet::BasicPoint3d{0.0, 0.0, 0.0}),
       lanelet::utils::conversion::toGeomMsgPt(lanelet::BasicPoint3d{3.0, 0.0, 0.0})},
      1.0, 4.0);

    ASSERT_TRUE(result);

    const auto start = result->compute(0);
    ASSERT_NEAR(start.x, 1.0, epsilon);
    ASSERT_NEAR(start.y, 0.0, epsilon);
    const auto end = result->compute(result->length());
    ASSERT_NEAR(end.x, 3.0, epsilon);
    ASSERT_NEAR(end.y, 0.0, epsilon);
  }

  {  // start of crop range is larger than end
    const auto result = utils::build_cropped_trajectory(
      {lanelet::utils::conversion::toGeomMsgPt(lanelet::BasicPoint3d{0.0, 0.0, 0.0}),
       lanelet::utils::conversion::toGeomMsgPt(lanelet::BasicPoint3d{3.0, 0.0, 0.0})},
      2.0, 1.0);

    ASSERT_FALSE(result);
  }
}

TEST_F(UtilsTest, GetArcLengthOnBounds)
{
  const auto epsilon = 1e-1;

  {  // lanelet sequence is empty
    const auto [left, right] = utils::get_arc_length_on_bounds({}, {});

    ASSERT_NEAR(left, {}, epsilon);
    ASSERT_NEAR(right, {}, epsilon);
  }

  {  // normal case
    const auto [left, right] = utils::get_arc_length_on_bounds(get_lanelets_from_ids({50}), 10.0);

    ASSERT_NEAR(left, 11.293, epsilon);
    ASSERT_NEAR(right, 8.823, epsilon);
  }

  {  // input arc length is negative
    const auto [left, right] = utils::get_arc_length_on_bounds(get_lanelets_from_ids({50}), -10.0);

    ASSERT_NEAR(left, 0.0, epsilon);
    ASSERT_NEAR(right, 0.0, epsilon);
  }

  {  // input arc length exceeds lanelet length
    const auto [left, right] = utils::get_arc_length_on_bounds(get_lanelets_from_ids({50}), 100.0);

    ASSERT_NEAR(left, 100.0, epsilon);
    ASSERT_NEAR(right, 100.0, epsilon);
  }
}

TEST_F(UtilsTest, GetArcLengthOnCenterline)
{
  const auto epsilon = 1e-1;

  {  // lanelet sequence is empty
    const auto [left, right] = utils::get_arc_length_on_centerline({}, {{}}, {{}});

    ASSERT_TRUE(left.has_value());
    ASSERT_NEAR(*left, {}, epsilon);
    ASSERT_TRUE(right.has_value());
    ASSERT_NEAR(*right, {}, epsilon);
  }

  {  // normal case
    const auto [left, right] =
      utils::get_arc_length_on_centerline(get_lanelets_from_ids({50}), 11.293, 8.823);

    ASSERT_TRUE(left.has_value());
    ASSERT_NEAR(*left, 10.0, epsilon);
    ASSERT_TRUE(right.has_value());
    ASSERT_NEAR(*right, 10.0, epsilon);
  }

  {  // input arc length is negative
    const auto [left, right] =
      utils::get_arc_length_on_centerline(get_lanelets_from_ids({50}), -10, -10);

    ASSERT_TRUE(left.has_value());
    ASSERT_NEAR(*left, 0.0, epsilon);
    ASSERT_TRUE(right.has_value());
    ASSERT_NEAR(*right, 0.0, epsilon);
  }

  {  // input arc length exceeds lanelet length
    const auto [left, right] =
      utils::get_arc_length_on_centerline(get_lanelets_from_ids({50}), 100.0, 100.0);

    ASSERT_TRUE(left.has_value());
    ASSERT_NEAR(*left, 100.0, epsilon);
    ASSERT_TRUE(right.has_value());
    ASSERT_NEAR(*right, 100.0, epsilon);
  }

  {  // input arc length is null
    const auto [left, right] =
      utils::get_arc_length_on_centerline(get_lanelets_from_ids({50}), std::nullopt, std::nullopt);

    ASSERT_FALSE(left.has_value());
    ASSERT_FALSE(right.has_value());
  }
}
}  // namespace autoware::path_generator
