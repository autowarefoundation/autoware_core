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
}  // namespace autoware::path_generator
