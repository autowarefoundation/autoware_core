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

#include <Eigen/Core>
#include <autoware/lanelet2_utils/geometry.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>

#include <filesystem>
#include <string>

namespace autoware
{

TEST(ExtrapolatedPointTest, ForwardExtrapolation)
{
  lanelet::ConstPoint3d p1(1, 0.0, 0.0, 0.0);
  lanelet::ConstPoint3d p2(2, 10.0, 0.0, 0.0);

  double distance = 5.0;
  auto point = lanelet2_utils::extrapolate_point(p1, p2, distance, true);

  EXPECT_NEAR(point.x(), 15.0, 1e-6);
  EXPECT_NEAR(point.y(), 0.0, 1e-6);
  EXPECT_NEAR(point.z(), 0.0, 1e-6);
}

TEST(ExtrapolatedPointTest, BackwardExtrapolation)
{
  lanelet::ConstPoint3d p1(1, 0.0, 0.0, 0.0);
  lanelet::ConstPoint3d p2(2, 10.0, 0.0, 0.0);

  double distance = 5.0;
  auto point = lanelet2_utils::extrapolate_point(p1, p2, distance, false);

  EXPECT_NEAR(point.x(), -5.0, 1e-6);
  EXPECT_NEAR(point.y(), 0.0, 1e-6);
  EXPECT_NEAR(point.z(), 0.0, 1e-6);
}

TEST(ExtrapolatedPointTest, ZeroDistanceReturnsOrigin)
{
  lanelet::ConstPoint3d p1(1, 1.0, 2.0, 3.0);
  lanelet::ConstPoint3d p2(2, 4.0, 5.0, 6.0);
  double distance = 0.0;
  auto point = lanelet2_utils::extrapolate_point(p1, p2, distance, false);

  EXPECT_NEAR(point.x(), p1.x(), 1e-6);
  EXPECT_NEAR(point.y(), p1.y(), 1e-6);
  EXPECT_NEAR(point.z(), p1.z(), 1e-6);
}

TEST(ExtrapolatedPointTest, ExtrapolateDiagonalLine)
{
  lanelet::ConstPoint3d p1(1, 1.0, 2.0, 3.0);
  lanelet::ConstPoint3d p2(2, 4.0, 5.0, 6.0);
  double distance = 2.0;
  auto point = lanelet2_utils::extrapolate_point(p1, p2, distance, true);

  EXPECT_NEAR(point.x(), 5.1547005383792515, 1e-6);
  EXPECT_NEAR(point.y(), 6.1547005383792515, 1e-6);
  EXPECT_NEAR(point.z(), 7.1547005383792515, 1e-6);
}

TEST(InterpolatePointTest, MidPointInterpolation)
{
  lanelet::ConstPoint3d p1(1, 0.0, 0.0, 0.0);
  lanelet::ConstPoint3d p2(2, 10.0, 0.0, 0.0);

  double distance = 5.0;
  auto point_from_first = lanelet2_utils::interpolate_point(p1, p2, distance, true);
  auto point_from_second = lanelet2_utils::interpolate_point(p1, p2, distance, false);

  EXPECT_NEAR(point_from_first.x(), 5.0, 1e-6);
  EXPECT_NEAR(point_from_first.y(), 0.0, 1e-6);
  EXPECT_NEAR(point_from_first.z(), 0.0, 1e-6);

  EXPECT_NEAR(point_from_second.x(), 5.0, 1e-6);
  EXPECT_NEAR(point_from_second.y(), 0.0, 1e-6);
  EXPECT_NEAR(point_from_second.z(), 0.0, 1e-6);
}

TEST(InterpolatePointTest, ZeroDistanceReturnsFirst)
{
  lanelet::ConstPoint3d p1(1, 1.0, 2.0, 3.0);
  lanelet::ConstPoint3d p2(2, 4.0, 5.0, 6.0);
  double distance = 0.0;
  auto pt_from_first = lanelet2_utils::interpolate_point(p1, p2, distance, true);
  auto pt_from_second = lanelet2_utils::interpolate_point(p1, p2, distance, false);

  EXPECT_NEAR(pt_from_first.x(), p1.x(), 1e-6);
  EXPECT_NEAR(pt_from_first.y(), p1.y(), 1e-6);
  EXPECT_NEAR(pt_from_first.z(), p1.z(), 1e-6);

  EXPECT_NEAR(pt_from_second.x(), p2.x(), 1e-6);
  EXPECT_NEAR(pt_from_second.y(), p2.y(), 1e-6);
  EXPECT_NEAR(pt_from_second.z(), p2.z(), 1e-6);
}

TEST(InterpolatePointTest, AtSegmentEnd)
{
  lanelet::ConstPoint3d p1(1, 1.0, 2.0, 3.0);
  lanelet::ConstPoint3d p2(2, 4.0, 5.0, 6.0);

  double segment_length = std::hypot(4.0 - 1.0, 5.0 - 2.0, 6.0 - 3.0);

  auto pt_from_first = lanelet2_utils::interpolate_point(p1, p2, segment_length, true);
  auto pt_from_second = lanelet2_utils::interpolate_point(p1, p2, segment_length, false);

  EXPECT_NEAR(pt_from_first.x(), p2.x(), 1e-6);
  EXPECT_NEAR(pt_from_first.y(), p2.y(), 1e-6);
  EXPECT_NEAR(pt_from_first.z(), p2.z(), 1e-6);

  EXPECT_NEAR(pt_from_second.x(), p1.x(), 1e-6);
  EXPECT_NEAR(pt_from_second.y(), p1.y(), 1e-6);
  EXPECT_NEAR(pt_from_second.z(), p1.z(), 1e-6);
}

TEST(InterpolatePointTest, OutOfBoundsDistanceReturnsFirst)
{
  lanelet::ConstPoint3d p1(1, 1.0, 2.0, 3.0);
  lanelet::ConstPoint3d p2(2, 4.0, 5.0, 6.0);

  double segment_length = std::hypot(4.0 - 1.0, 5.0 - 2.0, 6.0 - 3.0);

  double distance_positive = segment_length + 1.0;
  auto pt_positive = lanelet2_utils::interpolate_point(p1, p2, distance_positive, true);

  double distance_negative = -1.0;
  auto pt_negative = lanelet2_utils::interpolate_point(p1, p2, distance_negative, true);

  EXPECT_NEAR(pt_positive.x(), p1.x(), 1e-6);
  EXPECT_NEAR(pt_positive.y(), p1.y(), 1e-6);
  EXPECT_NEAR(pt_positive.z(), p1.z(), 1e-6);

  EXPECT_NEAR(pt_negative.x(), p1.x(), 1e-6);
  EXPECT_NEAR(pt_negative.y(), p1.y(), 1e-6);
  EXPECT_NEAR(pt_negative.z(), p1.z(), 1e-6);
}

TEST(InterpolatePointTest, DiagonalInterpolation)
{
  lanelet::ConstPoint3d p1(1, 1.0, 2.0, 3.0);
  lanelet::ConstPoint3d p2(2, 4.0, 6.0, 8.0);

  double distance = 2.0;
  auto point_from_first = lanelet2_utils::interpolate_point(p1, p2, distance, true);
  auto point_from_second = lanelet2_utils::interpolate_point(p1, p2, distance, false);

  EXPECT_NEAR(point_from_first.x(), 1.848528, 1e-5);
  EXPECT_NEAR(point_from_first.y(), 3.131370, 1e-5);
  EXPECT_NEAR(point_from_first.z(), 4.414213, 1e-5);

  EXPECT_NEAR(point_from_second.x(), 3.151471, 1e-5);
  EXPECT_NEAR(point_from_second.y(), 4.868629, 1e-5);
  EXPECT_NEAR(point_from_second.z(), 6.585786, 1e-5);
}

TEST(LineStringTest, InterpolateLineStringFromFirst)
{
  lanelet::ConstPoint3d p1(1, 0.0, 0.0, 0.0);
  lanelet::ConstPoint3d p2(2, 10.0, 0.0, 0.0);
  lanelet::ConstPoint3d p3(3, 20.0, 0.0, 0.0);

  std::vector<lanelet::Point3d> pts = {
    lanelet::Point3d(p1), lanelet::Point3d(p2), lanelet::Point3d(p3)};
  lanelet::ConstLineString3d line(lanelet::InvalId, pts);

  double s = 15.0;
  bool from_first = true;
  lanelet::ConstLineString3d interp_line =
    lanelet2_utils::interpolate_linestring(line, s, from_first);

  ASSERT_EQ(interp_line.size(), 4U);
  EXPECT_NEAR(interp_line[2].x(), 15.0, 1e-6);
  EXPECT_NEAR(interp_line[2].y(), 0.0, 1e-6);
  EXPECT_NEAR(interp_line[2].z(), 0.0, 1e-6);
}

TEST(LineStringTest, InterpolateLineStringFromLast)
{
  lanelet::ConstPoint3d p1(1, 0.0, 0.0, 0.0);
  lanelet::ConstPoint3d p2(2, 10.0, 0.0, 0.0);
  lanelet::ConstPoint3d p3(3, 20.0, 0.0, 0.0);

  std::vector<lanelet::Point3d> pts = {
    lanelet::Point3d(p1), lanelet::Point3d(p2), lanelet::Point3d(p3)};
  lanelet::ConstLineString3d line(lanelet::InvalId, pts);

  double s = 5.0;
  bool from_first = false;
  lanelet::ConstLineString3d interp_line =
    lanelet2_utils::interpolate_linestring(line, s, from_first);

  ASSERT_EQ(interp_line.size(), 4U);
  EXPECT_NEAR(interp_line[2].x(), 15.0, 1e-6);
  EXPECT_NEAR(interp_line[2].y(), 0.0, 1e-6);
  EXPECT_NEAR(interp_line[2].z(), 0.0, 1e-6);
}

TEST(LineStringTest, ExtrapolateLineStringForward)
{
  lanelet::ConstPoint3d p1(1, 0.0, 0.0, 0.0);
  lanelet::ConstPoint3d p2(2, 10.0, 0.0, 0.0);
  lanelet::ConstPoint3d p3(3, 20.0, 0.0, 0.0);

  std::vector<lanelet::Point3d> pts = {
    lanelet::Point3d(p1), lanelet::Point3d(p2), lanelet::Point3d(p3)};
  lanelet::ConstLineString3d line(lanelet::InvalId, pts);

  double dist = 5.0;
  bool is_forward = false;
  lanelet::ConstLineString3d extra_line =
    lanelet2_utils::extrapolate_linestring(line, dist, is_forward);

  ASSERT_EQ(extra_line.size(), 4U);
  EXPECT_NEAR(extra_line[0].x(), 0.0, 1e-6);
  EXPECT_NEAR(extra_line[1].x(), 10.0, 1e-6);
  EXPECT_NEAR(extra_line[2].x(), 20.0, 1e-6);
  EXPECT_NEAR(extra_line[3].x(), 25.0, 1e-6);
}

TEST(LineStringTest, ExtrapolateLineStringBackward)
{
  // Create the same line string.
  lanelet::ConstPoint3d p1(1, 0.0, 0.0, 0.0);
  lanelet::ConstPoint3d p2(2, 10.0, 0.0, 0.0);
  lanelet::ConstPoint3d p3(3, 20.0, 0.0, 0.0);

  std::vector<lanelet::Point3d> pts = {
    lanelet::Point3d(p1), lanelet::Point3d(p2), lanelet::Point3d(p3)};
  lanelet::ConstLineString3d line(lanelet::InvalId, pts);

  double dist = 5.0;
  bool is_forward = true;
  lanelet::ConstLineString3d extra_line =
    lanelet2_utils::extrapolate_linestring(line, dist, is_forward);

  ASSERT_EQ(extra_line.size(), 4U);
  EXPECT_NEAR(extra_line[0].x(), -5.0, 1e-6);
  EXPECT_NEAR(extra_line[1].x(), 0.0, 1e-6);
  EXPECT_NEAR(extra_line[2].x(), 10.0, 1e-6);
  EXPECT_NEAR(extra_line[3].x(), 20.0, 1e-6);
}

}  // namespace autoware

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
