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

#include "autoware/lanelet2_utils/geometry.hpp"

#include "autoware/lanelet2_utils/conversion.hpp"

#include <Eigen/Core>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <boost/geometry.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>

#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace autoware::experimental
{

class ExtrapolatedLaneletTest : public ::testing::Test
{
protected:
  lanelet::LaneletMapConstPtr lanelet_map_ptr_{nullptr};

  void SetUp() override
  {
    const auto sample_map_dir =
      fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
      "sample_map";
    const auto intersection_crossing_map_path = sample_map_dir / "intersection" / "crossing.osm";

    lanelet_map_ptr_ =
      lanelet2_utils::load_mgrs_coordinate_map(intersection_crossing_map_path.string());
  }
};

// Test 1: forward extrapolation
TEST(ExtrapolatedPointTest, ForwardExtrapolation)
{
  lanelet::ConstPoint3d p1(1, 0.0, 0.0, 0.0);
  lanelet::ConstPoint3d p2(2, 10.0, 0.0, 0.0);
  double distance = 5.0;

  auto interpolated_pt = lanelet2_utils::extrapolate_point(p1, p2, distance);

  EXPECT_NEAR(interpolated_pt.x(), 15.0, 1e-4);
  EXPECT_NEAR(interpolated_pt.y(), 0.0, 1e-4);
  EXPECT_NEAR(interpolated_pt.z(), 0.0, 1e-4);
}

// Test 2: Zero distance extrapolation
TEST(ExtrapolatedPointTest, ZeroDistanceReturnsOrigin)
{
  lanelet::ConstPoint3d p1(1, 1.0, 2.0, 3.0);
  lanelet::ConstPoint3d p2(2, 4.0, 5.0, 6.0);
  double distance = 0.0;

  auto interpolated_pt = lanelet2_utils::extrapolate_point(p1, p2, distance);

  EXPECT_NEAR(interpolated_pt.x(), p2.x(), 1e-4);
  EXPECT_NEAR(interpolated_pt.y(), p2.y(), 1e-4);
  EXPECT_NEAR(interpolated_pt.z(), p2.z(), 1e-4);
}

// Test 3: Zero distance interpolation
TEST(InterpolatePointTest, ZeroDistanceReturnsFirstOrLast)
{
  lanelet::ConstPoint3d p1(1, 1.0, 2.0, 3.0);
  lanelet::ConstPoint3d p2(2, 4.0, 5.0, 6.0);
  double distance = 0.0;

  auto interpolated_pt_first = lanelet2_utils::interpolate_point(p1, p2, distance);
  ASSERT_TRUE(interpolated_pt_first.has_value());
  EXPECT_NEAR(interpolated_pt_first->x(), p1.x(), 1e-4);
}

// Test 4: Interpolation at exact segment length
TEST(InterpolatePointTest, AtSegmentEnd)
{
  lanelet::ConstPoint3d p1(1, 1.0, 2.0, 3.0);
  lanelet::ConstPoint3d p2(2, 4.0, 5.0, 6.0);
  double segment_length = std::hypot(4.0 - 1.0, 5.0 - 2.0, 6.0 - 3.0);

  auto interpolated_pt_forward = lanelet2_utils::interpolate_point(p1, p2, segment_length);
  ASSERT_TRUE(interpolated_pt_forward.has_value());
  EXPECT_NEAR(interpolated_pt_forward->x(), p2.x(), 1e-4);
}

// Test 5: out‑of‑bounds interpolation (returns nullopt)
TEST(InterpolatePointTest, OutOfBoundsDistanceReturnsNullopt)
{
  lanelet::ConstPoint3d p1(1, 1.0, 2.0, 3.0);
  lanelet::ConstPoint3d p2(2, 4.0, 5.0, 6.0);
  double segment_length = std::hypot(4.0 - 1.0, 5.0 - 2.0, 6.0 - 3.0);

  auto interpolated_pt_pos = lanelet2_utils::interpolate_point(p1, p2, segment_length + 1.0);
  auto interpolated_pt_neg = lanelet2_utils::interpolate_point(p1, p2, -1.0);
  EXPECT_FALSE(interpolated_pt_pos.has_value());
  EXPECT_FALSE(interpolated_pt_neg.has_value());
}

// Test 6: interpolate_lanelet test from map
TEST_F(ExtrapolatedLaneletTest, InterpolateLanelet)
{
  const auto ll = lanelet_map_ptr_->laneletLayer.get(2287);
  auto opt_pt = lanelet2_utils::interpolate_lanelet(ll, 3.0);
  ASSERT_TRUE(opt_pt.has_value());
  // TODO(soblin): following are flaky
  EXPECT_NEAR(opt_pt->x(), 164.269030, 1e-4);
  EXPECT_NEAR(opt_pt->y(), 181.097588, 2e-3);
  EXPECT_NEAR(opt_pt->z(), 100.000000, 2e-3);
}

// Test 7: interpolate_lanelet_sequence test from map
TEST_F(ExtrapolatedLaneletTest, InterpolateLaneletSequence)
{
  lanelet::ConstLanelets lanelets;
  lanelets.reserve(3);
  for (const auto & id : {2287, 2288, 2289}) {
    lanelets.push_back(lanelet_map_ptr_->laneletLayer.get(id));
  }
  auto opt_pt = lanelet2_utils::interpolate_lanelet_sequence(lanelets, 3.0);
  ASSERT_TRUE(opt_pt.has_value());
  // TODO(soblin): following are flaky
  EXPECT_NEAR(opt_pt->x(), 164.269030, 2e-3);
  EXPECT_NEAR(opt_pt->y(), 181.097588, 2e-3);
  EXPECT_NEAR(opt_pt->z(), 100.000000, 2e-3);
}

// Test 8: concatenate_center_line empty input
TEST(ConcatenateCenterLineTest, EmptyInputReturnsNullopt)
{
  lanelet::ConstLanelets empty_seq;
  auto opt_ls = lanelet2_utils::concatenate_center_line(empty_seq);
  EXPECT_FALSE(opt_ls.has_value());
}

// Test 9: concatenate_center_line map
TEST_F(ExtrapolatedLaneletTest, ConcatenateCenterlinesSequence)
{
  lanelet::ConstLanelets lanelets;
  lanelets.reserve(3);
  for (auto id : {2287, 2288, 2289}) {
    lanelets.push_back(lanelet_map_ptr_->laneletLayer.get(id));
  }

  auto opt_ls = lanelet2_utils::concatenate_center_line(lanelets);

  ASSERT_TRUE(opt_ls.has_value());
  const auto & ls = *opt_ls;

  const auto first_expected = lanelets.front().centerline().front().basicPoint();
  EXPECT_NEAR(ls.front().x(), first_expected.x(), 1e-4);
  EXPECT_NEAR(ls.front().y(), first_expected.y(), 1e-4);
  EXPECT_NEAR(ls.front().z(), first_expected.z(), 1e-4);
  const auto last_expected = lanelets.back().centerline().back().basicPoint();
  EXPECT_NEAR(ls.back().x(), last_expected.x(), 1e-4);
  EXPECT_NEAR(ls.back().y(), last_expected.y(), 1e-4);
  EXPECT_NEAR(ls.back().z(), last_expected.z(), 1e-4);

  for (size_t i = 1; i < ls.size(); ++i) {
    EXPECT_FALSE(ls[i].basicPoint() == ls[i - 1].basicPoint());
  }
}

// Test 10: getLineStringFromArcLength empty linestring
TEST(GetLineStringFromArcLength, EmptyLinestringReturnsNullopt)
{
  lanelet::ConstLineString3d empty{lanelet::InvalId, lanelet::Points3d{}};
  auto opt =
    autoware::experimental::lanelet2_utils::get_linestring_from_arc_length(empty, 0.5, 1.0);
  EXPECT_FALSE(opt.has_value());
}

// Test 11: getLineStringFromArcLength out of bounds
TEST(GetLineStringFromArcLength, OutOfBoundsReturnsNullopt)
{
  std::vector<lanelet::Point3d> pts = {
    lanelet::Point3d{lanelet::ConstPoint3d(1, 0.0, 0.0, 0.0)},
    lanelet::Point3d{lanelet::ConstPoint3d(1, 1.0, 0.0, 0.0)},
    lanelet::Point3d{lanelet::ConstPoint3d(1, 2.0, 0.0, 0.0)}};
  lanelet::ConstLineString3d line{lanelet::InvalId, pts};
  auto opt1 =
    autoware::experimental::lanelet2_utils::get_linestring_from_arc_length(line, 0.0, 3.0);
  EXPECT_FALSE(opt1.has_value());
  auto opt2 =
    autoware::experimental::lanelet2_utils::get_linestring_from_arc_length(line, -1.0, 1.0);
  EXPECT_FALSE(opt2.has_value());
}

// Test 12: getLineStringFromArcLength full range
TEST(GetLineStringFromArcLength, FullRangeReturnsAllPoints)
{
  std::vector<lanelet::Point3d> pts = {
    lanelet::Point3d{lanelet::ConstPoint3d(1, 0.0, 0.0, 0.0)},
    lanelet::Point3d{lanelet::ConstPoint3d(1, 1.0, 0.0, 0.0)},
    lanelet::Point3d{lanelet::ConstPoint3d(1, 2.0, 0.0, 0.0)}};
  lanelet::ConstLineString3d line{lanelet::InvalId, pts};
  auto opt = autoware::experimental::lanelet2_utils::get_linestring_from_arc_length(line, 0.0, 2.0);
  ASSERT_TRUE(opt.has_value());
  const auto & out = *opt;
  ASSERT_EQ(out.size(), line.size());
  for (size_t i = 0; i < line.size(); ++i) {
    EXPECT_NEAR(out[i].x(), line[i].x(), 1e-4);
    EXPECT_NEAR(out[i].y(), line[i].y(), 1e-4);
    EXPECT_NEAR(out[i].z(), line[i].z(), 1e-4);
  }
}

// Test 13: getLineStringFromArcLength partial range
TEST(GetLineStringFromArcLength, PartialRangeExtractsCorrectSegment)
{
  std::vector<lanelet::Point3d> pts = {
    lanelet::Point3d{lanelet::ConstPoint3d(1, 0.0, 0.0, 0.0)},
    lanelet::Point3d{lanelet::ConstPoint3d(1, 1.0, 0.0, 0.0)},
    lanelet::Point3d{lanelet::ConstPoint3d(1, 1.7, 0.0, 0.0)},
    lanelet::Point3d{lanelet::ConstPoint3d(1, 2.0, 0.0, 0.0)}};
  lanelet::ConstLineString3d line{lanelet::InvalId, pts};
  auto opt = autoware::experimental::lanelet2_utils::get_linestring_from_arc_length(line, 0.5, 1.5);
  ASSERT_TRUE(opt.has_value());
  const auto & out = *opt;
  ASSERT_EQ(out.size(), 3u);
  EXPECT_NEAR(out[0].x(), 0.5, 1e-4);
  EXPECT_NEAR(out[2].x(), 1.5, 1e-4);
}

// Test 14: get_pose_from_2d_arc_length out of bound
TEST_F(ExtrapolatedLaneletTest, GetPoseFrom2dArcLength_OutOfBounds)
{
  lanelet::ConstLanelets lanelets;
  EXPECT_FALSE(
    autoware::experimental::lanelet2_utils::get_pose_from_2d_arc_length(lanelets, 0.0).has_value());

  lanelets.push_back(lanelet_map_ptr_->laneletLayer.get(2287));
  EXPECT_FALSE(autoware::experimental::lanelet2_utils::get_pose_from_2d_arc_length(lanelets, -1.0)
                 .has_value());
  EXPECT_FALSE(
    autoware::experimental::lanelet2_utils::get_pose_from_2d_arc_length(lanelets, 1e6).has_value());
}

// Test 15: get_pose_from_2d_arc_length
TEST_F(ExtrapolatedLaneletTest, GetPoseFrom2dArcLength_OnRealMapLanelets)
{
  lanelet::ConstLanelets lanelets;
  for (auto id : {2287, 2288, 2289}) {
    lanelets.push_back(lanelet_map_ptr_->laneletLayer.get(id));
  }
  auto opt_pose =
    autoware::experimental::lanelet2_utils::get_pose_from_2d_arc_length(lanelets, 3.0);
  ASSERT_TRUE(opt_pose.has_value());
  const auto & p = *opt_pose;
  // TODO(soblin): following are flaky
  EXPECT_NEAR(p.position.x, 164.269030, 2e-3);
  EXPECT_NEAR(p.position.y, 181.097588, 2e-3);
  EXPECT_NEAR(p.position.z, 100.000000, 2e-3);
  auto pt1 = lanelet_map_ptr_->laneletLayer.get(2287).centerline().front().basicPoint();
  auto pt2 = lanelet_map_ptr_->laneletLayer.get(2287).centerline()[1].basicPoint();
  double expected_yaw = std::atan2(pt2.y() - pt1.y(), pt2.x() - pt1.x());
  double half = expected_yaw * 0.5;
  geometry_msgs::msg::Quaternion eq;
  eq.x = 0.0;
  eq.y = 0.0;
  eq.z = std::sin(half);
  eq.w = std::cos(half);

  EXPECT_NEAR(p.orientation.x, eq.x, 1e-4);
  EXPECT_NEAR(p.orientation.y, eq.y, 1e-4);
  EXPECT_NEAR(p.orientation.z, eq.z, 1e-4);
  EXPECT_NEAR(p.orientation.w, eq.w, 1e-4);
}

}  // namespace autoware::experimental

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
