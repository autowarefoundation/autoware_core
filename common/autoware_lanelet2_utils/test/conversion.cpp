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

#include "autoware/lanelet2_utils/conversion.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <vector>

template <typename PointT1, typename PointT2>
static void expect_point_eq(PointT1 & p1, PointT2 & p2)
{
  EXPECT_EQ(p1.x(), p2.x()) << "x mismatch.";
  EXPECT_EQ(p1.y(), p2.y()) << "y mismatch.";
  EXPECT_EQ(p1.z(), p2.z()) << "z mismatch.";
}

template <typename PointT1, typename PointT2>
static void assert_float_point_eq(PointT1 & p1, PointT2 & p2)
{
  ASSERT_FLOAT_EQ(p1.x, p2.x()) << "x mismatch.";
  ASSERT_FLOAT_EQ(p1.y, p2.y()) << "y mismatch.";
  ASSERT_FLOAT_EQ(p1.z, p2.z()) << "z mismatch.";
}

template <typename PointT1, typename PointT2>
static void assert_float_point_eq_3d_to_2d(PointT1 & p1, PointT2 & p2, float z = 0.0)
{
  ASSERT_FLOAT_EQ(p1.x, p2.x()) << "x mismatch.";
  ASSERT_FLOAT_EQ(p1.y, p2.y()) << "y mismatch.";
  ASSERT_FLOAT_EQ(p1.z, z) << "z mismatch.";
}

template <typename PointT1, typename PointT2>
static void assert_float_point_eq_2d(PointT1 & p1, PointT2 & p2)
{
  ASSERT_FLOAT_EQ(p1.x, p2.x()) << "x mismatch.";
  ASSERT_FLOAT_EQ(p1.y, p2.y()) << "y mismatch.";
}

// Test 1: to ros (BasicPoint3d->Point)
TEST(TestConversion, BasicPoint3dToPoint)
{
  auto original = lanelet::BasicPoint3d(1.0, 2.0, 3.0);

  auto ros_pt = autoware::experimental::lanelet2_utils::to_ros(original);
  assert_float_point_eq(ros_pt, original);
  EXPECT_EQ(typeid(ros_pt), typeid(geometry_msgs::msg::Point))
    << "ros_pt is not geometry_msgs::msg::Point.";
}

// Test 2: to/from ros (Point<->ConstPoint3d)
TEST(TestConversion, RoundTripPointToConstPoint3d)
{
  auto original = lanelet::ConstPoint3d(lanelet::Point3d(lanelet::InvalId, 1.0, 2.0, 3.0));

  auto ros_pt = autoware::experimental::lanelet2_utils::to_ros(original);
  assert_float_point_eq(ros_pt, original);
  EXPECT_EQ(typeid(ros_pt), typeid(geometry_msgs::msg::Point))
    << "ros_pt is not geometry_msgs::msg::Point.";

  auto converted_pt = autoware::experimental::lanelet2_utils::from_ros(ros_pt);
  assert_float_point_eq(ros_pt, converted_pt);
  EXPECT_EQ(typeid(converted_pt), typeid(lanelet::ConstPoint3d))
    << "converted_pt is not lanelet::ConstPoint3d.";
}

// Test 3: from ros (Pose->ConstPoint3d)
TEST(TestConversion, PoseToConstPoint3d)
{
  geometry_msgs::msg::Pose original;
  original.position.x = 1.0;
  original.position.y = 2.0;
  original.position.z = 3.0;

  auto const_pt = autoware::experimental::lanelet2_utils::from_ros(original);
  assert_float_point_eq(original.position, const_pt);
  EXPECT_EQ(typeid(const_pt), typeid(lanelet::ConstPoint3d))
    << "const_pt is not lanelet::ConstPoint3d.";
}

// Test 4: to ros (BasicPoint2d->Point)
TEST(TestConversion, RoundTripPointToBasicPoint2d)
{
  auto original = lanelet::BasicPoint2d(lanelet::Point2d(lanelet::InvalId, 1.0, 2.0));

  auto ros_pt = autoware::experimental::lanelet2_utils::to_ros(original, 3.0);
  assert_float_point_eq_3d_to_2d(ros_pt, original, 3.0);
  EXPECT_EQ(typeid(ros_pt), typeid(geometry_msgs::msg::Point))
    << "ros_pt is not geometry_msgs::msg::Point.";
}

// Test 5: to/from ros (Point<->ConstPoint2d)
TEST(TestConversion, RoundTripPointToConstPoint2d)
{
  auto original = lanelet::BasicPoint2d(1.0, 2.0);

  auto ros_pt = autoware::experimental::lanelet2_utils::to_ros(original, 3.0);
  assert_float_point_eq_3d_to_2d(ros_pt, original, 3.0);
  EXPECT_EQ(typeid(ros_pt), typeid(geometry_msgs::msg::Point))
    << "ros_pt is not geometry_msgs::msg::Point.";

  auto converted_pt = autoware::experimental::lanelet2_utils::from_ros(ros_pt);
  auto converted_pt2d = lanelet::utils::to2D(converted_pt);
  assert_float_point_eq_2d(ros_pt, converted_pt2d);
  EXPECT_EQ(typeid(converted_pt2d), typeid(lanelet::ConstPoint2d))
    << "converted_pt is not lanelet::ConstPoint2d.";
}

// Test 5: remove const from ConstPoint3d
TEST(RemoveConst, RemoveConst)
{
  const lanelet::ConstPoint3d point = lanelet::Point3d(lanelet::InvalId, 1.0, 1.0, 1.0);
  ASSERT_EQ(typeid(point), typeid(lanelet::ConstPoint3d)) << "point is not lanelet::ConstPoint3d.";

  const auto without_const_point = autoware::experimental::lanelet2_utils::remove_const(point);
  // Check type
  EXPECT_EQ(typeid(without_const_point), typeid(lanelet::Point3d))
    << "without_const_point is not lanelet::Point3d.";

  // Check Content
  EXPECT_EQ(point.x(), without_const_point.x());
  EXPECT_EQ(point.y(), without_const_point.y());
  EXPECT_EQ(point.z(), without_const_point.z());
}

// Test 6: construct BasicLineString3d - one point
TEST(ArtificialLaneletObjectConstruction, OnePointBasicLineString3d)
{
  auto p1 = lanelet::BasicPoint3d(1.0, 1.0, 1.0);
  std::vector<lanelet::BasicPoint3d> vect_points = {p1};
  auto ls = autoware::experimental::lanelet2_utils::create_basic_linestring3d(vect_points);
  EXPECT_FALSE(ls.has_value());
}

// Test 7: construct BasicLineString3d - normal
TEST(ArtificialLaneletObjectConstruction, BasicLineString3d)
{
  auto p1 = lanelet::BasicPoint3d(1.0, 1.0, 1.0);
  auto p2 = lanelet::BasicPoint3d(2.0, 2.0, 2.0);
  auto p3 = lanelet::BasicPoint3d(3.0, 3.0, 3.0);
  std::vector<lanelet::BasicPoint3d> vect_points = {p1, p2, p3};
  auto ls = autoware::experimental::lanelet2_utils::create_basic_linestring3d(vect_points);
  EXPECT_TRUE(ls.has_value());

  EXPECT_EQ(typeid(*ls), typeid(lanelet::BasicLineString3d))
    << "ls is not lanelet::BasicLineString3d.";

  for (size_t i = 0; i < vect_points.size(); ++i) {
    expect_point_eq((*ls)[i], vect_points[i]);
  }
}

// Test 8: construct ConstLineString3d - one point
TEST(ArtificialLaneletObjectConstruction, OnePointConstLineString3d)
{
  auto p1 = lanelet::ConstPoint3d(lanelet::Point3d(lanelet::InvalId, 1.0, 1.0, 1.0));
  std::vector<lanelet::ConstPoint3d> vect_points = {p1};
  auto ls = autoware::experimental::lanelet2_utils::create_const_linestring3d(vect_points);
  EXPECT_FALSE(ls.has_value());
}

// Test 7: construct ConstLineString3d - normal
TEST(ArtificialLaneletObjectConstruction, ConstLineString3d)
{
  auto p1 = lanelet::ConstPoint3d(lanelet::Point3d(lanelet::InvalId, 1.0, 1.0, 1.0));
  auto p2 = lanelet::ConstPoint3d(lanelet::Point3d(lanelet::InvalId, 2.0, 2.0, 2.0));
  auto p3 = lanelet::ConstPoint3d(lanelet::Point3d(lanelet::InvalId, 3.0, 3.0, 3.0));
  std::vector<lanelet::ConstPoint3d> vect_points = {p1, p2, p3};
  auto ls = autoware::experimental::lanelet2_utils::create_const_linestring3d(vect_points);
  EXPECT_TRUE(ls.has_value());

  EXPECT_EQ(typeid(*ls), typeid(lanelet::ConstLineString3d))
    << "ls is not lanelet::ConstLineString3d.";

  for (size_t i = 0; i < vect_points.size(); ++i) {
    expect_point_eq((*ls)[i], vect_points[i]);
  }
}
