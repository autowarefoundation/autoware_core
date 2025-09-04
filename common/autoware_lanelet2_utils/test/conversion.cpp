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
#include <lanelet2_core/LaneletMap.h>

#include <cmath>
#include <iostream>
#include <string>

using lanelet::Lanelet;
using lanelet::LineString3d;
using lanelet::Point3d;
using lanelet::utils::getId;

static lanelet::ConstLanelets lanelet_layer(const lanelet::LaneletMapConstPtr & ll_map)
{
  lanelet::ConstLanelets lanelets;
  if (!ll_map) {
    std::cerr << "No map received!";
    return lanelets;
  }

  for (const auto & li : ll_map->laneletLayer) {
    lanelets.push_back(li);
  }

  return lanelets;
}

template <typename PointT1, typename PointT2>
static void assert_float_point_eq(
  PointT1 & p1, PointT2 & p2, std::string type_name = "Unknown Type")
{
  ASSERT_FLOAT_EQ(p1.x, p2.x) << "x mismatch, converted value is different from original ("
                              << type_name << ").";
  ASSERT_FLOAT_EQ(p1.y, p2.y) << "y mismatch, converted value is different from original ("
                              << type_name << ").";
  ASSERT_FLOAT_EQ(p1.z, p2.z) << "z mismatch, converted value is different from original ("
                              << type_name << ").";
}

template <typename PointT1, typename PointT2>
static void assert_double_point_eq(
  PointT1 & p1, PointT2 & p2, std::string type_name = "Unknown Type")
{
  ASSERT_DOUBLE_EQ(p1.x(), p2.x) << "x mismatch, converted value is different from original ("
                                 << type_name << ").";
  ASSERT_DOUBLE_EQ(p1.y(), p2.y) << "y mismatch, converted value is different from original ("
                                 << type_name << ").";
  ASSERT_DOUBLE_EQ(p1.z(), p2.z) << "z mismatch, converted value is different from original ("
                                 << type_name << ").";
}

template <typename PointT1, typename PointT2>
static void assert_double_point_eq_2d(
  PointT1 & p1, PointT2 & p2, std::string type_name = "Unknown Type")
{
  ASSERT_DOUBLE_EQ(p1.x(), p2.x) << "x mismatch, converted value is different from original ("
                                 << type_name << ").";
  ASSERT_DOUBLE_EQ(p1.y(), p2.y) << "y mismatch, converted value is different from original ("
                                 << type_name << ").";
  ASSERT_DOUBLE_EQ(0.0, p2.z) << "z mismatch, converted value is different from original ("
                              << type_name << ").";
}

class TestConversion : public ::testing::Test
{
public:
  TestConversion() : single_lanelet_map_ptr(new lanelet::LaneletMap())
  {
    Point3d p1;
    Point3d p2;
    Point3d p3;
    Point3d p4;
    LineString3d traffic_light_base;
    LineString3d traffic_light_bulbs;
    LineString3d stop_line;

    LineString3d ls_left(getId(), {p1, p2});
    LineString3d ls_right(getId(), {p3, p4});

    Lanelet lanelet(getId(), ls_left, ls_right);

    single_lanelet_map_ptr->add(lanelet);
  }
  ~TestConversion() override = default;

  lanelet::LaneletMapPtr single_lanelet_map_ptr;

private:
};

// Test 1: Bin message conversion (map to msg and msg to map)
TEST_F(TestConversion, BinMsgConversion)
{
  autoware_map_msgs::msg::LaneletMapBin bin_msg;
  lanelet::LaneletMapPtr regenerated_map(new lanelet::LaneletMap);

  autoware::experimental::lanelet2_utils::to_bin_msg(single_lanelet_map_ptr, &bin_msg);

  ASSERT_NE(0U, bin_msg.data.size()) << "converted bin message does not have any data";

  autoware::experimental::lanelet2_utils::from_bin_msg(bin_msg, regenerated_map);

  auto original_lanelet = lanelet_layer(single_lanelet_map_ptr);
  auto regenerated_lanelet = lanelet_layer(regenerated_map);

  ASSERT_EQ(original_lanelet.front().id(), regenerated_lanelet.front().id())
    << "regenerated map has different id";
}

// Test 2: to_geom_msg_pt
TEST_F(TestConversion, toGeomMsgPt)
{
  Point3d lanelet_pt(getId(), -0.1, 0.2, 3.0);

  geometry_msgs::msg::Point32 geom_pt32;
  geom_pt32.x = -0.1;
  geom_pt32.y = 0.2;
  geom_pt32.z = 3.0;

  geometry_msgs::msg::Point geom_pt;
  autoware::experimental::lanelet2_utils::to_geom_msg_pt(geom_pt32, &geom_pt);
  assert_float_point_eq(geom_pt32, geom_pt, "geometry_msgs::msg::Point32");

  geom_pt = autoware::experimental::lanelet2_utils::to_geom_msg_pt(geom_pt32);
  assert_float_point_eq(geom_pt32, geom_pt, "geometry_msgs::msg::Point32");

  autoware::experimental::lanelet2_utils::to_geom_msg_pt(lanelet_pt.basicPoint(), &geom_pt);
  assert_double_point_eq(lanelet_pt.basicPoint(), geom_pt, "lanelet::BasicPoint3d");

  geom_pt = autoware::experimental::lanelet2_utils::to_geom_msg_pt(lanelet_pt.basicPoint());
  assert_double_point_eq(lanelet_pt.basicPoint(), geom_pt, "lanelet::BasicPoint3d");

  autoware::experimental::lanelet2_utils::to_geom_msg_pt(lanelet_pt, &geom_pt);
  assert_double_point_eq(lanelet_pt, geom_pt, "lanelet::ConstPoint3d");

  geom_pt = autoware::experimental::lanelet2_utils::to_geom_msg_pt(lanelet_pt);
  assert_double_point_eq(lanelet_pt, geom_pt, "lanelet::ConstPoint3d");

  lanelet::ConstPoint2d point_2d = lanelet::utils::to2D(lanelet_pt);

  autoware::experimental::lanelet2_utils::to_geom_msg_pt(point_2d, &geom_pt);
  assert_double_point_eq_2d(point_2d, geom_pt, "lanelet::ConstPoint2d");

  geom_pt = autoware::experimental::lanelet2_utils::to_geom_msg_pt(point_2d);
  assert_double_point_eq_2d(point_2d, geom_pt, "lanelet::ConstPoint2d");
}

// Test 3: to_lanelet_point
TEST_F(TestConversion, ToLaneletPoint)
{
  geometry_msgs::msg::Point pt;
  pt.x = 1.0;
  pt.y = 2.0;
  pt.z = 3.0;

  lanelet::ConstPoint3d ll_pt;

  autoware::experimental::lanelet2_utils::to_lanelet_point(pt, &ll_pt);
  ASSERT_FLOAT_EQ(ll_pt.x(), pt.x);
  ASSERT_FLOAT_EQ(ll_pt.y(), pt.y);
  ASSERT_FLOAT_EQ(ll_pt.z(), pt.z);

  ll_pt = autoware::experimental::lanelet2_utils::to_lanelet_point(pt);
  ASSERT_FLOAT_EQ(ll_pt.x(), pt.x);
  ASSERT_FLOAT_EQ(ll_pt.y(), pt.y);
  ASSERT_FLOAT_EQ(ll_pt.z(), pt.z);
}

// Test 4: to_geom_msg_poly
TEST_F(TestConversion, ToGeomMsgPoly)
{
  // make lanelet polygon
  lanelet::Point3d p1(lanelet::InvalId, 1.0, 2.0, 3.0);
  lanelet::Point3d p2(lanelet::InvalId, 4.0, 5.0, 6.0);
  lanelet::Point3d p3(lanelet::InvalId, 7.0, 8.0, 9.0);

  lanelet::Polygon3d ll_poly(lanelet::InvalId, {p1, p2, p3});

  geometry_msgs::msg::Polygon poly;
  autoware::experimental::lanelet2_utils::to_geom_msg_poly(ll_poly, &poly);

  ASSERT_EQ(poly.points.size(), ll_poly.size());

  for (size_t i = 0; i < ll_poly.size(); i++) {
    EXPECT_FLOAT_EQ(poly.points[i].x, ll_poly[i].x());
    EXPECT_FLOAT_EQ(poly.points[i].y, ll_poly[i].y());
    EXPECT_FLOAT_EQ(poly.points[i].z, ll_poly[i].z());
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
