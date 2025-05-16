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
#include <autoware/marker_utils/marker_conversion.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <rclcpp/clock.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/primitives/CompoundPolygon.h>

#include <filesystem>
#include <string>

namespace fs = std::filesystem;

namespace autoware::experimental
{
using autoware_utils::create_default_marker;
using autoware_utils::create_marker_color;
using autoware_utils::create_marker_scale;

class MarkerConversionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    now_ = rclcpp::Clock().now();
    color_.r = 1.0f;
    color_.g = 0.0f;
    color_.b = 0.0f;
    color_.a = 1.0f;
    header_.frame_id = "map";
    header_.stamp = now_;
  }

  rclcpp::Time now_;
  std_msgs::msg::ColorRGBA color_;
  std_msgs::msg::Header header_;
};

auto make_point = [](float x, float y, float z) {
  geometry_msgs::msg::Point32 p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
};

template <typename PointT>
void expect_point_eq(const PointT & p, double x, double y, double z)
{
  EXPECT_DOUBLE_EQ(p.x, x);
  EXPECT_DOUBLE_EQ(p.y, y);
  EXPECT_DOUBLE_EQ(p.z, z);
}

// Test 1: verify LINE_STRIP marker closes polygon by repeating first point
TEST_F(MarkerConversionTest, MakeMarkerFromPolygonLineStrip)
{
  geometry_msgs::msg::Polygon poly;
  poly.points.push_back(make_point(0.0f, 0.0f, 0.0f));
  poly.points.push_back(make_point(1.0f, 0.0f, 0.0f));
  poly.points.push_back(make_point(1.0f, 1.0f, 0.0f));

  auto arr = autoware::marker_utils::create_polygon_marker_array(
    poly, header_.frame_id, now_, "ns", 42, visualization_msgs::msg::Marker::LINE_STRIP,
    create_marker_scale(0.1, 0.1, 0.1), color_);

  ASSERT_EQ(arr.markers.size(), 1u);
  const auto & pts = arr.markers[0].points;
  EXPECT_EQ(pts.size(), 4u);
  expect_point_eq(pts[0], 0.0f, 0.0f, 0.0f);
  expect_point_eq(pts[1], 1.0f, 0.0f, 0.0f);
  expect_point_eq(pts[2], 1.0f, 1.0f, 0.0f);
  expect_point_eq(pts[3], 0.0f, 0.0f, 0.0f);
}

// Test 2: verify LINE_LIST marker draws each edge as separate line segments
TEST_F(MarkerConversionTest, MakeMarkerFromPolygonLineList)
{
  geometry_msgs::msg::Polygon poly;
  poly.points.push_back(make_point(0.0f, 0.0f, 0.0f));
  poly.points.push_back(make_point(1.0f, 0.0f, 0.0f));
  poly.points.push_back(make_point(.5f, 1.0f, 0.0f));

  auto arr = autoware::marker_utils::create_polygon_marker_array(
    poly, header_.frame_id, now_, "ns", 7, visualization_msgs::msg::Marker::LINE_LIST,
    create_marker_scale(0.2, 0.2, 0.2), color_);

  ASSERT_EQ(arr.markers.size(), 1u);
  const auto & pts = arr.markers[0].points;
  EXPECT_EQ(pts.size(), 6u);
  expect_point_eq(pts[0], 0.0f, 0.0f, 0.0f);
  expect_point_eq(pts[1], 1.0f, 0.0f, 0.0f);
  expect_point_eq(pts[2], 1.0f, 0.0f, 0.0f);
  expect_point_eq(pts[3], 0.5f, 1.0f, 0.0f);
  expect_point_eq(pts[4], 0.5f, 1.0f, 0.0f);
  expect_point_eq(pts[5], 0.0f, 0.0f, 0.0f);
}

// Test 3: confirm boost Polygon2d converts to marker at constant z height
TEST_F(MarkerConversionTest, CreateBoostPolygonMarker)
{
  using autoware_utils::Point2d;
  using autoware_utils::Polygon2d;

  Polygon2d poly;
  auto & ring = poly.outer();
  ring.push_back(Point2d{0.0, 0.0});
  ring.push_back(Point2d{1.0, 0.0});
  ring.push_back(Point2d{1.0, 1.0});

  double z = 1.5;
  auto marker = autoware::marker_utils::create_boost_polygon_marker(
    poly, header_, "map", 0, visualization_msgs::msg::Marker::LINE_STRIP,
    create_marker_scale(0.1, 0.1, 0.1), color_, z);

  const auto & pts = marker.points;
  EXPECT_EQ(pts.size(), ring.size());
  for (size_t i = 0; i < ring.size(); ++i) {
    expect_point_eq(pts[i], ring[i].x(), ring[i].y(), z);
  }
}

// Test 4: validate pull-over area MultiPolygon2d flattens to single marker ring
TEST_F(MarkerConversionTest, CreatePullOverAreaMarkerArray)
{
  using autoware_utils::MultiPolygon2d;
  using autoware_utils::Point2d;
  using autoware_utils::Polygon2d;

  Polygon2d square;
  auto & ring = square.outer();
  ring.push_back(Point2d{0.0, 0.0});
  ring.push_back(Point2d{1.0, 0.0});
  ring.push_back(Point2d{1.0, 1.0});
  ring.push_back(Point2d{0.0, 1.0});

  MultiPolygon2d mp;
  mp.push_back(square);

  double z = 2.5;
  auto arr = autoware::marker_utils::create_pull_over_area_marker_array(mp, header_, color_, z);

  ASSERT_EQ(arr.markers.size(), 1u);
  const auto & pts = arr.markers[0].points;
  EXPECT_EQ(pts.size(), ring.size());
  for (size_t i = 0; i < ring.size(); ++i) {
    expect_point_eq(pts[i], ring[i].x(), ring[i].y(), z);
  }
}

// Test 5: ensure PredictedObjects produce markers with correct id and pose
TEST_F(MarkerConversionTest, CreateObjectsMakerArray)
{
  autoware_perception_msgs::msg::PredictedObjects objs;
  autoware_perception_msgs::msg::PredictedObject o;
  o.kinematics.initial_pose_with_covariance.pose.position.x = 5.0;
  o.kinematics.initial_pose_with_covariance.pose.position.y = -3.0;
  objs.objects.push_back(o);

  int64_t module_id = 0x1234;
  double r = 0.1, g = 0.2, b = 0.3;
  auto arr =
    autoware::marker_utils::create_objects_marker_array(objs, "obj_ns", module_id, now_, r, g, b);

  ASSERT_EQ(arr.markers.size(), 1u);
  int64_t expected_id = (module_id << (sizeof(int32_t) * 8 / 2)) + 0;
  EXPECT_EQ(arr.markers[0].id, expected_id);
  const auto & m = arr.markers[0];
  EXPECT_DOUBLE_EQ(m.pose.position.x, 5.0);
  EXPECT_DOUBLE_EQ(m.pose.position.y, -3.0);
  EXPECT_DOUBLE_EQ(m.pose.position.z, 0.0);

  EXPECT_DOUBLE_EQ(m.pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(m.pose.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(m.pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(m.pose.orientation.w, 1.0);
}

// Test 6: empty CompoundPolygon3d returns single marker with no points
TEST_F(MarkerConversionTest, CreateLaneletPolygonMarkerArrayEmpty)
{
  lanelet::CompoundPolygon3d cp;
  auto arr =
    autoware::marker_utils::create_lanelet_polygon_marker_array(cp, header_, "ns3d", color_);

  ASSERT_EQ(arr.markers.size(), 1u);
  EXPECT_TRUE(arr.markers[0].points.empty());
}

// Test 7: debug footprint draws full closed ring including first point at end
TEST_F(MarkerConversionTest, VisualizeDebugFootprint)
{
  using autoware_utils::LinearRing2d;
  using autoware_utils::Point2d;

  LinearRing2d ring;
  ring.push_back(Point2d{0.0, 0.0});
  ring.push_back(Point2d{1.0, 0.0});
  ring.push_back(Point2d{1.0, 1.0});

  auto arr = autoware::marker_utils::visualize_debug_footprint(ring);
  ASSERT_EQ(arr.markers.size(), 1u);
  const auto & pts = arr.markers[0].points;
  EXPECT_EQ(pts.size(), ring.size() + 1);
  for (size_t i = 0; i < ring.size(); ++i) {
    expect_point_eq(pts[i], ring[i].x(), ring[i].y(), 0.0);
  }
  expect_point_eq(pts.back(), ring.front().x(), ring.front().y(), 0.0);
}

}  // namespace autoware::experimental

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
