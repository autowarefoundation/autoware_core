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
#include <autoware_lanelet2_extension/regulatory_elements/detection_area.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/no_stopping_area.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>
#include <rclcpp/clock.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/primitives/CompoundPolygon.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{
using autoware_utils_visualization::create_default_marker;
using autoware_utils_visualization::create_marker_color;
using autoware_utils_visualization::create_marker_scale;

class MarkerConversionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    now = rclcpp::Clock().now();
    color_.r = 1.0f;
    color_.g = 0.0f;
    color_.b = 0.0f;
    color_.a = 1.0f;
  }

  rclcpp::Time now;
  std_msgs::msg::ColorRGBA color_;
};

auto make_point = [](float x, float y, float z) {
  geometry_msgs::msg::Point32 p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
};

auto make_point_plain = [](double x, double y, double z) {
  geometry_msgs::msg::Point p;
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

template <typename PointT>
void expect_point_eq(const PointT & p, double x, double y)
{
  EXPECT_DOUBLE_EQ(p.x, x);
  EXPECT_DOUBLE_EQ(p.y, y);
}

// Test 1: verify LINE_STRIP marker closes polygon by repeating first point
TEST_F(MarkerConversionTest, MakeMarkerFromPolygonLINESTRIP)
{
  geometry_msgs::msg::Polygon poly;
  poly.points.push_back(make_point(0.0f, 0.0f, 0.0f));
  poly.points.push_back(make_point(1.0f, 0.0f, 0.0f));
  poly.points.push_back(make_point(1.0f, 1.0f, 0.0f));

  int32_t id = 42;
  auto arr = autoware::experimental::marker_utils::create_geometry_msgs_marker_array(
    poly, now, "ns", id, visualization_msgs::msg::Marker::LINE_STRIP,
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
TEST_F(MarkerConversionTest, MakeMarkerFromPolygonLINELIST)
{
  geometry_msgs::msg::Polygon poly;
  poly.points.push_back(make_point(0.0f, 0.0f, 0.0f));
  poly.points.push_back(make_point(1.0f, 0.0f, 0.0f));
  poly.points.push_back(make_point(.5f, 1.0f, 0.0f));

  auto arr = autoware::experimental::marker_utils::create_geometry_msgs_marker_array(
    poly, now, "ns", 7, visualization_msgs::msg::Marker::LINE_LIST,
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

// Test 3: text marker from point
TEST_F(MarkerConversionTest, CreateTextMarkerFromPoint)
{
  geometry_msgs::msg::Point pt;
  pt.x = 1.0;
  pt.y = 2.0;
  pt.z = 3.0;

  auto arr = autoware::experimental::marker_utils::create_geometry_msgs_marker_array(
    pt, now, "no_start_obstacle_text", 0, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
    create_marker_scale(0.0, 0.0, 1.0), create_marker_color(1.0, 1.0, 1.0, 0.999));
  ASSERT_EQ(arr.markers.size(), 1u);
  const auto & m = arr.markers[0];
  EXPECT_EQ(m.text, "!");
  EXPECT_DOUBLE_EQ(m.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(m.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(m.pose.position.z, 5.0);  // 3 + 2 offset
}

// Test 4: create_ros_pose_marker_array with the pose (to create yaw line)
TEST_F(MarkerConversionTest, CreateMarkerArrayYawLine)
{
  geometry_msgs::msg::Pose pose;

  pose.position.x = 1.0;
  pose.position.y = 2.0;
  pose.position.z = 0.0;

  const double yaw = tf2::getYaw(pose.orientation);

  auto marker_array = autoware::experimental::marker_utils::create_geometry_msgs_marker_array(
    pose, now, "test_ns", 0, create_marker_scale(1.0, 1.0, 0.1), color_);

  EXPECT_EQ(marker_array.markers.size(), 1u);
  auto marker_line = marker_array.markers[0];
  EXPECT_EQ(marker_line.points.size(), 2u);
  expect_point_eq(
    marker_line.points[0], pose.position.x - 3 * std::sin(yaw), pose.position.y + 3 * std::cos(yaw),
    pose.position.z);
  expect_point_eq(
    marker_line.points[1], pose.position.x + 3 * std::sin(yaw), pose.position.y - 3 * std::cos(yaw),
    pose.position.z);
}

// Test 5: create_geometry_msgs_marker_array with an arrow
TEST_F(MarkerConversionTest, CreateMarkerArrayArrow)
{
  using geometry_msgs::msg::Point;
  auto arrow_type = visualization_msgs::msg::Marker::ARROW;

  Point p_start = make_point_plain(0.0f, 0.0f, 0.0f);
  Point p_end = make_point_plain(1.0f, 1.0f, 1.0f);
  std::vector<Point> test_vector;
  test_vector.push_back(p_start);

  // Only one point in vector.
  auto marker_array_one_point =
    autoware::experimental::marker_utils::create_geometry_msgs_marker_array(
      test_vector, now, "test_ns", 0, arrow_type, create_marker_scale(1.0, 1.0, 1.0), color_);
  EXPECT_EQ(marker_array_one_point.markers.size(), 0u);

  test_vector.push_back(p_end);

  // There are exactly two points in vector.
  auto marker_array = autoware::experimental::marker_utils::create_geometry_msgs_marker_array(
    test_vector, now, "test_ns", 0, arrow_type, create_marker_scale(1.0, 1.0, 1.0), color_);

  EXPECT_EQ(marker_array.markers.size(), 1u);
  EXPECT_EQ(marker_array.markers[0].type, visualization_msgs::msg::Marker::ARROW);
  expect_point_eq(marker_array.markers[0].points[0], 0.0f, 0.0f, 0.0f);
  expect_point_eq(marker_array.markers[0].points[1], 1.0f, 1.0f, 1.0f);

  Point p_extra = make_point_plain(2.0f, 2.0f, 2.0f);
  test_vector.push_back(p_extra);

  // There are more than 2 points in vector.
  auto marker_array_extra = autoware::experimental::marker_utils::create_geometry_msgs_marker_array(
    test_vector, now, "test_ns", 0, arrow_type, create_marker_scale(1.0, 1.0, 1.0), color_);

  EXPECT_EQ(marker_array_extra.markers.size(), 1u);
  EXPECT_EQ(marker_array_extra.markers[0].type, visualization_msgs::msg::Marker::ARROW);
  expect_point_eq(marker_array_extra.markers[0].points[0], 0.0f, 0.0f, 0.0f);
  expect_point_eq(marker_array_extra.markers[0].points[1], 1.0f, 1.0f, 1.0f);
}

// Test 6: create_autoware_geometry_marker_array with a vector of geometry_msgs::msg::Point
// (both SPHERE and LINE_STRIP, and other)
TEST_F(MarkerConversionTest, CreateMarkerArrayGeometryPoints)
{
  using geometry_msgs::msg::Point;

  std::vector<Point> test_vector;
  test_vector.push_back(make_point_plain(1.0f, 2.0f, 3.0f));
  test_vector.push_back(make_point_plain(0.0f, 4.0f, -1.0f));
  test_vector.push_back(make_point_plain(3.0f, 3.0f, 1.0f));
  test_vector.push_back(make_point_plain(4.0f, 1.0f, 0.0f));

  auto line_strip_type = visualization_msgs::msg::Marker::LINE_STRIP;
  auto marker_array = autoware::experimental::marker_utils::create_geometry_msgs_marker_array(
    test_vector, now, "test_ns", 0, line_strip_type, create_marker_scale(1.0, 1.0, 1.0), color_);

  EXPECT_EQ(marker_array.markers.size(), 1u);
  auto marker = marker_array.markers[0];
  expect_point_eq(marker.points[0], 1.0f, 2.0f, 3.0);
  expect_point_eq(marker.points[1], 0.0f, 4.0f, -1.0f);
  expect_point_eq(marker.points[2], 3.0f, 3.0f, 1.0f);
  expect_point_eq(marker.points[3], 4.0f, 1.0f, 0.0f);

  auto sphere_type = visualization_msgs::msg::Marker::SPHERE;
  auto marker_array_sphere =
    autoware::experimental::marker_utils::create_geometry_msgs_marker_array(
      test_vector, now, "test_ns", 0, sphere_type, create_marker_scale(1.0, 1.0, 1.0), color_);

  EXPECT_EQ(marker_array_sphere.markers.size(), test_vector.size());
  for (auto i = 0u; i < marker_array_sphere.markers.size(); ++i) {
    auto marker = marker_array_sphere.markers[i];
    expect_point_eq(marker.pose.position, test_vector[i].x, test_vector[i].y, test_vector[i].z);
  }

  auto line_list_type = visualization_msgs::msg::Marker::LINE_LIST;
  auto marker_array_line_list =
    autoware::experimental::marker_utils::create_geometry_msgs_marker_array(
      test_vector, now, "test_ns", 0, line_list_type, create_marker_scale(1.0, 1.0, 1.0), color_);

  EXPECT_EQ(marker_array_line_list.markers.size(), 0u);
}

// Test 7: debug footprint draws full closed ring including first point at end
TEST_F(MarkerConversionTest, VisualizeDebugFootprint)
{
  using autoware_utils_geometry::LinearRing2d;
  using autoware_utils_geometry::Point2d;

  LinearRing2d ring;
  ring.push_back(Point2d{0.0, 0.0});
  ring.push_back(Point2d{1.0, 0.0});
  ring.push_back(Point2d{1.0, 1.0});

  auto arr = autoware::experimental::marker_utils::create_autoware_geometry_marker_array(
    ring, now, "goal_footprint", 0, visualization_msgs::msg::Marker::LINE_STRIP,
    create_marker_scale(0.05, 0.0, 0.0), create_marker_color(0.99, 0.99, 0.2, 1.0));
  ASSERT_EQ(arr.markers.size(), 1u);
  const auto & pts = arr.markers[0].points;
  EXPECT_EQ(pts.size(), ring.size() + 1);
  for (size_t i = 0; i < ring.size(); ++i) {
    expect_point_eq(pts[i], ring[i].x(), ring[i].y(), 0.0);
  }
  expect_point_eq(pts.back(), ring.front().x(), ring.front().y(), 0.0);
}

// Test 8: validate pull-over area MultiPolygon2d flattens to single marker ring
TEST_F(MarkerConversionTest, CreatePullOverAreaMarkerArray)
{
  using autoware_utils_geometry::MultiPolygon2d;
  using autoware_utils_geometry::Point2d;
  using autoware_utils_geometry::Polygon2d;

  Polygon2d square;
  auto & ring = square.outer();
  ring.push_back(Point2d{0.0, 0.0});
  ring.push_back(Point2d{1.0, 0.0});
  ring.push_back(Point2d{1.0, 1.0});
  ring.push_back(Point2d{0.0, 1.0});

  Polygon2d square_2;
  auto & ring_2 = square_2.outer();
  ring_2.push_back(Point2d{1.0, 1.0});
  ring_2.push_back(Point2d{2.0, 1.0});
  ring_2.push_back(Point2d{2.0, 2.0});
  ring_2.push_back(Point2d{1.0, 2.0});

  MultiPolygon2d mp;
  mp.push_back(square);
  mp.push_back(square_2);

  double z = 2.5;
  int32_t id = 42;
  auto arr = autoware::experimental::marker_utils::create_autoware_geometry_marker_array(
    mp, now, "ns", id, visualization_msgs::msg::Marker::LINE_STRIP,
    create_marker_scale(0.1, 0.1, 0.1), color_, z);

  ASSERT_EQ(arr.markers.size(), 2u);
  const auto & pts = arr.markers[0].points;
  EXPECT_EQ(pts.size(), 4u);
  for (size_t i = 0; i < ring.size(); ++i) {
    expect_point_eq(pts[i], ring[i].x(), ring[i].y(), z);
  }
}

// Test 9: create_lanelet_linestring_marker
TEST_F(MarkerConversionTest, CreateLaneletLineStringMarker)
{
  using lanelet::BasicLineString2d;
  using lanelet::BasicPoint2d;

  BasicLineString2d ls;

  ls.push_back(BasicPoint2d(0, 0));
  ls.push_back(BasicPoint2d(1, 0));
  ls.push_back(BasicPoint2d(1, 1));

  double z = 3;
  auto marker = autoware::experimental::marker_utils::create_lanelet_linestring_marker(
    ls, now, "test_ns", 0, create_marker_scale(1.0, 1.0, 0.1), color_, z);

  // size of points = 4
  ASSERT_EQ(marker.points.size(), (ls.size() - 1) * 2);
  expect_point_eq(marker.points[0], 0, 0, z);
  expect_point_eq(marker.points[1], 1, 0, z);
  expect_point_eq(marker.points[2], 1, 0, z);
  expect_point_eq(marker.points[3], 1, 1, z);
}

// Test 10: one simple rectangular lanelet → one marker, closed ring
TEST_F(MarkerConversionTest, CreateLaneletsMarkerArrayOne)
{
  // build a 1×1 rectangular lanelet
  using lanelet::ConstLanelet;
  using lanelet::Lanelet;
  using lanelet::LineString3d;
  using lanelet::Point3d;

  LineString3d leftBound{1, {Point3d{0, 0, 0}, Point3d{1, 0, 0}}};
  LineString3d rightBound{2, {Point3d{1, 1, 0}, Point3d{0, 1, 0}}};
  Lanelet raw{3, leftBound, rightBound};
  ConstLanelet cl{raw};
  lanelet::ConstLanelets lls{cl};

  const double Z = 1.0;
  const auto & arr = autoware::experimental::marker_utils::create_lanelets_marker_array(
    lls, "book", create_marker_scale(0.1, 0.1, 0.1), create_marker_color(0.0, 1.0, 0.0, 1.0), Z);

  ASSERT_EQ(arr.markers.size(), 1u);
  const auto & m_out = arr.markers[0];

  // get the expected basicPolygon and ensure it was closed
  auto poly = cl.polygon2d().basicPolygon();
  EXPECT_EQ(m_out.points.size(), poly.size() + 1);

  // verify closure: last == first
  EXPECT_DOUBLE_EQ(m_out.points.back().x, poly.front().x());
  EXPECT_DOUBLE_EQ(m_out.points.back().y, poly.front().y());
  EXPECT_DOUBLE_EQ(m_out.points.back().z, Z + 0.5);
}

// Test 11: create_autoware_geometry_marker_array with empty lanelet
TEST_F(MarkerConversionTest, EmptyLaneletsCustomNS)
{
  lanelet::ConstLanelets empty;
  const auto & markers = autoware::experimental::marker_utils::create_lanelets_marker_array(
    empty, "foo", create_marker_scale(0.1, 0.1, 0.1), color_, 0, true);

  ASSERT_EQ(markers.markers.size(), 0u);
}

// Test 12: create_autoware_geometry_marker_array with lanelet as triangle marker array
TEST_F(MarkerConversionTest, SingleLaneletClosedRing)
{
  using lanelet::ConstLanelet;
  using lanelet::Lanelet;
  using lanelet::LineString3d;
  using lanelet::Point3d;

  LineString3d lb{1, {Point3d{0, 0, 0}, Point3d{1, 0, 0}}};
  LineString3d rb{2, {Point3d{1, 1, 0}, Point3d{0, 1, 0}}};
  Lanelet raw{3, lb, rb};
  ConstLanelet cl{raw};
  lanelet::ConstLanelets lls{cl};

  const auto & markers = autoware::experimental::marker_utils::create_lanelets_marker_array(
    lls, "ns", create_marker_scale(0.1, 0.1, 0.1), color_, 0, true);

  ASSERT_EQ(markers.markers.size(), 1u);
  const auto & m = markers.markers.back();
  auto poly = cl.polygon2d().basicPolygon();
  EXPECT_EQ(m.points.size(), poly.size() + 2);
  expect_point_eq(m.points.back(), poly.front().x(), poly.front().y(), 0.0);
}

// Test 13: create_lanelet_linestring_marker_array
TEST_F(MarkerConversionTest, CreateLaneletLineStringMarkerArray)
{
  using autoware_utils_geometry::LineString2d;
  using autoware_utils_geometry::MultiLineString2d;
  using autoware_utils_geometry::Point2d;

  LineString2d ls1{{Point2d{0, 0}, Point2d{1, 0}, Point2d{1, 1}}};
  LineString2d ls2{{Point2d{1, 1}, Point2d{2, 0}, Point2d{2, 2}}};
  LineString2d ls3{{Point2d{2, 2}, Point2d{3, 0}, Point2d{3, 3}}};

  MultiLineString2d mls;
  mls.push_back(ls1);
  mls.push_back(ls2);
  mls.push_back(ls3);

  double z = 3;
  auto marker_array = autoware::experimental::marker_utils::create_lanelet_linestring_marker_array(
    mls, now, "test_ns", 0, create_marker_scale(1.0, 1.0, 0.1), color_, z);

  EXPECT_EQ(marker_array.markers.size(), 3u);
  EXPECT_EQ(marker_array.markers[0].id, 0);
  EXPECT_EQ(marker_array.markers[1].id, 1);
  EXPECT_EQ(marker_array.markers[2].id, 2);

  auto marker1 = marker_array.markers[0];
  expect_point_eq(marker1.points[0], 0, 0, z);
  expect_point_eq(marker1.points[1], 1, 0, z);
  expect_point_eq(marker1.points[2], 1, 1, z);

  auto marker2 = marker_array.markers[1];
  expect_point_eq(marker2.points[0], 1, 1, z);
  expect_point_eq(marker2.points[1], 2, 0, z);
  expect_point_eq(marker2.points[2], 2, 2, z);

  auto marker3 = marker_array.markers[2];
  expect_point_eq(marker3.points[0], 2, 2, z);
  expect_point_eq(marker3.points[1], 3, 0, z);
  expect_point_eq(marker3.points[2], 3, 3, z);
}

// Test 14: create_autoware_geometry_marker_array with lanelet as triangle marker array
TEST_F(MarkerConversionTest, CreateLaneletPolygonMarkerArray)
{
  lanelet::LineString3d ring{
    1,
    {lanelet::Point3d(1, 0.0, 0.0, 0.0), lanelet::Point3d(2, 1.0, 0.0, 0.0),
     lanelet::Point3d(3, 1.0, 1.0, 0.0), lanelet::Point3d(4, 0.0, 1.0, 0.0)}};
  lanelet::ConstLineString3d const_ring = lanelet::ConstLineString3d(ring);
  std::vector<lanelet::ConstLineString3d> rings{const_ring};
  lanelet::CompoundPolygon3d polygon(rings);

  std_msgs::msg::ColorRGBA color;
  color.r = 0.5f;
  color.g = 0.5f;
  color.b = 1.0f;
  color.a = 1.0f;

  auto arr = autoware::experimental::marker_utils::create_lanelet_polygon_marker_array(
    polygon, now, "test_ns", 0, color);

  ASSERT_EQ(arr.markers.size(), 1u);
  const auto & marker = arr.markers[0];
  ASSERT_EQ(marker.points.size(), ring.size());

  for (size_t i = 0; i < ring.size(); ++i) {
    EXPECT_DOUBLE_EQ(marker.points[i].x, ring[i].x());
    EXPECT_DOUBLE_EQ(marker.points[i].y, ring[i].y());
    EXPECT_DOUBLE_EQ(marker.points[i].z, ring[i].z());
  }
  EXPECT_EQ(marker.ns, "test_ns");
  EXPECT_EQ(marker.id, 0);
  EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::LINE_STRIP);
}

// Test 15: confirm BasicPolygon2d convert to MarkerArray of one Marker at constant z + 0.5
TEST_F(MarkerConversionTest, OneBasicPolygon2d)
{
  using lanelet::BasicPoint2d;
  using lanelet::BasicPolygon2d;
  using lanelet::BasicPolygons2d;
  using visualization_msgs::msg::Marker;

  BasicPolygon2d polygon;
  polygon.emplace_back(BasicPoint2d{0.0f, 0.0f});
  polygon.emplace_back(BasicPoint2d{1.0f, 0.0f});
  polygon.emplace_back(BasicPoint2d{1.0f, 1.0f});
  polygon.emplace_back(BasicPoint2d{0.0f, 1.0f});

  auto marker_array = autoware::experimental::marker_utils::create_lanelet_polygon_marker_array(
    polygon, now, "test_ns", 0, create_marker_scale(0.1, 0.1, 0.1), color_, 0.0f);

  ASSERT_EQ(marker_array.markers.size(), 1u);
  const auto & marker = marker_array.markers[0];
  const auto & pts = marker.points;

  ASSERT_EQ(pts.size(), polygon.size() + 1);

  expect_point_eq(pts[0], 0.0f, 0.0f, 0.5f);
  expect_point_eq(pts[1], 1.0f, 0.0f, 0.5f);
  expect_point_eq(pts[2], 1.0f, 1.0f, 0.5f);
  expect_point_eq(pts[3], 0.0f, 1.0f, 0.5f);
  expect_point_eq(pts[4], 0.0f, 0.0f, 0.5f);
  EXPECT_EQ(marker.ns, "test_ns");
  EXPECT_EQ(marker.id, 0);
  EXPECT_EQ(marker.type, Marker::LINE_STRIP);
}

// Test 16: create_lanelet_polygon_marker_array - empty BasicPolygons2d
TEST_F(MarkerConversionTest, EmptyBasicPolygons2d)
{
  using visualization_msgs::msg::Marker;
  lanelet::BasicPolygons2d empty;

  auto marker_array = autoware::experimental::marker_utils::create_lanelet_polygon_marker_array(
    empty, now, "foo", 0, Marker::LINE_LIST, create_marker_scale(0.1, 0.1, 0.1), color_, 0);
  ASSERT_EQ(marker_array.markers.size(), 0u);
}

// Test 17: create_lanelet_polygon_marker_array - BasicPolygons2d with LINE_LIST marker type
TEST_F(MarkerConversionTest, BasicPolygons2dLINELIST)
{
  using lanelet::BasicPoint2d;
  using lanelet::BasicPolygon2d;
  using lanelet::BasicPolygons2d;
  using visualization_msgs::msg::Marker;

  BasicPolygon2d polygon;
  polygon.emplace_back(BasicPoint2d{0.0f, 0.0f});
  polygon.emplace_back(BasicPoint2d{1.0f, 0.0f});
  polygon.emplace_back(BasicPoint2d{1.0f, 1.0f});
  polygon.emplace_back(BasicPoint2d{0.0f, 1.0f});

  lanelet::BasicPolygons2d polygons{polygon, polygon};

  auto marker_array = autoware::experimental::marker_utils::create_lanelet_polygon_marker_array(
    polygons, now, "test_ns", 0, Marker::LINE_LIST, create_marker_scale(0.1, 0.1, 0.1), color_, 0);

  ASSERT_EQ(marker_array.markers.size(), 1u);
  const auto & marker = marker_array.markers[0];
  const auto & pts = marker.points;
  EXPECT_EQ(pts.size(), 8u * 2);
  expect_point_eq(pts[0], 0.0f, 0.0f);
  expect_point_eq(pts[1], 1.0f, 0.0f);
  expect_point_eq(pts[2], 1.0f, 0.0f);
  expect_point_eq(pts[3], 1.0f, 1.0f);
  expect_point_eq(pts[4], 1.0f, 1.0f);
  expect_point_eq(pts[5], 0.0f, 1.0f);
  expect_point_eq(pts[6], 0.0f, 1.0f);
  expect_point_eq(pts[7], 0.0f, 0.0f);

  EXPECT_EQ(marker.ns, "test_ns");
  EXPECT_EQ(marker.id, 0);
  EXPECT_EQ(marker.type, Marker::LINE_LIST);
}

// Test 18: create_lanelet_polygon_marker_array - BasicPolygons2d with LINE_STRIP marker type
TEST_F(MarkerConversionTest, BasicPolygons2dLINESTRIP)
{
  using lanelet::BasicPoint2d;
  using lanelet::BasicPolygon2d;
  using lanelet::BasicPolygons2d;
  using visualization_msgs::msg::Marker;

  BasicPolygon2d polygon;
  polygon.emplace_back(BasicPoint2d{0.0f, 0.0f});
  polygon.emplace_back(BasicPoint2d{1.0f, 0.0f});
  polygon.emplace_back(BasicPoint2d{1.0f, 1.0f});
  polygon.emplace_back(BasicPoint2d{0.0f, 1.0f});

  lanelet::BasicPolygons2d polygons{polygon, polygon};

  auto marker_array = autoware::experimental::marker_utils::create_lanelet_polygon_marker_array(
    polygons, now, "test_ns", 0, Marker::LINE_STRIP, create_marker_scale(0.1, 0.1, 0.1), color_, 0);

  ASSERT_EQ(marker_array.markers.size(), 2u);
  const auto & marker = marker_array.markers[0];
  const auto & pts = marker.points;
  // doesn't close the polygon
  EXPECT_EQ(pts.size(), 4u);
  expect_point_eq(pts[0], 0.0f, 0.0f);
  expect_point_eq(pts[1], 1.0f, 0.0f);
  expect_point_eq(pts[2], 1.0f, 1.0f);
  expect_point_eq(pts[3], 0.0f, 1.0f);

  EXPECT_EQ(marker.ns, "test_ns");
  EXPECT_EQ(marker.id, 0);
  EXPECT_EQ(marker.type, Marker::LINE_STRIP);
}

// Test 19: ensure PredictedObjects produce markers with correct id and pose
TEST_F(MarkerConversionTest, CreateObjectsMakerArray)
{
  autoware_perception_msgs::msg::PredictedObjects objs;
  autoware_perception_msgs::msg::PredictedObject o;
  o.kinematics.initial_pose_with_covariance.pose.position.x = 5.0;
  o.kinematics.initial_pose_with_covariance.pose.position.y = -3.0;
  objs.objects.push_back(o);

  int64_t module_id = 0x1234;
  auto arr = autoware::experimental::marker_utils::create_predicted_objects_marker_array(
    objs, now, "obj_ns", module_id, color_);

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

// Test 20: create_predicted_path_marker_array - empty
TEST_F(MarkerConversionTest, CreatePredictedPathMarkerArrayEmpty)
{
  autoware_perception_msgs::msg::PredictedPath pp;
  autoware::vehicle_info_utils::VehicleInfo info;
  info.vehicle_width_m = 2.0;
  info.rear_overhang_m = 0.5;
  info.vehicle_length_m = 3.0;

  auto arr = autoware::experimental::marker_utils::create_predicted_path_marker_array(
    pp, info, "ns_", 5, color_);
  EXPECT_TRUE(arr.markers.empty());
}

// Test 21: create_predicted_path_marker_array - one element
TEST_F(MarkerConversionTest, CreatePredictedPathMarkerArrayOne)
{
  autoware_perception_msgs::msg::PredictedPath pp;
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 1.0;
  pp.path.push_back(pose);

  autoware::vehicle_info_utils::VehicleInfo info;
  info.vehicle_width_m = 2.0;
  info.rear_overhang_m = 0.5;
  info.vehicle_length_m = 3.0;

  auto arr = autoware::experimental::marker_utils::create_predicted_path_marker_array(
    pp, info, "ns_", 5, color_);
  ASSERT_EQ(arr.markers.size(), 1u);
  const auto & m = arr.markers[0];
  EXPECT_EQ(m.id, 5);
  EXPECT_EQ(m.points.size(), 5u);
}

// Test 22: create_path_with_lane_id_marker_array without text
TEST_F(MarkerConversionTest, CreatePathWithLaneIdMarkerArrayNoText)
{
  autoware_internal_planning_msgs::msg::PathWithLaneId path;
  autoware_internal_planning_msgs::msg::PathPointWithLaneId pp;
  pp.point.pose.position.x = 0.0;
  pp.point.pose.position.y = 0.0;
  pp.lane_ids = {1, 2};
  path.points.push_back(pp);

  auto arr = autoware::experimental::marker_utils::create_path_with_lane_id_marker_array(
    path, "ns", 1, now, geometry_msgs::msg::Vector3(), color_, false);
  ASSERT_EQ(arr.markers.size(), 1u);
  const auto & m = arr.markers[0];
  EXPECT_EQ(m.type, visualization_msgs::msg::Marker::ARROW);
  EXPECT_DOUBLE_EQ(m.pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(m.pose.position.y, 0.0);
}

// Test 23: create_path_with_lane_id_marker_array with text
TEST_F(MarkerConversionTest, CreatePathWithLaneIdMarkerArrayWithText)
{
  autoware_internal_planning_msgs::msg::PathWithLaneId path;
  for (int i = 0; i < 12; ++i) {
    autoware_internal_planning_msgs::msg::PathPointWithLaneId pp;
    pp.point.pose.position.x = static_cast<double>(i);
    pp.point.pose.position.y = static_cast<double>(i);
    pp.lane_ids = {1};
    path.points.push_back(pp);
  }

  auto arr = autoware::experimental::marker_utils::create_path_with_lane_id_marker_array(
    path, "ns_", 1, now, geometry_msgs::msg::Vector3(), color_, true);
  // expect arrow at n_point + text => 13 markers
  ASSERT_EQ(arr.markers.size(), 13u);
  bool found_text = false;
  for (const auto & m : arr.markers) {
    if (m.type == visualization_msgs::msg::Marker::TEXT_VIEW_FACING) {
      found_text = true;
    }
  }
  EXPECT_TRUE(found_text);
}

// Test 24: create_vehicle_trajectory_point_marker_array
TEST_F(MarkerConversionTest, CreateVehicleTrajectoryPointMarkerArray)
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> traj(3);
  for (size_t i = 0; i < traj.size(); ++i) {
    traj[i].pose.position.x = static_cast<double>(i);
  }
  autoware::vehicle_info_utils::VehicleInfo info;
  info.wheel_tread_m = 1.0;
  info.right_overhang_m = 0.5;
  info.left_overhang_m = 0.5;
  info.vehicle_length_m = 3.0;
  info.rear_overhang_m = 1.0;

  auto arr = autoware::experimental::marker_utils::create_vehicle_trajectory_point_marker_array(
    traj, info, "mpt_footprints", 1);
  ASSERT_EQ(arr.markers.size(), traj.size());
  for (const auto & m : arr.markers) {
    ASSERT_EQ(m.points.size(), 5u);
  }
}

// Test 25: confirm boost Polygon2d converts to marker at constant z height (LINE_STRIP)
TEST_F(MarkerConversionTest, CreateBoostPolygonMarkerLINESTRIP)
{
  using autoware_utils_geometry::Point2d;
  using autoware_utils_geometry::Polygon2d;

  Polygon2d poly;
  auto & ring = poly.outer();
  ring.push_back(Point2d{0.0, 0.0});
  ring.push_back(Point2d{1.0, 0.0});
  ring.push_back(Point2d{1.0, 1.0});

  double z = 1.5;
  auto marker = autoware::experimental::marker_utils::create_autoware_geometry_marker(
    poly, now, "map", 0, visualization_msgs::msg::Marker::LINE_STRIP,
    create_marker_scale(0.1, 0.1, 0.1), color_, z);

  const auto & pts = marker.points;
  EXPECT_EQ(pts.size(), ring.size());
  for (size_t i = 0; i < ring.size(); ++i) {
    expect_point_eq(pts[i], ring[i].x(), ring[i].y(), z);
  }
}

// Test 26: confirm boost Polygon2d converts to marker at constant z height (LINE_LIST)
TEST_F(MarkerConversionTest, CreateBoostPolygonMarkerLINELIST)
{
  using autoware_utils_geometry::Point2d;
  using autoware_utils_geometry::Polygon2d;

  Polygon2d poly;
  auto & ring = poly.outer();
  ring.push_back(Point2d{0.0, 0.0});
  ring.push_back(Point2d{1.0, 0.0});
  ring.push_back(Point2d{1.0, 1.0});

  double z = 1.5;
  auto marker = autoware::experimental::marker_utils::create_autoware_geometry_marker(
    poly, now, "map", 0, visualization_msgs::msg::Marker::LINE_LIST,
    create_marker_scale(0.1, 0.1, 0.1), color_, z);

  const auto & pts = marker.points;
  EXPECT_EQ(pts.size(), ring.size() * 2);
  for (size_t i = 0; i < ring.size(); ++i) {
    expect_point_eq(pts[2 * i], ring[i].x(), ring[i].y(), z);
    expect_point_eq(
      pts[2 * i + 1], ring[(i + 1) % ring.size()].x(), ring[(i + 1) % ring.size()].y(), z);
  }
}

// Test 27: create_autoware_geometry_marker
TEST_F(MarkerConversionTest, CreateLineStringMarker)
{
  using autoware_utils_geometry::Point2d;

  autoware_utils_geometry::LineString2d ls{{Point2d{0, 0}, Point2d{1, 0}, Point2d{1, 1}}};

  double z = 3;
  auto marker = autoware::experimental::marker_utils::create_autoware_geometry_marker(
    ls, now, "test_ns", 0, create_marker_scale(1.0, 1.0, 0.1), color_, z);

  ASSERT_EQ(marker.points.size(), 3u);
  expect_point_eq(marker.points[0], 0, 0, z);
  expect_point_eq(marker.points[1], 1, 0, z);
  expect_point_eq(marker.points[2], 1, 1, z);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
