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
#include <lanelet2_core/primitives/Lanelet.h>

#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
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

  auto arr = autoware::experimental::marker_utils::create_autoware_geometry_marker_array(
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

  auto arr = autoware::experimental::marker_utils::create_autoware_geometry_marker_array(
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
  auto marker = autoware::experimental::marker_utils::create_autoware_geometry_marker(
    poly, now_, "map", 0, visualization_msgs::msg::Marker::LINE_STRIP,
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
  auto arr = autoware::experimental::marker_utils::create_autoware_geometry_marker_array(
    mp, now_, "ns", 42, visualization_msgs::msg::Marker::LINE_STRIP,
    create_marker_scale(0.1, 0.1, 0.1), color_, z);

  ASSERT_EQ(arr.markers.size(), 2u);
  const auto & pts = arr.markers[0].points;
  EXPECT_EQ(pts.size(), 4u);
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
  auto arr = autoware::experimental::marker_utils::create_predicted_objects_marker_array(
    objs, "obj_ns", module_id, now_, color_);

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

// Test 7: debug footprint draws full closed ring including first point at end
TEST_F(MarkerConversionTest, VisualizeDebugFootprint)
{
  using autoware_utils::LinearRing2d;
  using autoware_utils::Point2d;

  LinearRing2d ring;
  ring.push_back(Point2d{0.0, 0.0});
  ring.push_back(Point2d{1.0, 0.0});
  ring.push_back(Point2d{1.0, 1.0});

  auto arr = autoware::experimental::marker_utils::create_autoware_geometry_marker_array(ring);
  ASSERT_EQ(arr.markers.size(), 1u);
  const auto & pts = arr.markers[0].points;
  EXPECT_EQ(pts.size(), ring.size() + 1);
  for (size_t i = 0; i < ring.size(); ++i) {
    expect_point_eq(pts[i], ring[i].x(), ring[i].y(), 0.0);
  }
  expect_point_eq(pts.back(), ring.front().x(), ring.front().y(), 0.0);
}

// Test 8: text marker from point
TEST_F(MarkerConversionTest, CreateTextMarkerFromPoint)
{
  geometry_msgs::msg::Point pt;
  pt.x = 1.0;
  pt.y = 2.0;
  pt.z = 3.0;

  auto arr = autoware::experimental::marker_utils::create_autoware_geometry_marker_array(now_, pt);
  ASSERT_EQ(arr.markers.size(), 1u);
  const auto & m = arr.markers[0];
  EXPECT_EQ(m.text, "!");
  EXPECT_DOUBLE_EQ(m.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(m.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(m.pose.position.z, 5.0);  // 3 + 2 offset
}

// Test 9: create_path_with_lane_id_marker_array without text
TEST_F(MarkerConversionTest, CreatePathWithLaneIdMarkerArrayNoText)
{
  autoware_internal_planning_msgs::msg::PathWithLaneId path;
  autoware_internal_planning_msgs::msg::PathPointWithLaneId pp;
  pp.point.pose.position.x = 0.0;
  pp.point.pose.position.y = 0.0;
  pp.lane_ids = {1, 2};
  path.points.push_back(pp);

  auto arr = autoware::experimental::marker_utils::create_path_with_lane_id_marker_array(
    path, "ns", 1, now_, geometry_msgs::msg::Vector3(), color_, false);
  ASSERT_EQ(arr.markers.size(), 1u);
  const auto & m = arr.markers[0];
  EXPECT_EQ(m.type, visualization_msgs::msg::Marker::ARROW);
  EXPECT_DOUBLE_EQ(m.pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(m.pose.position.y, 0.0);
}

// Test 10: create_path_with_lane_id_marker_array with text
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
    path, "ns_", 1, now_, geometry_msgs::msg::Vector3(), color_, true);
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

// Test 11: create_vehicle_trajectory_point_marker_array
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
    traj, info, 1);
  ASSERT_EQ(arr.markers.size(), traj.size());
  for (const auto & m : arr.markers) {
    ASSERT_EQ(m.points.size(), 5u);
  }
}

// Test 12: create_predicted_path_marker_array - empty
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

// Test 13: create_predicted_path_marker_array - one element
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

// Test 14: one PredictedObject + one pose within range → one marker with >0 points
TEST_F(MarkerConversionTest, CreatePredictedObjectsMarkerArrayOne)
{
  visualization_msgs::msg::MarkerArray arr;
  visualization_msgs::msg::Marker base = create_default_marker(
    header_.frame_id, now_, "objects", 7, visualization_msgs::msg::Marker::LINE_LIST,
    create_marker_scale(0.1, 0.1, 0.1), color_);
  // build one object with one predicted path and one pose
  autoware_perception_msgs::msg::PredictedObjects objs;
  autoware_perception_msgs::msg::PredictedObject obj;
  // a very simple shape: a unit‐square box
  obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  autoware_perception_msgs::msg::PredictedPath path;
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 2.0;
  pose.position.z = 0.0;
  path.path.push_back(pose);
  obj.kinematics.predicted_paths.push_back(path);
  objs.objects.push_back(obj);

  geometry_msgs::msg::Pose ego;
  ego.position.x = 0.0;
  ego.position.y = 0.0;
  ego.position.z = 0.0;

  autoware::experimental::marker_utils::create_predicted_objects_marker_array(arr, base, objs, ego);

  // should have exactly one marker
  ASSERT_EQ(arr.markers.size(), 1u);
  const auto & m = arr.markers[0];
  // each corner of the unit‐box plus closing segment: 4 edges → 10 points
  EXPECT_EQ(m.points.size(), 10u);
  // check first two points are the first edge of the box
  EXPECT_DOUBLE_EQ(m.points[0].x, 1.0);
  EXPECT_DOUBLE_EQ(m.points[0].y, 2.0);
  EXPECT_DOUBLE_EQ(m.points[1].x, 1.0);
  EXPECT_DOUBLE_EQ(m.points[1].y, 2.0);
}

// Test 15: one simple rectangular lanelet → one marker, closed ring
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

  visualization_msgs::msg::MarkerArray arr;
  visualization_msgs::msg::Marker marker = create_default_marker(
    header_.frame_id, now_, "lanelets", 5, visualization_msgs::msg::Marker::LINE_STRIP,
    create_marker_scale(0.1, 0.1, 0.1), color_);

  const double Z = 1.0;
  autoware::experimental::marker_utils::create_lanelets_marker_array(lls, marker, arr, Z);

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

// Test 16: create_autoware_geometry_marker_array with a MultiPolygon2d of two rings
TEST_F(MarkerConversionTest, CreateAutowareGeometryMarkerArrayMultiPolygon)
{
  using autoware_utils::MultiPolygon2d;
  using autoware_utils::Polygon2d;

  // build a triangle and a square
  Polygon2d tri;
  {
    auto & r = tri.outer();
    r.push_back({0.0, 0.0});
    r.push_back({1.0, 0.0});
    r.push_back({0.0, 1.0});
  }
  Polygon2d sq;
  {
    auto & r = sq.outer();
    r.push_back({0.0, 0.0});
    r.push_back({2.0, 0.0});
    r.push_back({2.0, 2.0});
    r.push_back({0.0, 2.0});
  }
  MultiPolygon2d mp{tri, sq};

  visualization_msgs::msg::MarkerArray arr;
  auto base = create_default_marker(
    header_.frame_id, now_, "mp_test", 42, visualization_msgs::msg::Marker::LINE_LIST,
    create_marker_scale(0.1, 0.1, 0.1), color_);
  arr.markers.push_back(base);

  // single trajectory point at (5,6,7)
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> traj(1);
  traj[0].pose.position.x = 5.0;
  traj[0].pose.position.y = 6.0;
  traj[0].pose.position.z = 7.0;

  autoware::experimental::marker_utils::create_autoware_geometry_marker_array(mp, arr, 0, traj);

  ASSERT_EQ(arr.markers.size(), 1u);
  const auto & pts = arr.markers[0].points;

  ASSERT_EQ(pts.size(), 4u);

  // first push: trajectory point
  EXPECT_DOUBLE_EQ(pts[0].x, 5.0);
  EXPECT_DOUBLE_EQ(pts[0].y, 6.0);
  EXPECT_DOUBLE_EQ(pts[0].z, 7.0);

  // centroid of triangle (0,0),(1,0),(0,1) is (1/3,1/3)
  EXPECT_DOUBLE_EQ(pts[1].x, 1.0 / 3.0);
  EXPECT_DOUBLE_EQ(pts[1].y, 1.0 / 3.0);
  EXPECT_DOUBLE_EQ(pts[1].z, 0.0);

  // second loop: trajectory point again
  EXPECT_DOUBLE_EQ(pts[2].x, 5.0);
  EXPECT_DOUBLE_EQ(pts[2].y, 6.0);
  EXPECT_DOUBLE_EQ(pts[2].z, 7.0);

  // centroid of square (0,0),(2,0),(2,2),(0,2) is (1,1)
  EXPECT_DOUBLE_EQ(pts[3].x, 1.0);
  EXPECT_DOUBLE_EQ(pts[3].y, 1.0);
  EXPECT_DOUBLE_EQ(pts[3].z, 0.0);
}

// Test 17: create_autoware_geometry_marker_array with empty lanelet
TEST_F(MarkerConversionTest, EmptyLaneletsCustomNS)
{
  lanelet::ConstLanelets empty;
  visualization_msgs::msg::MarkerArray markers;
  markers.markers.push_back(create_default_marker(
    header_.frame_id, now_, "end_lanelets", 2, visualization_msgs::msg::Marker::LINE_LIST,
    create_marker_scale(0.2, 0.2, 0.2), color_));

  autoware::experimental::marker_utils::create_lanelets_marker_array(empty, color_, markers, "foo");

  ASSERT_EQ(markers.markers.size(), 1u);
  EXPECT_EQ(markers.markers[0].ns, "end_lanelets");
}

// Test 18: create_autoware_geometry_marker_array with triangle lanelet
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

  visualization_msgs::msg::MarkerArray markers;
  markers.markers.push_back(create_default_marker(
    header_.frame_id, now_, "goal_lanelets", 5, visualization_msgs::msg::Marker::LINE_STRIP,
    create_marker_scale(0.1, 0.1, 0.1), color_));

  autoware::experimental::marker_utils::create_lanelets_marker_array(lls, color_, markers, "ns");

  ASSERT_EQ(markers.markers.size(), 2u);
  const auto & m = markers.markers.back();
  auto poly = cl.polygon2d().basicPolygon();
  EXPECT_EQ(m.points.size(), poly.size() + 2);
  expect_point_eq(m.points.back(), poly.front().x(), poly.front().y(), 0.0);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
