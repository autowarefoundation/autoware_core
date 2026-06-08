// Copyright 2026 The Autoware Contributors
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

#include "../src/lanelet2_map_visualization.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <memory>
#include <string>

namespace
{
using autoware::lanelet2_map_visualizer::create_lanelet_map_marker_array;

// Build a single straight road lanelet (left/right bounds) and add it to a fresh map.
lanelet::LaneletMapPtr make_road_lanelet_map()
{
  lanelet::Point3d p1{1, 0, 0, 0};
  lanelet::Point3d p2{2, 10, 0, 0};
  lanelet::Point3d p3{3, 0, 5, 0};
  lanelet::Point3d p4{4, 10, 5, 0};

  lanelet::LineString3d left_ls(5, {p1, p2});
  lanelet::LineString3d right_ls(6, {p3, p4});

  lanelet::Lanelet ll(7, left_ls, right_ls);
  ll.attributes()["subtype"] = "road";

  lanelet::LaneletMapPtr map = std::make_shared<lanelet::LaneletMap>();
  map->add(ll);
  return map;
}

// Collect the set of namespaces present in a MarkerArray.
bool has_namespace(const visualization_msgs::msg::MarkerArray & arr, const std::string & ns)
{
  return std::any_of(
    arr.markers.begin(), arr.markers.end(), [&](const auto & m) { return m.ns == ns; });
}

const visualization_msgs::msg::Marker * find_namespace(
  const visualization_msgs::msg::MarkerArray & arr, const std::string & ns)
{
  for (const auto & m : arr.markers) {
    if (m.ns == ns) {
      return &m;
    }
  }
  return nullptr;
}
}  // namespace

TEST(CreateLaneletMapMarkerArray, EmptyMapProducesEmptyArray)
{
  const auto map = std::make_shared<lanelet::LaneletMap>();

  const auto markers = create_lanelet_map_marker_array(map, /*viz_centerline=*/true);

  EXPECT_TRUE(markers.markers.empty());
}

TEST(CreateLaneletMapMarkerArray, RoadLaneletProducesRoadTriangleAndBoundaryMarkers)
{
  const auto map = make_road_lanelet_map();

  const auto markers = create_lanelet_map_marker_array(map, /*viz_centerline=*/true);

  ASSERT_FALSE(markers.markers.empty());

  // The road triangle marker must be present under the "road_lanelets" namespace.
  const auto * road_triangle = find_namespace(markers, "road_lanelets");
  ASSERT_NE(road_triangle, nullptr);
  EXPECT_EQ(road_triangle->type, visualization_msgs::msg::Marker::TRIANGLE_LIST);
  EXPECT_EQ(road_triangle->header.frame_id, "map");

  // Every produced marker is anchored to the "map" frame.
  for (const auto & marker : markers.markers) {
    EXPECT_EQ(marker.header.frame_id, "map");
  }

  // The road boundary markers must be present.
  EXPECT_TRUE(has_namespace(markers, "left_lane_bound"));
  EXPECT_TRUE(has_namespace(markers, "right_lane_bound"));
}

TEST(CreateLaneletMapMarkerArray, RoadTriangleUsesRoadColor)
{
  const auto map = make_road_lanelet_map();

  const auto markers = create_lanelet_map_marker_array(map, /*viz_centerline=*/true);

  const auto * road_triangle = find_namespace(markers, "road_lanelets");
  ASSERT_NE(road_triangle, nullptr);
  ASSERT_FALSE(road_triangle->colors.empty());

  // The per-vertex colors carry the hard-coded road color (0.27, 0.27, 0.27, 0.999).
  const auto & color = road_triangle->colors.front();
  EXPECT_FLOAT_EQ(color.r, 0.27f);
  EXPECT_FLOAT_EQ(color.g, 0.27f);
  EXPECT_FLOAT_EQ(color.b, 0.27f);
  EXPECT_FLOAT_EQ(color.a, 0.999f);
}

TEST(CreateLaneletMapMarkerArray, CurbstoneLineStringProducesCurbstoneMarkerWithCurbstoneColor)
{
  lanelet::Point3d c1{10, 0, 0, 0};
  lanelet::Point3d c2{11, 5, 0, 0};
  lanelet::LineString3d curb(12, {c1, c2});
  curb.attributes()["type"] = "curbstone";

  const auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(curb);

  const auto markers = create_lanelet_map_marker_array(map, /*viz_centerline=*/true);

  const auto * curbstone_marker = find_namespace(markers, "curbstone");
  ASSERT_NE(curbstone_marker, nullptr);

  // The curbstone marker carries the hard-coded curbstone color (0.1, 0.1, 0.2, 0.999).
  EXPECT_FLOAT_EQ(curbstone_marker->color.r, 0.1f);
  EXPECT_FLOAT_EQ(curbstone_marker->color.g, 0.1f);
  EXPECT_FLOAT_EQ(curbstone_marker->color.b, 0.2f);
  EXPECT_FLOAT_EQ(curbstone_marker->color.a, 0.999f);
}

TEST(CreateLaneletMapMarkerArray, CenterlineFlagTogglesCenterlineMarkers)
{
  const auto map = make_road_lanelet_map();

  const auto with_centerline = create_lanelet_map_marker_array(map, /*viz_centerline=*/true);
  const auto without_centerline = create_lanelet_map_marker_array(map, /*viz_centerline=*/false);

  // With the centerline enabled, the center-line markers are emitted; the boundary markers are
  // present in both cases, so this isolates the centerline branch.
  EXPECT_TRUE(has_namespace(with_centerline, "center_lane_line"));
  EXPECT_TRUE(has_namespace(with_centerline, "center_line_arrows"));

  EXPECT_FALSE(has_namespace(without_centerline, "center_lane_line"));
  EXPECT_FALSE(has_namespace(without_centerline, "center_line_arrows"));

  // The left boundary is unaffected by the centerline flag.
  EXPECT_TRUE(has_namespace(with_centerline, "left_lane_bound"));
  EXPECT_TRUE(has_namespace(without_centerline, "left_lane_bound"));

  // Disabling the centerline strictly reduces the marker count.
  EXPECT_LT(without_centerline.markers.size(), with_centerline.markers.size());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
