// Copyright 2026 TIER IV, Inc.
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

#include "autoware/objects_of_interest_marker_interface/coloring.hpp"
#include "autoware/objects_of_interest_marker_interface/marker_data.hpp"
#include "autoware/objects_of_interest_marker_interface/marker_utils.hpp"
#include "autoware/objects_of_interest_marker_interface/objects_of_interest_marker_interface.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/time.hpp>

#include <gtest/gtest.h>

#include <string>

namespace autoware::objects_of_interest_marker_interface
{
namespace
{
constexpr float kAlpha = 0.5f;
constexpr float kEps = 1e-4f;

// Compares a marker's header.stamp against an expected ROS time by raw fields,
// avoiding rclcpp::Time clock-source comparison (the message stores a
// source-less builtin_interfaces::msg::Time).
void expect_stamp_eq(const builtin_interfaces::msg::Time & actual, const rclcpp::Time & expected)
{
  const builtin_interfaces::msg::Time expected_msg = expected;
  EXPECT_EQ(actual.sec, expected_msg.sec);
  EXPECT_EQ(actual.nanosec, expected_msg.nanosec);
}

ObjectMarkerData make_data()
{
  ObjectMarkerData data;
  data.pose.position.x = 1.0;
  data.pose.position.y = 2.0;
  data.pose.position.z = 3.0;
  data.shape.dimensions.z = 4.0;  // height = 0.5 * 4.0 = 2.0
  data.color.r = 0.1f;
  data.color.g = 0.2f;
  data.color.b = 0.3f;
  data.color.a = 0.9f;
  return data;
}
}  // namespace

// ---------------------------------------------------------------------------
// coloring: pin the exact hex-decode results (the >>16 / <<48>>56 / <<56>>56
// bit math is non-obvious and worth a regression test).
// ---------------------------------------------------------------------------
TEST(Coloring, GetGreenDecodes0x00e676)
{
  const auto color = coloring::getGreen(kAlpha);
  EXPECT_NEAR(color.r, 0x00 / 255.0f, kEps);
  EXPECT_NEAR(color.g, 0xe6 / 255.0f, kEps);
  EXPECT_NEAR(color.b, 0x76 / 255.0f, kEps);
  EXPECT_FLOAT_EQ(color.a, kAlpha);
}

TEST(Coloring, GetAmberDecodes0xffea00)
{
  const auto color = coloring::getAmber(kAlpha);
  EXPECT_NEAR(color.r, 0xff / 255.0f, kEps);
  EXPECT_NEAR(color.g, 0xea / 255.0f, kEps);
  EXPECT_NEAR(color.b, 0x00 / 255.0f, kEps);
  EXPECT_FLOAT_EQ(color.a, kAlpha);
}

TEST(Coloring, GetRedDecodes0xff3d00)
{
  const auto color = coloring::getRed(kAlpha);
  EXPECT_NEAR(color.r, 0xff / 255.0f, kEps);
  EXPECT_NEAR(color.g, 0x3d / 255.0f, kEps);
  EXPECT_NEAR(color.b, 0x00 / 255.0f, kEps);
  EXPECT_FLOAT_EQ(color.a, kAlpha);
}

TEST(Coloring, GetGrayDecodes0xbdbdbd)
{
  const auto color = coloring::getGray(kAlpha);
  EXPECT_NEAR(color.r, 0xbd / 255.0f, kEps);
  EXPECT_NEAR(color.g, 0xbd / 255.0f, kEps);
  EXPECT_NEAR(color.b, 0xbd / 255.0f, kEps);
  EXPECT_FLOAT_EQ(color.a, kAlpha);
}

TEST(Coloring, DefaultAlphaIsApplied)
{
  // The free functions take alpha explicitly; verify alpha is forwarded.
  EXPECT_FLOAT_EQ(coloring::getGreen(0.25f).a, 0.25f);
  EXPECT_FLOAT_EQ(coloring::getGray(1.0f).a, 1.0f);
}

// ---------------------------------------------------------------------------
// getColor: each ColorName maps to the matching coloring helper, and an
// out-of-range value falls through to the GRAY default branch.
// ---------------------------------------------------------------------------
TEST(GetColor, MapsEachColorName)
{
  const float a = 0.7f;
  const auto green = ObjectsOfInterestMarkerInterface::getColor(ColorName::GREEN, a);
  EXPECT_NEAR(green.r, 0x00 / 255.0f, kEps);
  EXPECT_NEAR(green.g, 0xe6 / 255.0f, kEps);
  EXPECT_NEAR(green.b, 0x76 / 255.0f, kEps);

  const auto amber = ObjectsOfInterestMarkerInterface::getColor(ColorName::AMBER, a);
  EXPECT_NEAR(amber.r, 0xff / 255.0f, kEps);
  EXPECT_NEAR(amber.g, 0xea / 255.0f, kEps);

  const auto red = ObjectsOfInterestMarkerInterface::getColor(ColorName::RED, a);
  EXPECT_NEAR(red.r, 0xff / 255.0f, kEps);
  EXPECT_NEAR(red.g, 0x3d / 255.0f, kEps);

  const auto gray = ObjectsOfInterestMarkerInterface::getColor(ColorName::GRAY, a);
  EXPECT_NEAR(gray.r, 0xbd / 255.0f, kEps);
}

TEST(GetColor, DefaultBranchReturnsGray)
{
  // Force the switch default branch with an out-of-enum value.
  const auto color = ObjectsOfInterestMarkerInterface::getColor(static_cast<ColorName>(999), 0.4f);
  EXPECT_NEAR(color.r, 0xbd / 255.0f, kEps);
  EXPECT_NEAR(color.g, 0xbd / 255.0f, kEps);
  EXPECT_NEAR(color.b, 0xbd / 255.0f, kEps);
  EXPECT_FLOAT_EQ(color.a, 0.4f);
}

TEST(GetColor, DefaultAlphaArgument)
{
  // The default alpha argument (0.99f) is applied when omitted.
  const auto color = ObjectsOfInterestMarkerInterface::getColor(ColorName::GRAY);
  EXPECT_FLOAT_EQ(color.a, 0.99f);
}

// ---------------------------------------------------------------------------
// createArrowMarker: 2 points, ARROW type, "_arrow" namespace, z-stacking.
// ---------------------------------------------------------------------------
TEST(MarkerUtils, CreateArrowMarker)
{
  const auto data = make_data();
  const rclcpp::Time stamp(123, 456);
  const auto marker = marker_utils::createArrowMarker(
    7, data, "mod", /*height_offset=*/0.5, /*arrow_length=*/2.0, stamp);

  EXPECT_EQ(marker.ns, "mod_arrow");
  EXPECT_EQ(marker.id, 7);
  EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::ARROW);
  EXPECT_EQ(marker.color.r, data.color.r);
  EXPECT_EQ(marker.color.a, data.color.a);

  // scale = (0.25*len, 0.5*len, 0.5*len)
  EXPECT_DOUBLE_EQ(marker.scale.x, 0.25 * 2.0);
  EXPECT_DOUBLE_EQ(marker.scale.y, 0.5 * 2.0);
  EXPECT_DOUBLE_EQ(marker.scale.z, 0.5 * 2.0);

  ASSERT_EQ(marker.points.size(), 2u);
  const double height = 0.5 * data.shape.dimensions.z;  // 2.0
  // src is higher than dst by arrow_length.
  EXPECT_DOUBLE_EQ(marker.points[0].z, data.pose.position.z + height + 0.5 + 2.0);
  EXPECT_DOUBLE_EQ(marker.points[1].z, data.pose.position.z + height + 0.5);
  EXPECT_DOUBLE_EQ(marker.points[0].x, data.pose.position.x);
  EXPECT_DOUBLE_EQ(marker.points[1].y, data.pose.position.y);

  // Injected stamp is used verbatim (no internal clock).
  expect_stamp_eq(marker.header.stamp, stamp);
}

// ---------------------------------------------------------------------------
// createCircleMarker: 21 points with first == last closure, LINE_STRIP type.
// ---------------------------------------------------------------------------
TEST(MarkerUtils, CreateCircleMarkerClosesLoop)
{
  const auto data = make_data();
  const rclcpp::Time stamp(10, 20);
  const auto marker = marker_utils::createCircleMarker(
    3, data, "ring", /*radius=*/1.0, /*height_offset=*/0.25, /*line_width=*/0.1, stamp);

  EXPECT_EQ(marker.ns, "ring");
  EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::LINE_STRIP);
  ASSERT_EQ(marker.points.size(), 21u);  // 20 + closing point

  // First and last point are identical (closure).
  EXPECT_DOUBLE_EQ(marker.points.front().x, marker.points.back().x);
  EXPECT_DOUBLE_EQ(marker.points.front().y, marker.points.back().y);
  EXPECT_DOUBLE_EQ(marker.points.front().z, marker.points.back().z);

  // First point: theta = 0 -> (x + radius, y), shared z.
  const double height = 0.5 * data.shape.dimensions.z;
  EXPECT_NEAR(marker.points.front().x, data.pose.position.x + 1.0, 1e-3);
  EXPECT_NEAR(marker.points.front().y, data.pose.position.y, 1e-3);
  EXPECT_DOUBLE_EQ(marker.points.front().z, data.pose.position.z + height + 0.25);

  expect_stamp_eq(marker.header.stamp, stamp);
}

// ---------------------------------------------------------------------------
// createNameTextMarker: text == name, gray color, TEXT_VIEW_FACING type.
// ---------------------------------------------------------------------------
TEST(MarkerUtils, CreateNameTextMarker)
{
  const auto data = make_data();
  const rclcpp::Time stamp(5, 0);
  const auto marker = marker_utils::createNameTextMarker(
    9, data, "label", /*height_offset=*/1.0, /*text_size=*/0.5, stamp);

  EXPECT_EQ(marker.ns, "label_name_text");
  EXPECT_EQ(marker.text, "label");
  EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);

  // Color is gray with the data's alpha.
  EXPECT_NEAR(marker.color.r, 0xbd / 255.0f, kEps);
  EXPECT_NEAR(marker.color.g, 0xbd / 255.0f, kEps);
  EXPECT_NEAR(marker.color.b, 0xbd / 255.0f, kEps);
  EXPECT_FLOAT_EQ(marker.color.a, data.color.a);

  // Pose copied from data with z raised by height + offset.
  const double height = 0.5 * data.shape.dimensions.z;
  EXPECT_DOUBLE_EQ(marker.pose.position.x, data.pose.position.x);
  EXPECT_DOUBLE_EQ(marker.pose.position.z, data.pose.position.z + height + 1.0);

  expect_stamp_eq(marker.header.stamp, stamp);
}

// ---------------------------------------------------------------------------
// createTargetMarker: 4 markers with the expected namespaces, all sharing the
// injected stamp.
// ---------------------------------------------------------------------------
TEST(MarkerUtils, CreateTargetMarkerAssembly)
{
  const auto data = make_data();
  const rclcpp::Time stamp(99, 1);
  const auto array = marker_utils::createTargetMarker(
    2, data, "tgt", /*height_offset=*/0.5, /*arrow_length=*/1.0, /*line_width=*/0.1, stamp);

  ASSERT_EQ(array.markers.size(), 4u);
  EXPECT_EQ(array.markers[0].ns, "tgt_arrow");
  EXPECT_EQ(array.markers[1].ns, "tgt_circle1");
  EXPECT_EQ(array.markers[2].ns, "tgt_circle2");
  EXPECT_EQ(array.markers[3].ns, "tgt_name_text");

  // All markers share the single injected stamp.
  for (const auto & marker : array.markers) {
    expect_stamp_eq(marker.header.stamp, stamp);
  }

  // circle2 has a larger radius than circle1.
  EXPECT_GT(array.markers[2].points.front().x, array.markers[1].points.front().x);
}

}  // namespace autoware::objects_of_interest_marker_interface

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
