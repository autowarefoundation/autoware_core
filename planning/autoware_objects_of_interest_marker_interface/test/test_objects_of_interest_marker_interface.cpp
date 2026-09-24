// Copyright 2024 Tier IV, Inc.
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
#include "autoware/objects_of_interest_marker_interface/marker_utils.hpp"
#include "autoware/objects_of_interest_marker_interface/objects_of_interest_marker_interface.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

using autoware::objects_of_interest_marker_interface::ColorName;
using autoware::objects_of_interest_marker_interface::ObjectMarkerData;
using autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface;
namespace coloring = autoware::objects_of_interest_marker_interface::coloring;
namespace marker_utils = autoware::objects_of_interest_marker_interface::marker_utils;

namespace
{
// Floating point tolerance at EXPECT_NEAR and similar checks
constexpr float near_tol = 1e-4F;
}  // namespace

TEST(TestColoring, BasicColors)
{
  const float alpha = 0.8f;
  const auto green = coloring::getGreen(alpha);
  EXPECT_NEAR(green.a, alpha, near_tol);
  EXPECT_GT(green.g, 0.5f);

  const auto amber = coloring::getAmber(alpha);
  EXPECT_NEAR(amber.a, alpha, near_tol);
  EXPECT_GT(amber.r, 0.5f);
  EXPECT_GT(amber.g, 0.5f);

  const auto red = coloring::getRed(alpha);
  EXPECT_NEAR(red.a, alpha, near_tol);
  EXPECT_GT(red.r, 0.5f);

  const auto gray = coloring::getGray(alpha);
  EXPECT_NEAR(gray.a, alpha, near_tol);
  EXPECT_GT(gray.r, 0.4f);
  EXPECT_NEAR(gray.r, gray.g, near_tol);
  EXPECT_NEAR(gray.g, gray.b, near_tol);
}

TEST(TestMarkerUtils, CreateMarkers)
{
  ObjectMarkerData data;
  data.pose.position.x = 1.0;
  data.pose.position.y = 2.0;
  data.pose.position.z = 0.5;
  data.pose.orientation.w = 1.0;
  data.shape.dimensions.x = 2.0;
  data.shape.dimensions.y = 1.5;
  data.shape.dimensions.z = 1.0;
  data.color = coloring::getRed(0.9f);

  // Arrow marker
  const auto arrow = marker_utils::createArrowMarker(1, data, "test", 0.5, 1.0);
  EXPECT_EQ(arrow.id, 1);
  EXPECT_EQ(arrow.type, visualization_msgs::msg::Marker::ARROW);
  EXPECT_EQ(arrow.ns, "test_arrow");
  ASSERT_EQ(arrow.points.size(), 2u);
  EXPECT_NEAR(arrow.points[0].x, 1.0f, near_tol);
  EXPECT_NEAR(arrow.points[0].y, 2.0f, near_tol);

  // Circle marker
  const auto circle = marker_utils::createCircleMarker(2, data, "test_circle", 1.0, 0.5, 0.1);
  EXPECT_EQ(circle.id, 2);
  EXPECT_EQ(circle.type, visualization_msgs::msg::Marker::LINE_STRIP);
  EXPECT_EQ(circle.points.size(), 21u);  // 20 points + closing point

  // Name text marker
  const auto text = marker_utils::createNameTextMarker(3, data, "test_name", 0.5, 0.8);
  EXPECT_EQ(text.id, 3);
  EXPECT_EQ(text.type, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  EXPECT_EQ(text.text, "test_name");

  // Target marker array
  const auto marker_array = marker_utils::createTargetMarker(4, data, "test_target", 0.5, 1.0, 0.1);
  EXPECT_EQ(marker_array.markers.size(), 4u);
}

TEST(TestObjectsOfInterestMarkerInterface, InterfaceOperations)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_objects_of_interest_node");
  ObjectsOfInterestMarkerInterface interface(node.get(), "test_module");

  EXPECT_EQ(interface.getName(), "test_module");
  EXPECT_DOUBLE_EQ(interface.getHeightOffset(), 0.5);

  interface.setHeightOffset(1.2);
  EXPECT_DOUBLE_EQ(interface.getHeightOffset(), 1.2);

  // Test getColor for each enum
  const auto gray = ObjectsOfInterestMarkerInterface::getColor(ColorName::GRAY, 0.5f);
  EXPECT_NEAR(gray.a, 0.5f, near_tol);
  const auto green = ObjectsOfInterestMarkerInterface::getColor(ColorName::GREEN, 0.6f);
  EXPECT_NEAR(green.a, 0.6f, near_tol);
  const auto amber = ObjectsOfInterestMarkerInterface::getColor(ColorName::AMBER, 0.7f);
  EXPECT_NEAR(amber.a, 0.7f, near_tol);
  const auto red = ObjectsOfInterestMarkerInterface::getColor(ColorName::RED, 0.8f);
  EXPECT_NEAR(red.a, 0.8f, near_tol);
  // Default branch
  const auto def = ObjectsOfInterestMarkerInterface::getColor(static_cast<ColorName>(999), 0.9f);
  EXPECT_NEAR(def.a, 0.9f, near_tol);

  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  autoware_perception_msgs::msg::Shape shape;
  shape.dimensions.x = 2.0;
  shape.dimensions.y = 1.0;
  shape.dimensions.z = 1.0;

  // Insert data with enum color
  interface.insertObjectData(pose, shape, ColorName::GREEN);

  // Insert data with custom color
  interface.insertObjectDataWithCustomColor(pose, shape, red);

  // Publish with 0 subscribers: should do nothing safely
  interface.publishMarkerArray();

  // Create subscriber and publish
  size_t received_count = 0;
  auto sub = node->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/planning/debug/objects_of_interest/test_module", 1,
    [&received_count](const visualization_msgs::msg::MarkerArray::ConstSharedPtr) {
      ++received_count;
    });

  // Re-insert data
  interface.insertObjectData(pose, shape, ColorName::AMBER);

  // Spin once to match subscriber
  rclcpp::WallRate rate(100);
  for (int i = 0; i < 10; ++i) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  interface.publishMarkerArray();

  rclcpp::shutdown();
}
