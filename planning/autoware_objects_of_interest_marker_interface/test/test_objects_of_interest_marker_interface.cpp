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

#include "autoware/objects_of_interest_marker_interface/objects_of_interest_marker_interface.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/shape.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>

using autoware::objects_of_interest_marker_interface::ColorName;
using autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface;

namespace
{
constexpr float near_tol = 1e-4F;
}  // namespace

class ObjectsOfInterestMarkerInterfaceTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(ObjectsOfInterestMarkerInterfaceTest, PublishMarkerArrayPublishesMarkersToSubscribedTopic)
{
  // Arrange
  const auto node = std::make_shared<rclcpp::Node>("test_pub_sub_node");
  ObjectsOfInterestMarkerInterface interface(node.get(), "test_module");

  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 2.0;
  pose.position.z = 0.5;
  pose.orientation.w = 1.0;

  autoware_perception_msgs::msg::Shape shape;
  shape.dimensions.x = 2.0;
  shape.dimensions.y = 1.0;
  shape.dimensions.z = 1.0;

  visualization_msgs::msg::MarkerArray received_msg;
  bool message_received = false;
  const auto sub = node->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/planning/debug/objects_of_interest/test_module", 1,
    [&](const visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
      received_msg = *msg;
      message_received = true;
    });

  interface.insertObjectData(pose, shape, ColorName::GREEN);

  // Wait briefly for intra-process publisher-subscriber discovery
  rclcpp::WallRate discovery_rate(100);
  for (int i = 0; i < 20 && sub->get_publisher_count() == 0; ++i) {
    rclcpp::spin_some(node);
    discovery_rate.sleep();
  }

  // Act
  interface.publishMarkerArray();

  // Spin to receive the published message
  rclcpp::WallRate spin_rate(100);
  for (int i = 0; i < 50 && !message_received; ++i) {
    rclcpp::spin_some(node);
    spin_rate.sleep();
  }

  // Assert
  ASSERT_TRUE(message_received);
  ASSERT_EQ(received_msg.markers.size(), 4u);

  // 1. Arrow marker
  EXPECT_EQ(received_msg.markers[0].type, visualization_msgs::msg::Marker::ARROW);
  EXPECT_EQ(received_msg.markers[0].ns, "test_module_arrow");
  ASSERT_EQ(received_msg.markers[0].points.size(), 2u);
  EXPECT_NEAR(received_msg.markers[0].points[0].x, 1.0, near_tol);
  EXPECT_NEAR(received_msg.markers[0].points[0].y, 2.0, near_tol);
  EXPECT_NEAR(
    received_msg.markers[0].points[1].z,
    pose.position.z + 0.5 * shape.dimensions.z + interface.getHeightOffset(), near_tol);
  EXPECT_GT(received_msg.markers[0].color.g, 0.5f);

  // 2. Circle markers
  EXPECT_EQ(received_msg.markers[1].type, visualization_msgs::msg::Marker::LINE_STRIP);
  EXPECT_EQ(received_msg.markers[1].ns, "test_module_circle1");
  EXPECT_EQ(received_msg.markers[1].points.size(), 21u);

  EXPECT_EQ(received_msg.markers[2].type, visualization_msgs::msg::Marker::LINE_STRIP);
  EXPECT_EQ(received_msg.markers[2].ns, "test_module_circle2");
  EXPECT_EQ(received_msg.markers[2].points.size(), 21u);

  // 3. Name text marker
  EXPECT_EQ(received_msg.markers[3].type, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  EXPECT_EQ(received_msg.markers[3].ns, "test_module_name_text");
  EXPECT_EQ(received_msg.markers[3].text, "test_module");
}

TEST_F(ObjectsOfInterestMarkerInterfaceTest, PublishMarkerArrayDoesNotPublishWithoutSubscriber)
{
  // Arrange
  const auto node = std::make_shared<rclcpp::Node>("test_no_sub_node");
  ObjectsOfInterestMarkerInterface interface(node.get(), "test_module_no_sub");

  geometry_msgs::msg::Pose pose;
  autoware_perception_msgs::msg::Shape shape;
  shape.dimensions.z = 1.0;
  interface.insertObjectData(pose, shape, ColorName::AMBER);

  // Act & Assert
  EXPECT_NO_THROW(interface.publishMarkerArray());
}

TEST_F(
  ObjectsOfInterestMarkerInterfaceTest,
  InsertObjectDataWithCustomColorStoresAndPublishesCustomColor)
{
  // Arrange
  const auto node = std::make_shared<rclcpp::Node>("test_custom_color_node");
  ObjectsOfInterestMarkerInterface interface(node.get(), "test_module_custom");

  geometry_msgs::msg::Pose pose;
  autoware_perception_msgs::msg::Shape shape;
  shape.dimensions.z = 1.0;

  std_msgs::msg::ColorRGBA custom_color;
  custom_color.r = 0.2f;
  custom_color.g = 0.4f;
  custom_color.b = 0.6f;
  custom_color.a = 0.8f;

  visualization_msgs::msg::MarkerArray received_msg;
  bool message_received = false;
  const auto sub = node->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/planning/debug/objects_of_interest/test_module_custom", 1,
    [&](const visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
      received_msg = *msg;
      message_received = true;
    });

  interface.insertObjectDataWithCustomColor(pose, shape, custom_color);

  // Wait briefly for discovery
  rclcpp::WallRate discovery_rate(100);
  for (int i = 0; i < 20 && sub->get_publisher_count() == 0; ++i) {
    rclcpp::spin_some(node);
    discovery_rate.sleep();
  }

  // Act
  interface.publishMarkerArray();

  // Spin to receive
  rclcpp::WallRate spin_rate(100);
  for (int i = 0; i < 50 && !message_received; ++i) {
    rclcpp::spin_some(node);
    spin_rate.sleep();
  }

  // Assert
  ASSERT_TRUE(message_received);
  ASSERT_FALSE(received_msg.markers.empty());
  EXPECT_NEAR(received_msg.markers[0].color.r, 0.2f, near_tol);
  EXPECT_NEAR(received_msg.markers[0].color.g, 0.4f, near_tol);
  EXPECT_NEAR(received_msg.markers[0].color.b, 0.6f, near_tol);
  EXPECT_NEAR(received_msg.markers[0].color.a, 0.8f, near_tol);
}

TEST_F(ObjectsOfInterestMarkerInterfaceTest, SetAndGetHeightOffsetUpdatesHeightOffsetCorrectly)
{
  // Arrange
  const auto node = std::make_shared<rclcpp::Node>("test_offset_node");
  ObjectsOfInterestMarkerInterface interface(node.get(), "test_module");

  // Assert initial default
  EXPECT_DOUBLE_EQ(interface.getHeightOffset(), 0.5);

  // Act
  interface.setHeightOffset(1.2);

  // Assert updated
  EXPECT_DOUBLE_EQ(interface.getHeightOffset(), 1.2);
}

TEST_F(ObjectsOfInterestMarkerInterfaceTest, GetNameReturnsModuleName)
{
  // Arrange
  const auto node = std::make_shared<rclcpp::Node>("test_get_name_node");
  ObjectsOfInterestMarkerInterface interface(node.get(), "unique_module_name");

  // Act
  const auto name = interface.getName();

  // Assert
  EXPECT_EQ(name, "unique_module_name");
}

TEST_F(ObjectsOfInterestMarkerInterfaceTest, GetColorReturnsExpectedRgbaColorForEachColorName)
{
  // Act & Assert: Green
  const auto green = ObjectsOfInterestMarkerInterface::getColor(ColorName::GREEN, 0.6f);
  EXPECT_NEAR(green.a, 0.6f, near_tol);
  EXPECT_GT(green.g, 0.5f);

  // Act & Assert: Amber
  const auto amber = ObjectsOfInterestMarkerInterface::getColor(ColorName::AMBER, 0.7f);
  EXPECT_NEAR(amber.a, 0.7f, near_tol);
  EXPECT_GT(amber.r, 0.5f);
  EXPECT_GT(amber.g, 0.5f);

  // Act & Assert: Red
  const auto red = ObjectsOfInterestMarkerInterface::getColor(ColorName::RED, 0.8f);
  EXPECT_NEAR(red.a, 0.8f, near_tol);
  EXPECT_GT(red.r, 0.5f);

  // Act & Assert: Gray
  const auto gray = ObjectsOfInterestMarkerInterface::getColor(ColorName::GRAY, 0.5f);
  EXPECT_NEAR(gray.a, 0.5f, near_tol);
  EXPECT_GT(gray.r, 0.4f);
  EXPECT_NEAR(gray.r, gray.g, near_tol);
  EXPECT_NEAR(gray.g, gray.b, near_tol);

  // Act & Assert: Default fallback branch
  const auto def = ObjectsOfInterestMarkerInterface::getColor(static_cast<ColorName>(999), 0.9f);
  EXPECT_NEAR(def.a, 0.9f, near_tol);
  EXPECT_GT(def.r, 0.4f);
}
