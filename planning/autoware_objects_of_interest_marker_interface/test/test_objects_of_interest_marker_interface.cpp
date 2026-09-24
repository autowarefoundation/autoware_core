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
