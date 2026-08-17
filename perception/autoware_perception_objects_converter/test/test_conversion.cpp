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

#include "../src/conversion.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <set>
#include <vector>

namespace
{
using autoware::perception_objects_converter::convert;
using autoware::perception_objects_converter::convert_object;
using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;
using autoware_perception_msgs::msg::ObjectClassification;

DetectedObject make_detected_object(const bool has_twist)
{
  DetectedObject object;
  object.existence_probability = 0.75F;

  ObjectClassification classification;
  classification.label = ObjectClassification::CAR;
  classification.probability = 0.9F;
  object.classification.push_back(classification);

  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 4.0;
  object.shape.dimensions.y = 1.8;
  object.shape.dimensions.z = 1.5;

  object.kinematics.pose_with_covariance.pose.position.x = 1.0;
  object.kinematics.pose_with_covariance.pose.position.y = 2.0;
  object.kinematics.pose_with_covariance.pose.position.z = 3.0;
  object.kinematics.pose_with_covariance.pose.orientation.w = 1.0;

  object.kinematics.has_twist = has_twist;
  object.kinematics.twist_with_covariance.twist.linear.x = 5.0;
  object.kinematics.twist_with_covariance.twist.angular.z = 0.5;

  return object;
}

// Deterministic UUID generator so id assignment can be asserted without relying on randomness.
autoware::perception_objects_converter::UuidGenerator make_counting_uuid_generator()
{
  auto counter = std::make_shared<uint8_t>(0);
  return [counter]() {
    unique_identifier_msgs::msg::UUID uuid;
    uuid.uuid.fill(*counter);
    ++(*counter);
    return uuid;
  };
}
}  // namespace

TEST(ConversionTest, HeaderIsPropagated)
{
  DetectedObjects detected_objects;
  detected_objects.header.frame_id = "map";
  detected_objects.header.stamp.sec = 42;
  detected_objects.header.stamp.nanosec = 7;

  const auto predicted_objects = convert(detected_objects, make_counting_uuid_generator());

  EXPECT_EQ(predicted_objects.header.frame_id, "map");
  EXPECT_EQ(predicted_objects.header.stamp.sec, 42);
  EXPECT_EQ(predicted_objects.header.stamp.nanosec, 7U);
  EXPECT_TRUE(predicted_objects.objects.empty());
}

TEST(ConversionTest, FieldMappingIsCopiedExactly)
{
  const auto detected = make_detected_object(/*has_twist=*/true);

  const auto predicted = convert_object(detected, make_counting_uuid_generator());

  EXPECT_FLOAT_EQ(predicted.existence_probability, detected.existence_probability);

  ASSERT_EQ(predicted.classification.size(), 1U);
  EXPECT_EQ(predicted.classification[0].label, ObjectClassification::CAR);
  EXPECT_FLOAT_EQ(predicted.classification[0].probability, 0.9F);

  EXPECT_EQ(predicted.shape.type, autoware_perception_msgs::msg::Shape::BOUNDING_BOX);
  EXPECT_DOUBLE_EQ(predicted.shape.dimensions.x, 4.0);
  EXPECT_DOUBLE_EQ(predicted.shape.dimensions.y, 1.8);
  EXPECT_DOUBLE_EQ(predicted.shape.dimensions.z, 1.5);

  EXPECT_DOUBLE_EQ(predicted.kinematics.initial_pose_with_covariance.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(predicted.kinematics.initial_pose_with_covariance.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(predicted.kinematics.initial_pose_with_covariance.pose.position.z, 3.0);
  EXPECT_DOUBLE_EQ(predicted.kinematics.initial_pose_with_covariance.pose.orientation.w, 1.0);
}

TEST(ConversionTest, TwistCopiedWhenHasTwistIsTrue)
{
  const auto detected = make_detected_object(/*has_twist=*/true);

  const auto predicted = convert_object(detected, make_counting_uuid_generator());

  EXPECT_DOUBLE_EQ(predicted.kinematics.initial_twist_with_covariance.twist.linear.x, 5.0);
  EXPECT_DOUBLE_EQ(predicted.kinematics.initial_twist_with_covariance.twist.angular.z, 0.5);
}

TEST(ConversionTest, TwistLeftDefaultWhenHasTwistIsFalse)
{
  const auto detected = make_detected_object(/*has_twist=*/false);

  const auto predicted = convert_object(detected, make_counting_uuid_generator());

  // has_twist=false => twist must remain at the message default (all zeros), not the detected one.
  EXPECT_DOUBLE_EQ(predicted.kinematics.initial_twist_with_covariance.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(predicted.kinematics.initial_twist_with_covariance.twist.angular.z, 0.0);
}

TEST(ConversionTest, EveryObjectGetsADistinctUuid)
{
  DetectedObjects detected_objects;
  detected_objects.objects.push_back(make_detected_object(/*has_twist=*/true));
  detected_objects.objects.push_back(make_detected_object(/*has_twist=*/false));
  detected_objects.objects.push_back(make_detected_object(/*has_twist=*/true));

  const auto predicted_objects = convert(detected_objects, make_counting_uuid_generator());

  ASSERT_EQ(predicted_objects.objects.size(), 3U);

  std::set<std::array<uint8_t, 16>> seen;
  for (const auto & object : predicted_objects.objects) {
    seen.insert(object.object_id.uuid);
  }
  EXPECT_EQ(seen.size(), 3U) << "Each converted object must receive a distinct UUID";
}

TEST(ConversionTest, DefaultGeneratorProducesNonZeroDistinctUuids)
{
  DetectedObjects detected_objects;
  detected_objects.objects.push_back(make_detected_object(/*has_twist=*/true));
  detected_objects.objects.push_back(make_detected_object(/*has_twist=*/true));

  // Exercise the production default generator (autoware_utils_uuid::generate_uuid).
  const auto predicted_objects = convert(detected_objects);

  ASSERT_EQ(predicted_objects.objects.size(), 2U);

  const std::array<uint8_t, 16> zero_uuid{};
  for (const auto & object : predicted_objects.objects) {
    EXPECT_NE(object.object_id.uuid, zero_uuid) << "UUID must be non-zero";
  }
  EXPECT_NE(
    predicted_objects.objects[0].object_id.uuid, predicted_objects.objects[1].object_id.uuid)
    << "Distinct objects should receive distinct UUIDs";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
