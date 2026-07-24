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

#include "autoware/object_recognition_utils/pointcloud_classification.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace autoware::object_recognition_utils
{

TEST(PointCloudClassification, EnumValues)
{
  EXPECT_EQ(static_cast<std::uint8_t>(PointCloudClassification::CAR), 0U);
  EXPECT_EQ(static_cast<std::uint8_t>(PointCloudClassification::TRUCK), 1U);
  EXPECT_EQ(static_cast<std::uint8_t>(PointCloudClassification::BUS), 2U);
  EXPECT_EQ(static_cast<std::uint8_t>(PointCloudClassification::MOTORCYCLE), 3U);
  EXPECT_EQ(static_cast<std::uint8_t>(PointCloudClassification::BICYCLE), 4U);
  EXPECT_EQ(static_cast<std::uint8_t>(PointCloudClassification::PEDESTRIAN), 5U);
  EXPECT_EQ(static_cast<std::uint8_t>(PointCloudClassification::ANIMAL), 6U);
  EXPECT_EQ(static_cast<std::uint8_t>(PointCloudClassification::HAZARD), 7U);
  EXPECT_EQ(static_cast<std::uint8_t>(PointCloudClassification::FLAT_SURFACE), 8U);
  EXPECT_EQ(static_cast<std::uint8_t>(PointCloudClassification::STRUCTURE), 9U);
  EXPECT_EQ(static_cast<std::uint8_t>(PointCloudClassification::VEGETATION), 10U);
  EXPECT_EQ(static_cast<std::uint8_t>(PointCloudClassification::NOISE), 11U);
}

TEST(PointCloudClassification, ToString)
{
  EXPECT_EQ(to_string(PointCloudClassification::CAR), "CAR");
  EXPECT_EQ(to_string(PointCloudClassification::TRUCK), "TRUCK");
  EXPECT_EQ(to_string(PointCloudClassification::BUS), "BUS");
  EXPECT_EQ(to_string(PointCloudClassification::MOTORCYCLE), "MOTORCYCLE");
  EXPECT_EQ(to_string(PointCloudClassification::BICYCLE), "BICYCLE");
  EXPECT_EQ(to_string(PointCloudClassification::PEDESTRIAN), "PEDESTRIAN");
  EXPECT_EQ(to_string(PointCloudClassification::ANIMAL), "ANIMAL");
  EXPECT_EQ(to_string(PointCloudClassification::HAZARD), "HAZARD");
  EXPECT_EQ(to_string(PointCloudClassification::FLAT_SURFACE), "FLAT_SURFACE");
  EXPECT_EQ(to_string(PointCloudClassification::STRUCTURE), "STRUCTURE");
  EXPECT_EQ(to_string(PointCloudClassification::VEGETATION), "VEGETATION");
  EXPECT_EQ(to_string(PointCloudClassification::NOISE), "NOISE");
  EXPECT_EQ(to_string(static_cast<PointCloudClassification>(255U)), "UNKNOWN");
}

TEST(PointCloudClassification, TryIntoObject)
{
  EXPECT_EQ(try_into_object(PointCloudClassification::CAR), ObjectClassification::CAR);
  EXPECT_EQ(try_into_object(PointCloudClassification::TRUCK), ObjectClassification::TRUCK);
  EXPECT_EQ(try_into_object(PointCloudClassification::BUS), ObjectClassification::BUS);
  EXPECT_EQ(
    try_into_object(PointCloudClassification::MOTORCYCLE), ObjectClassification::MOTORCYCLE);
  EXPECT_EQ(try_into_object(PointCloudClassification::BICYCLE), ObjectClassification::BICYCLE);
  EXPECT_EQ(
    try_into_object(PointCloudClassification::PEDESTRIAN), ObjectClassification::PEDESTRIAN);
  EXPECT_EQ(try_into_object(PointCloudClassification::ANIMAL), ObjectClassification::ANIMAL);
  EXPECT_EQ(try_into_object(PointCloudClassification::HAZARD), ObjectClassification::HAZARD);

  EXPECT_FALSE(try_into_object(PointCloudClassification::FLAT_SURFACE).has_value());
  EXPECT_FALSE(try_into_object(PointCloudClassification::STRUCTURE).has_value());
  EXPECT_FALSE(try_into_object(PointCloudClassification::VEGETATION).has_value());
  EXPECT_FALSE(try_into_object(PointCloudClassification::NOISE).has_value());
  EXPECT_FALSE(try_into_object(static_cast<PointCloudClassification>(255U)).has_value());
}

TEST(PointCloudClassification, TryIntoSemantic)
{
  EXPECT_EQ(try_into_semantic(ObjectClassification::CAR), PointCloudClassification::CAR);
  EXPECT_EQ(try_into_semantic(ObjectClassification::TRUCK), PointCloudClassification::TRUCK);
  EXPECT_EQ(try_into_semantic(ObjectClassification::BUS), PointCloudClassification::BUS);
  EXPECT_EQ(try_into_semantic(ObjectClassification::TRAILER), PointCloudClassification::TRUCK);
  EXPECT_EQ(
    try_into_semantic(ObjectClassification::MOTORCYCLE), PointCloudClassification::MOTORCYCLE);
  EXPECT_EQ(try_into_semantic(ObjectClassification::BICYCLE), PointCloudClassification::BICYCLE);
  EXPECT_EQ(
    try_into_semantic(ObjectClassification::PEDESTRIAN), PointCloudClassification::PEDESTRIAN);
  EXPECT_EQ(try_into_semantic(ObjectClassification::ANIMAL), PointCloudClassification::ANIMAL);
  EXPECT_EQ(try_into_semantic(ObjectClassification::HAZARD), PointCloudClassification::HAZARD);

  EXPECT_FALSE(try_into_semantic(ObjectClassification::UNKNOWN).has_value());
  EXPECT_FALSE(try_into_semantic(ObjectClassification::OVER_DRIVABLE).has_value());
  EXPECT_FALSE(try_into_semantic(ObjectClassification::UNDER_DRIVABLE).has_value());
}

TEST(PointCloudClassification, IsObjectCompatible)
{
  EXPECT_TRUE(is_object_compatible(PointCloudClassification::CAR));
  EXPECT_TRUE(is_object_compatible(PointCloudClassification::TRUCK));
  EXPECT_TRUE(is_object_compatible(PointCloudClassification::BUS));
  EXPECT_TRUE(is_object_compatible(PointCloudClassification::MOTORCYCLE));
  EXPECT_TRUE(is_object_compatible(PointCloudClassification::BICYCLE));
  EXPECT_TRUE(is_object_compatible(PointCloudClassification::PEDESTRIAN));
  EXPECT_TRUE(is_object_compatible(PointCloudClassification::ANIMAL));
  EXPECT_TRUE(is_object_compatible(PointCloudClassification::HAZARD));
  EXPECT_FALSE(is_object_compatible(PointCloudClassification::FLAT_SURFACE));
  EXPECT_FALSE(is_object_compatible(PointCloudClassification::STRUCTURE));
  EXPECT_FALSE(is_object_compatible(PointCloudClassification::VEGETATION));
  EXPECT_FALSE(is_object_compatible(PointCloudClassification::NOISE));
}

TEST(PointCloudClassification, Roundtrip)
{
  struct RoundtripExpectation
  {
    ObjectLabel input;
    ObjectLabel expected_output;
  };

  constexpr RoundtripExpectation object_labels[] = {
    {ObjectClassification::CAR, ObjectClassification::CAR},
    {ObjectClassification::TRUCK, ObjectClassification::TRUCK},
    {ObjectClassification::BUS, ObjectClassification::BUS},
    {ObjectClassification::TRAILER, ObjectClassification::TRUCK},
    {ObjectClassification::MOTORCYCLE, ObjectClassification::MOTORCYCLE},
    {ObjectClassification::BICYCLE, ObjectClassification::BICYCLE},
    {ObjectClassification::PEDESTRIAN, ObjectClassification::PEDESTRIAN},
    {ObjectClassification::ANIMAL, ObjectClassification::ANIMAL},
    {ObjectClassification::HAZARD, ObjectClassification::HAZARD}};

  for (const auto & expectation : object_labels) {
    const auto pointcloud = try_into_semantic(expectation.input);
    ASSERT_TRUE(pointcloud.has_value());
    EXPECT_EQ(try_into_object(pointcloud.value()), expectation.expected_output);
  }
}

TEST(PointCloudClassification, ConstexprToString)
{
  constexpr auto classification = PointCloudClassification::CAR;
  constexpr auto str = to_string(classification);
  static_assert(str == "CAR");
  EXPECT_EQ(str, "CAR");
}

}  // namespace autoware::object_recognition_utils
