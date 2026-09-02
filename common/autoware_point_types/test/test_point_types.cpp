// Copyright 2021 Tier IV, Inc.
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

#include "autoware/point_types/types.hpp"

#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

TEST(PointEquality, PointXYZI)
{
  using autoware::point_types::PointXYZI;

  PointXYZI pt0{0, 1, 2, 3};
  PointXYZI pt1{0, 1, 2, 3};
  EXPECT_EQ(pt0, pt1);
  EXPECT_TRUE(pt0 == pt1);
}

TEST(PointEquality, PointXYZIRCT)
{
  using autoware::point_types::PointXYZIRCT;

  PointXYZIRCT pt0{0, 1, 2, 3, 4, 5, 6};
  PointXYZIRCT pt1{0, 1, 2, 3, 4, 5, 6};
  EXPECT_EQ(pt0, pt1);
  EXPECT_TRUE(pt0 == pt1);

  pt1.time_stamp = 7;
  EXPECT_FALSE(pt0 == pt1);
}

TEST(PointEquality, PointXYZIRADRT)
{
  using autoware::point_types::PointXYZIRADRT;

  PointXYZIRADRT pt0{0, 1, 2, 3, 4, 5, 6, 7, 8};
  PointXYZIRADRT pt1{0, 1, 2, 3, 4, 5, 6, 7, 8};
  EXPECT_EQ(pt0, pt1);
  EXPECT_TRUE(pt0 == pt1);
}

TEST(PointEquality, PointXYZIRCAEDT)
{
  using autoware::point_types::PointXYZIRCAEDT;

  PointXYZIRCAEDT pt0{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  PointXYZIRCAEDT pt1{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  EXPECT_EQ(pt0, pt1);
  EXPECT_TRUE(pt0 == pt1);
}

TEST(PointEquality, PointXYZCPE)
{
  using autoware::point_types::PointXYZCPE;

  {
    // test defaults are the unclassified/unset markers
    PointXYZCPE pt0;
    PointXYZCPE pt1;
    EXPECT_EQ(
      pt0.class_id,
      static_cast<std::uint8_t>(autoware::point_types::PointCloudClassification::INVALID));
    EXPECT_TRUE(std::isnan(pt0.entropy));
    EXPECT_EQ(pt0, pt1);

    pt1.entropy = 0.0F;
    EXPECT_FALSE(pt0 == pt1);
  }

  {
    // test with specific values
    PointXYZCPE pt0{0, 1, 2, 3, 4, 5};
    PointXYZCPE pt1{0, 1, 2, 3, 4, 5};
    EXPECT_EQ(pt0, pt1);
    EXPECT_TRUE(pt0 == pt1);
  }
}

TEST(PointCloudClassification, EnumValues)
{
  using autoware::point_types::PointCloudClassification;

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
  EXPECT_EQ(static_cast<std::uint8_t>(PointCloudClassification::INVALID), 255U);
}

TEST(PointCloudClassification, ToString)
{
  using autoware::point_types::PointCloudClassification;
  using autoware::point_types::to_string;

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
  EXPECT_EQ(to_string(PointCloudClassification::INVALID), "INVALID");
  EXPECT_THROW(to_string(static_cast<PointCloudClassification>(254U)), std::invalid_argument);
}

TEST(PointCloudClassification, ToPointcloudClassification)
{
  using autoware::point_types::PointCloudClassification;
  using autoware::point_types::to_pointcloud_classification;
  using autoware::point_types::to_string;

  // Round trip with every name to_string() can produce, including INVALID.
  for (const auto classification :
       {PointCloudClassification::CAR, PointCloudClassification::TRUCK,
        PointCloudClassification::BUS, PointCloudClassification::MOTORCYCLE,
        PointCloudClassification::BICYCLE, PointCloudClassification::PEDESTRIAN,
        PointCloudClassification::ANIMAL, PointCloudClassification::HAZARD,
        PointCloudClassification::FLAT_SURFACE, PointCloudClassification::STRUCTURE,
        PointCloudClassification::VEGETATION, PointCloudClassification::NOISE,
        PointCloudClassification::INVALID}) {
    EXPECT_EQ(to_pointcloud_classification(to_string(classification)), classification);
  }

  // Matching is case-insensitive for every name, NOISE included.
  EXPECT_EQ(to_pointcloud_classification("car"), PointCloudClassification::CAR);
  EXPECT_EQ(to_pointcloud_classification("flat_surface"), PointCloudClassification::FLAT_SURFACE);
  EXPECT_EQ(to_pointcloud_classification("noise"), PointCloudClassification::NOISE);
  EXPECT_EQ(to_pointcloud_classification("Noise"), PointCloudClassification::NOISE);

  EXPECT_THROW(to_pointcloud_classification("UNKNOWN"), std::invalid_argument);
  EXPECT_THROW(to_pointcloud_classification(""), std::invalid_argument);
}

TEST(PointCloudClassification, ConstexprToString)
{
  using autoware::point_types::PointCloudClassification;
  using autoware::point_types::to_string;

  constexpr auto classification = PointCloudClassification::CAR;
  constexpr auto str = to_string(classification);
  static_assert(str == "CAR");
  EXPECT_EQ(str, "CAR");
}

TEST(PointEquality, FloatEq)
{
  // test template
  EXPECT_TRUE(autoware::point_types::float_eq<float>(1, 1));
  EXPECT_TRUE(autoware::point_types::float_eq<double>(1, 1));

  // test floating point error
  EXPECT_TRUE(autoware::point_types::float_eq<float>(1, 1 + std::numeric_limits<float>::epsilon()));

  // test difference of sign
  EXPECT_FALSE(autoware::point_types::float_eq<float>(2, -2));
  EXPECT_FALSE(autoware::point_types::float_eq<float>(-2, 2));

  // small value difference
  EXPECT_FALSE(autoware::point_types::float_eq<float>(2, 2 + 10e-6));

  // expect same value if epsilon is larger than difference
  EXPECT_TRUE(autoware::point_types::float_eq<float>(2, 2 + 10e-6, 10e-5));
}

TEST(PointCloudModifier, PointXYZIRCT)
{
  using autoware::point_types::PointXYZIRCT;
  using autoware::point_types::PointXYZIRCTGenerator;
  using point_cloud_msg_wrapper::PointCloud2Modifier;
  using point_cloud_msg_wrapper::PointCloud2View;

  sensor_msgs::msg::PointCloud2 msg;
  PointCloud2Modifier<PointXYZIRCT, PointXYZIRCTGenerator> modifier{msg, "base_link"};

  const PointXYZIRCT point{1.0F, 2.0F, 3.0F, 4U, 5U, 6U, 7U};
  modifier.push_back(point);

  ASSERT_EQ(msg.fields.size(), 7U);
  EXPECT_EQ(msg.fields.back().name, "time_stamp");
  EXPECT_EQ(msg.point_step, sizeof(PointXYZIRCT));

  PointCloud2View<PointXYZIRCT, PointXYZIRCTGenerator> view{msg};
  ASSERT_EQ(view.size(), 1U);
  EXPECT_EQ(view[0], point);
}
