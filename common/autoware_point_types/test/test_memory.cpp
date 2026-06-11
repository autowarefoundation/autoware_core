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

#include "autoware/point_types/memory.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <vector>

namespace
{
using autoware::point_types::create_fields_point_xyzi;
using autoware::point_types::create_fields_point_xyziradrt;
using autoware::point_types::create_fields_point_xyzirc;
using autoware::point_types::create_fields_point_xyzircaedt;
using autoware::point_types::is_data_layout_compatible_with_point_xyzi;
using autoware::point_types::is_data_layout_compatible_with_point_xyziradrt;
using autoware::point_types::is_data_layout_compatible_with_point_xyzirc;
using autoware::point_types::is_data_layout_compatible_with_point_xyzircaedt;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;
}  // namespace

//
// create_fields_* exact contents
//

TEST(CreateFields, Xyzi)
{
  const auto fields = create_fields_point_xyzi();
  ASSERT_EQ(fields.size(), 4U);

  EXPECT_EQ(fields[0].name, "x");
  EXPECT_EQ(fields[0].offset, offsetof(autoware::point_types::PointXYZI, x));
  EXPECT_EQ(fields[0].datatype, PointField::FLOAT32);
  EXPECT_EQ(fields[0].count, 1U);

  EXPECT_EQ(fields[1].name, "y");
  EXPECT_EQ(fields[1].datatype, PointField::FLOAT32);
  EXPECT_EQ(fields[2].name, "z");
  EXPECT_EQ(fields[2].datatype, PointField::FLOAT32);

  EXPECT_EQ(fields[3].name, "intensity");
  EXPECT_EQ(fields[3].offset, offsetof(autoware::point_types::PointXYZI, intensity));
  EXPECT_EQ(fields[3].datatype, PointField::FLOAT32);
  EXPECT_EQ(fields[3].count, 1U);
}

TEST(CreateFields, Xyzirc)
{
  const auto fields = create_fields_point_xyzirc();
  ASSERT_EQ(fields.size(), 6U);

  EXPECT_EQ(fields[3].name, "intensity");
  EXPECT_EQ(fields[3].datatype, PointField::UINT8);
  EXPECT_EQ(fields[4].name, "return_type");
  EXPECT_EQ(fields[4].datatype, PointField::UINT8);
  EXPECT_EQ(fields[5].name, "channel");
  EXPECT_EQ(fields[5].datatype, PointField::UINT16);
  EXPECT_EQ(fields[5].offset, offsetof(autoware::point_types::PointXYZIRC, channel));
}

TEST(CreateFields, Xyziradrt)
{
  const auto fields = create_fields_point_xyziradrt();
  ASSERT_EQ(fields.size(), 9U);

  EXPECT_EQ(fields[4].name, "ring");
  EXPECT_EQ(fields[4].datatype, PointField::UINT16);
  EXPECT_EQ(fields[5].name, "azimuth");
  EXPECT_EQ(fields[5].datatype, PointField::FLOAT32);
  EXPECT_EQ(fields[6].name, "distance");
  EXPECT_EQ(fields[6].datatype, PointField::FLOAT32);
  EXPECT_EQ(fields[7].name, "return_type");
  EXPECT_EQ(fields[7].datatype, PointField::UINT8);
  EXPECT_EQ(fields[8].name, "time_stamp");
  EXPECT_EQ(fields[8].datatype, PointField::FLOAT64);
  EXPECT_EQ(fields[8].offset, offsetof(autoware::point_types::PointXYZIRADRT, time_stamp));
}

TEST(CreateFields, Xyzircaedt)
{
  const auto fields = create_fields_point_xyzircaedt();
  ASSERT_EQ(fields.size(), 10U);

  EXPECT_EQ(fields[3].name, "intensity");
  EXPECT_EQ(fields[3].datatype, PointField::UINT8);
  EXPECT_EQ(fields[4].name, "return_type");
  EXPECT_EQ(fields[4].datatype, PointField::UINT8);
  EXPECT_EQ(fields[5].name, "channel");
  EXPECT_EQ(fields[5].datatype, PointField::UINT16);
  EXPECT_EQ(fields[6].name, "azimuth");
  EXPECT_EQ(fields[6].datatype, PointField::FLOAT32);
  EXPECT_EQ(fields[7].name, "elevation");
  EXPECT_EQ(fields[7].datatype, PointField::FLOAT32);
  EXPECT_EQ(fields[8].name, "distance");
  EXPECT_EQ(fields[8].datatype, PointField::FLOAT32);
  EXPECT_EQ(fields[9].name, "time_stamp");
  EXPECT_EQ(fields[9].datatype, PointField::UINT32);
  EXPECT_EQ(fields[9].offset, offsetof(autoware::point_types::PointXYZIRCAEDT, time_stamp));
}

//
// Round-trip: a freshly-created field layout must be reported compatible
//

TEST(LayoutCompatibleRoundTrip, Xyzi)
{
  EXPECT_TRUE(is_data_layout_compatible_with_point_xyzi(create_fields_point_xyzi()));
}

TEST(LayoutCompatibleRoundTrip, Xyzirc)
{
  EXPECT_TRUE(is_data_layout_compatible_with_point_xyzirc(create_fields_point_xyzirc()));
}

TEST(LayoutCompatibleRoundTrip, Xyziradrt)
{
  EXPECT_TRUE(is_data_layout_compatible_with_point_xyziradrt(create_fields_point_xyziradrt()));
}

TEST(LayoutCompatibleRoundTrip, Xyzircaedt)
{
  EXPECT_TRUE(is_data_layout_compatible_with_point_xyzircaedt(create_fields_point_xyzircaedt()));
}

//
// The four functions are type-specific: a layout for one type must not match another
//

TEST(LayoutCompatibleCrossType, MismatchedTypesReturnFalse)
{
  // xyzi layout has FLOAT32 intensity and only 4 fields -> not xyzirc/xyziradrt/xyzircaedt
  EXPECT_FALSE(is_data_layout_compatible_with_point_xyzirc(create_fields_point_xyzi()));
  EXPECT_FALSE(is_data_layout_compatible_with_point_xyziradrt(create_fields_point_xyzi()));
  EXPECT_FALSE(is_data_layout_compatible_with_point_xyzircaedt(create_fields_point_xyzi()));

  // xyzircaedt layout (10 fields, UINT8 intensity) -> not the others
  EXPECT_FALSE(is_data_layout_compatible_with_point_xyziradrt(create_fields_point_xyzircaedt()));
}

//
// PointCloud2 overloads forward to the std::vector<PointField> overloads
//

TEST(LayoutCompatiblePointCloud2Overload, ForwardsToFields)
{
  PointCloud2 cloud_xyzi;
  cloud_xyzi.fields = create_fields_point_xyzi();
  EXPECT_TRUE(is_data_layout_compatible_with_point_xyzi(cloud_xyzi));

  PointCloud2 cloud_xyzirc;
  cloud_xyzirc.fields = create_fields_point_xyzirc();
  EXPECT_TRUE(is_data_layout_compatible_with_point_xyzirc(cloud_xyzirc));

  PointCloud2 cloud_xyziradrt;
  cloud_xyziradrt.fields = create_fields_point_xyziradrt();
  EXPECT_TRUE(is_data_layout_compatible_with_point_xyziradrt(cloud_xyziradrt));

  PointCloud2 cloud_xyzircaedt;
  cloud_xyzircaedt.fields = create_fields_point_xyzircaedt();
  EXPECT_TRUE(is_data_layout_compatible_with_point_xyzircaedt(cloud_xyzircaedt));

  // empty cloud is not compatible with any
  PointCloud2 empty;
  EXPECT_FALSE(is_data_layout_compatible_with_point_xyzi(empty));
  EXPECT_FALSE(is_data_layout_compatible_with_point_xyzirc(empty));
  EXPECT_FALSE(is_data_layout_compatible_with_point_xyziradrt(empty));
  EXPECT_FALSE(is_data_layout_compatible_with_point_xyzircaedt(empty));
}

//
// Negative cases: a single corrupted field flips the result to false
//

TEST(LayoutCompatibleNegative, WrongName)
{
  auto fields = create_fields_point_xyzi();
  fields[1].name = "Y";  // expected lowercase "y"
  EXPECT_FALSE(is_data_layout_compatible_with_point_xyzi(fields));
}

TEST(LayoutCompatibleNegative, WrongOffset)
{
  auto fields = create_fields_point_xyzirc();
  fields[2].offset = fields[2].offset + 1U;  // z offset perturbed
  EXPECT_FALSE(is_data_layout_compatible_with_point_xyzirc(fields));
}

TEST(LayoutCompatibleNegative, WrongDatatype)
{
  auto fields = create_fields_point_xyziradrt();
  fields[8].datatype = PointField::FLOAT32;  // time_stamp must be FLOAT64
  EXPECT_FALSE(is_data_layout_compatible_with_point_xyziradrt(fields));
}

TEST(LayoutCompatibleNegative, WrongCount)
{
  auto fields = create_fields_point_xyzircaedt();
  fields[0].count = 2U;  // x must have count 1
  EXPECT_FALSE(is_data_layout_compatible_with_point_xyzircaedt(fields));
}

//
// Field-count edge behavior. This pins the CURRENT contract, which intentionally
// differs by type: xyzi/xyzirc/xyziradrt use a `size() < N` guard (extra trailing
// fields are ignored and the cloud is still accepted), while xyzircaedt uses a
// strict `size() != 10` guard (extra fields are rejected).
//

TEST(LayoutCompatibleFieldCount, TooFewFieldsRejected)
{
  {
    auto fields = create_fields_point_xyzi();
    fields.pop_back();  // 3 < 4
    EXPECT_FALSE(is_data_layout_compatible_with_point_xyzi(fields));
  }
  {
    auto fields = create_fields_point_xyzirc();
    fields.pop_back();  // 5 < 6
    EXPECT_FALSE(is_data_layout_compatible_with_point_xyzirc(fields));
  }
  {
    auto fields = create_fields_point_xyziradrt();
    fields.pop_back();  // 8 < 9
    EXPECT_FALSE(is_data_layout_compatible_with_point_xyziradrt(fields));
  }
  {
    auto fields = create_fields_point_xyzircaedt();
    fields.pop_back();  // 9 != 10
    EXPECT_FALSE(is_data_layout_compatible_with_point_xyzircaedt(fields));
  }
}

TEST(LayoutCompatibleFieldCount, ExtraTrailingFieldsAcceptedExceptXyzircaedt)
{
  // For the `< N` guard variants, an extra trailing field is ignored -> still compatible.
  {
    auto fields = create_fields_point_xyzi();
    fields.emplace_back();  // 5 fields, but only first 4 are checked
    EXPECT_TRUE(is_data_layout_compatible_with_point_xyzi(fields));
  }
  {
    auto fields = create_fields_point_xyzirc();
    fields.emplace_back();
    EXPECT_TRUE(is_data_layout_compatible_with_point_xyzirc(fields));
  }
  {
    auto fields = create_fields_point_xyziradrt();
    fields.emplace_back();
    EXPECT_TRUE(is_data_layout_compatible_with_point_xyziradrt(fields));
  }
  // For the strict `!= 10` guard, an extra trailing field is rejected.
  {
    auto fields = create_fields_point_xyzircaedt();
    fields.emplace_back();  // 11 != 10
    EXPECT_FALSE(is_data_layout_compatible_with_point_xyzircaedt(fields));
  }
}
