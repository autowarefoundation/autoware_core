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

#include "autoware/ground_filter/sanity_check.hpp"

#include <autoware/point_types/types.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

using PointXYZI = autoware::point_types::PointXYZI;
using PointXYZIRC = autoware::point_types::PointXYZIRC;
using PointXYZIRADRT = autoware::point_types::PointXYZIRADRT;
using PointXYZIRCAEDT = autoware::point_types::PointXYZIRCAEDT;

class SanityCheckTest : public ::testing::Test
{
protected:
  void SetUp() override {}

  // Helper function to create a compatible PointXYZI PointCloud2 message
  sensor_msgs::msg::PointCloud2 createPointXYZICloud()
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.fields.resize(4);

    // X field
    cloud.fields[0].name = "x";
    cloud.fields[0].offset = offsetof(PointXYZI, x);
    cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[0].count = 1;

    // Y field
    cloud.fields[1].name = "y";
    cloud.fields[1].offset = offsetof(PointXYZI, y);
    cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[1].count = 1;

    // Z field
    cloud.fields[2].name = "z";
    cloud.fields[2].offset = offsetof(PointXYZI, z);
    cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[2].count = 1;

    // Intensity field
    cloud.fields[3].name = "intensity";
    cloud.fields[3].offset = offsetof(PointXYZI, intensity);
    cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[3].count = 1;

    return cloud;
  }

  // Helper function to create a compatible PointXYZIRC PointCloud2 message
  sensor_msgs::msg::PointCloud2 createPointXYZIRCCloud()
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.fields.resize(6);

    // X field
    cloud.fields[0].name = "x";
    cloud.fields[0].offset = offsetof(PointXYZIRC, x);
    cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[0].count = 1;

    // Y field
    cloud.fields[1].name = "y";
    cloud.fields[1].offset = offsetof(PointXYZIRC, y);
    cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[1].count = 1;

    // Z field
    cloud.fields[2].name = "z";
    cloud.fields[2].offset = offsetof(PointXYZIRC, z);
    cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[2].count = 1;

    // Intensity field
    cloud.fields[3].name = "intensity";
    cloud.fields[3].offset = offsetof(PointXYZIRC, intensity);
    cloud.fields[3].datatype = sensor_msgs::msg::PointField::UINT8;
    cloud.fields[3].count = 1;

    // Return type field
    cloud.fields[4].name = "return_type";
    cloud.fields[4].offset = offsetof(PointXYZIRC, return_type);
    cloud.fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
    cloud.fields[4].count = 1;

    // Channel field
    cloud.fields[5].name = "channel";
    cloud.fields[5].offset = offsetof(PointXYZIRC, channel);
    cloud.fields[5].datatype = sensor_msgs::msg::PointField::UINT16;
    cloud.fields[5].count = 1;

    return cloud;
  }

  // Helper function to create a compatible PointXYZIRADRT PointCloud2 message
  sensor_msgs::msg::PointCloud2 createPointXYZIRADRTCloud()
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.fields.resize(9);

    // X field
    cloud.fields[0].name = "x";
    cloud.fields[0].offset = offsetof(PointXYZIRADRT, x);
    cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[0].count = 1;

    // Y field
    cloud.fields[1].name = "y";
    cloud.fields[1].offset = offsetof(PointXYZIRADRT, y);
    cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[1].count = 1;

    // Z field
    cloud.fields[2].name = "z";
    cloud.fields[2].offset = offsetof(PointXYZIRADRT, z);
    cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[2].count = 1;

    // Intensity field
    cloud.fields[3].name = "intensity";
    cloud.fields[3].offset = offsetof(PointXYZIRADRT, intensity);
    cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[3].count = 1;

    // Ring field
    cloud.fields[4].name = "ring";
    cloud.fields[4].offset = offsetof(PointXYZIRADRT, ring);
    cloud.fields[4].datatype = sensor_msgs::msg::PointField::UINT16;
    cloud.fields[4].count = 1;

    // Azimuth field
    cloud.fields[5].name = "azimuth";
    cloud.fields[5].offset = offsetof(PointXYZIRADRT, azimuth);
    cloud.fields[5].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[5].count = 1;

    // Distance field
    cloud.fields[6].name = "distance";
    cloud.fields[6].offset = offsetof(PointXYZIRADRT, distance);
    cloud.fields[6].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[6].count = 1;

    // Return type field
    cloud.fields[7].name = "return_type";
    cloud.fields[7].offset = offsetof(PointXYZIRADRT, return_type);
    cloud.fields[7].datatype = sensor_msgs::msg::PointField::UINT8;
    cloud.fields[7].count = 1;

    // Time stamp field
    cloud.fields[8].name = "time_stamp";
    cloud.fields[8].offset = offsetof(PointXYZIRADRT, time_stamp);
    cloud.fields[8].datatype = sensor_msgs::msg::PointField::FLOAT64;
    cloud.fields[8].count = 1;

    return cloud;
  }

  // Helper function to create a compatible PointXYZIRCAEDT PointCloud2 message
  sensor_msgs::msg::PointCloud2 createPointXYZIRCAEDTCloud()
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.fields.resize(10);

    // X field
    cloud.fields[0].name = "x";
    cloud.fields[0].offset = offsetof(PointXYZIRCAEDT, x);
    cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[0].count = 1;

    // Y field
    cloud.fields[1].name = "y";
    cloud.fields[1].offset = offsetof(PointXYZIRCAEDT, y);
    cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[1].count = 1;

    // Z field
    cloud.fields[2].name = "z";
    cloud.fields[2].offset = offsetof(PointXYZIRCAEDT, z);
    cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[2].count = 1;

    // Intensity field
    cloud.fields[3].name = "intensity";
    cloud.fields[3].offset = offsetof(PointXYZIRCAEDT, intensity);
    cloud.fields[3].datatype = sensor_msgs::msg::PointField::UINT8;
    cloud.fields[3].count = 1;

    // Return type field
    cloud.fields[4].name = "return_type";
    cloud.fields[4].offset = offsetof(PointXYZIRCAEDT, return_type);
    cloud.fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
    cloud.fields[4].count = 1;

    // Channel field
    cloud.fields[5].name = "channel";
    cloud.fields[5].offset = offsetof(PointXYZIRCAEDT, channel);
    cloud.fields[5].datatype = sensor_msgs::msg::PointField::UINT16;
    cloud.fields[5].count = 1;

    // Azimuth field
    cloud.fields[6].name = "azimuth";
    cloud.fields[6].offset = offsetof(PointXYZIRCAEDT, azimuth);
    cloud.fields[6].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[6].count = 1;

    // Elevation field
    cloud.fields[7].name = "elevation";
    cloud.fields[7].offset = offsetof(PointXYZIRCAEDT, elevation);
    cloud.fields[7].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[7].count = 1;

    // Distance field
    cloud.fields[8].name = "distance";
    cloud.fields[8].offset = offsetof(PointXYZIRCAEDT, distance);
    cloud.fields[8].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[8].count = 1;

    // Time stamp field
    cloud.fields[9].name = "time_stamp";
    cloud.fields[9].offset = offsetof(PointXYZIRCAEDT, time_stamp);
    cloud.fields[9].datatype = sensor_msgs::msg::PointField::UINT32;
    cloud.fields[9].count = 1;

    return cloud;
  }
};

TEST_F(SanityCheckTest, TestEmptyPointCloud)
{
  sensor_msgs::msg::PointCloud2 cloud;

  // Test empty cloud with all layout check functions
  EXPECT_FALSE(autoware::ground_filter::is_data_layout_compatible_with_point_xyzi(cloud));
  EXPECT_FALSE(autoware::ground_filter::is_data_layout_compatible_with_point_xyzirc(cloud));
  EXPECT_FALSE(autoware::ground_filter::is_data_layout_compatible_with_point_xyziradrt(cloud));
  EXPECT_FALSE(autoware::ground_filter::is_data_layout_compatible_with_point_xyzircaedt(cloud));
}

TEST_F(SanityCheckTest, TestPointXYZILayout)
{
  auto cloud = createPointXYZICloud();

  // Test with all layout check functions
  EXPECT_TRUE(autoware::ground_filter::is_data_layout_compatible_with_point_xyzi(cloud));
  EXPECT_FALSE(autoware::ground_filter::is_data_layout_compatible_with_point_xyzirc(cloud));
  EXPECT_FALSE(autoware::ground_filter::is_data_layout_compatible_with_point_xyziradrt(cloud));
  EXPECT_FALSE(autoware::ground_filter::is_data_layout_compatible_with_point_xyzircaedt(cloud));

  // Modify the cloud to break compatibility
  cloud.fields[0].name = "not_x";
  EXPECT_FALSE(autoware::ground_filter::is_data_layout_compatible_with_point_xyzi(cloud));

  // Reset and modify a different field
  cloud = createPointXYZICloud();
  cloud.fields[3].datatype = sensor_msgs::msg::PointField::UINT8;  // Wrong intensity type
  EXPECT_FALSE(autoware::ground_filter::is_data_layout_compatible_with_point_xyzi(cloud));
}

TEST_F(SanityCheckTest, TestPointXYZIRCLayout)
{
  auto cloud = createPointXYZIRCCloud();

  // Test with all layout check functions
  EXPECT_FALSE(autoware::ground_filter::is_data_layout_compatible_with_point_xyzi(cloud));
  EXPECT_TRUE(autoware::ground_filter::is_data_layout_compatible_with_point_xyzirc(cloud));
  EXPECT_FALSE(autoware::ground_filter::is_data_layout_compatible_with_point_xyziradrt(cloud));
  EXPECT_FALSE(autoware::ground_filter::is_data_layout_compatible_with_point_xyzircaedt(cloud));

  // Modify the cloud to break compatibility
  cloud.fields[4].name = "not_return_type";
  EXPECT_FALSE(autoware::ground_filter::is_data_layout_compatible_with_point_xyzirc(cloud));

  // Reset and modify a different field
  cloud = createPointXYZIRCCloud();
  cloud.fields[5].datatype = sensor_msgs::msg::PointField::UINT8;  // Wrong channel type
  EXPECT_FALSE(autoware::ground_filter::is_data_layout_compatible_with_point_xyzirc(cloud));
}

TEST_F(SanityCheckTest, TestPointXYZIRADRTLayout)
{
  auto cloud = createPointXYZIRADRTCloud();

  // Test with all layout check functions
  EXPECT_TRUE(autoware::ground_filter::is_data_layout_compatible_with_point_xyzi(cloud));
  EXPECT_FALSE(autoware::ground_filter::is_data_layout_compatible_with_point_xyzirc(cloud));
  EXPECT_TRUE(autoware::ground_filter::is_data_layout_compatible_with_point_xyziradrt(cloud));
  EXPECT_FALSE(autoware::ground_filter::is_data_layout_compatible_with_point_xyzircaedt(cloud));

  // Modify the cloud to break compatibility
  cloud.fields[6].name = "not_distance";
  EXPECT_FALSE(autoware::ground_filter::is_data_layout_compatible_with_point_xyziradrt(cloud));

  // Reset and modify a different field
  cloud = createPointXYZIRADRTCloud();
  cloud.fields[4].datatype = sensor_msgs::msg::PointField::UINT8;  // Wrong ring type
  EXPECT_FALSE(autoware::ground_filter::is_data_layout_compatible_with_point_xyziradrt(cloud));

  // Test with fewer fields
  cloud.fields.resize(8);  // Missing time_stamp field
  EXPECT_FALSE(autoware::ground_filter::is_data_layout_compatible_with_point_xyziradrt(cloud));
}

TEST_F(SanityCheckTest, TestPointXYZIRCAEDTLayout)
{
  auto cloud = createPointXYZIRCAEDTCloud();

  // Test with all layout check functions
  EXPECT_FALSE(autoware::ground_filter::is_data_layout_compatible_with_point_xyzi(cloud));
  EXPECT_TRUE(autoware::ground_filter::is_data_layout_compatible_with_point_xyzirc(cloud));
  EXPECT_FALSE(autoware::ground_filter::is_data_layout_compatible_with_point_xyziradrt(cloud));
  EXPECT_TRUE(autoware::ground_filter::is_data_layout_compatible_with_point_xyzircaedt(cloud));

  // Modify the cloud to break compatibility
  cloud.fields[7].name = "not_elevation";
  EXPECT_FALSE(autoware::ground_filter::is_data_layout_compatible_with_point_xyzircaedt(cloud));

  // Reset and modify a different field
  cloud = createPointXYZIRCAEDTCloud();
  cloud.fields[8].datatype = sensor_msgs::msg::PointField::UINT8;  // Wrong distance type
  EXPECT_FALSE(autoware::ground_filter::is_data_layout_compatible_with_point_xyzircaedt(cloud));

  // Test with more fields than expected
  cloud = createPointXYZIRCAEDTCloud();
  cloud.fields.push_back(sensor_msgs::msg::PointField());  // Add an extra field
  EXPECT_FALSE(autoware::ground_filter::is_data_layout_compatible_with_point_xyzircaedt(cloud));

  // Test with fewer fields
  cloud = createPointXYZIRCAEDTCloud();
  cloud.fields.resize(9);  // Missing time_stamp field
  EXPECT_FALSE(autoware::ground_filter::is_data_layout_compatible_with_point_xyzircaedt(cloud));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
