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

#include "crop_box_filter.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

sensor_msgs::msg::PointCloud2 make_cloud(
  std::vector<sensor_msgs::msg::PointField> fields, uint32_t point_step)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.fields = fields;
  cloud.point_step = point_step;
  cloud.width = 1;
  cloud.height = 1;
  cloud.row_step = point_step;
  cloud.data.resize(point_step, 0);
  return cloud;
}

sensor_msgs::msg::PointField make_field(
  const std::string & name, uint32_t offset, uint8_t datatype, uint32_t count = 1)
{
  sensor_msgs::msg::PointField f;
  f.name = name;
  f.offset = offset;
  f.datatype = datatype;
  f.count = count;
  return f;
}

TEST(ValidatePointCloud2Test, AcceptsXyzOnly)
{
  const auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(make_cloud(
    {make_field("x", 0, sensor_msgs::msg::PointField::FLOAT32),
     make_field("y", 4, sensor_msgs::msg::PointField::FLOAT32),
     make_field("z", 8, sensor_msgs::msg::PointField::FLOAT32)},
    12));

  const auto result = autoware::crop_box_filter::validate_pointcloud2(cloud);

  EXPECT_TRUE(result.is_valid);
}

TEST(ValidatePointCloud2Test, AcceptsXyzirc)
{
  const auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(make_cloud(
    {make_field("x", 0, sensor_msgs::msg::PointField::FLOAT32),
     make_field("y", 4, sensor_msgs::msg::PointField::FLOAT32),
     make_field("z", 8, sensor_msgs::msg::PointField::FLOAT32),
     make_field("intensity", 12, sensor_msgs::msg::PointField::UINT8),
     make_field("return_type", 13, sensor_msgs::msg::PointField::UINT8),
     make_field("channel", 14, sensor_msgs::msg::PointField::UINT16)},
    16));

  const auto result = autoware::crop_box_filter::validate_pointcloud2(cloud);

  EXPECT_TRUE(result.is_valid);
}

TEST(ValidatePointCloud2Test, RejectsMissingZ)
{
  const auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(make_cloud(
    {make_field("x", 0, sensor_msgs::msg::PointField::FLOAT32),
     make_field("y", 4, sensor_msgs::msg::PointField::FLOAT32)},
    8));

  const auto result = autoware::crop_box_filter::validate_pointcloud2(cloud);

  EXPECT_FALSE(result.is_valid);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
