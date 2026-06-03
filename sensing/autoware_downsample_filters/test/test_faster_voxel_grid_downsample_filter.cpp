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

#include "voxel_grid_downsample_filter/faster_voxel_grid_downsample_filter.hpp"
#include "voxel_grid_downsample_filter/transform_info.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>

#include <array>
#include <vector>

namespace
{
using PointXYZ = std::array<float, 3>;
using autoware::downsample_filters::FasterVoxelGridDownsampleFilter;
using autoware::downsample_filters::TransformInfo;

sensor_msgs::msg::PointCloud2 make_cloud(const std::vector<PointXYZ> & points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    6, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
    sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::UINT8, "return_type", 1,
    sensor_msgs::msg::PointField::UINT8, "channel", 1, sensor_msgs::msg::PointField::UINT16);
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_intensity(cloud, "intensity");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_return_type(cloud, "return_type");
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_channel(cloud, "channel");

  for (const auto & point : points) {
    *iter_x = point[0];
    *iter_y = point[1];
    *iter_z = point[2];
    *iter_intensity = 100U;
    *iter_return_type = 0U;
    *iter_channel = 0U;
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_intensity;
    ++iter_return_type;
    ++iter_channel;
  }

  cloud.header.frame_id = "sensor_frame";
  return cloud;
}

std::vector<PointXYZ> read_xyz(const sensor_msgs::msg::PointCloud2 & cloud)
{
  std::vector<PointXYZ> points;
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    points.push_back({*iter_x, *iter_y, *iter_z});
  }

  return points;
}
}  // namespace

TEST(FasterVoxelGridDownsampleFilterTest, ReturnsCentroidForSingleVoxel)
{
  const auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(
    make_cloud({{0.1f, 0.1f, 0.1f}, {0.2f, 0.2f, 0.2f}, {0.9f, 0.9f, 0.9f}}));

  FasterVoxelGridDownsampleFilter filter;
  filter.set_voxel_size(1.0f, 1.0f, 1.0f);

  sensor_msgs::msg::PointCloud2 output;
  TransformInfo transform_info;
  const auto status = filter.filter(cloud, output, transform_info);

  ASSERT_EQ(status, FasterVoxelGridDownsampleFilter::Status::kSuccess);
  ASSERT_EQ(output.width, 1U);

  const auto points = read_xyz(output);
  ASSERT_EQ(points.size(), 1U);
  EXPECT_NEAR(points.front()[0], 0.4f, 1.0e-4f);
  EXPECT_NEAR(points.front()[1], 0.4f, 1.0e-4f);
  EXPECT_NEAR(points.front()[2], 0.4f, 1.0e-4f);
}

TEST(FasterVoxelGridDownsampleFilterTest, FailsForInvalidIntensityFieldType)
{
  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(make_cloud({{0.1f, 0.1f, 0.1f}}));

  for (auto & field : cloud->fields) {
    if (field.name == "intensity") {
      field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      break;
    }
  }

  FasterVoxelGridDownsampleFilter filter;
  filter.set_voxel_size(1.0f, 1.0f, 1.0f);

  sensor_msgs::msg::PointCloud2 output;
  TransformInfo transform_info;
  const auto status = filter.filter(cloud, output, transform_info);

  EXPECT_EQ(status, FasterVoxelGridDownsampleFilter::Status::kIntensityFieldNotFoundOrInvalidType);
}
