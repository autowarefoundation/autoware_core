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

#include "voxel_grid_downsample_filter/voxel_grid_downsample_filter.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

using PointXYZ = std::array<float, 3>;
using PointXYZI = std::array<float, 4>;

struct PointXYZIu8
{
  float x;
  float y;
  float z;
  uint8_t intensity;
};

sensor_msgs::msg::PointCloud2 create_xyzirc_pointcloud2(const std::vector<PointXYZI> & points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    6, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
    sensor_msgs::msg::PointField::UINT8, "return_type", 1, sensor_msgs::msg::PointField::UINT8,
    "channel", 1, sensor_msgs::msg::PointField::UINT16);
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
    *iter_intensity = static_cast<uint8_t>(std::clamp(point[3], 0.0f, 255.0f));
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
  cloud.height = 1;
  cloud.is_dense = true;
  return cloud;
}

sensor_msgs::msg::PointCloud2 create_xyzi_pointcloud2(const std::vector<PointXYZI> & points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
    sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");

  for (const auto & point : points) {
    *iter_x = point[0];
    *iter_y = point[1];
    *iter_z = point[2];
    *iter_intensity = point[3];
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_intensity;
  }

  cloud.header.frame_id = "sensor_frame";
  cloud.height = 1;
  cloud.is_dense = true;
  return cloud;
}

std::vector<PointXYZIu8> extract_points_with_intensity_from_cloud(
  const sensor_msgs::msg::PointCloud2 & cloud)
{
  std::vector<PointXYZIu8> points;
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_intensity(cloud, "intensity");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
    points.push_back({*iter_x, *iter_y, *iter_z, *iter_intensity});
  }

  return points;
}

void expect_points_with_intensity_near(
  std::vector<PointXYZIu8> actual, std::vector<PointXYZIu8> expected, const float tolerance)
{
  ASSERT_EQ(actual.size(), expected.size());

  const auto less = [](const PointXYZIu8 & a, const PointXYZIu8 & b) {
    return std::tie(a.x, a.y, a.z, a.intensity) < std::tie(b.x, b.y, b.z, b.intensity);
  };
  std::sort(actual.begin(), actual.end(), less);
  std::sort(expected.begin(), expected.end(), less);

  for (size_t i = 0; i < expected.size(); ++i) {
    SCOPED_TRACE("point index " + std::to_string(i));
    EXPECT_NEAR(actual[i].x, expected[i].x, tolerance);
    EXPECT_NEAR(actual[i].y, expected[i].y, tolerance);
    EXPECT_NEAR(actual[i].z, expected[i].z, tolerance);
    EXPECT_EQ(actual[i].intensity, expected[i].intensity);
  }
}

TEST(VoxelGridDownsampleFilterCoreTest, RejectsInvalidDataBufferSize)
{
  autoware::downsample_filters::VoxelGridDownsampleFilter core({1.0f, 1.0f, 1.0f});

  auto cloud = create_xyzirc_pointcloud2({{0.1f, 0.1f, 0.1f, 100.0f}});
  cloud.data.pop_back();

  const auto result = core.filter(std::make_shared<const sensor_msgs::msg::PointCloud2>(cloud));
  EXPECT_FALSE(result);
}

TEST(VoxelGridDownsampleFilterCoreTest, RejectsUnsupportedPointLayout)
{
  autoware::downsample_filters::VoxelGridDownsampleFilter core({1.0f, 1.0f, 1.0f});

  const auto cloud = create_xyzi_pointcloud2({{0.1f, 0.1f, 0.1f, 10.0f}});

  const auto result = core.filter(std::make_shared<const sensor_msgs::msg::PointCloud2>(cloud));
  EXPECT_FALSE(result);
}

TEST(VoxelGridDownsampleFilterCoreTest, DownsamplesPointsInSameVoxelToSingleCentroid)
{
  autoware::downsample_filters::VoxelGridDownsampleFilter core({1.0f, 1.0f, 1.0f});

  const auto cloud = create_xyzirc_pointcloud2(
    {{0.1f, 0.1f, 0.1f, 100.0f}, {0.2f, 0.2f, 0.2f, 100.0f}, {0.9f, 0.9f, 0.9f, 100.0f}});

  const auto result = core.filter(std::make_shared<const sensor_msgs::msg::PointCloud2>(cloud));
  ASSERT_TRUE(result) << result.error();

  const auto & output = result.value();
  EXPECT_EQ(output.header.frame_id, cloud.header.frame_id);
  EXPECT_EQ(output.header.stamp, cloud.header.stamp);

  const std::vector<PointXYZIu8> expected_points = {{0.4f, 0.4f, 0.4f, 100U}};
  expect_points_with_intensity_near(
    extract_points_with_intensity_from_cloud(output), expected_points, 1.0e-4f);
}

TEST(VoxelGridDownsampleFilterCoreTest, PreservesSeparateVoxelsAsMultipleCentroids)
{
  autoware::downsample_filters::VoxelGridDownsampleFilter core({1.0f, 1.0f, 1.0f});

  const auto cloud = create_xyzirc_pointcloud2(
    {{0.1f, 0.1f, 0.1f, 10.0f}, {0.9f, 0.9f, 0.9f, 50.0f}, {1.1f, 1.1f, 1.1f, 90.0f}});

  const auto result = core.filter(std::make_shared<const sensor_msgs::msg::PointCloud2>(cloud));
  ASSERT_TRUE(result) << result.error();

  const auto & output = result.value();
  const std::vector<PointXYZIu8> expected_points = {
    {0.5f, 0.5f, 0.5f, 30U}, {1.1f, 1.1f, 1.1f, 90U}};
  expect_points_with_intensity_near(
    extract_points_with_intensity_from_cloud(output), expected_points, 1.0e-4f);
}

TEST(VoxelGridDownsampleFilterCoreTest, IgnoresNonFinitePoints)
{
  autoware::downsample_filters::VoxelGridDownsampleFilter core({1.0f, 1.0f, 1.0f});

  const auto nan = std::numeric_limits<float>::quiet_NaN();
  const auto inf = std::numeric_limits<float>::infinity();
  const auto cloud = create_xyzirc_pointcloud2(
    {{0.5f, 0.5f, 0.5f, 12.0f}, {nan, 0.0f, 0.0f, 20.0f}, {0.0f, inf, 0.0f, 30.0f}});

  const auto result = core.filter(std::make_shared<const sensor_msgs::msg::PointCloud2>(cloud));
  ASSERT_TRUE(result) << result.error();

  const auto & output = result.value();
  const std::vector<PointXYZIu8> expected_points = {{0.5f, 0.5f, 0.5f, 12U}};
  expect_points_with_intensity_near(
    extract_points_with_intensity_from_cloud(output), expected_points, 1.0e-4f);
}

TEST(VoxelGridDownsampleFilterCoreTest, FallsBackToInputWhenVoxelIndexWouldOverflow)
{
  autoware::downsample_filters::VoxelGridDownsampleFilter core({1.0e-4f, 1.0e-4f, 1.0e-4f});

  const auto cloud =
    create_xyzirc_pointcloud2({{0.0f, 0.0f, 0.0f, 10.0f}, {500000.0f, 0.0f, 0.0f, 20.0f}});

  const auto result = core.filter(std::make_shared<const sensor_msgs::msg::PointCloud2>(cloud));
  ASSERT_FALSE(result);
}
