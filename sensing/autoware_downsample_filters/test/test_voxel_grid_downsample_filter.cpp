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

sensor_msgs::msg::PointCloud2 create_xyzirc_pointcloud2(const std::vector<PointXYZI> & points)
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
    4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
    sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
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

std::vector<PointXYZ> extract_points_from_cloud(const sensor_msgs::msg::PointCloud2 & cloud)
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

std::vector<uint8_t> extract_intensities_from_cloud(const sensor_msgs::msg::PointCloud2 & cloud)
{
  std::vector<uint8_t> intensities;
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_intensity(cloud, "intensity");

  for (; iter_intensity != iter_intensity.end(); ++iter_intensity) {
    intensities.push_back(*iter_intensity);
  }

  return intensities;
}

void expect_points_near(
  std::vector<PointXYZ> actual, std::vector<PointXYZ> expected, const float tolerance)
{
  ASSERT_EQ(actual.size(), expected.size());

  const auto less = [](const PointXYZ & a, const PointXYZ & b) {
    return std::tie(a[0], a[1], a[2]) < std::tie(b[0], b[1], b[2]);
  };
  std::sort(actual.begin(), actual.end(), less);
  std::sort(expected.begin(), expected.end(), less);

  for (size_t i = 0; i < expected.size(); ++i) {
    SCOPED_TRACE("point index " + std::to_string(i));
    EXPECT_NEAR(actual[i][0], expected[i][0], tolerance);
    EXPECT_NEAR(actual[i][1], expected[i][1], tolerance);
    EXPECT_NEAR(actual[i][2], expected[i][2], tolerance);
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

  const std::vector<PointXYZ> expected_points = {{0.4f, 0.4f, 0.4f}};
  expect_points_near(extract_points_from_cloud(output), expected_points, 1.0e-4f);

  const auto intensities = extract_intensities_from_cloud(output);
  ASSERT_EQ(intensities.size(), 1U);
  EXPECT_EQ(intensities.front(), 100U);
}

TEST(VoxelGridDownsampleFilterCoreTest, PreservesSeparateVoxelsAsMultipleCentroids)
{
  autoware::downsample_filters::VoxelGridDownsampleFilter core({1.0f, 1.0f, 1.0f});

  const auto cloud = create_xyzirc_pointcloud2(
    {{0.1f, 0.1f, 0.1f, 10.0f}, {0.9f, 0.9f, 0.9f, 50.0f}, {1.1f, 1.1f, 1.1f, 90.0f}});

  const auto result = core.filter(std::make_shared<const sensor_msgs::msg::PointCloud2>(cloud));
  ASSERT_TRUE(result) << result.error();

  const auto & output = result.value();
  const std::vector<PointXYZ> expected_points = {{0.5f, 0.5f, 0.5f}, {1.1f, 1.1f, 1.1f}};
  expect_points_near(extract_points_from_cloud(output), expected_points, 1.0e-4f);

  auto intensities = extract_intensities_from_cloud(output);
  ASSERT_EQ(intensities.size(), 2U);
  std::sort(intensities.begin(), intensities.end());
  EXPECT_EQ(intensities[0], 30U);
  EXPECT_EQ(intensities[1], 90U);
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
  const std::vector<PointXYZ> expected_points = {{0.5f, 0.5f, 0.5f}};
  expect_points_near(extract_points_from_cloud(output), expected_points, 1.0e-4f);

  const auto intensities = extract_intensities_from_cloud(output);
  ASSERT_EQ(intensities.size(), 1U);
  EXPECT_EQ(intensities.front(), 12U);
}

TEST(VoxelGridDownsampleFilterCoreTest, FallsBackToInputWhenVoxelIndexWouldOverflow)
{
  autoware::downsample_filters::VoxelGridDownsampleFilter core({1.0e-4f, 1.0e-4f, 1.0e-4f});

  const auto cloud = create_xyzirc_pointcloud2(
    {{0.0f, 0.0f, 0.0f, 10.0f}, {500000.0f, 0.0f, 0.0f, 20.0f}});

  const auto result = core.filter(std::make_shared<const sensor_msgs::msg::PointCloud2>(cloud));
  ASSERT_FALSE(result);
}