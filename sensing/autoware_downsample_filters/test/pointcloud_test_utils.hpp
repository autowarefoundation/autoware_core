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

#ifndef POINTCLOUD_TEST_UTILS_HPP_
#define POINTCLOUD_TEST_UTILS_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <string>
#include <tuple>
#include <vector>

namespace autoware::downsample_filters::test_utils
{
using PointXYZ = std::array<float, 3>;

inline sensor_msgs::msg::PointCloud2 create_pointcloud2(const std::vector<PointXYZ> & points)
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

  return cloud;
}

inline std::vector<PointXYZ> extract_points_from_cloud(const sensor_msgs::msg::PointCloud2 & cloud)
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

inline void expect_points_near(
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

}  // namespace autoware::downsample_filters::test_utils

#endif  // POINTCLOUD_TEST_UTILS_HPP_
