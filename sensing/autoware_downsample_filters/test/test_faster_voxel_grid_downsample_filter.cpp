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

#include "pointcloud_test_utils.hpp"
#include "voxel_grid_downsample_filter/faster_voxel_grid_downsample_filter.hpp"
#include "voxel_grid_downsample_filter/transform_info.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>

#include <vector>

namespace
{
using autoware::downsample_filters::FasterVoxelGridDownsampleFilter;
using autoware::downsample_filters::TransformInfo;
using autoware::downsample_filters::test_utils::create_pointcloud2;
using autoware::downsample_filters::test_utils::expect_points_near;
using autoware::downsample_filters::test_utils::extract_points_from_cloud;
using autoware::downsample_filters::test_utils::PointXYZ;
}  // namespace

TEST(FasterVoxelGridDownsampleFilterTest, ReturnsCentroidForSingleVoxel)
{
  const auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(
    create_pointcloud2({{0.1f, 0.1f, 0.1f}, {0.2f, 0.2f, 0.2f}, {0.9f, 0.9f, 0.9f}}));

  FasterVoxelGridDownsampleFilter filter;
  filter.set_voxel_size(1.0f, 1.0f, 1.0f);

  sensor_msgs::msg::PointCloud2 output;
  TransformInfo transform_info;
  const auto status = filter.filter(cloud, output, transform_info);

  ASSERT_EQ(status, FasterVoxelGridDownsampleFilter::Status::kSuccess);
  const auto output_points = extract_points_from_cloud(output);
  const std::vector<PointXYZ> expected_points = {
    {0.4f, 0.4f, 0.4f},
  };
  expect_points_near(output_points, expected_points, 1.0e-4f);
}

TEST(FasterVoxelGridDownsampleFilterTest, FailsForInvalidIntensityFieldType)
{
  auto cloud =
    std::make_shared<sensor_msgs::msg::PointCloud2>(create_pointcloud2({{0.1f, 0.1f, 0.1f}}));

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
