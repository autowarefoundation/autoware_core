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

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <string>
#include <vector>

using PointXYZ = std::array<float, 3>;
using PointXYZList = std::vector<PointXYZ>;

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
  const auto cloud = make_cloud(
    {make_field("x", 0, sensor_msgs::msg::PointField::FLOAT32),
     make_field("y", 4, sensor_msgs::msg::PointField::FLOAT32),
     make_field("z", 8, sensor_msgs::msg::PointField::FLOAT32)},
    12);

  const auto result = autoware::crop_box_filter::validate_pointcloud2(cloud);

  EXPECT_TRUE(result.is_valid);
}

TEST(ValidatePointCloud2Test, AcceptsXyzirc)
{
  const auto cloud = make_cloud(
    {make_field("x", 0, sensor_msgs::msg::PointField::FLOAT32),
     make_field("y", 4, sensor_msgs::msg::PointField::FLOAT32),
     make_field("z", 8, sensor_msgs::msg::PointField::FLOAT32),
     make_field("intensity", 12, sensor_msgs::msg::PointField::UINT8),
     make_field("return_type", 13, sensor_msgs::msg::PointField::UINT8),
     make_field("channel", 14, sensor_msgs::msg::PointField::UINT16)},
    16);

  const auto result = autoware::crop_box_filter::validate_pointcloud2(cloud);

  EXPECT_TRUE(result.is_valid);
}

TEST(ValidatePointCloud2Test, RejectsMissingZ)
{
  const auto cloud = make_cloud(
    {make_field("x", 0, sensor_msgs::msg::PointField::FLOAT32),
     make_field("y", 4, sensor_msgs::msg::PointField::FLOAT32)},
    8);

  const auto result = autoware::crop_box_filter::validate_pointcloud2(cloud);

  EXPECT_FALSE(result.is_valid);
}

sensor_msgs::msg::PointCloud2 create_pointcloud2(PointXYZList & points)
{
  sensor_msgs::msg::PointCloud2 pointcloud;
  sensor_msgs::PointCloud2Modifier modifier(pointcloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud, "z");

  for (const auto & point : points) {
    *iter_x = point[0];
    *iter_y = point[1];
    *iter_z = point[2];
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  return pointcloud;
}

PointXYZList extract_points_from_cloud(const sensor_msgs::msg::PointCloud2 & cloud)
{
  PointXYZList points;
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    points.push_back({*iter_x, *iter_y, *iter_z});
  }
  return points;
}

bool is_same_points(const PointXYZList & points1, const PointXYZList & points2)
{
  if (points1.size() != points2.size()) {
    return false;
  }
  // sort both vectors to ensure order does not affect comparison
  PointXYZList sorted_points1 = points1;
  PointXYZList sorted_points2 = points2;
  std::sort(sorted_points1.begin(), sorted_points1.end());
  std::sort(sorted_points2.begin(), sorted_points2.end());
  // compare each point
  for (size_t i = 0; i < sorted_points1.size(); ++i) {
    if (sorted_points1[i] != sorted_points2[i]) {
      return false;
    }
  }
  return true;
}

PointXYZList apply_crop_box_filter(
  const PointXYZList & points, const autoware::crop_box_filter::CropBoxFilterConfig & config)
{
  const auto input_cloud = create_pointcloud2(const_cast<PointXYZList &>(points));
  const autoware::crop_box_filter::CropBoxFilter filter(config);
  const auto result = filter.filter(input_cloud);
  return extract_points_from_cloud(result.pointcloud);
}

TEST(CropBoxFilterTest, FilterZeroPointReturnZeroPoint)
{
  // Arrange
  autoware::crop_box_filter::CropBoxFilterConfig config;
  config.param = {-5.0f, 5.0f, -5.0f, 5.0f, -5.0f, 5.0f};
  config.keep_outside_box = true;
  config.output_frame = "base_link";

  // Act
  const auto output_points = apply_crop_box_filter({}, config);

  // Assert
  EXPECT_TRUE(output_points.empty());
}

TEST(CropBoxFilterTest, FilterExcludePointsInsideBoxWhenKeepOutsideBox)
{
  // Arrange
  autoware::crop_box_filter::CropBoxFilterConfig config;
  config.param = {-5.0f, 5.0f, -5.0f, 5.0f, -5.0f, 5.0f};
  config.keep_outside_box = true;
  config.output_frame = "base_link";
  // clang-format off
  PointXYZList input_points = {
    // points inside the box
    {0.5f, 0.5f, 0.1f},
    {1.5f, 1.5f, 1.1f},
    {2.5f, 2.5f, 2.1f},
    {3.5f, 3.5f, 3.1f},
    {4.5f, 4.5f, 4.1f},
    // points outside the box
    {5.5f, 5.5f, 5.1f},
    {6.5f, 6.5f, 6.1f},
    {7.5f, 7.5f, 7.1f},
    {8.5f, 8.5f, 8.1f},
    {9.5f, 9.5f, 9.1f},
    {-5.5f, -5.5f, -5.1f},
    {-6.5f, -6.5f, -6.1f},
    {-7.5f, -7.5f, -7.1f},
    {-8.5f, -8.5f, -8.1f},
    {-9.5f, -9.5f, -9.1f}
  };
  PointXYZList expected_points = {
    {5.5f, 5.5f, 5.1f},
    {6.5f, 6.5f, 6.1f},
    {7.5f, 7.5f, 7.1f},
    {8.5f, 8.5f, 8.1f},
    {9.5f, 9.5f, 9.1f},
    {-5.5f, -5.5f, -5.1f},
    {-6.5f, -6.5f, -6.1f},
    {-7.5f, -7.5f, -7.1f},
    {-8.5f, -8.5f, -8.1f},
    {-9.5f, -9.5f, -9.1f}
  };
  // clang-format on

  // Act
  const auto output_points = apply_crop_box_filter(input_points, config);

  // Assert
  EXPECT_TRUE(is_same_points(expected_points, output_points));
}

TEST(CropBoxFilterTest, FilterExcludePointsOutsideBoxWhenKeepInsideBox)
{
  // Arrange
  autoware::crop_box_filter::CropBoxFilterConfig config;
  config.param = {-5.0f, 5.0f, -5.0f, 5.0f, -5.0f, 5.0f};
  config.keep_outside_box = false;
  config.output_frame = "base_link";
  // clang-format off
  PointXYZList input_points = {
    // points inside the box
    {0.5f, 0.5f, 0.1f},
    {1.5f, 1.5f, 1.1f},
    {2.5f, 2.5f, 2.1f},
    {3.5f, 3.5f, 3.1f},
    {4.5f, 4.5f, 4.1f},
    // points outside the box
    {5.5f, 5.5f, 5.1f},
    {6.5f, 6.5f, 6.1f},
    {7.5f, 7.5f, 7.1f},
    {8.5f, 8.5f, 8.1f},
    {9.5f, 9.5f, 9.1f},
    {-5.5f, -5.5f, -5.1f},
    {-6.5f, -6.5f, -6.1f},
    {-7.5f, -7.5f, -7.1f},
    {-8.5f, -8.5f, -8.1f},
    {-9.5f, -9.5f, -9.1f}
  };
  PointXYZList expected_points = {
    {0.5f, 0.5f, 0.1f},
    {1.5f, 1.5f, 1.1f},
    {2.5f, 2.5f, 2.1f},
    {3.5f, 3.5f, 3.1f},
    {4.5f, 4.5f, 4.1f}
  };
  // clang-format on

  // Act
  const auto output_points = apply_crop_box_filter(input_points, config);

  // Assert
  EXPECT_TRUE(is_same_points(expected_points, output_points));
}

TEST(GenerateCropBoxPolygonTest, SetsFrameIdStampAndPointCount)
{
  // Arrange
  autoware::crop_box_filter::CropBoxParam param = {-5.0f, 5.0f, -5.0f, 5.0f, -5.0f, 5.0f};
  const std::string frame_id = "base_link";
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 123;
  stamp.nanosec = 456;

  // Act
  const auto polygon = autoware::crop_box_filter::generate_crop_box_polygon(param, frame_id, stamp);

  // Assert
  EXPECT_EQ(polygon.header.frame_id, frame_id);
  EXPECT_EQ(polygon.header.stamp.sec, 123);
  EXPECT_EQ(polygon.header.stamp.nanosec, 456u);
  EXPECT_EQ(polygon.polygon.points.size(), 16u);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
