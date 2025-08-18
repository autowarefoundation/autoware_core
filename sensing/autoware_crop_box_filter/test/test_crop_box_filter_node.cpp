// Copyright(c) 2025 AutoCore Technology (Nanjing) Co., Ltd. All rights reserved.
//
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

#include "autoware/crop_box_filter/crop_box_filter_node.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

sensor_msgs::msg::PointCloud2 create_pointcloud2(std::vector<std::array<float, 3>> & points)
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

std::vector<std::array<float, 3>> extract_points_from_cloud(
  const sensor_msgs::msg::PointCloud2 & cloud)
{
  std::vector<std::array<float, 3>> points;
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    points.push_back({*iter_x, *iter_y, *iter_z});
  }
  return points;
}

bool is_same_points(
  const std::vector<std::array<float, 3>> & points1,
  const std::vector<std::array<float, 3>> & points2)
{
  if (points1.size() != points2.size()) {
    return false;
  }
  // sort both vectors to ensure order does not affect comparison
  std::vector<std::array<float, 3>> sorted_points1 = points1;
  std::vector<std::array<float, 3>> sorted_points2 = points2;
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

// Test helper structure to hold crop box parameters
struct CropBoxParams
{
  double min_x = -5.0;
  double max_x = 5.0;
  double min_y = -5.0;
  double max_y = 5.0;
  double min_z = -5.0;
  double max_z = 5.0;
  bool negative = true;
  std::string input_pointcloud_frame = "base_link";
  std::string input_frame = "base_link";
  std::string output_frame = "base_link";
  int64_t max_queue_size = 5;
};

// Helper function to create crop box filter node with given parameters
std::unique_ptr<autoware::crop_box_filter::CropBoxFilter> create_crop_box_filter_node(
  const CropBoxParams & params)
{
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides({
    {"min_x", params.min_x},
    {"min_y", params.min_y},
    {"min_z", params.min_z},
    {"max_x", params.max_x},
    {"max_y", params.max_y},
    {"max_z", params.max_z},
    {"negative", params.negative},
    {"input_pointcloud_frame", params.input_pointcloud_frame},
    {"input_frame", params.input_frame},
    {"output_frame", params.output_frame},
    {"max_queue_size", params.max_queue_size},
  });

  return std::make_unique<autoware::crop_box_filter::CropBoxFilter>(node_options);
}

// Helper function to run crop box filter test
void run_crop_box_filter_test(
  const CropBoxParams & params, const std::vector<std::array<float, 3>> & input_points,
  const std::vector<std::array<float, 3>> & expected_points)
{
  // Create the node with the specified options
  auto node = create_crop_box_filter_node(params);

  // Create pointcloud using helper function
  auto mutable_input_points = const_cast<std::vector<std::array<float, 3>> &>(input_points);
  sensor_msgs::msg::PointCloud2 pointcloud = create_pointcloud2(mutable_input_points);

  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_msg =
    std::make_shared<sensor_msgs::msg::PointCloud2>(pointcloud);

  auto output = sensor_msgs::msg::PointCloud2();
  // filtering
  node->filter_pointcloud(pointcloud_msg, output);

  // Extract points from output using helper function
  std::vector<std::array<float, 3>> output_points = extract_points_from_cloud(output);

  // check if the points are inside/outside the box as expected
  EXPECT_TRUE(is_same_points(expected_points, output_points));
}

TEST(CropBoxFilterTest, FilterZeroPointReturnZeroPoint)
{
  CropBoxParams params;

  // input points
  std::vector<std::array<float, 3>> input_points = {};

  // expected points after filtering
  std::vector<std::array<float, 3>> expected_points = {};

  run_crop_box_filter_test(params, input_points, expected_points);
}

TEST(CropBoxFilterTest, FilterExcludePointsInsideBoxWhenNegative)
{
  CropBoxParams params;
  params.negative = true;

  // clang-format off

  // input points
  std::vector<std::array<float, 3>> input_points = {
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

  // expected points after filtering
  std::vector<std::array<float, 3>> expected_points = {
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

  run_crop_box_filter_test(params, input_points, expected_points);
}

TEST(CropBoxFilterTest, FilterExcludePointsOutsideBoxWhenPositive)
{
  CropBoxParams params;
  params.negative = false;

  // clang-format off

  // input points
  std::vector<std::array<float, 3>> input_points = {
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

  // expected points after filtering
  std::vector<std::array<float, 3>> expected_points = {
    {0.5f, 0.5f, 0.1f},
    {1.5f, 1.5f, 1.1f},
    {2.5f, 2.5f, 2.1f},
    {3.5f, 3.5f, 3.1f},
    {4.5f, 4.5f, 4.1f}
  };

  // clang-format on

  run_crop_box_filter_test(params, input_points, expected_points);
}

int main(int argc, char ** argv)
{
  rclcpp::init(0, nullptr);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
