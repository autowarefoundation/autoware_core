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

#include "voxel_grid_downsample_filter/voxel_grid_downsample_filter_node.hpp"

#include <rclcpp/executors.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

using PointXYZ = std::array<float, 3>;

sensor_msgs::msg::PointCloud2 create_pointcloud2(const std::vector<PointXYZ> & points)
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

class VoxelGridIntegrationHarness
{
public:
  // TODO(sasakisasaki): refactor to src+lib (src = ROS dependent, lib = application logic), then
  // simplify this test
  explicit VoxelGridIntegrationHarness(const rclcpp::NodeOptions & filter_options)
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    filter_node_ =
      std::make_shared<autoware::downsample_filters::VoxelGridDownsampleFilter>(filter_options);

    input_pub_node_ = rclcpp::Node::make_shared("voxel_grid_test_input_publisher");
    input_pub_ = input_pub_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "input", rclcpp::SensorDataQoS());

    output_sub_node_ = rclcpp::Node::make_shared("voxel_grid_test_output_subscriber");
    output_sub_ = output_sub_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "output", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        received_cloud_ = msg;
        is_received_.store(true);
      });

    executor_->add_node(filter_node_);
    executor_->add_node(input_pub_node_);
    executor_->add_node(output_sub_node_);

    executor_thread_ = std::thread([this]() { executor_->spin(); });
    (void)wait_for_connections(std::chrono::milliseconds(3000));
  }

  ~VoxelGridIntegrationHarness()
  {
    executor_->cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }

    output_sub_.reset();
    input_pub_.reset();
    output_sub_node_.reset();
    input_pub_node_.reset();
    filter_node_.reset();
  }

  void publish_points(const std::vector<PointXYZ> & points, const std::string & frame_id)
  {
    auto cloud = create_pointcloud2(points);
    cloud.header.frame_id = frame_id;
    cloud.header.stamp = input_pub_node_->get_clock()->now();
    input_pub_->publish(cloud);
  }

  bool wait_for_output(const std::chrono::milliseconds timeout)
  {
    const auto start = std::chrono::steady_clock::now();
    while (!is_received_.load()) {
      if (std::chrono::steady_clock::now() - start > timeout) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return true;
  }

  const sensor_msgs::msg::PointCloud2::SharedPtr & received_cloud() const
  {
    return received_cloud_;
  }

private:
  bool wait_for_connections(const std::chrono::milliseconds timeout)
  {
    const auto start = std::chrono::steady_clock::now();
    while (
      (input_pub_->get_subscription_count() == 0U || output_sub_->get_publisher_count() == 0U) &&
      std::chrono::steady_clock::now() - start <= timeout) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return input_pub_->get_subscription_count() > 0U && output_sub_->get_publisher_count() > 0U;
  }

private:
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;

  std::shared_ptr<autoware::downsample_filters::VoxelGridDownsampleFilter> filter_node_;

  rclcpp::Node::SharedPtr input_pub_node_;
  rclcpp::Node::SharedPtr output_sub_node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr input_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr output_sub_;

  std::atomic_bool is_received_{false};
  sensor_msgs::msg::PointCloud2::SharedPtr received_cloud_;
};

TEST(VoxelGridDownsampleFilterIntegrationTest, CollapsesPointsInSameVoxelToSingleCentroid)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"voxel_size_x", 1.0},
    {"voxel_size_y", 1.0},
    {"voxel_size_z", 1.0},
    {"input_frame", ""},
    {"output_frame", ""},
    {"max_queue_size", static_cast<int64_t>(5)},
  });

  VoxelGridIntegrationHarness harness(options);

  const std::vector<PointXYZ> input_points = {
    {0.1f, 0.1f, 0.1f},
    {0.2f, 0.2f, 0.2f},
    {0.9f, 0.9f, 0.9f},
  };
  const std::vector<PointXYZ> expected_points = {
    {0.4f, 0.4f, 0.4f},
  };
  harness.publish_points(input_points, "sensor_frame");

  ASSERT_TRUE(harness.wait_for_output(std::chrono::milliseconds(5000)));
  ASSERT_NE(harness.received_cloud(), nullptr);

  const auto output_points = extract_points_from_cloud(*harness.received_cloud());
  EXPECT_EQ(harness.received_cloud()->header.frame_id, "sensor_frame");
  expect_points_near(output_points, expected_points, 1.0e-4f);
}

TEST(
  VoxelGridDownsampleFilterIntegrationTest,
  PublishesOutputEvenWhenInputFrameParameterCannotBeTransformed)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"voxel_size_x", 1.0},
    {"voxel_size_y", 1.0},
    {"voxel_size_z", 1.0},
    {"input_frame", "missing_target_frame"},
    {"output_frame", ""},
    {"max_queue_size", static_cast<int64_t>(5)},
  });

  VoxelGridIntegrationHarness harness(options);

  const std::vector<PointXYZ> input_points = {
    {0.1f, 0.1f, 0.1f},
    {0.2f, 0.2f, 0.2f},
  };
  const std::vector<PointXYZ> expected_points = {
    {0.15f, 0.15f, 0.15f},
  };
  harness.publish_points(input_points, "missing_source_frame");

  ASSERT_TRUE(harness.wait_for_output(std::chrono::milliseconds(5000)));
  ASSERT_NE(harness.received_cloud(), nullptr);

  const auto output_points = extract_points_from_cloud(*harness.received_cloud());
  EXPECT_EQ(harness.received_cloud()->header.frame_id, "missing_source_frame");
  expect_points_near(output_points, expected_points, 1.0e-4f);
}

TEST(VoxelGridDownsampleFilterIntegrationTest, EmptyInputResultsInEmptyOutput)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"voxel_size_x", 1.0},
    {"voxel_size_y", 1.0},
    {"voxel_size_z", 1.0},
    {"input_frame", ""},
    {"output_frame", ""},
    {"max_queue_size", static_cast<int64_t>(5)},
  });

  VoxelGridIntegrationHarness harness(options);
  harness.publish_points({}, "sensor_frame");

  ASSERT_TRUE(harness.wait_for_output(std::chrono::milliseconds(5000)));
  ASSERT_NE(harness.received_cloud(), nullptr);
  EXPECT_EQ(harness.received_cloud()->width, 0U);
  EXPECT_TRUE(harness.received_cloud()->data.empty());
}

TEST(VoxelGridDownsampleFilterIntegrationTest, KeepsDistinctPointsForMultipleVoxels)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"voxel_size_x", 1.0},
    {"voxel_size_y", 1.0},
    {"voxel_size_z", 1.0},
    {"input_frame", ""},
    {"output_frame", ""},
    {"max_queue_size", static_cast<int64_t>(5)},
  });

  VoxelGridIntegrationHarness harness(options);
  const std::vector<PointXYZ> input_points = {
    {0.1f, 0.1f, 0.1f},
    {0.2f, 0.2f, 0.2f},
    {1.2f, 1.2f, 1.2f},
    {1.4f, 1.4f, 1.4f},
  };
  const std::vector<PointXYZ> expected_points = {
    {0.15f, 0.15f, 0.15f},
    {1.3f, 1.3f, 1.3f},
  };
  harness.publish_points(input_points, "sensor_frame");

  ASSERT_TRUE(harness.wait_for_output(std::chrono::milliseconds(5000)));
  ASSERT_NE(harness.received_cloud(), nullptr);

  const auto output_points = extract_points_from_cloud(*harness.received_cloud());
  expect_points_near(output_points, expected_points, 1.0e-4f);
}

TEST(
  VoxelGridDownsampleFilterIntegrationTest, BoundaryPointBelongsToExactlyOneVoxelDeterministically)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"voxel_size_x", 1.0},
    {"voxel_size_y", 1.0},
    {"voxel_size_z", 1.0},
    {"input_frame", ""},
    {"output_frame", ""},
    {"max_queue_size", static_cast<int64_t>(5)},
  });

  const std::vector<PointXYZ> input_points = {
    {0.2f, 0.0f, 0.0f},
    {1.0f, 0.0f, 0.0f},
    {1.0f, 0.0f, 0.0f},
  };
  const std::vector<PointXYZ> expected_points = {
    {0.2f, 0.0f, 0.0f},
    {1.0f, 0.0f, 0.0f},
  };

  VoxelGridIntegrationHarness harness(options);
  harness.publish_points(input_points, "sensor_frame");

  ASSERT_TRUE(harness.wait_for_output(std::chrono::milliseconds(5000)));
  ASSERT_NE(harness.received_cloud(), nullptr);

  const auto output_points = extract_points_from_cloud(*harness.received_cloud());
  expect_points_near(output_points, expected_points, 0.01f);
}

TEST(
  VoxelGridDownsampleFilterIntegrationTest,
  OutputFrameParameterDoesNotRewriteFrameIdOrTransformPoints)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"voxel_size_x", 0.5},
    {"voxel_size_y", 0.5},
    {"voxel_size_z", 0.5},
    {"input_frame", ""},
    {"output_frame", "map"},
    {"max_queue_size", static_cast<int64_t>(5)},
  });

  VoxelGridIntegrationHarness harness(options);

  const std::vector<PointXYZ> input_points = {
    {1.0f, 2.0f, 3.0f},
  };
  const std::vector<PointXYZ> expected_points = {
    {1.0f, 2.0f, 3.0f},
  };
  harness.publish_points(input_points, "sensor_frame");

  ASSERT_TRUE(harness.wait_for_output(std::chrono::milliseconds(5000)));
  ASSERT_NE(harness.received_cloud(), nullptr);

  const auto output_points = extract_points_from_cloud(*harness.received_cloud());
  EXPECT_EQ(harness.received_cloud()->header.frame_id, "sensor_frame");
  expect_points_near(output_points, expected_points, 1.0e-4f);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  const auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
