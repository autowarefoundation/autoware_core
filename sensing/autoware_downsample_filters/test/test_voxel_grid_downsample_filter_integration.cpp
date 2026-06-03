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
#include "voxel_grid_downsample_filter/voxel_grid_downsample_filter_node.hpp"

#include <rclcpp/executors.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using autoware::downsample_filters::test_utils::create_pointcloud2;
using autoware::downsample_filters::test_utils::expect_points_near;
using autoware::downsample_filters::test_utils::extract_points_from_cloud;
using autoware::downsample_filters::test_utils::PointXYZ;

class VoxelGridIntegrationHarness
{
public:
  // TODO(sasakisasaki): refactor to src+lib (src = ROS dependent, lib = application logic), then
  // simplify this test
  explicit VoxelGridIntegrationHarness(const rclcpp::NodeOptions & filter_options)
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    tf_node_ = rclcpp::Node::make_shared("voxel_grid_tf_publisher");
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(tf_node_);

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

    executor_->add_node(tf_node_);
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
    static_tf_broadcaster_.reset();
    output_sub_node_.reset();
    input_pub_node_.reset();
    filter_node_.reset();
    tf_node_.reset();
  }

  void publish_static_transform(
    const std::string & parent_frame, const std::string & child_frame, const double tx,
    const double ty, const double tz)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = tf_node_->get_clock()->now();
    transform.header.frame_id = parent_frame;
    transform.child_frame_id = child_frame;
    transform.transform.translation.x = tx;
    transform.transform.translation.y = ty;
    transform.transform.translation.z = tz;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(transform);
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

  rclcpp::Node::SharedPtr tf_node_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

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

TEST(VoxelGridDownsampleFilterIntegrationTest, DoesNotPublishWhenInputTransformIsMissing)
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
  harness.publish_points(input_points, "missing_source_frame");

  EXPECT_FALSE(harness.wait_for_output(std::chrono::milliseconds(1000)));
  EXPECT_EQ(harness.received_cloud(), nullptr);
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

// TODO(sasakisasaki): Re-enable after fixing the bug
TEST(
  VoxelGridDownsampleFilterIntegrationTest,
  DISABLED_OutputFrameParameterShouldSetOutputFrameIdToOutputFrame)
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
  harness.publish_static_transform("map", "sensor_frame", 1.5, 0.0, 0.0);

  const std::vector<PointXYZ> input_points = {
    {1.0f, 2.0f, 3.0f},
  };
  const std::vector<PointXYZ> expected_points = {
    {2.5f, 2.0f, 3.0f},
  };
  harness.publish_points(input_points, "sensor_frame");

  ASSERT_TRUE(harness.wait_for_output(std::chrono::milliseconds(5000)));
  ASSERT_NE(harness.received_cloud(), nullptr);

  const auto output_points = extract_points_from_cloud(*harness.received_cloud());
  // FIXME: Expected behavior after bug fix: output cloud should use output_frame in header.
  EXPECT_EQ(harness.received_cloud()->header.frame_id, "map");
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
