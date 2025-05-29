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

#include "../src/euclidean_cluster_node.hpp"
#include "../src/voxel_grid_based_euclidean_cluster_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>

using autoware::euclidean_cluster::EuclideanClusterNode;
using autoware::euclidean_cluster::VoxelGridBasedEuclideanClusterNode;

// Create a test framework class for testing EuclideanClusterNode
class TestEuclideanClusterNode : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    // Create node
    rclcpp::NodeOptions options;
    options.append_parameter_override("use_height", true);
    options.append_parameter_override("min_cluster_size", 3);
    options.append_parameter_override("max_cluster_size", 200);
    options.append_parameter_override("tolerance", 0.5);

    node_ = std::make_shared<EuclideanClusterNode>(options);

    // Wait for publishers and subscribers to establish connections
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<EuclideanClusterNode> node_;
};

// Create a test framework class for testing VoxelGridBasedEuclideanClusterNode
class TestVoxelGridBasedEuclideanClusterNode : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    // Create node
    rclcpp::NodeOptions options;
    options.append_parameter_override("use_height", true);
    options.append_parameter_override("min_cluster_size", 3);
    options.append_parameter_override("max_cluster_size", 200);
    options.append_parameter_override("tolerance", 0.5);
    options.append_parameter_override("voxel_leaf_size", 0.2);
    options.append_parameter_override("min_points_number_per_voxel", 2);

    node_ = std::make_shared<VoxelGridBasedEuclideanClusterNode>(options);

    // Wait for publishers and subscribers to establish connections
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<VoxelGridBasedEuclideanClusterNode> node_;
};

// Test creation of EuclideanClusterNode
TEST_F(TestEuclideanClusterNode, NodeCreation)
{
  EXPECT_NE(node_, nullptr);
}

// Test creation of VoxelGridBasedEuclideanClusterNode
TEST_F(TestVoxelGridBasedEuclideanClusterNode, NodeCreation)
{
  EXPECT_NE(node_, nullptr);
}

// Implement a test to check if the node can process point cloud messages
TEST_F(TestEuclideanClusterNode, MessageProcessing)
{
  // Create a test node for subscribing to output
  auto test_node = std::make_shared<rclcpp::Node>("test_node");
  bool message_received = false;

  auto sub = test_node->create_subscription<autoware_perception_msgs::msg::DetectedObjects>(
    "output", rclcpp::QoS{1},
    [&message_received](const autoware_perception_msgs::msg::DetectedObjects::SharedPtr) {
      message_received = true;
    });

  // Create a publisher to publish point cloud messages to the cluster node
  auto pub = test_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(1));

  // Create a simple point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width = 10;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);

  // Fill point cloud data - create two different clusters
  for (size_t i = 0; i < 5; ++i) {
    cloud->points[i].x = 0.1f * static_cast<float>(i);
    cloud->points[i].y = 0.1f * static_cast<float>(i);
    cloud->points[i].z = 0.1f * static_cast<float>(i);
  }

  for (size_t i = 5; i < 10; ++i) {
    cloud->points[i].x = 10.0f + 0.1f * static_cast<float>(i);
    cloud->points[i].y = 10.0f + 0.1f * static_cast<float>(i);
    cloud->points[i].z = 10.0f + 0.1f * static_cast<float>(i);
  }

  // Convert to ROS message
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.frame_id = "base_link";
  cloud_msg.header.stamp = test_node->now();

  // Publish message
  pub->publish(cloud_msg);

  // Process callbacks
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(test_node);
  executor.add_node(node_);

  // Wait for message reception or timeout
  const auto start = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::seconds(5);
  while (!message_received) {
    executor.spin_some();

    // Timeout check
    if (std::chrono::steady_clock::now() - start > timeout) {
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_TRUE(message_received) << "Did not receive expected message within timeout";
}

// Test message processing for VoxelGridBasedEuclideanClusterNode
TEST_F(TestVoxelGridBasedEuclideanClusterNode, MessageProcessing)
{
  // Create a test node for subscribing to output
  auto test_node = std::make_shared<rclcpp::Node>("test_node");
  bool message_received = false;

  auto sub = test_node->create_subscription<autoware_perception_msgs::msg::DetectedObjects>(
    "output", rclcpp::QoS{1},
    [&message_received](const autoware_perception_msgs::msg::DetectedObjects::SharedPtr) {
      message_received = true;
    });

  // Create a publisher to publish point cloud messages to the cluster node
  auto pub = test_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(1));

  // Create a simple point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width = 10;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);

  // Fill point cloud data - create two different clusters
  for (size_t i = 0; i < 5; ++i) {
    cloud->points[i].x = 0.1f * static_cast<float>(i);
    cloud->points[i].y = 0.1f * static_cast<float>(i);
    cloud->points[i].z = 0.1f * static_cast<float>(i);
  }

  for (size_t i = 5; i < 10; ++i) {
    cloud->points[i].x = 10.0f + 0.1f * static_cast<float>(i);
    cloud->points[i].y = 10.0f + 0.1f * static_cast<float>(i);
    cloud->points[i].z = 10.0f + 0.1f * static_cast<float>(i);
  }

  // Convert to ROS message
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.frame_id = "base_link";
  cloud_msg.header.stamp = test_node->now();

  // Publish message
  pub->publish(cloud_msg);

  // Process callbacks
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(test_node);
  executor.add_node(node_);

  // Wait for message reception or timeout
  const auto start = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::seconds(5);
  while (!message_received) {
    executor.spin_some();

    // Timeout check
    if (std::chrono::steady_clock::now() - start > timeout) {
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_TRUE(message_received) << "Did not receive expected message within timeout";
}

// Test empty message processing (VoxelGridBasedEuclideanClusterNode specific code branch)
TEST_F(TestVoxelGridBasedEuclideanClusterNode, EmptyMessageProcessing)
{
  // Create a test node for subscribing to output
  auto test_node = std::make_shared<rclcpp::Node>("test_node");
  bool message_received = false;

  auto sub = test_node->create_subscription<autoware_perception_msgs::msg::DetectedObjects>(
    "output", rclcpp::QoS{1},
    [&message_received](const autoware_perception_msgs::msg::DetectedObjects::SharedPtr) {
      message_received = true;
    });

  // Create a publisher to publish point cloud messages to the cluster node
  auto pub = test_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(1));

  // Create an empty point cloud message
  sensor_msgs::msg::PointCloud2 empty_cloud_msg;
  empty_cloud_msg.header.frame_id = "base_link";
  empty_cloud_msg.header.stamp = test_node->now();

  pub->publish(empty_cloud_msg);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(test_node);
  executor.add_node(node_);

  // Wait for message reception or timeout
  const auto start = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::seconds(5);
  while (!message_received) {
    executor.spin_some();

    // Timeout check
    if (std::chrono::steady_clock::now() - start > timeout) {
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Expect to receive a message here, even if the input is empty, the node should still process and
  // output
  EXPECT_TRUE(message_received) << "Did not receive expected message within timeout";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
