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

#include "../src/voxel_grid_based_euclidean_cluster_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <vector>

namespace autoware::euclidean_cluster
{

class EuclideanClusterObjectDetectorIntegrationHarness : public ::testing::Test
{
protected:
  std::shared_ptr<VoxelGridBasedEuclideanClusterNode> target_node_;
  std::shared_ptr<rclcpp::Node> interceptor_node_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_input_;
  rclcpp::Subscription<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr sub_output_;

  bool message_received_{false};
  autoware_perception_msgs::msg::DetectedObjects::SharedPtr last_output_{nullptr};

  // Test env setup
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    // Initialize default parameters
    // (for now matching legacy node, should be changed later after refactoring)
    rclcpp::NodeOptions options;
    options.append_parameter_override("use_height", true);
    options.append_parameter_override("min_cluster_size", 3);
    options.append_parameter_override("max_cluster_size", 200);
    options.append_parameter_override("tolerance", 0.5);
    options.append_parameter_override("voxel_leaf_size", 0.1);
    options.append_parameter_override("min_points_number_per_voxel", 1);

    target_node_ = std::make_shared<VoxelGridBasedEuclideanClusterNode>(options);

    interceptor_node_ = std::make_shared<rclcpp::Node>("test_interceptor_node");

    pub_input_ = interceptor_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "input", rclcpp::SensorDataQoS().keep_last(1));

    sub_output_ =
      interceptor_node_->create_subscription<autoware_perception_msgs::msg::DetectedObjects>(
        "output", rclcpp::QoS{1},
        [this](const autoware_perception_msgs::msg::DetectedObjects::SharedPtr msg) {
          last_output_ = msg;
          message_received_ = true;
        });

    // Wait for pub/sub handshake
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Test env teardown & cleanup
  void TearDown() override
  {
    target_node_.reset();
    interceptor_node_.reset();
    rclcpp::shutdown();
  }

  /**
   * @brief Helper func to publish a point cloud and wait for output message from target node.
   *
   * @param input_msg Point cloud message to publish.
   *
   * @note Current timeout is 30 seconds. Pub/sub handshake checked every 10 ms.
   */
  void publish_and_wait(const sensor_msgs::msg::PointCloud2 & input_msg)
  {
    message_received_ = false;
    last_output_.reset();

    pub_input_->publish(input_msg);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(interceptor_node_);
    executor.add_node(target_node_);

    const auto start = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::seconds(30);
    while (!message_received_) {
      executor.spin_some();
      if (std::chrono::steady_clock::now() - start > timeout) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  /**
   * @brief Helper func to create a mock point cloud message from a vector of points.
   *
   * @param points Vector of points, each point is an array of 3 floats (x, y, z).
   *
   * @return sensor_msgs::msg::PointCloud2 A point cloud message containing above points.
   */
  sensor_msgs::msg::PointCloud2 create_mock_cloud(const std::vector<std::array<float, 3>> & points)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "base_link";
    cloud.header.stamp = interceptor_node_->now();

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    for (const auto & p : points) {
      *iter_x = p[0];
      *iter_y = p[1];
      *iter_z = p[2];
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }

    return cloud;
  }
};

};  // namespace autoware::euclidean_cluster
