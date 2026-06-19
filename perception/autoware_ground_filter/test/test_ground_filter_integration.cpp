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

#include "node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/point_types/types.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pcl_msgs/msg/point_indices.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using autoware::ground_filter::GroundFilterComponent;

class GroundFilterIntegrationHarness : public ::testing::Test
{
protected:
  // Node, pub, sub for testing
  std::shared_ptr<GroundFilterComponent> node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr input_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr result_sub_;

  // Sanity check flags and point cloud storage
  sensor_msgs::msg::PointCloud2::SharedPtr received_cloud_;
  bool result_received_;

  /**
   * @brief Construct a new GroundFilterIntegrationHarness object, which sets up
   *        test env for integration testing of GroundFilterComponent.
   */
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    const auto autoware_test_utils_dir =
      ament_index_cpp::get_package_share_directory("autoware_test_utils");
    const auto autoware_ground_filter_dir =
      ament_index_cpp::get_package_share_directory("autoware_ground_filter");

    // Load standard vehicle params
    rclcpp::NodeOptions options;
    autoware::test_utils::updateNodeOptions(
      options, {autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml",
                autoware_ground_filter_dir + "/config/ground_filter.param.yaml"});

    // Specific params overriding for this test
    options.append_parameter_override("elevation_grid_mode", false);
    options.append_parameter_override("input_frame", "base_link");
    options.append_parameter_override("output_frame", "base_link");
    options.append_parameter_override("use_indices", false);
    options.append_parameter_override("approximate_sync", false);

    // Node pub/sub init
    node_ = std::make_shared<GroundFilterComponent>(options);

    result_received_ = false;
    result_sub_ = rclcpp::create_subscription<sensor_msgs::msg::PointCloud2>(
      node_, "output", rclcpp::SensorDataQoS().keep_last(1),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        received_cloud_ = msg;
        result_received_ = true;
      });

    input_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "input", rclcpp::SensorDataQoS().keep_last(1));
  }

  /**
   * @brief Tear down test harness, shutting down ROS and cleaning up resources.
   */
  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

  /**
   * @brief Create a deterministic point cloud, with known ground, obstacle,
   *        as groundtruth for testing this ground filter.
   *
   * @note This point cloud includes:
   *       - Ground points:
   *         - P1: (2,0,0) (near)
   *         - P2: (5,0,0) (far)
   *       - Obstacle points:
   *         - P3: (2,0,1) (floating above P1)
   *         - P4: (5,0,1) (floating above P2)
   *
   * @return sensor_msgs::msg::PointCloud2 A deterministic point cloud message.
   */
  sensor_msgs::msg::PointCloud2 create_deterministic_point_cloud()
  {
    pcl::PointCloud<autoware::point_types::PointXYZIRC> pcl_cloud;

    // P1
    autoware::point_types::PointXYZIRC p1;
    p1.x = 2.0f;
    p1.y = 0.0f;
    p1.z = 0.0f;
    pcl_cloud.push_back(p1);

    // P2
    autoware::point_types::PointXYZIRC p2;
    p2.x = 5.0f;
    p2.y = 0.0f;
    p2.z = 0.0f;
    pcl_cloud.push_back(p2);

    // P3
    autoware::point_types::PointXYZIRC p3;
    p3.x = 2.0f;
    p3.y = 0.0f;
    p3.z = 1.0f;
    pcl_cloud.push_back(p3);

    // P4
    autoware::point_types::PointXYZIRC p4;
    p4.x = 5.0f;
    p4.y = 0.0f;
    p4.z = 1.0f;
    pcl_cloud.push_back(p4);

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(pcl_cloud, cloud_msg);
    cloud_msg.header.frame_id = "base_link";
    cloud_msg.header.stamp = node_->now();

    return cloud_msg;
  }

  /**
   * @brief Helper func to spin node until either result is there no timeout
   */
  void spin_to_process()
  {
    auto start_time = std::chrono::steady_clock::now();
    while (!result_received_ &&
           std::chrono::steady_clock::now() - start_time < std::chrono::seconds(2)) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
};

// ================== TESTING AREA HERE ==================

// TEST 1. Standard filtering test for above deterministic point cloud
// Should filter out ground points (z=0.0) and retain obstacle points (z=1.0)
TEST_F(GroundFilterIntegrationHarness, FiltersGroundPointsAndKeepsObstacles)
{
  auto input_cloud = create_deterministic_point_cloud();

  input_pub_->publish(input_cloud);
  spin_to_process();

  // Result actually generated and received
  ASSERT_TRUE(result_received_);
  ASSERT_NE(received_cloud_, nullptr);

  // Extract points from output ROS2 message
  std::vector<float> output_z_values;
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*received_cloud_, "z");
  for (; iter_z != iter_z.end(); ++iter_z) {
    output_z_values.push_back(*iter_z);
  }

  // Output should contain exactly those 2 obstacle points
  EXPECT_EQ(output_z_values.size(), 2);
  for (const float z : output_z_values) {
    EXPECT_NEAR(z, 1.0f, 1e-4f);  // Should be the obstacle points at z=1.0
  }
}

// TEST 2. Empty or invalid point clouds are rejected and won't produce output
// This is what I consolidated from the only 5 test cases in the original test file
// that were actually worthy to keep, and they were all about sanity checks, so I
// merged them all here.
// BTW you should see a red ERROR log upon `colcon test` for this test
TEST_F(GroundFilterIntegrationHarness, RejectsEmptyOrInvalidPointClouds)
{
  // Test with completely empty point cloud
  sensor_msgs::msg::PointCloud2 empty_cloud;
  empty_cloud.header.frame_id = "base_link";
  empty_cloud.header.stamp = node_->now();
  empty_cloud.width = 0;
  empty_cloud.height = 0;
  empty_cloud.point_step = 16;
  empty_cloud.data.clear();

  input_pub_->publish(empty_cloud);

  // Give the node a tiny bit of time to "theoretically" process
  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(500)) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Expects no result received for empty point cloud
  EXPECT_FALSE(result_received_);
}
