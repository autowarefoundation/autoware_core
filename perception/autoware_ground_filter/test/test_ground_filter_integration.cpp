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

// Floating point tolerance at EXPECT_NEAR and similar checks
constexpr float near_tol = 1e-4F;

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
   *        as ground truth for testing this ground filter.
   *
   * @note I wanna design this deterministic point cloud (DPC) in a way that
   *       it could COMPREHENSIVELY test our ground filter algorithm to its
   *       extent, cover as much scenarios/edges as possible.
   *       Thus, this DPC includes 3 rays of points, each with 3-4 points:
   *       - Ray A (front, Y = 0) : 4 points
   *           - A1 (R = 2.0, Z = 0.0) : pure ground, baseline
   *           - A2 (R = 3.0, Z = 0.0) : pure ground, test "point follow" logic from A1
   *           - A3 (R = 4.0, Z = 0.6) : obstacle, test local slope threshold after A2
   *           - A4 (R = 5.0, Z = 2.0) : obstacle, test global slope threshold
   *       - Ray B (frontleft, X = Y) : 3 points
   *           - B1 (R = 2.0, Z = 0.0) : pure ground, baseline
   *           - B2 (R = 3.0, Z = 0.5) : obstacle, bump after B1
   *           - B3 (R = 5.0, Z = 0.0) : pure ground, behind B2, test baseline reset logic
   *       - Ray C (frontright, X = -Y) : 3 points
   *           - C1 (R = 2.0, Z = 0.2) : ground, start of slope
   *           - C2 (R = 4.0, Z = 0.6) : ground, slope continues after C1 (~11 deg)
   *           - C3 (R = 6.0, Z = 1.3) : obstacle, slope jump after C2 (~17 deg)
   *       With a hardcoded local/global slope threshold of 15 deg (same as previous test suites),
   *       we gonna have 6 ground 4 obstacle. I hope this DPC could cover all characteristics of
   *       this ground filter algorithm.
   *
   * @return sensor_msgs::msg::PointCloud2 A deterministic point cloud message.
   */
  sensor_msgs::msg::PointCloud2 create_deterministic_point_cloud()
  {
    pcl::PointCloud<autoware::point_types::PointXYZIRC> pcl_cloud;

    // Helper lambda func to add points
    auto add_point = [&](float x, float y, float z) {
      autoware::point_types::PointXYZIRC p;

      p.x = x;
      p.y = y;
      p.z = z;
      p.intensity = 100;
      p.return_type = 1;
      p.channel = 0;

      pcl_cloud.push_back(p);
    };

    // Ray A (front, Y = 0) : 4 points
    add_point(2.0f, 0.0f, 0.0f);
    add_point(3.0f, 0.0f, 0.0f);
    add_point(4.0f, 0.0f, 0.6f);
    add_point(5.0f, 0.0f, 2.0f);

    // Ray B (frontleft, X = Y) : 3 points
    add_point(1.414f, 1.414f, 0.0f);
    add_point(2.121f, 2.121f, 0.5f);
    add_point(3.535f, 3.535f, 0.0f);

    // Ray C (frontright, X = -Y) : 3 points
    add_point(1.414f, -1.414f, 0.2f);
    add_point(2.828f, -2.828f, 0.6f);
    add_point(4.242f, -4.242f, 1.3f);

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
// Should filter out ground points and retain only obstacles as our logic dictates
TEST_F(GroundFilterIntegrationHarness, FiltersGroundPointsAndKeepsObstacles)
{
  auto input_cloud = create_deterministic_point_cloud();

  input_pub_->publish(input_cloud);
  spin_to_process();

  // Result actually generated and received
  ASSERT_TRUE(result_received_);
  ASSERT_NE(received_cloud_, nullptr);

  // Extract points from output ROS message
  std::vector<float> output_z_values;
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*received_cloud_, "z");
  for (; iter_z != iter_z.end(); ++iter_z) {
    output_z_values.push_back(*iter_z);
  }

  // 1. Output should contain exactly those 4 obstacle points
  EXPECT_EQ(output_z_values.size(), 4);

  // 2. Here checking exact Z values (heights) of those obstacles

  // Helper func
  auto contains_z = [&](float expected_z) {
    return std::find_if(output_z_values.begin(), output_z_values.end(), [expected_z](float z) {
             return std::abs(z - expected_z) < near_tol;
           }) != output_z_values.end();
  };

  // Check ray A
  EXPECT_TRUE(contains_z(0.6f));  // A3
  EXPECT_TRUE(contains_z(2.0f));  // A4

  // Check ray B
  EXPECT_TRUE(contains_z(0.5f));  // B2

  // Check ray C
  EXPECT_TRUE(contains_z(1.3f));  // C3
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
