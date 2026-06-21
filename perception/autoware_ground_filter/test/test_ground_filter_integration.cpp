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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

using autoware::ground_filter::GroundFilterComponent;

namespace
{
// Floating point tolerance at EXPECT_NEAR and similar checks
constexpr float near_tol = 1e-4F;
}  // namespace

class GroundFilterIntegrationHarness : public ::testing::Test
{
protected:
  // Node, pub, sub for testing
  std::shared_ptr<GroundFilterComponent> node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr input_pub_;
  rclcpp::Publisher<pcl_msgs::msg::PointIndices>::SharedPtr indices_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr result_sub_;

  // Sanity check flags and point cloud storage
  sensor_msgs::msg::PointCloud2::SharedPtr received_cloud_;
  bool result_received_{false};

  /**
   * @brief Initialize GroundFilterComponent node with given params.
   *
   * @param use_indices Whether to enable indices input subscription.
   * @param approximate_sync Whether to enable approximate synchronization for inputs.
   */
  void init_node(bool use_indices = false, bool approximate_sync = false)
  {
    // Reset all current pub, sub, node
    result_sub_.reset();
    input_pub_.reset();
    indices_pub_.reset();
    node_.reset();

    // Prepare node options with params
    const auto autoware_test_utils_dir =
      ament_index_cpp::get_package_share_directory("autoware_test_utils");
    const auto autoware_ground_filter_dir =
      ament_index_cpp::get_package_share_directory("autoware_ground_filter");

    rclcpp::NodeOptions options;
    autoware::test_utils::updateNodeOptions(
      options, {autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml",
                autoware_ground_filter_dir + "/config/ground_filter.param.yaml"});

    options.append_parameter_override("elevation_grid_mode", false);
    options.append_parameter_override("input_frame", "base_link");
    options.append_parameter_override("output_frame", "base_link");
    options.append_parameter_override("use_indices", use_indices);
    options.append_parameter_override("approximate_sync", approximate_sync);

    // Create node
    node_ = std::make_shared<GroundFilterComponent>(options);

    // Create result subscription
    reset_result();
    result_sub_ = rclcpp::create_subscription<sensor_msgs::msg::PointCloud2>(
      node_, "output", rclcpp::SensorDataQoS().keep_last(1),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        received_cloud_ = msg;
        result_received_ = true;
      });

    // Create input pub
    input_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "input", rclcpp::SensorDataQoS().keep_last(1));

    // Create indices pub if needed
    if (use_indices) {
      indices_pub_ = node_->create_publisher<pcl_msgs::msg::PointIndices>(
        "indices", rclcpp::SensorDataQoS().keep_last(1));
    }

    ASSERT_TRUE(wait_for_connections()) << "Timed out waiting for test pub/sub discovery";
  }

  /**
   * @brief Helper func to spin node until a condition is met or timeout occurs.
   *
   * @param condition Callable that returns a bool indicating if condition is met.
   * @param timeout Maximum wait duration for condition. Default 2000ms.
   *
   * @return true if condition is met within timeout, false otherwise.
   */
  bool spin_until(
    const std::function<bool()> & condition,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(2000))
  {
    const auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < timeout) {
      if (condition()) {
        return true;
      }

      rclcpp::spin_some(node_);

      // Refresh rate before next check (10ms)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return condition();
  }

  /**
   * @brief Wait for all pubs and subs to be connected. Just a wrapper of spin_until().
   *
   * @return true if all connections are established within timeout, false otherwise.
   */
  bool wait_for_connections()
  {
    return spin_until(
      [this]() {
        const bool output_ready = result_sub_ && result_sub_->get_publisher_count() > 0;
        const bool input_ready = input_pub_ && input_pub_->get_subscription_count() > 0;
        const bool indices_ready = !indices_pub_ || indices_pub_->get_subscription_count() > 0;
        return output_ready && input_ready && indices_ready;
      },
      std::chrono::milliseconds(2000));
  }

  /**
   * @brief Wait for result to be received, within timeout.
   *
   * @param timeout Maximum wait duration for result. Default 2000ms.
   *
   * @return true if result is received within timeout, false otherwise.
   */
  bool wait_for_result(std::chrono::milliseconds timeout = std::chrono::milliseconds(2000))
  {
    return spin_until([this]() { return result_received_; }, timeout);
  }

  // Helper func to reset result flag and received cloud
  void reset_result()
  {
    received_cloud_.reset();
    result_received_ = false;
  }

  /**
   * @brief Helper func to create a PointIndices message with given indices and timestamp.
   *
   * @param indices Vector of point indices to include in message.
   * @param stamp Timestamp to set in message header.
   *
   * @return Shared pointer to created PointIndices message.
   */
  static pcl_msgs::msg::PointIndices::SharedPtr create_indices(
    const std::vector<int> & indices, const rclcpp::Time & stamp)
  {
    auto message = std::make_shared<pcl_msgs::msg::PointIndices>();
    message->header.frame_id = "base_link";
    const auto nanoseconds = stamp.nanoseconds();
    message->header.stamp.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
    message->header.stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
    message->indices = indices;
    return message;
  }

  /**
   * @brief Helper func to publish input point cloud and optional indices, resetting result state
   * before publishing.
   *
   * @param cloud PointCloud2 message to publish as input.
   * @param indices Optional PointIndices message to publish if indices input is enabled.
   */
  void publish_input(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const pcl_msgs::msg::PointIndices::SharedPtr & indices = nullptr)
  {
    reset_result();
    input_pub_->publish(cloud);
    if (indices) {
      ASSERT_NE(indices_pub_, nullptr);
      indices_pub_->publish(*indices);
    }
  }

  /**
   * @brief Helper func to extract Z values from a point cloud. Used in test cases.
   *
   * @return Vector of Z values from received point cloud.
   */
  [[nodiscard]] std::vector<float> collect_output_z_values() const
  {
    std::vector<float> output_z_values;
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*received_cloud_, "z");
    for (; iter_z != iter_z.end(); ++iter_z) {
      output_z_values.push_back(*iter_z);
    }
    return output_z_values;
  }

  /**
   * @brief Helper func to check if a given Z value is approximately contained in output Z values.
   * Used in test cases.
   *
   * @param output_z_values Vector of Z values to check against.
   * @param expected_z Z value to check for approximate presence in output_z_values.
   *
   * @return true if expected_z is approximately contained in output_z_values, false otherwise.
   */
  static bool contains_z(const std::vector<float> & output_z_values, float expected_z)
  {
    return std::find_if(output_z_values.begin(), output_z_values.end(), [expected_z](float z) {
             return std::abs(z - expected_z) < near_tol;
           }) != output_z_values.end();
  }

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
};

// ================== TESTING AREA HERE ==================

// TEST 1. Standard filtering test for above deterministic point cloud
// Should filter out ground points and retain only obstacles as our logic dictates
TEST_F(GroundFilterIntegrationHarness, FiltersGroundPointsAndKeepsObstacles)
{
  auto input_cloud = create_deterministic_point_cloud();

  publish_input(input_cloud);

  // Result actually generated and received
  ASSERT_TRUE(wait_for_result());
  ASSERT_NE(received_cloud_, nullptr);

  const auto output_z_values = collect_output_z_values();

  // 1. Output should contain exactly those 4 obstacle points
  EXPECT_EQ(output_z_values.size(), 4);

  // 2. Here checking exact Z values (heights) of those obstacles

  // Check ray A
  EXPECT_TRUE(contains_z(output_z_values, 0.6f));  // A3
  EXPECT_TRUE(contains_z(output_z_values, 2.0f));  // A4

  // Check ray B
  EXPECT_TRUE(contains_z(output_z_values, 0.5f));  // B2

  // Check ray C
  EXPECT_TRUE(contains_z(output_z_values, 1.3f));  // C3
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

// TEST 3. When indices input is enabled, node should still publish output
// (but currently without subsetting by indices vector).
// This is to verify that enabling indices input won't break node functionality, and also
// to keep a record of current behavior that we might want to change in the future.
// Btw note that current implementation of GroundFilterComponent does not actually subset
// filtering by supplied indices vector, but it still uses indices subscription path if
// enabled, yeah just want to make sure it still publishes results as expected in this case.
TEST_F(GroundFilterIntegrationHarness, PublishesWhenIndicesInputIsEnabled)
{
  init_node(true, false);

  const auto input_cloud = create_deterministic_point_cloud();
  const auto indices =
    create_indices({0, 1, 2, 3, 4, 5, 6}, rclcpp::Time(input_cloud.header.stamp));

  publish_input(input_cloud, indices);

  ASSERT_TRUE(wait_for_result());
  ASSERT_NE(received_cloud_, nullptr);

  const auto output_z_values = collect_output_z_values();
  // Current implementation uses indices subscription path but does not subset filtering
  // by supplied indices vector.
  EXPECT_EQ(output_z_values.size(), 4);
  EXPECT_TRUE(contains_z(output_z_values, 0.6f));
  EXPECT_TRUE(contains_z(output_z_values, 2.0f));
  EXPECT_TRUE(contains_z(output_z_values, 0.5f));
  EXPECT_TRUE(contains_z(output_z_values, 1.3f));
}

// TEST 4. When approximate sync is enabled, node should still publish correct output.
// This is to verify that enabling approximate sync won't break node functionality.
// Although this looks kinda tautological with TEST 3, I still wanna keep it as separate
// test to explicitly verify this approximate sync functionality.
TEST_F(GroundFilterIntegrationHarness, FiltersWithApproximateSync)
{
  init_node(true, true);

  const auto input_cloud = create_deterministic_point_cloud();
  const auto input_stamp = rclcpp::Time(input_cloud.header.stamp);
  const auto indices = create_indices({0, 1, 2, 3, 4, 5, 6, 7, 8, 9}, input_stamp);

  publish_input(input_cloud, indices);

  ASSERT_TRUE(wait_for_result());
  ASSERT_NE(received_cloud_, nullptr);

  const auto output_z_values = collect_output_z_values();
  EXPECT_EQ(output_z_values.size(), 4);
  EXPECT_TRUE(contains_z(output_z_values, 0.6f));
  EXPECT_TRUE(contains_z(output_z_values, 2.0f));
  EXPECT_TRUE(contains_z(output_z_values, 0.5f));
  EXPECT_TRUE(contains_z(output_z_values, 1.3f));
}
