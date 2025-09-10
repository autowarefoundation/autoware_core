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

#include <rclcpp/executors.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <memory>
#include <thread>
#include <vector>

sensor_msgs::msg::PointCloud2 create_pointcloud2(std::vector<std::array<float, 3>> & points)
{
  sensor_msgs::msg::PointCloud2 pointcloud;
  sensor_msgs::PointCloud2Modifier modifier(pointcloud);

  // Create a PointCloud2 with XYZIRC fields (minimum required for this integration test)
  modifier.setPointCloud2Fields(
    6, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
    sensor_msgs::msg::PointField::UINT8, "return_type", 1, sensor_msgs::msg::PointField::UINT8,
    "channel", 1, sensor_msgs::msg::PointField::UINT16);

  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_intensity(pointcloud, "intensity");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_return_type(pointcloud, "return_type");
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_channel(pointcloud, "channel");

  for (const auto & point : points) {
    *iter_x = point[0];
    *iter_y = point[1];
    *iter_z = point[2];
    *iter_intensity = 100;
    *iter_return_type = 0;
    *iter_channel = 0;
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_intensity;
    ++iter_return_type;
    ++iter_channel;
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

// Integration test class for launch test
class CropBoxFilterIntegrationTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    // Create executor for running multiple nodes
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    // Setup TF publisher node
    setupTfPublisher();

    // Setup CropBoxFilter node
    setupCropBoxFilter();

    // Setup PointCloud2 publisher and subscriber
    setupPointCloudNodes();

    // Add all nodes to executor
    executor_->add_node(tf_publisher_node_);
    executor_->add_node(crop_box_filter_node_);
    executor_->add_node(pointcloud_publisher_node_);
    executor_->add_node(pointcloud_subscriber_node_);

    // Start executor in separate thread
    executor_thread_ = std::thread([this]() { executor_->spin(); });

    // Wait for nodes to initialize
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  void TearDown() override
  {
    executor_->cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }

    // Reset all shared pointers to clean up nodes
    crop_box_filter_node_.reset();
    tf_publisher_node_.reset();
    pointcloud_publisher_node_.reset();
    pointcloud_subscriber_node_.reset();
    static_tf_broadcaster_.reset();
    pointcloud_publisher_.reset();
    pointcloud_subscriber_.reset();
  }

protected:
  void setupTfPublisher()
  {
    tf_publisher_node_ = rclcpp::Node::make_shared("tf_publisher");

    // Create static transform broadcaster
    static_tf_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(tf_publisher_node_);

    // Create transform from sensor_frame to base_link
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = tf_publisher_node_->get_clock()->now();
    transform_stamped.header.frame_id = "base_link";
    transform_stamped.child_frame_id = "sensor_frame";
    transform_stamped.transform.translation.x = 1.0;
    transform_stamped.transform.translation.y = 0.0;
    transform_stamped.transform.translation.z = 0.0;
    transform_stamped.transform.rotation.x = 0.0;
    transform_stamped.transform.rotation.y = 0.0;
    transform_stamped.transform.rotation.z = 0.0;
    transform_stamped.transform.rotation.w = 1.0;

    static_tf_broadcaster_->sendTransform(transform_stamped);

    // Create transform from base_link to output_frame
    geometry_msgs::msg::TransformStamped transform_stamped2;
    transform_stamped2.header.stamp = tf_publisher_node_->get_clock()->now();
    transform_stamped2.header.frame_id = "output_frame";
    transform_stamped2.child_frame_id = "base_link";
    transform_stamped2.transform.translation.x = -0.5;
    transform_stamped2.transform.translation.y = 0.0;
    transform_stamped2.transform.translation.z = 0.0;
    transform_stamped2.transform.rotation.x = 0.0;
    transform_stamped2.transform.rotation.y = 0.0;
    transform_stamped2.transform.rotation.z = 0.0;
    transform_stamped2.transform.rotation.w = 1.0;

    static_tf_broadcaster_->sendTransform(transform_stamped2);
  }

  void setupCropBoxFilter()
  {
    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides({
      {"min_x", -2.0},
      {"min_y", -2.0},
      {"min_z", -2.0},
      {"max_x", 2.0},
      {"max_y", 2.0},
      {"max_z", 2.0},
      {"negative", false},
      {"input_pointcloud_frame", "sensor_frame"},
      {"input_frame", "base_link"},
      {"output_frame", "output_frame"},
      {"max_queue_size", 5},
    });

    crop_box_filter_node_ =
      std::make_shared<autoware::crop_box_filter::CropBoxFilter>(node_options);
  }

  void setupPointCloudNodes()
  {
    // PointCloud2 publisher node
    pointcloud_publisher_node_ = rclcpp::Node::make_shared("pointcloud_publisher");
    pointcloud_publisher_ =
      pointcloud_publisher_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "input", rclcpp::SensorDataQoS());

    // PointCloud2 subscriber node
    pointcloud_subscriber_node_ = rclcpp::Node::make_shared("pointcloud_subscriber");
    pointcloud_subscriber_ =
      pointcloud_subscriber_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "output", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          received_pointcloud_ = msg;
          pointcloud_received_ = true;
        });
  }

  void publishTestPointCloud(const std::vector<std::array<float, 3>> & points)
  {
    auto mutable_points = const_cast<std::vector<std::array<float, 3>> &>(points);
    auto pointcloud = create_pointcloud2(mutable_points);
    pointcloud.header.frame_id = "sensor_frame";
    pointcloud.header.stamp = pointcloud_publisher_node_->get_clock()->now();

    pointcloud_publisher_->publish(pointcloud);
  }

  bool waitForPointCloudReceived(
    std::chrono::milliseconds timeout = std::chrono::milliseconds(5000))
  {
    auto start_time = std::chrono::steady_clock::now();
    while (!pointcloud_received_) {
      if (std::chrono::steady_clock::now() - start_time > timeout) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return true;
  }

  void resetReceivedData()
  {
    pointcloud_received_ = false;
    received_pointcloud_.reset();
  }

protected:
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;

  // TF nodes
  rclcpp::Node::SharedPtr tf_publisher_node_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  // CropBoxFilter node
  std::shared_ptr<autoware::crop_box_filter::CropBoxFilter> crop_box_filter_node_;

  // PointCloud2 nodes
  rclcpp::Node::SharedPtr pointcloud_publisher_node_;
  rclcpp::Node::SharedPtr pointcloud_subscriber_node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;

  // Test data
  bool pointcloud_received_ = false;
  sensor_msgs::msg::PointCloud2::SharedPtr received_pointcloud_;
};

/**
 * @test
 * @brief Integration test for CropBoxFilter with TF transform.
 *
 * This test verifies that the CropBoxFilter correctly transforms and filters points
 * in a PointCloud2 message when multiple TF transforms are present between input, base, and output
 * frames.
 */
TEST_F(CropBoxFilterIntegrationTest, IntegrationTestWithTfTransform)
{
  // Test points in sensor_frame coordinates
  std::vector<std::array<float, 3>> input_points = {
    // These points will be inside the crop box after transformation
    {0.5f, 0.5f, 0.5f},     // In sensor_frame, will be (1.5, 0.5, 0.5) in base_link,
                            // then (1.0, 0.5, 0.5) in output_frame
    {-0.5f, -0.5f, -0.5f},  // In sensor_frame, will be (0.5, -0.5, -0.5) in base_link,
                            // then (0.0, -0.5, -0.5) in output_frame

    // These points will be outside the crop box after transformation
    {3.0f, 3.0f, 3.0f},    // In sensor_frame, will be (4.0, 3.0, 3.0) in base_link,
                           // then (3.5, 3.0, 3.0) in output_frame - outside box
    {-3.0f, -3.0f, -3.0f}  // In sensor_frame, will be (-2.0, -3.0, -3.0) in base_link,
                           // then (-2.5, -3.0, -3.0) in output_frame - outside box
  };

  // Expected points after transformation:
  std::vector<std::array<float, 3>> expected_points = {
    {1.0f, 0.5f, 0.5f},    // Inside crop box after transformation
    {0.0f, -0.5f, -0.5f},  // Inside crop box after transformation
    // The other two points should be filtered out
  };

  // Publish test pointcloud
  publishTestPointCloud(input_points);

  // Wait for the filtered pointcloud to be received
  ASSERT_TRUE(waitForPointCloudReceived()) << "Failed to receive filtered pointcloud";

  // Check that we received a pointcloud
  ASSERT_NE(received_pointcloud_, nullptr) << "Received pointcloud is null";

  // Extract points from received pointcloud
  auto received_points = extract_points_from_cloud(*received_pointcloud_);

  // We expect 2 points to pass through the filter (the ones inside the box)
  EXPECT_EQ(received_points.size(), 2) << "Expected 2 points, got " << received_points.size();

  // Check that the output frame is correct
  EXPECT_EQ(received_pointcloud_->header.frame_id, "output_frame");

  // Check that the received points match the expected points
  for (size_t i = 0; i < expected_points.size(); ++i) {
    EXPECT_NEAR(received_points[i][0], expected_points[i][0], 0.01)
      << "X coordinate mismatch for point " << i;
    EXPECT_NEAR(received_points[i][1], expected_points[i][1], 0.01)
      << "Y coordinate mismatch for point " << i;
    EXPECT_NEAR(received_points[i][2], expected_points[i][2], 0.01)
      << "Z coordinate mismatch for point " << i;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
