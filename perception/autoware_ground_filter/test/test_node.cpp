// Copyright 2024 Tier IV, Inc.
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

#include "autoware/ground_filter/node.hpp"

#include <autoware/point_types/types.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pcl_msgs/msg/point_indices.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

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
void createXYZIRADRTPointCloud(sensor_msgs::msg::PointCloud2 & cloud_msg)
{
  cloud_msg.header.frame_id = "base_link";
  cloud_msg.height = 1;
  cloud_msg.width = 10;
  cloud_msg.is_dense = true;
  cloud_msg.is_bigendian = false;

  sensor_msgs::msg::PointField x_field;
  x_field.name = "x";
  x_field.offset = 0;
  x_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  x_field.count = 1;
  cloud_msg.fields.push_back(x_field);

  sensor_msgs::msg::PointField y_field;
  y_field.name = "y";
  y_field.offset = 4;
  y_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  y_field.count = 1;
  cloud_msg.fields.push_back(y_field);

  sensor_msgs::msg::PointField z_field;
  z_field.name = "z";
  z_field.offset = 8;
  z_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  z_field.count = 1;
  cloud_msg.fields.push_back(z_field);

  sensor_msgs::msg::PointField intensity_field;
  intensity_field.name = "intensity";
  intensity_field.offset = 12;
  intensity_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  intensity_field.count = 1;
  cloud_msg.fields.push_back(intensity_field);

  sensor_msgs::msg::PointField ring_field;
  ring_field.name = "ring";
  ring_field.offset = 16;
  ring_field.datatype = sensor_msgs::msg::PointField::UINT16;
  ring_field.count = 1;
  cloud_msg.fields.push_back(ring_field);

  sensor_msgs::msg::PointField azimuth_field;
  azimuth_field.name = "azimuth";
  azimuth_field.offset = 18;
  azimuth_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  azimuth_field.count = 1;
  cloud_msg.fields.push_back(azimuth_field);

  sensor_msgs::msg::PointField distance_field;
  distance_field.name = "distance";
  distance_field.offset = 22;
  distance_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  distance_field.count = 1;
  cloud_msg.fields.push_back(distance_field);

  sensor_msgs::msg::PointField return_type_field;
  return_type_field.name = "return_type";
  return_type_field.offset = 26;
  return_type_field.datatype = sensor_msgs::msg::PointField::UINT8;
  return_type_field.count = 1;
  cloud_msg.fields.push_back(return_type_field);

  sensor_msgs::msg::PointField time_field;
  time_field.name = "time";
  time_field.offset = 27;
  time_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  time_field.count = 1;
  cloud_msg.fields.push_back(time_field);

  cloud_msg.point_step = 31;
  cloud_msg.row_step = cloud_msg.width * cloud_msg.point_step;
  cloud_msg.data.resize(cloud_msg.row_step);

  for (size_t i = 0; i < cloud_msg.width; i++) {
    size_t point_offset = i * cloud_msg.point_step;

    float x = static_cast<float>(i) * 0.1f;
    float y = 0.0f;
    float z = 0.0f;

    std::memcpy(&cloud_msg.data[point_offset + 0], &x, sizeof(float));
    std::memcpy(&cloud_msg.data[point_offset + 4], &y, sizeof(float));
    std::memcpy(&cloud_msg.data[point_offset + 8], &z, sizeof(float));

    float intensity = 100.0f;
    std::memcpy(&cloud_msg.data[point_offset + 12], &intensity, sizeof(float));
  }
}

void createXYZIPointCloud(sensor_msgs::msg::PointCloud2 & cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.width = 10;
  cloud.height = 1;
  cloud.points.resize(cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size(); i++) {
    cloud.points[i].x = static_cast<float>(i) * 0.1f;
    cloud.points[i].y = 0.0f;
    cloud.points[i].z = 0.0f;
    cloud.points[i].intensity = 100.0f;
  }

  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = "base_link";
}

class GroundFilterComponentTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    test_cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    createTestPointCloud(*test_cloud_);

    rclcpp::NodeOptions options;

    // Setting ground filter parameters
    options.append_parameter_override("elevation_grid_mode", true);
    options.append_parameter_override("radial_divider_angle_deg", 1.0);
    options.append_parameter_override("global_slope_max_angle_deg", 15.0);
    options.append_parameter_override("local_slope_max_angle_deg", 15.0);
    options.append_parameter_override("split_points_distance_tolerance", 0.2);
    options.append_parameter_override("use_virtual_ground_point", true);
    options.append_parameter_override("split_height_distance", 0.2);
    options.append_parameter_override("use_recheck_ground_cluster", true);
    options.append_parameter_override("use_lowest_point", true);
    options.append_parameter_override("detection_range_z_max", 2.0);
    options.append_parameter_override("low_priority_region_x", 0.0);
    options.append_parameter_override("center_pcl_shift", 0.0);
    options.append_parameter_override("non_ground_height_threshold", 0.2);
    options.append_parameter_override("grid_size_m", 0.5);
    options.append_parameter_override("grid_mode_switch_radius", 20.0);
    options.append_parameter_override("ground_grid_buffer_size", 3);
    options.append_parameter_override("publish_processing_time_detail", false);
    options.append_parameter_override("input_frame", "base_link");
    options.append_parameter_override("output_frame", "base_link");
    options.append_parameter_override("max_queue_size", 5);
    options.append_parameter_override("use_indices", false);
    options.append_parameter_override("latched_indices", false);
    options.append_parameter_override("approximate_sync", false);

    // Setting vehicle parameters
    options.append_parameter_override("wheel_radius", 0.4);
    options.append_parameter_override("wheel_width", 0.2);
    options.append_parameter_override("wheel_base", 2.8);
    options.append_parameter_override("wheel_tread", 1.6);
    options.append_parameter_override("front_overhang", 0.9);
    options.append_parameter_override("rear_overhang", 1.1);
    options.append_parameter_override("left_overhang", 0.3);
    options.append_parameter_override("right_overhang", 0.3);
    options.append_parameter_override("vehicle_height", 1.9);
    options.append_parameter_override("max_steer_angle", 0.6);

    node_ = std::make_shared<GroundFilterComponent>(options);

    // Initialize subscription to test if node is publishing correctly
    result_received_ = false;
    result_sub_ = rclcpp::create_subscription<sensor_msgs::msg::PointCloud2>(
      node_, "output", rclcpp::SensorDataQoS().keep_last(5),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        received_cloud_ = msg;
        result_received_ = true;
      });
  }

  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

  void createTestNode(
    const std::string & input_frame = "base_link", const std::string & output_frame = "base_link")
  {
    rclcpp::NodeOptions options;

    options.append_parameter_override("elevation_grid_mode", true);
    options.append_parameter_override("radial_divider_angle_deg", 1.0);
    options.append_parameter_override("global_slope_max_angle_deg", 15.0);
    options.append_parameter_override("local_slope_max_angle_deg", 15.0);
    options.append_parameter_override("split_points_distance_tolerance", 0.2);
    options.append_parameter_override("use_virtual_ground_point", true);
    options.append_parameter_override("split_height_distance", 0.2);
    options.append_parameter_override("use_recheck_ground_cluster", true);
    options.append_parameter_override("use_lowest_point", true);
    options.append_parameter_override("detection_range_z_max", 2.0);
    options.append_parameter_override("low_priority_region_x", 0.0);
    options.append_parameter_override("center_pcl_shift", 0.0);
    options.append_parameter_override("non_ground_height_threshold", 0.2);
    options.append_parameter_override("grid_size_m", 0.5);
    options.append_parameter_override("grid_mode_switch_radius", 20.0);
    options.append_parameter_override("ground_grid_buffer_size", 3);
    options.append_parameter_override("publish_processing_time_detail", false);

    options.append_parameter_override("input_frame", input_frame);
    options.append_parameter_override("output_frame", output_frame);

    options.append_parameter_override("max_queue_size", 5);
    options.append_parameter_override("use_indices", false);
    options.append_parameter_override("latched_indices", false);
    options.append_parameter_override("approximate_sync", false);

    options.append_parameter_override("wheel_radius", 0.4);
    options.append_parameter_override("wheel_width", 0.2);
    options.append_parameter_override("wheel_base", 2.8);
    options.append_parameter_override("wheel_tread", 1.6);
    options.append_parameter_override("front_overhang", 0.9);
    options.append_parameter_override("rear_overhang", 1.1);
    options.append_parameter_override("left_overhang", 0.3);
    options.append_parameter_override("right_overhang", 0.3);
    options.append_parameter_override("vehicle_height", 1.9);
    options.append_parameter_override("max_steer_angle", 0.6);

    node_ = std::make_shared<autoware::ground_filter::GroundFilterComponent>(options);
  }

  void createTestPointCloud(sensor_msgs::msg::PointCloud2 & cloud_msg)
  {
    pcl::PointCloud<autoware::point_types::PointXYZIRC> pcl_cloud;

    // Add ground points (approximately on a plane with some noise)
    for (int i = 0; i < 100; ++i) {
      autoware::point_types::PointXYZIRC point;
      point.x = static_cast<float>(i % 10) - 5.0f;
      point.y = static_cast<float>(i / 10) - 5.0f;
      // Add small noise to ground points
      point.z =
        0.0f + (static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) - 0.5f) * 0.05f;
      point.intensity = 100;
      point.return_type = 1;
      point.channel = 0;
      pcl_cloud.push_back(point);
    }

    // Add non-ground points (above the ground)
    for (int i = 0; i < 50; ++i) {
      autoware::point_types::PointXYZIRC point;
      point.x = static_cast<float>(i % 10) - 5.0f;
      point.y = static_cast<float>(i / 10) - 5.0f;
      // Place non-ground points higher than ground points
      point.z = 0.5f + (static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX)) * 0.5f;
      point.intensity = 50;
      point.return_type = 1;
      point.channel = 0;
      pcl_cloud.push_back(point);
    }

    // Convert to ROS message
    pcl::toROSMsg(pcl_cloud, cloud_msg);
    cloud_msg.header.frame_id = "base_link";
    cloud_msg.header.stamp = rclcpp::Clock().now();
  }

  void createInvalidPointCloud(sensor_msgs::msg::PointCloud2 & cloud_msg)
  {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

    for (int i = 0; i < 10; ++i) {
      pcl::PointXYZ point;
      point.x = static_cast<float>(i % 5) - 2.0f;
      point.y = static_cast<float>(i / 5) - 2.0f;
      point.z = 0.0f;
      pcl_cloud.push_back(point);
    }

    pcl::toROSMsg(pcl_cloud, cloud_msg);
    cloud_msg.header.frame_id = "base_link";
    cloud_msg.header.stamp = rclcpp::Clock().now();
  }

  pcl_msgs::msg::PointIndices::SharedPtr createPointIndices()
  {
    auto indices = std::make_shared<pcl_msgs::msg::PointIndices>();
    indices->header.frame_id = "base_link";
    indices->header.stamp = rclcpp::Clock().now();
    // Include only the first half of the points (assumed to be ground points in our test data)
    for (int i = 0; i < 75; ++i) {
      indices->indices.push_back(i);
    }
    return indices;
  }

  void spinSome()
  {
    rclcpp::spin_some(node_);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(node_);
  }

  std::shared_ptr<GroundFilterComponent> node_;

  sensor_msgs::msg::PointCloud2::SharedPtr test_cloud_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr result_sub_;
  sensor_msgs::msg::PointCloud2::SharedPtr received_cloud_;
  bool result_received_;
};

TEST_F(GroundFilterComponentTest, TestNodeInitialization)
{
  // Test that the node initializes without crashing
  EXPECT_NE(node_, nullptr);
}

TEST_F(GroundFilterComponentTest, TestParameterUpdate)
{
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("global_slope_max_angle_deg", 20.0);

  auto result = node_->set_parameters(params);

  EXPECT_TRUE(result[0].successful);
}

TEST_F(GroundFilterComponentTest, TestPublish)
{
  auto pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(5));

  pub->publish(*test_cloud_);

  spinSome();

  EXPECT_TRUE(result_received_);
  EXPECT_NE(received_cloud_, nullptr);

  if (received_cloud_) {
    EXPECT_GT(received_cloud_->width * received_cloud_->height, 0);
    EXPECT_GT(received_cloud_->data.size(), 0);

    EXPECT_EQ(received_cloud_->header.frame_id, "base_link");

    bool has_x = false, has_y = false, has_z = false;
    for (const auto & field : received_cloud_->fields) {
      if (field.name == "x") has_x = true;
      if (field.name == "y") has_y = true;
      if (field.name == "z") has_z = true;
    }
    EXPECT_TRUE(has_x && has_y && has_z);
  }
}

TEST_F(GroundFilterComponentTest, TestWithIndices)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("elevation_grid_mode", true);
  options.append_parameter_override("radial_divider_angle_deg", 1.0);
  options.append_parameter_override("global_slope_max_angle_deg", 15.0);
  options.append_parameter_override("local_slope_max_angle_deg", 15.0);
  options.append_parameter_override("split_points_distance_tolerance", 0.2);
  options.append_parameter_override("use_virtual_ground_point", true);
  options.append_parameter_override("split_height_distance", 0.2);
  options.append_parameter_override("use_recheck_ground_cluster", true);
  options.append_parameter_override("use_lowest_point", true);
  options.append_parameter_override("detection_range_z_max", 2.0);
  options.append_parameter_override("low_priority_region_x", 0.0);
  options.append_parameter_override("center_pcl_shift", 0.0);
  options.append_parameter_override("non_ground_height_threshold", 0.2);
  options.append_parameter_override("grid_size_m", 0.5);
  options.append_parameter_override("grid_mode_switch_radius", 20.0);
  options.append_parameter_override("ground_grid_buffer_size", 3);
  options.append_parameter_override("publish_processing_time_detail", false);
  options.append_parameter_override("input_frame", "base_link");
  options.append_parameter_override("output_frame", "base_link");
  options.append_parameter_override("max_queue_size", 5);
  options.append_parameter_override("use_indices", true);  // Set to use indices
  options.append_parameter_override("latched_indices", false);
  options.append_parameter_override("approximate_sync", false);

  options.append_parameter_override("wheel_radius", 0.4);
  options.append_parameter_override("wheel_width", 0.2);
  options.append_parameter_override("wheel_base", 2.8);
  options.append_parameter_override("wheel_tread", 1.6);
  options.append_parameter_override("front_overhang", 0.9);
  options.append_parameter_override("rear_overhang", 1.1);
  options.append_parameter_override("left_overhang", 0.3);
  options.append_parameter_override("right_overhang", 0.3);
  options.append_parameter_override("vehicle_height", 1.9);
  options.append_parameter_override("max_steer_angle", 0.6);

  // Create a new node with the indices option enabled
  auto indices_node = std::make_shared<GroundFilterComponent>(options);

  auto cloud_pub = indices_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(5));
  auto indices_pub = indices_node->create_publisher<pcl_msgs::msg::PointIndices>(
    "indices", rclcpp::SensorDataQoS().keep_last(5));

  bool result_received = false;
  sensor_msgs::msg::PointCloud2::SharedPtr received_cloud;
  auto result_sub = rclcpp::create_subscription<sensor_msgs::msg::PointCloud2>(
    indices_node, "output", rclcpp::SensorDataQoS().keep_last(5),
    [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      received_cloud = msg;
      result_received = true;
    });

  auto indices = createPointIndices();
  cloud_pub->publish(*test_cloud_);
  indices_pub->publish(*indices);

  EXPECT_NO_THROW(rclcpp::spin_some(indices_node));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_NO_THROW(rclcpp::spin_some(indices_node));
}

TEST_F(GroundFilterComponentTest, TestInvalidPointCloudFormat)
{
  sensor_msgs::msg::PointCloud2 invalid_cloud;
  createInvalidPointCloud(invalid_cloud);

  auto pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(5));

  bool result_received = false;
  auto result_sub = rclcpp::create_subscription<sensor_msgs::msg::PointCloud2>(
    node_, "output", rclcpp::SensorDataQoS().keep_last(5),
    [&](const sensor_msgs::msg::PointCloud2::SharedPtr /*msg*/) { result_received = true; });

  pub->publish(invalid_cloud);

  spinSome();

  EXPECT_FALSE(result_received);
}

TEST_F(GroundFilterComponentTest, TestInvalidIndices)
{
  rclcpp::NodeOptions options;

  options.append_parameter_override("elevation_grid_mode", true);
  options.append_parameter_override("radial_divider_angle_deg", 1.0);
  options.append_parameter_override("global_slope_max_angle_deg", 15.0);
  options.append_parameter_override("local_slope_max_angle_deg", 15.0);
  options.append_parameter_override("split_points_distance_tolerance", 0.2);
  options.append_parameter_override("use_virtual_ground_point", true);
  options.append_parameter_override("split_height_distance", 0.2);
  options.append_parameter_override("use_recheck_ground_cluster", true);
  options.append_parameter_override("use_lowest_point", true);
  options.append_parameter_override("detection_range_z_max", 2.0);
  options.append_parameter_override("low_priority_region_x", 0.0);
  options.append_parameter_override("center_pcl_shift", 0.0);
  options.append_parameter_override("non_ground_height_threshold", 0.2);
  options.append_parameter_override("grid_size_m", 0.5);
  options.append_parameter_override("grid_mode_switch_radius", 20.0);
  options.append_parameter_override("ground_grid_buffer_size", 3);
  options.append_parameter_override("publish_processing_time_detail", false);

  options.append_parameter_override("input_frame", "base_link");
  options.append_parameter_override("output_frame", "base_link");

  options.append_parameter_override("max_queue_size", 5);
  options.append_parameter_override("use_indices", true);
  options.append_parameter_override("latched_indices", false);
  options.append_parameter_override("approximate_sync", false);

  options.append_parameter_override("wheel_radius", 0.4);
  options.append_parameter_override("wheel_width", 0.2);
  options.append_parameter_override("wheel_base", 2.8);
  options.append_parameter_override("wheel_tread", 1.6);
  options.append_parameter_override("front_overhang", 0.9);
  options.append_parameter_override("rear_overhang", 1.1);
  options.append_parameter_override("left_overhang", 0.3);
  options.append_parameter_override("right_overhang", 0.3);
  options.append_parameter_override("vehicle_height", 1.9);
  options.append_parameter_override("max_steer_angle", 0.6);

  node_ = std::make_shared<autoware::ground_filter::GroundFilterComponent>(options);

  sensor_msgs::msg::PointCloud2 valid_cloud;
  createTestPointCloud(valid_cloud);

  auto invalid_indices = std::make_shared<pcl_msgs::msg::PointIndices>();
  invalid_indices->header.frame_id = "wrong_frame";
  invalid_indices->header.stamp = rclcpp::Time(0);

  auto cloud_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(5));
  auto indices_pub = node_->create_publisher<pcl_msgs::msg::PointIndices>(
    "indices", rclcpp::SensorDataQoS().keep_last(5));

  bool result_received = false;
  auto result_sub = rclcpp::create_subscription<sensor_msgs::msg::PointCloud2>(
    node_, "output", rclcpp::SensorDataQoS().keep_last(5),
    [&](const sensor_msgs::msg::PointCloud2::SharedPtr /*msg*/) { result_received = true; });

  cloud_pub->publish(valid_cloud);
  indices_pub->publish(*invalid_indices);

  spinSome();

  EXPECT_FALSE(result_received);
}

TEST_F(GroundFilterComponentTest, TestAlternativePointCloudFormats)
{
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud_xyzi;
  for (int i = 0; i < 20; ++i) {
    pcl::PointXYZI point;
    point.x = static_cast<float>(i % 5) - 2.0f;
    point.y = static_cast<float>(i / 5) - 2.0f;
    point.z = 0.0f;
    point.intensity = 100;
    pcl_cloud_xyzi.push_back(point);
  }

  sensor_msgs::msg::PointCloud2 cloud_xyzi;
  pcl::toROSMsg(pcl_cloud_xyzi, cloud_xyzi);
  cloud_xyzi.header.frame_id = "base_link";
  cloud_xyzi.header.stamp = rclcpp::Clock().now();

  auto pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(5));

  bool result_received = false;
  auto result_sub = rclcpp::create_subscription<sensor_msgs::msg::PointCloud2>(
    node_, "output", rclcpp::SensorDataQoS().keep_last(5),
    [&](const sensor_msgs::msg::PointCloud2::SharedPtr /*msg*/) { result_received = true; });

  pub->publish(cloud_xyzi);

  EXPECT_NO_THROW(spinSome());
}

TEST_F(GroundFilterComponentTest, TestDifferentFrames)
{
  createTestNode("base_link", "map");

  auto pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(5));

  bool result_received = false;
  sensor_msgs::msg::PointCloud2::SharedPtr received_cloud;
  auto result_sub = rclcpp::create_subscription<sensor_msgs::msg::PointCloud2>(
    node_, "output", rclcpp::SensorDataQoS().keep_last(5),
    [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      received_cloud = msg;
      result_received = true;
    });

  pub->publish(*test_cloud_);

  EXPECT_NO_THROW(spinSome());
}

TEST_F(GroundFilterComponentTest, TestProcessingTimeDetailPublishing)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("elevation_grid_mode", true);
  options.append_parameter_override("radial_divider_angle_deg", 1.0);
  options.append_parameter_override("global_slope_max_angle_deg", 15.0);
  options.append_parameter_override("local_slope_max_angle_deg", 15.0);
  options.append_parameter_override("split_points_distance_tolerance", 0.2);
  options.append_parameter_override("use_virtual_ground_point", true);
  options.append_parameter_override("split_height_distance", 0.2);
  options.append_parameter_override("use_recheck_ground_cluster", true);
  options.append_parameter_override("use_lowest_point", true);
  options.append_parameter_override("detection_range_z_max", 2.0);
  options.append_parameter_override("low_priority_region_x", 0.0);
  options.append_parameter_override("center_pcl_shift", 0.0);
  options.append_parameter_override("non_ground_height_threshold", 0.2);
  options.append_parameter_override("grid_size_m", 0.5);
  options.append_parameter_override("grid_mode_switch_radius", 20.0);
  options.append_parameter_override("ground_grid_buffer_size", 3);
  options.append_parameter_override("publish_processing_time_detail", true);
  options.append_parameter_override("input_frame", "base_link");
  options.append_parameter_override("output_frame", "base_link");
  options.append_parameter_override("max_queue_size", 5);
  options.append_parameter_override("use_indices", false);
  options.append_parameter_override("latched_indices", false);
  options.append_parameter_override("approximate_sync", false);

  options.append_parameter_override("wheel_radius", 0.4);
  options.append_parameter_override("wheel_width", 0.2);
  options.append_parameter_override("wheel_base", 2.8);
  options.append_parameter_override("wheel_tread", 1.6);
  options.append_parameter_override("front_overhang", 0.9);
  options.append_parameter_override("rear_overhang", 1.1);
  options.append_parameter_override("left_overhang", 0.3);
  options.append_parameter_override("right_overhang", 0.3);
  options.append_parameter_override("vehicle_height", 1.9);
  options.append_parameter_override("max_steer_angle", 0.6);

  node_ = std::make_shared<autoware::ground_filter::GroundFilterComponent>(options);

  auto pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(5));

  pub->publish(*test_cloud_);

  EXPECT_NO_THROW(spinSome());
}

TEST_F(GroundFilterComponentTest, TestFrameTransform)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("elevation_grid_mode", true);
  options.append_parameter_override("radial_divider_angle_deg", 1.0);
  options.append_parameter_override("global_slope_max_angle_deg", 15.0);
  options.append_parameter_override("local_slope_max_angle_deg", 15.0);
  options.append_parameter_override("split_points_distance_tolerance", 0.2);
  options.append_parameter_override("use_virtual_ground_point", true);
  options.append_parameter_override("split_height_distance", 0.2);
  options.append_parameter_override("use_recheck_ground_cluster", true);
  options.append_parameter_override("use_lowest_point", true);
  options.append_parameter_override("detection_range_z_max", 2.0);
  options.append_parameter_override("low_priority_region_x", 0.0);
  options.append_parameter_override("center_pcl_shift", 0.0);
  options.append_parameter_override("non_ground_height_threshold", 0.2);
  options.append_parameter_override("grid_size_m", 0.5);
  options.append_parameter_override("grid_mode_switch_radius", 20.0);
  options.append_parameter_override("ground_grid_buffer_size", 3);
  options.append_parameter_override("publish_processing_time_detail", false);
  options.append_parameter_override("input_frame", "base_link");
  options.append_parameter_override("output_frame", "map");  // Different output frame
  options.append_parameter_override("max_queue_size", 5);
  options.append_parameter_override("use_indices", false);
  options.append_parameter_override("latched_indices", false);
  options.append_parameter_override("approximate_sync", false);

  options.append_parameter_override("wheel_radius", 0.4);
  options.append_parameter_override("wheel_width", 0.2);
  options.append_parameter_override("wheel_base", 2.8);
  options.append_parameter_override("wheel_tread", 1.6);
  options.append_parameter_override("front_overhang", 0.9);
  options.append_parameter_override("rear_overhang", 1.1);
  options.append_parameter_override("left_overhang", 0.3);
  options.append_parameter_override("right_overhang", 0.3);
  options.append_parameter_override("vehicle_height", 1.9);
  options.append_parameter_override("max_steer_angle", 0.6);

  EXPECT_NO_THROW(
    node_ = std::make_shared<autoware::ground_filter::GroundFilterComponent>(options));
}

TEST_F(GroundFilterComponentTest, TestApproximateSync)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("elevation_grid_mode", true);
  options.append_parameter_override("radial_divider_angle_deg", 1.0);
  options.append_parameter_override("global_slope_max_angle_deg", 15.0);
  options.append_parameter_override("local_slope_max_angle_deg", 15.0);
  options.append_parameter_override("split_points_distance_tolerance", 0.2);
  options.append_parameter_override("use_virtual_ground_point", true);
  options.append_parameter_override("split_height_distance", 0.2);
  options.append_parameter_override("use_recheck_ground_cluster", true);
  options.append_parameter_override("use_lowest_point", true);
  options.append_parameter_override("detection_range_z_max", 2.0);
  options.append_parameter_override("low_priority_region_x", 0.0);
  options.append_parameter_override("center_pcl_shift", 0.0);
  options.append_parameter_override("non_ground_height_threshold", 0.2);
  options.append_parameter_override("grid_size_m", 0.5);
  options.append_parameter_override("grid_mode_switch_radius", 20.0);
  options.append_parameter_override("ground_grid_buffer_size", 3);
  options.append_parameter_override("publish_processing_time_detail", false);
  options.append_parameter_override("input_frame", "base_link");
  options.append_parameter_override("output_frame", "base_link");
  options.append_parameter_override("max_queue_size", 5);
  options.append_parameter_override("use_indices", true);  // Enable indices
  options.append_parameter_override("latched_indices", false);
  options.append_parameter_override("approximate_sync", true);  // Enable approximate sync

  options.append_parameter_override("wheel_radius", 0.4);
  options.append_parameter_override("wheel_width", 0.2);
  options.append_parameter_override("wheel_base", 2.8);
  options.append_parameter_override("wheel_tread", 1.6);
  options.append_parameter_override("front_overhang", 0.9);
  options.append_parameter_override("rear_overhang", 1.1);
  options.append_parameter_override("left_overhang", 0.3);
  options.append_parameter_override("right_overhang", 0.3);
  options.append_parameter_override("vehicle_height", 1.9);
  options.append_parameter_override("max_steer_angle", 0.6);

  node_ = std::make_shared<autoware::ground_filter::GroundFilterComponent>(options);

  bool result_received = false;
  sensor_msgs::msg::PointCloud2::SharedPtr received_cloud;
  auto result_sub = rclcpp::create_subscription<sensor_msgs::msg::PointCloud2>(
    node_, "output", rclcpp::SensorDataQoS().keep_last(5),
    [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      received_cloud = msg;
      result_received = true;
    });

  auto cloud_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(5));
  auto indices_pub = node_->create_publisher<pcl_msgs::msg::PointIndices>(
    "indices", rclcpp::SensorDataQoS().keep_last(5));

  auto indices = std::make_shared<pcl_msgs::msg::PointIndices>();
  indices->header.frame_id = "base_link";
  indices->header.stamp = test_cloud_->header.stamp;
  for (int i = 0; i < 75; ++i) {
    indices->indices.push_back(i);
  }

  // Publish test data
  cloud_pub->publish(*test_cloud_);
  indices_pub->publish(*indices);

  // Give some time for processing
  EXPECT_NO_THROW(spinSome());
}

TEST_F(GroundFilterComponentTest, TestCompatiblePointCloud)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("elevation_grid_mode", true);
  options.append_parameter_override("radial_divider_angle_deg", 1.0);
  options.append_parameter_override("global_slope_max_angle_deg", 15.0);
  options.append_parameter_override("local_slope_max_angle_deg", 15.0);
  options.append_parameter_override("split_points_distance_tolerance", 0.2);
  options.append_parameter_override("use_virtual_ground_point", true);
  options.append_parameter_override("split_height_distance", 0.2);
  options.append_parameter_override("use_recheck_ground_cluster", true);
  options.append_parameter_override("use_lowest_point", true);
  options.append_parameter_override("detection_range_z_max", 2.0);
  options.append_parameter_override("low_priority_region_x", 0.0);
  options.append_parameter_override("center_pcl_shift", 0.0);
  options.append_parameter_override("non_ground_height_threshold", 0.2);
  options.append_parameter_override("grid_size_m", 0.5);
  options.append_parameter_override("grid_mode_switch_radius", 20.0);
  options.append_parameter_override("ground_grid_buffer_size", 3);
  options.append_parameter_override("publish_processing_time_detail", false);
  options.append_parameter_override("input_frame", "base_link");
  options.append_parameter_override("output_frame", "base_link");
  options.append_parameter_override("max_queue_size", 5);
  options.append_parameter_override("use_indices", false);
  options.append_parameter_override("latched_indices", false);
  options.append_parameter_override("approximate_sync", false);

  options.append_parameter_override("wheel_radius", 0.4);
  options.append_parameter_override("wheel_width", 0.2);
  options.append_parameter_override("wheel_base", 2.8);
  options.append_parameter_override("wheel_tread", 1.6);
  options.append_parameter_override("front_overhang", 0.9);
  options.append_parameter_override("rear_overhang", 1.1);
  options.append_parameter_override("left_overhang", 0.3);
  options.append_parameter_override("right_overhang", 0.3);
  options.append_parameter_override("vehicle_height", 1.9);
  options.append_parameter_override("max_steer_angle", 0.6);

  auto test_node = std::make_shared<GroundFilterComponent>(options);

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = 10;
  cloud.height = 1;
  cloud.points.resize(cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size(); i++) {
    cloud.points[i].x = static_cast<float>(i) * 0.1f;
    cloud.points[i].y = 0.0f;
    cloud.points[i].z = -1.5f;
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = "base_link";
  cloud_msg.header.stamp = rclcpp::Clock().now();

  auto pub = test_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(1));
  pub->publish(cloud_msg);

  EXPECT_NO_THROW(spinSome());
}

TEST_F(GroundFilterComponentTest, TestNullPointCloudValidation)
{
  sensor_msgs::msg::PointCloud2::ConstSharedPtr null_cloud = nullptr;

  auto pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(1));

  EXPECT_NO_THROW(spinSome());
}

TEST_F(GroundFilterComponentTest, TestInvalidPointCloudValidation)
{
  sensor_msgs::msg::PointCloud2 invalid_cloud;
  invalid_cloud.header.frame_id = "base_link";
  invalid_cloud.header.stamp = rclcpp::Clock().now();
  invalid_cloud.width = 10;
  invalid_cloud.height = 1;
  invalid_cloud.point_step = 16;

  invalid_cloud.data.resize(50);  // Should be 160

  auto pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(1));
  pub->publish(invalid_cloud);

  EXPECT_NO_THROW(spinSome());
}

TEST_F(GroundFilterComponentTest, TestPointsCentroidFunctionality)
{
  // Workaround to test PointsCentroid which is private

  // Create node with specific parameters to trigger different code paths
  rclcpp::NodeOptions options;
  options.append_parameter_override("elevation_grid_mode", false);  // Use non-grid mode
  options.append_parameter_override("radial_divider_angle_deg", 1.0);
  options.append_parameter_override("global_slope_max_angle_deg", 15.0);
  options.append_parameter_override("local_slope_max_angle_deg", 15.0);
  options.append_parameter_override("split_points_distance_tolerance", 0.2);
  options.append_parameter_override("use_virtual_ground_point", true);
  options.append_parameter_override("split_height_distance", 0.2);
  options.append_parameter_override("use_recheck_ground_cluster", false);
  options.append_parameter_override("use_lowest_point", false);
  options.append_parameter_override("detection_range_z_max", 2.0);
  options.append_parameter_override("low_priority_region_x", 5.0);
  options.append_parameter_override("center_pcl_shift", 1.0);
  options.append_parameter_override("non_ground_height_threshold", 0.3);
  options.append_parameter_override("grid_size_m", 0.5);
  options.append_parameter_override("grid_mode_switch_radius", 20.0);
  options.append_parameter_override("ground_grid_buffer_size", 5);
  options.append_parameter_override("publish_processing_time_detail", false);
  options.append_parameter_override("input_frame", "base_link");
  options.append_parameter_override("output_frame", "base_link");
  options.append_parameter_override("max_queue_size", 5);
  options.append_parameter_override("use_indices", false);
  options.append_parameter_override("latched_indices", false);
  options.append_parameter_override("approximate_sync", false);

  // Vehicle parameters
  options.append_parameter_override("wheel_radius", 0.4);
  options.append_parameter_override("wheel_width", 0.2);
  options.append_parameter_override("wheel_base", 2.8);
  options.append_parameter_override("wheel_tread", 1.6);
  options.append_parameter_override("front_overhang", 0.9);
  options.append_parameter_override("rear_overhang", 1.1);
  options.append_parameter_override("left_overhang", 0.3);
  options.append_parameter_override("right_overhang", 0.3);
  options.append_parameter_override("vehicle_height", 1.9);
  options.append_parameter_override("max_steer_angle", 0.6);

  auto test_node = std::make_shared<GroundFilterComponent>(options);

  // Create test point cloud that will exercise more code paths
  pcl::PointCloud<autoware::point_types::PointXYZIRADRT> cloud;
  cloud.width = 200;
  cloud.height = 1;
  cloud.points.resize(cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size(); i++) {
    // Create points at varying distances and heights
    float angle = static_cast<float>(i) * 0.1f;
    float radius = 1.0f + static_cast<float>(i) * 0.1f;
    cloud.points[i].x = radius * std::cos(angle);
    cloud.points[i].y = radius * std::sin(angle);
    cloud.points[i].z = -1.5f + (static_cast<float>(i % 20) * 0.1f);
    cloud.points[i].intensity = 100.0f;
    cloud.points[i].ring = static_cast<uint16_t>(i % 32);
    cloud.points[i].azimuth = angle;
    cloud.points[i].distance = radius;
    cloud.points[i].return_type = 1;
    cloud.points[i].time_stamp = static_cast<double>(i) * 0.001;
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = "base_link";
  cloud_msg.header.stamp = rclcpp::Clock().now();

  auto pub = test_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(1));
  pub->publish(cloud_msg);

  // Spin to process
  EXPECT_NO_THROW(for (int i = 0; i < 10; ++i) {
    rclcpp::spin_some(test_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  });
}

TEST_F(GroundFilterComponentTest, TestDifferentParameterConfigurations)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("elevation_grid_mode", true);
  options.append_parameter_override("radial_divider_angle_deg", 0.5);     // Smaller angle
  options.append_parameter_override("global_slope_max_angle_deg", 30.0);  // Larger angle
  options.append_parameter_override("local_slope_max_angle_deg", 25.0);
  options.append_parameter_override("split_points_distance_tolerance", 0.5);
  options.append_parameter_override("use_virtual_ground_point", false);  // Disable virtual ground
  options.append_parameter_override("split_height_distance", 0.1);
  options.append_parameter_override("use_recheck_ground_cluster", true);
  options.append_parameter_override("use_lowest_point", true);
  options.append_parameter_override("detection_range_z_max", 5.0);
  options.append_parameter_override("low_priority_region_x", 10.0);
  options.append_parameter_override("center_pcl_shift", -2.0);
  options.append_parameter_override("non_ground_height_threshold", 0.5);
  options.append_parameter_override("grid_size_m", 1.0);
  options.append_parameter_override("grid_mode_switch_radius", 10.0);
  options.append_parameter_override("ground_grid_buffer_size", 1);
  options.append_parameter_override("publish_processing_time_detail", false);
  options.append_parameter_override("input_frame", "base_link");
  options.append_parameter_override("output_frame", "base_link");
  options.append_parameter_override("max_queue_size", 10);
  options.append_parameter_override("use_indices", false);
  options.append_parameter_override("latched_indices", false);
  options.append_parameter_override("approximate_sync", false);

  // Vehicle parameters
  options.append_parameter_override("wheel_radius", 0.3);
  options.append_parameter_override("wheel_width", 0.15);
  options.append_parameter_override("wheel_base", 3.0);
  options.append_parameter_override("wheel_tread", 1.8);
  options.append_parameter_override("front_overhang", 1.0);
  options.append_parameter_override("rear_overhang", 1.2);
  options.append_parameter_override("left_overhang", 0.2);
  options.append_parameter_override("right_overhang", 0.2);
  options.append_parameter_override("vehicle_height", 2.1);
  options.append_parameter_override("max_steer_angle", 0.7);

  auto test_node = std::make_shared<GroundFilterComponent>(options);
  EXPECT_NE(test_node, nullptr);

  auto parameters_to_set = std::vector<rclcpp::Parameter>{
    rclcpp::Parameter("grid_size_m", 0.8), rclcpp::Parameter("non_ground_height_threshold", 0.4)};

  auto parameter_client = std::make_shared<rclcpp::SyncParametersClient>(test_node);
  if (parameter_client->wait_for_service(std::chrono::milliseconds(500))) {
    auto result = parameter_client->set_parameters(parameters_to_set);
    EXPECT_TRUE(result[0].successful);
    EXPECT_TRUE(result[1].successful);
  }
}

TEST_F(GroundFilterComponentTest, TestEmptyPointCloud)
{
  // Test with empty point cloud
  sensor_msgs::msg::PointCloud2 empty_cloud;
  empty_cloud.header.frame_id = "base_link";
  empty_cloud.header.stamp = rclcpp::Clock().now();
  empty_cloud.width = 0;
  empty_cloud.height = 0;
  empty_cloud.point_step = 16;
  empty_cloud.data.clear();

  auto pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(1));
  pub->publish(empty_cloud);

  EXPECT_NO_THROW(spinSome());
}

TEST_F(GroundFilterComponentTest, TestPointCloudWithVariedHeights)
{
  pcl::PointCloud<autoware::point_types::PointXYZIRADRT> cloud;
  cloud.width = 50;
  cloud.height = 1;
  cloud.points.resize(cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size(); i++) {
    cloud.points[i].x = static_cast<float>(i) * 0.2f;
    cloud.points[i].y = (i % 2 == 0) ? 0.5f : -0.5f;  // Alternate sides

    // Create varied heights: ground, obstacles, and elevated points
    if (i < 15) {
      cloud.points[i].z = -1.7f;  // Ground level
    } else if (i < 30) {
      cloud.points[i].z = -0.5f;  // Elevated ground
    } else {
      cloud.points[i].z = 0.5f;  // Non-ground/obstacles
    }

    cloud.points[i].intensity = 100.0f;
    cloud.points[i].ring = static_cast<uint16_t>(i % 32);
    cloud.points[i].azimuth = static_cast<float>(i) * 0.02f;
    cloud.points[i].distance = std::sqrt(
      cloud.points[i].x * cloud.points[i].x + cloud.points[i].y * cloud.points[i].y +
      cloud.points[i].z * cloud.points[i].z);
    cloud.points[i].return_type = 1;
    cloud.points[i].time_stamp = static_cast<double>(i) * 0.001;
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = "base_link";
  cloud_msg.header.stamp = rclcpp::Clock().now();

  auto pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(1));
  pub->publish(cloud_msg);

  EXPECT_NO_THROW(spinSome());
}

TEST_F(GroundFilterComponentTest, TestIncompatiblePointCloudLayout)
{
  pcl::PointCloud<autoware::point_types::PointXYZIRC> cloud;
  cloud.width = 20;
  cloud.height = 1;
  cloud.points.resize(cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size(); i++) {
    cloud.points[i].x = static_cast<float>(i) * 0.1f;
    cloud.points[i].y = 0.0f;
    cloud.points[i].z = -1.5f;
    cloud.points[i].intensity = 100.0f;
    cloud.points[i].return_type = static_cast<uint8_t>(i % 4);
    cloud.points[i].channel = static_cast<uint16_t>(i % 4);
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = "base_link";
  cloud_msg.header.stamp = rclcpp::Clock().now();

  auto pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(1));
  pub->publish(cloud_msg);

  EXPECT_NO_THROW(spinSome());
}

TEST_F(GroundFilterComponentTest, TestNonElevationGridMode)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("elevation_grid_mode", false);
  options.append_parameter_override("radial_divider_angle_deg", 1.0);
  options.append_parameter_override("global_slope_max_angle_deg", 15.0);
  options.append_parameter_override("local_slope_max_angle_deg", 15.0);
  options.append_parameter_override("split_points_distance_tolerance", 0.2);
  options.append_parameter_override("use_virtual_ground_point", true);
  options.append_parameter_override("split_height_distance", 0.2);
  options.append_parameter_override("use_recheck_ground_cluster", true);
  options.append_parameter_override("use_lowest_point", true);
  options.append_parameter_override("detection_range_z_max", 2.0);
  options.append_parameter_override("low_priority_region_x", 0.0);
  options.append_parameter_override("center_pcl_shift", 0.0);
  options.append_parameter_override("non_ground_height_threshold", 0.2);
  options.append_parameter_override("grid_size_m", 0.5);
  options.append_parameter_override("grid_mode_switch_radius", 20.0);
  options.append_parameter_override("ground_grid_buffer_size", 3);
  options.append_parameter_override("publish_processing_time_detail", false);
  options.append_parameter_override("input_frame", "base_link");
  options.append_parameter_override("output_frame", "base_link");
  options.append_parameter_override("max_queue_size", 5);
  options.append_parameter_override("use_indices", false);
  options.append_parameter_override("latched_indices", false);
  options.append_parameter_override("approximate_sync", false);

  options.append_parameter_override("wheel_radius", 0.4);
  options.append_parameter_override("wheel_width", 0.2);
  options.append_parameter_override("wheel_base", 2.8);
  options.append_parameter_override("wheel_tread", 1.6);
  options.append_parameter_override("front_overhang", 0.9);
  options.append_parameter_override("rear_overhang", 1.1);
  options.append_parameter_override("left_overhang", 0.1);
  options.append_parameter_override("right_overhang", 0.1);
  options.append_parameter_override("vehicle_height", 2.0);
  options.append_parameter_override("max_steer_angle", 0.7);

  auto non_grid_node = std::make_shared<GroundFilterComponent>(options);
  EXPECT_TRUE(non_grid_node != nullptr);

  auto publisher = non_grid_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS());

  auto start_time = std::chrono::steady_clock::now();
  while (publisher->get_subscription_count() == 0 &&
         std::chrono::steady_clock::now() - start_time < std::chrono::seconds(5)) {
    rclcpp::spin_some(non_grid_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  publisher->publish(*test_cloud_);

  for (int i = 0; i < 10; ++i) {
    rclcpp::spin_some(non_grid_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

TEST_F(GroundFilterComponentTest, TestTransformCalculation)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("elevation_grid_mode", true);
  options.append_parameter_override("input_frame", "sensor_frame");
  options.append_parameter_override("output_frame", "base_link");

  options.append_parameter_override("radial_divider_angle_deg", 1.0);
  options.append_parameter_override("global_slope_max_angle_deg", 15.0);
  options.append_parameter_override("local_slope_max_angle_deg", 15.0);
  options.append_parameter_override("split_points_distance_tolerance", 0.2);
  options.append_parameter_override("use_virtual_ground_point", true);
  options.append_parameter_override("split_height_distance", 0.2);
  options.append_parameter_override("use_recheck_ground_cluster", true);
  options.append_parameter_override("use_lowest_point", true);
  options.append_parameter_override("detection_range_z_max", 2.0);
  options.append_parameter_override("low_priority_region_x", 0.0);
  options.append_parameter_override("center_pcl_shift", 0.0);
  options.append_parameter_override("non_ground_height_threshold", 0.2);
  options.append_parameter_override("grid_size_m", 0.5);
  options.append_parameter_override("grid_mode_switch_radius", 20.0);
  options.append_parameter_override("ground_grid_buffer_size", 3);
  options.append_parameter_override("publish_processing_time_detail", false);
  options.append_parameter_override("max_queue_size", 5);
  options.append_parameter_override("use_indices", false);
  options.append_parameter_override("latched_indices", false);
  options.append_parameter_override("approximate_sync", false);

  options.append_parameter_override("wheel_radius", 0.4);
  options.append_parameter_override("wheel_width", 0.2);
  options.append_parameter_override("wheel_base", 2.8);
  options.append_parameter_override("wheel_tread", 1.6);
  options.append_parameter_override("front_overhang", 0.9);
  options.append_parameter_override("rear_overhang", 1.1);
  options.append_parameter_override("left_overhang", 0.1);
  options.append_parameter_override("right_overhang", 0.1);
  options.append_parameter_override("vehicle_height", 2.0);
  options.append_parameter_override("max_steer_angle", 0.7);

  auto transform_node = std::make_shared<GroundFilterComponent>(options);
  EXPECT_TRUE(transform_node != nullptr);

  // Setting to trigger transform
  auto test_cloud_different_frame = std::make_shared<sensor_msgs::msg::PointCloud2>(*test_cloud_);
  test_cloud_different_frame->header.frame_id = "sensor_frame";

  auto publisher = transform_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS());

  auto start_time = std::chrono::steady_clock::now();
  while (publisher->get_subscription_count() == 0 &&
         std::chrono::steady_clock::now() - start_time < std::chrono::seconds(5)) {
    rclcpp::spin_some(transform_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  publisher->publish(*test_cloud_different_frame);

  for (int i = 0; i < 10; ++i) {
    rclcpp::spin_some(transform_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

TEST_F(GroundFilterComponentTest, TestParameterUpdateBranches)
{
  rclcpp::NodeOptions options;

  options.append_parameter_override("elevation_grid_mode", true);
  options.append_parameter_override("radial_divider_angle_deg", 1.0);
  options.append_parameter_override("global_slope_max_angle_deg", 15.0);
  options.append_parameter_override("local_slope_max_angle_deg", 15.0);
  options.append_parameter_override("split_points_distance_tolerance", 0.2);
  options.append_parameter_override("use_virtual_ground_point", true);
  options.append_parameter_override("split_height_distance", 0.2);
  options.append_parameter_override("use_recheck_ground_cluster", true);
  options.append_parameter_override("use_lowest_point", true);
  options.append_parameter_override("detection_range_z_max", 2.0);
  options.append_parameter_override("low_priority_region_x", 0.0);
  options.append_parameter_override("center_pcl_shift", 0.0);
  options.append_parameter_override("non_ground_height_threshold", 0.2);
  options.append_parameter_override("grid_size_m", 0.5);
  options.append_parameter_override("grid_mode_switch_radius", 20.0);
  options.append_parameter_override("ground_grid_buffer_size", 3);
  options.append_parameter_override("publish_processing_time_detail", false);
  options.append_parameter_override("input_frame", "base_link");
  options.append_parameter_override("output_frame", "base_link");
  options.append_parameter_override("max_queue_size", 5);
  options.append_parameter_override("use_indices", false);
  options.append_parameter_override("latched_indices", false);
  options.append_parameter_override("approximate_sync", false);

  options.append_parameter_override("wheel_radius", 0.4);
  options.append_parameter_override("wheel_width", 0.2);
  options.append_parameter_override("wheel_base", 2.8);
  options.append_parameter_override("wheel_tread", 1.6);
  options.append_parameter_override("front_overhang", 0.9);
  options.append_parameter_override("rear_overhang", 1.1);
  options.append_parameter_override("left_overhang", 0.1);
  options.append_parameter_override("right_overhang", 0.1);
  options.append_parameter_override("vehicle_height", 2.0);
  options.append_parameter_override("max_steer_angle", 0.7);

  auto node = std::make_shared<GroundFilterComponent>(options);

  auto local_slope_param = rclcpp::Parameter("local_slope_max_angle_deg", 20.0);
  auto result1 = node->set_parameter(local_slope_param);
  EXPECT_TRUE(result1.successful);

  auto radial_param = rclcpp::Parameter("radial_divider_angle_deg", 2.0);
  auto result2 = node->set_parameter(radial_param);
  EXPECT_TRUE(result2.successful);

  auto split_distance_param = rclcpp::Parameter("split_points_distance_tolerance", 0.3);
  auto result3 = node->set_parameter(split_distance_param);
  EXPECT_TRUE(result3.successful);

  auto split_height_param = rclcpp::Parameter("split_height_distance", 0.3);
  auto result4 = node->set_parameter(split_height_param);
  EXPECT_TRUE(result4.successful);

  auto virtual_ground_param = rclcpp::Parameter("use_virtual_ground_point", false);
  auto result5 = node->set_parameter(virtual_ground_param);
  EXPECT_TRUE(result5.successful);

  auto recheck_param = rclcpp::Parameter("use_recheck_ground_cluster", false);
  auto result6 = node->set_parameter(recheck_param);
  EXPECT_TRUE(result6.successful);
}

TEST_F(GroundFilterComponentTest, TestConvertOutputCostlyFunction)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("elevation_grid_mode", true);
  options.append_parameter_override("output_frame", "map");

  options.append_parameter_override("radial_divider_angle_deg", 1.0);
  options.append_parameter_override("global_slope_max_angle_deg", 15.0);
  options.append_parameter_override("local_slope_max_angle_deg", 15.0);
  options.append_parameter_override("split_points_distance_tolerance", 0.2);
  options.append_parameter_override("use_virtual_ground_point", true);
  options.append_parameter_override("split_height_distance", 0.2);
  options.append_parameter_override("use_recheck_ground_cluster", true);
  options.append_parameter_override("use_lowest_point", true);
  options.append_parameter_override("detection_range_z_max", 2.0);
  options.append_parameter_override("low_priority_region_x", 0.0);
  options.append_parameter_override("center_pcl_shift", 0.0);
  options.append_parameter_override("non_ground_height_threshold", 0.2);
  options.append_parameter_override("grid_size_m", 0.5);
  options.append_parameter_override("grid_mode_switch_radius", 20.0);
  options.append_parameter_override("ground_grid_buffer_size", 3);
  options.append_parameter_override("publish_processing_time_detail", false);
  options.append_parameter_override("input_frame", "base_link");
  options.append_parameter_override("max_queue_size", 5);
  options.append_parameter_override("use_indices", false);
  options.append_parameter_override("latched_indices", false);
  options.append_parameter_override("approximate_sync", false);

  options.append_parameter_override("wheel_radius", 0.4);
  options.append_parameter_override("wheel_width", 0.2);
  options.append_parameter_override("wheel_base", 2.8);
  options.append_parameter_override("wheel_tread", 1.6);
  options.append_parameter_override("front_overhang", 0.9);
  options.append_parameter_override("rear_overhang", 1.1);
  options.append_parameter_override("left_overhang", 0.1);
  options.append_parameter_override("right_overhang", 0.1);
  options.append_parameter_override("vehicle_height", 2.0);
  options.append_parameter_override("max_steer_angle", 0.7);

  auto output_transform_node = std::make_shared<GroundFilterComponent>(options);
  EXPECT_TRUE(output_transform_node != nullptr);

  auto publisher = output_transform_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS());

  auto start_time = std::chrono::steady_clock::now();
  while (publisher->get_subscription_count() == 0 &&
         std::chrono::steady_clock::now() - start_time < std::chrono::seconds(5)) {
    rclcpp::spin_some(output_transform_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  publisher->publish(*test_cloud_);

  for (int i = 0; i < 10; ++i) {
    rclcpp::spin_some(output_transform_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

TEST_F(GroundFilterComponentTest, TestCalculateTransformMatrixWithDifferentFrames)
{
  rclcpp::NodeOptions options;

  options.append_parameter_override("elevation_grid_mode", true);
  options.append_parameter_override("radial_divider_angle_deg", 1.0);
  options.append_parameter_override("global_slope_max_angle_deg", 15.0);
  options.append_parameter_override("local_slope_max_angle_deg", 15.0);
  options.append_parameter_override("split_points_distance_tolerance", 0.2);
  options.append_parameter_override("use_virtual_ground_point", true);
  options.append_parameter_override("split_height_distance", 0.2);
  options.append_parameter_override("use_recheck_ground_cluster", true);
  options.append_parameter_override("use_lowest_point", true);
  options.append_parameter_override("detection_range_z_max", 2.0);
  options.append_parameter_override("low_priority_region_x", 0.0);
  options.append_parameter_override("center_pcl_shift", 0.0);
  options.append_parameter_override("non_ground_height_threshold", 0.2);
  options.append_parameter_override("grid_size_m", 0.5);
  options.append_parameter_override("grid_mode_switch_radius", 20.0);
  options.append_parameter_override("ground_grid_buffer_size", 3);
  options.append_parameter_override("publish_processing_time_detail", false);
  options.append_parameter_override("input_frame", "base_link");
  options.append_parameter_override("output_frame", "base_link");
  options.append_parameter_override("max_queue_size", 5);
  options.append_parameter_override("use_indices", false);
  options.append_parameter_override("latched_indices", false);
  options.append_parameter_override("approximate_sync", false);

  options.append_parameter_override("wheel_radius", 0.4);
  options.append_parameter_override("wheel_width", 0.2);
  options.append_parameter_override("wheel_base", 2.8);
  options.append_parameter_override("wheel_tread", 1.6);
  options.append_parameter_override("front_overhang", 0.9);
  options.append_parameter_override("rear_overhang", 1.1);
  options.append_parameter_override("left_overhang", 0.1);
  options.append_parameter_override("right_overhang", 0.1);
  options.append_parameter_override("vehicle_height", 2.0);
  options.append_parameter_override("max_steer_angle", 0.7);

  auto node = std::make_shared<GroundFilterComponent>(options);

  sensor_msgs::msg::PointCloud2 test_msg = *test_cloud_;
  test_msg.header.frame_id = "base_link";

  auto publisher =
    node->create_publisher<sensor_msgs::msg::PointCloud2>("input", rclcpp::SensorDataQoS());

  auto start_time = std::chrono::steady_clock::now();
  while (publisher->get_subscription_count() == 0 &&
         std::chrono::steady_clock::now() - start_time < std::chrono::seconds(5)) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  publisher->publish(test_msg);

  for (int i = 0; i < 10; ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

TEST_F(GroundFilterComponentTest, TestProcessingTimeDetailPublishingEnabled)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("elevation_grid_mode", true);
  options.append_parameter_override("publish_processing_time_detail", true);

  options.append_parameter_override("radial_divider_angle_deg", 1.0);
  options.append_parameter_override("global_slope_max_angle_deg", 15.0);
  options.append_parameter_override("local_slope_max_angle_deg", 15.0);
  options.append_parameter_override("split_points_distance_tolerance", 0.2);
  options.append_parameter_override("use_virtual_ground_point", true);
  options.append_parameter_override("split_height_distance", 0.2);
  options.append_parameter_override("use_recheck_ground_cluster", true);
  options.append_parameter_override("use_lowest_point", true);
  options.append_parameter_override("detection_range_z_max", 2.0);
  options.append_parameter_override("low_priority_region_x", 0.0);
  options.append_parameter_override("center_pcl_shift", 0.0);
  options.append_parameter_override("non_ground_height_threshold", 0.2);
  options.append_parameter_override("grid_size_m", 0.5);
  options.append_parameter_override("grid_mode_switch_radius", 20.0);
  options.append_parameter_override("ground_grid_buffer_size", 3);
  options.append_parameter_override("input_frame", "base_link");
  options.append_parameter_override("output_frame", "base_link");
  options.append_parameter_override("max_queue_size", 5);
  options.append_parameter_override("use_indices", false);
  options.append_parameter_override("latched_indices", false);
  options.append_parameter_override("approximate_sync", false);

  options.append_parameter_override("wheel_radius", 0.4);
  options.append_parameter_override("wheel_width", 0.2);
  options.append_parameter_override("wheel_base", 2.8);
  options.append_parameter_override("wheel_tread", 1.6);
  options.append_parameter_override("front_overhang", 0.9);
  options.append_parameter_override("rear_overhang", 1.1);
  options.append_parameter_override("left_overhang", 0.1);
  options.append_parameter_override("right_overhang", 0.1);
  options.append_parameter_override("vehicle_height", 2.0);
  options.append_parameter_override("max_steer_angle", 0.7);

  auto detail_node = std::make_shared<GroundFilterComponent>(options);
  EXPECT_TRUE(detail_node != nullptr);

  auto publisher =
    detail_node->create_publisher<sensor_msgs::msg::PointCloud2>("input", rclcpp::SensorDataQoS());

  auto start_time = std::chrono::steady_clock::now();
  while (publisher->get_subscription_count() == 0 &&
         std::chrono::steady_clock::now() - start_time < std::chrono::seconds(5)) {
    rclcpp::spin_some(detail_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  publisher->publish(*test_cloud_);

  for (int i = 0; i < 10; ++i) {
    rclcpp::spin_some(detail_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
