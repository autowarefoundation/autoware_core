// Copyright 2023 The Autoware Contributors
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

#include "../src/pointcloud_map_loader/pointcloud_map_loader_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/srv/get_selected_point_cloud_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

using autoware_map_msgs::srv::GetSelectedPointCloudMap;

class TestPointcloudMapLoaderModule : public ::testing::Test
{
protected:
  std::shared_ptr<autoware::map_loader::PointCloudMapLoaderNode> map_loader_node_;
  std::string temp_pcd_path;

  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    // Create a temporary PCD file with dummy point cloud data
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = 5;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    for (size_t i = 0; i < cloud.points.size(); ++i) {
      cloud.points[i].x = static_cast<float>(i);
      cloud.points[i].y = static_cast<float>(i * 2);
      cloud.points[i].z = static_cast<float>(i * 3);
    }

    temp_pcd_path = "/tmp/test_pointcloud_map_loader_module.pcd";
    pcl::io::savePCDFileASCII(temp_pcd_path, cloud);

    rclcpp::NodeOptions node_options;
    node_options.append_parameter_override(
      "pcd_paths_or_directory", std::vector<std::string>{temp_pcd_path});
    node_options.append_parameter_override("pcd_metadata_path", std::string("/tmp/not_used.yaml"));
    node_options.append_parameter_override("enable_whole_load", true);
    node_options.append_parameter_override("enable_downsampled_whole_load", false);
    node_options.append_parameter_override("enable_partial_load", false);
    node_options.append_parameter_override("enable_selected_load", false);
    map_loader_node_ =
      std::make_shared<autoware::map_loader::PointCloudMapLoaderNode>(node_options);
  }

  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(TestPointcloudMapLoaderModule, LoadPCDFilesNoDownsampleTest)
{
  using namespace std::literals::chrono_literals;

  // Subscribe after node initialization; transient_local publisher should provide the latest map.
  auto pointcloud_received = std::make_shared<bool>(false);
  auto pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();

  auto pointcloud_sub = map_loader_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "output/pointcloud_map", durable_qos,
    [pointcloud_received, pointcloud_msg](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      *pointcloud_received = true;
      *pointcloud_msg = *msg;
    });

  // Spin until pointcloud is received or timeout occurs
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(map_loader_node_);
  auto start_time = map_loader_node_->now();
  while (!*pointcloud_received && (map_loader_node_->now() - start_time).seconds() < 3) {
    executor.spin_some(50ms);
  }

  // Check if the point cloud is received and the content is as expected
  ASSERT_TRUE(*pointcloud_received);

  // Convert the received point cloud to pcl::PointCloud<pcl::PointXYZ>
  pcl::PointCloud<pcl::PointXYZ> received_cloud;
  pcl::fromROSMsg(*pointcloud_msg, received_cloud);

  // Check if the received point cloud matches the content of the temporary PCD file
  ASSERT_EQ(static_cast<int>(received_cloud.width), 5);
  ASSERT_EQ(static_cast<int>(received_cloud.height), 1);

  for (size_t i = 0; i < received_cloud.points.size(); ++i) {
    EXPECT_FLOAT_EQ(received_cloud.points[i].x, static_cast<float>(i));
    EXPECT_FLOAT_EQ(received_cloud.points[i].y, static_cast<float>(i * 2));
    EXPECT_FLOAT_EQ(received_cloud.points[i].z, static_cast<float>(i * 3));
  }
}

TEST(PointcloudMapLoaderModuleSelected, LoadSelectedPCDFiles)
{
  rclcpp::init(0, nullptr);

  pcl::PointCloud<pcl::PointXYZ> dummy_cloud;
  dummy_cloud.width = 3;
  dummy_cloud.height = 1;
  dummy_cloud.points.resize(dummy_cloud.width * dummy_cloud.height);
  dummy_cloud.points[0] = pcl::PointXYZ(-1.0, -1.0, -1.0);
  dummy_cloud.points[1] = pcl::PointXYZ(0.0, 0.0, 0.0);
  dummy_cloud.points[2] = pcl::PointXYZ(1.0, 1.0, 1.0);

  const std::string selected_test_pcd_path = "/tmp/test_pointcloud_map_loader_module_selected.pcd";
  pcl::io::savePCDFileASCII(selected_test_pcd_path, dummy_cloud);

  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override(
    "pcd_paths_or_directory", std::vector<std::string>{selected_test_pcd_path});
  node_options.append_parameter_override("pcd_metadata_path", std::string("/tmp/not_used.yaml"));
  node_options.append_parameter_override("enable_whole_load", false);
  node_options.append_parameter_override("enable_downsampled_whole_load", false);
  node_options.append_parameter_override("enable_partial_load", false);
  node_options.append_parameter_override("enable_selected_load", true);

  auto map_loader_node =
    std::make_shared<autoware::map_loader::PointCloudMapLoaderNode>(node_options);
  auto client =
    map_loader_node->create_client<GetSelectedPointCloudMap>("service/get_selected_pcd_map");

  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(3)));

  auto request = std::make_shared<GetSelectedPointCloudMap::Request>();
  request->cell_ids.push_back(selected_test_pcd_path);

  auto result_future = client->async_send_request(request);
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(map_loader_node, result_future),
    rclcpp::FutureReturnCode::SUCCESS);

  auto result = result_future.get();
  ASSERT_EQ(static_cast<int>(result->new_pointcloud_with_ids.size()), 1);
  const auto & cell = result->new_pointcloud_with_ids[0];
  EXPECT_EQ(cell.cell_id, selected_test_pcd_path);
  EXPECT_FLOAT_EQ(cell.metadata.min_x, -1.0F);
  EXPECT_FLOAT_EQ(cell.metadata.min_y, -1.0F);
  EXPECT_FLOAT_EQ(cell.metadata.max_x, 1.0F);
  EXPECT_FLOAT_EQ(cell.metadata.max_y, 1.0F);
  EXPECT_EQ(result->header.frame_id, "map");

  rclcpp::shutdown();
}

TEST(PointcloudMapLoaderModuleSelected, RequestedIdNotFound)
{
  rclcpp::init(0, nullptr);

  pcl::PointCloud<pcl::PointXYZ> dummy_cloud;
  dummy_cloud.width = 3;
  dummy_cloud.height = 1;
  dummy_cloud.points.resize(dummy_cloud.width * dummy_cloud.height);
  dummy_cloud.points[0] = pcl::PointXYZ(-1.0, -1.0, -1.0);
  dummy_cloud.points[1] = pcl::PointXYZ(0.0, 0.0, 0.0);
  dummy_cloud.points[2] = pcl::PointXYZ(1.0, 1.0, 1.0);

  const std::string selected_test_pcd_path =
    "/tmp/test_pointcloud_map_loader_module_selected_not_found.pcd";
  pcl::io::savePCDFileASCII(selected_test_pcd_path, dummy_cloud);

  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override(
    "pcd_paths_or_directory", std::vector<std::string>{selected_test_pcd_path});
  node_options.append_parameter_override("pcd_metadata_path", std::string("/tmp/not_used.yaml"));
  node_options.append_parameter_override("enable_whole_load", false);
  node_options.append_parameter_override("enable_downsampled_whole_load", false);
  node_options.append_parameter_override("enable_partial_load", false);
  node_options.append_parameter_override("enable_selected_load", true);

  auto map_loader_node =
    std::make_shared<autoware::map_loader::PointCloudMapLoaderNode>(node_options);
  auto client =
    map_loader_node->create_client<GetSelectedPointCloudMap>("service/get_selected_pcd_map");

  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(3)));

  auto request = std::make_shared<GetSelectedPointCloudMap::Request>();
  request->cell_ids.push_back("/tmp/does_not_exist.pcd");

  auto result_future = client->async_send_request(request);
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(map_loader_node, result_future),
    rclcpp::FutureReturnCode::SUCCESS);

  auto result = result_future.get();
  EXPECT_EQ(static_cast<int>(result->new_pointcloud_with_ids.size()), 0);
  EXPECT_EQ(result->header.frame_id, "map");

  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
