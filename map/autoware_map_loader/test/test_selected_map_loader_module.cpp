// Copyright 2024 The Autoware Contributors
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

#include "../src/pointcloud_map_loader/selected_map_loader_module.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/srv/get_selected_point_cloud_map.hpp>

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>

#include <map>
#include <memory>
#include <string>

using autoware::map_loader::SelectedMapLoaderModule;
using autoware_map_msgs::srv::GetSelectedPointCloudMap;

class TestSelectedMapLoaderModule : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize ROS node
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_selected_map_loader_module");

    // Generate a sample dummy pointcloud and save it to a file
    pcl::PointCloud<pcl::PointXYZ> dummy_cloud;
    dummy_cloud.width = 3;
    dummy_cloud.height = 1;
    dummy_cloud.points.resize(dummy_cloud.width * dummy_cloud.height);
    dummy_cloud.points[0] = pcl::PointXYZ(-1.0, -1.0, -1.0);
    dummy_cloud.points[1] = pcl::PointXYZ(0.0, 0.0, 0.0);
    dummy_cloud.points[2] = pcl::PointXYZ(1.0, 1.0, 1.0);
    pcl::io::savePCDFileASCII("/tmp/dummy.pcd", dummy_cloud);

    // Generate a sample dummy pointcloud metadata dictionary
    autoware::map_loader::PCDFileMetadata dummy_metadata;
    dummy_metadata.min = pcl::PointXYZ(-1.0, -2.0, -3.0);
    dummy_metadata.max = pcl::PointXYZ(1.0, 2.0, 3.0);
    dummy_metadata_dict_["/tmp/dummy.pcd"] = dummy_metadata;

    module_ = std::make_unique<SelectedMapLoaderModule>(dummy_metadata_dict_);

    service_ = node_->create_service<GetSelectedPointCloudMap>(
      "service/get_selected_pcd_map", [this](
                                        GetSelectedPointCloudMap::Request::SharedPtr req,
                                        GetSelectedPointCloudMap::Response::SharedPtr res) {
        return module_->create_response(req, res);
      });

    client_ = node_->create_client<GetSelectedPointCloudMap>("service/get_selected_pcd_map");
  }

  void TearDown() override { rclcpp::shutdown(); }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<SelectedMapLoaderModule> module_;
  rclcpp::Service<GetSelectedPointCloudMap>::SharedPtr service_;
  rclcpp::Client<GetSelectedPointCloudMap>::SharedPtr client_;
  std::map<std::string, autoware::map_loader::PCDFileMetadata> dummy_metadata_dict_;
};

TEST_F(TestSelectedMapLoaderModule, LoadSelectedPCDFiles)
{
  // Wait for the GetSelectedPointCloudMap service to be available
  ASSERT_TRUE(client_->wait_for_service(std::chrono::seconds(3)));

  // Prepare a request for an existing cell id
  auto request = std::make_shared<GetSelectedPointCloudMap::Request>();
  request->cell_ids.push_back("/tmp/dummy.pcd");

  // Call the service
  auto result_future = client_->async_send_request(request);
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(node_, result_future), rclcpp::FutureReturnCode::SUCCESS);

  // The requested cell is found and populated with its metadata bounds
  auto result = result_future.get();
  ASSERT_EQ(static_cast<int>(result->new_pointcloud_with_ids.size()), 1);
  const auto & cell = result->new_pointcloud_with_ids[0];
  EXPECT_EQ(cell.cell_id, "/tmp/dummy.pcd");
  EXPECT_FLOAT_EQ(cell.metadata.min_x, -1.0F);
  EXPECT_FLOAT_EQ(cell.metadata.min_y, -2.0F);
  EXPECT_FLOAT_EQ(cell.metadata.max_x, 1.0F);
  EXPECT_FLOAT_EQ(cell.metadata.max_y, 2.0F);
  EXPECT_EQ(result->header.frame_id, "map");
}

TEST_F(TestSelectedMapLoaderModule, RequestedIdNotFound)
{
  // Wait for the GetSelectedPointCloudMap service to be available
  ASSERT_TRUE(client_->wait_for_service(std::chrono::seconds(3)));

  // Prepare a request for a cell id that does not exist in the metadata dict
  auto request = std::make_shared<GetSelectedPointCloudMap::Request>();
  request->cell_ids.push_back("/tmp/does_not_exist.pcd");

  // Call the service
  auto result_future = client_->async_send_request(request);
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(node_, result_future), rclcpp::FutureReturnCode::SUCCESS);

  // The not-found branch is taken: nothing is returned but the call still succeeds
  auto result = result_future.get();
  EXPECT_EQ(static_cast<int>(result->new_pointcloud_with_ids.size()), 0);
  EXPECT_EQ(result->header.frame_id, "map");
}

TEST(TestCreateMetadata, PopulatesCellMetadataFromDict)
{
  std::map<std::string, autoware::map_loader::PCDFileMetadata> dict;
  autoware::map_loader::PCDFileMetadata m0;
  m0.min = pcl::PointXYZ(0.0, 1.0, 2.0);
  m0.max = pcl::PointXYZ(10.0, 11.0, 12.0);
  dict["a.pcd"] = m0;
  autoware::map_loader::PCDFileMetadata m1;
  m1.min = pcl::PointXYZ(-5.0, -6.0, -7.0);
  m1.max = pcl::PointXYZ(5.0, 6.0, 7.0);
  dict["b.pcd"] = m1;

  const auto msg = autoware::map_loader::create_metadata(dict);

  EXPECT_EQ(msg.header.frame_id, "map");
  ASSERT_EQ(msg.metadata_list.size(), 2U);

  // std::map iterates in sorted key order, so "a.pcd" comes first
  EXPECT_EQ(msg.metadata_list[0].cell_id, "a.pcd");
  EXPECT_FLOAT_EQ(msg.metadata_list[0].metadata.min_x, 0.0F);
  EXPECT_FLOAT_EQ(msg.metadata_list[0].metadata.min_y, 1.0F);
  EXPECT_FLOAT_EQ(msg.metadata_list[0].metadata.max_x, 10.0F);
  EXPECT_FLOAT_EQ(msg.metadata_list[0].metadata.max_y, 11.0F);

  EXPECT_EQ(msg.metadata_list[1].cell_id, "b.pcd");
  EXPECT_FLOAT_EQ(msg.metadata_list[1].metadata.min_x, -5.0F);
  EXPECT_FLOAT_EQ(msg.metadata_list[1].metadata.min_y, -6.0F);
  EXPECT_FLOAT_EQ(msg.metadata_list[1].metadata.max_x, 5.0F);
  EXPECT_FLOAT_EQ(msg.metadata_list[1].metadata.max_y, 6.0F);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
