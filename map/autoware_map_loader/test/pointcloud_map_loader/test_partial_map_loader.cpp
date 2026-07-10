// Copyright 2026 The Autoware Contributors
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

#include "../src/pointcloud_map_loader/partial_map_loader.hpp"

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::map_loader
{
namespace
{
using GetPartialPointCloudMap = autoware_map_msgs::srv::GetPartialPointCloudMap;

PCDFileMetadata make_metadata(
  const float min_x, const float min_y, const float min_z, const float max_x, const float max_y,
  const float max_z)
{
  PCDFileMetadata metadata;
  metadata.min.x = min_x;
  metadata.min.y = min_y;
  metadata.min.z = min_z;
  metadata.max.x = max_x;
  metadata.max.y = max_y;
  metadata.max.z = max_z;
  return metadata;
}

void write_dummy_pcd(const std::string & path)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = 1;
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.points.resize(1);
  cloud.points[0] = pcl::PointXYZ(0.0F, 0.0F, 0.0F);
  pcl::io::savePCDFileASCII(path, cloud);
}

std::vector<std::string> loaded_ids(const GetPartialPointCloudMap::Response::SharedPtr & response)
{
  std::vector<std::string> ids;
  ids.reserve(response->new_pointcloud_with_ids.size());
  for (const auto & pointcloud_with_id : response->new_pointcloud_with_ids) {
    ids.push_back(pointcloud_with_id.cell_id);
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

TEST(PartialMapLoader, ReturnsAllIntersectingCells)
{
  const std::string in_a = "/tmp/test_partial_map_loader_in_a.pcd";
  const std::string in_b = "/tmp/test_partial_map_loader_in_b.pcd";
  write_dummy_pcd(in_a);
  write_dummy_pcd(in_b);

  autoware_map_msgs::msg::AreaInfo area_info;
  area_info.center_x = 0.0;
  area_info.center_y = 0.0;
  area_info.radius = 2.0;

  const std::map<std::string, PCDFileMetadata> metadata_dict = {
    {in_a, make_metadata(-1.0F, -1.0F, -1.0F, 1.0F, 1.0F, 1.0F)},
    {in_b, make_metadata(1.0F, 0.0F, -1.0F, 3.0F, 1.0F, 1.0F)},
    {"/tmp/test_partial_map_loader_out_a.pcd", make_metadata(5.0F, 5.0F, -1.0F, 6.0F, 6.0F, 1.0F)},
  };

  PartialMapLoaderModule module(metadata_dict);

  auto request = std::make_shared<GetPartialPointCloudMap::Request>();
  request->area = area_info;
  auto response = std::make_shared<GetPartialPointCloudMap::Response>();

  EXPECT_TRUE(module.create_response(request, response));

  EXPECT_EQ(loaded_ids(response), (std::vector<std::string>{in_a, in_b}));
  EXPECT_EQ(response->header.frame_id, "map");
}

TEST(PartialMapLoader, ReturnsEmptyWhenNoCellsIntersect)
{
  autoware_map_msgs::msg::AreaInfo area_info;
  area_info.center_x = 0.0;
  area_info.center_y = 0.0;
  area_info.radius = 1.0;

  const std::map<std::string, PCDFileMetadata> metadata_dict = {
    {"/tmp/test_partial_map_loader_none_out_a.pcd", make_metadata(3.0F, 3.0F, -1.0F, 4.0F, 4.0F, 1.0F)},
    {"/tmp/test_partial_map_loader_none_out_b.pcd", make_metadata(-4.0F, -4.0F, -1.0F, -3.0F, -3.0F, 1.0F)},
  };

  PartialMapLoaderModule module(metadata_dict);

  auto request = std::make_shared<GetPartialPointCloudMap::Request>();
  request->area = area_info;
  auto response = std::make_shared<GetPartialPointCloudMap::Response>();

  EXPECT_TRUE(module.create_response(request, response));

  EXPECT_TRUE(response->new_pointcloud_with_ids.empty());
  EXPECT_EQ(response->header.frame_id, "map");
}

TEST(PartialMapLoader, IncludesBoundaryTouchingCell)
{
  const std::string boundary_touch = "/tmp/test_partial_map_loader_boundary_touch.pcd";
  write_dummy_pcd(boundary_touch);

  autoware_map_msgs::msg::AreaInfo area_info;
  area_info.center_x = 0.0;
  area_info.center_y = 0.0;
  area_info.radius = 2.0;

  const std::map<std::string, PCDFileMetadata> metadata_dict = {
    {boundary_touch, make_metadata(2.0F, 0.0F, -1.0F, 3.0F, 1.0F, 1.0F)},
  };

  PartialMapLoaderModule module(metadata_dict);

  auto request = std::make_shared<GetPartialPointCloudMap::Request>();
  request->area = area_info;
  auto response = std::make_shared<GetPartialPointCloudMap::Response>();

  EXPECT_TRUE(module.create_response(request, response));

  EXPECT_EQ(loaded_ids(response), (std::vector<std::string>{boundary_touch}));
  EXPECT_EQ(response->header.frame_id, "map");
}
}  // namespace
}  // namespace autoware::map_loader

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}