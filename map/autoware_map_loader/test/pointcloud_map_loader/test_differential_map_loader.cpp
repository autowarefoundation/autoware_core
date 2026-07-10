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

#include "../src/pointcloud_map_loader/differential_map_loader.hpp"

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

using GetDifferentialPointCloudMap = autoware_map_msgs::srv::GetDifferentialPointCloudMap;

std::vector<std::string> loaded_ids(const GetDifferentialPointCloudMap::Response::SharedPtr & response)
{
  std::vector<std::string> ids;
  ids.reserve(response->new_pointcloud_with_ids.size());
  for (const auto & pointcloud_with_id : response->new_pointcloud_with_ids) {
    ids.push_back(pointcloud_with_id.cell_id);
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

TEST(DifferentialMapLoader, EmptyCacheLoadsAllCellsInArea)
{
  const std::string in_a = "/tmp/test_differential_map_loader_in_a.pcd";
  const std::string in_b = "/tmp/test_differential_map_loader_in_b.pcd";
  write_dummy_pcd(in_a);
  write_dummy_pcd(in_b);

  autoware_map_msgs::msg::AreaInfo area_info;
  area_info.center_x = 0.0;
  area_info.center_y = 0.0;
  area_info.radius = 2.0;

  const std::map<std::string, PCDFileMetadata> metadata_dict = {
    {in_a, make_metadata(-1.0F, -1.0F, -1.0F, 1.0F, 1.0F, 1.0F)},
    {in_b, make_metadata(1.0F, 0.0F, -1.0F, 3.0F, 1.0F, 1.0F)},
    {"/tmp/test_differential_map_loader_out_a.pcd",
     make_metadata(5.0F, 5.0F, -1.0F, 6.0F, 6.0F, 1.0F)},
  };

  DifferentialMapLoaderModule module(metadata_dict);

  auto request = std::make_shared<GetDifferentialPointCloudMap::Request>();
  request->area = area_info;
  request->cached_ids = {};
  auto response = std::make_shared<GetDifferentialPointCloudMap::Response>();

  EXPECT_TRUE(module.create_response(request, response));

  EXPECT_EQ(loaded_ids(response), (std::vector<std::string>{in_a, in_b}));
  EXPECT_TRUE(response->ids_to_remove.empty());
  EXPECT_EQ(response->header.frame_id, "map");
}

TEST(DifferentialMapLoader, CachedInAreaIsKeptAndNotReloaded)
{
  const std::string in_a = "/tmp/test_differential_map_loader_cached_in_a.pcd";
  const std::string in_b = "/tmp/test_differential_map_loader_cached_in_b.pcd";
  write_dummy_pcd(in_b);

  autoware_map_msgs::msg::AreaInfo area_info;
  area_info.center_x = 0.0;
  area_info.center_y = 0.0;
  area_info.radius = 2.0;

  const std::map<std::string, PCDFileMetadata> metadata_dict = {
    {in_a, make_metadata(-1.0F, -1.0F, -1.0F, 1.0F, 1.0F, 1.0F)},
    {in_b, make_metadata(1.0F, 0.0F, -1.0F, 3.0F, 1.0F, 1.0F)},
  };

  DifferentialMapLoaderModule module(metadata_dict);

  auto request = std::make_shared<GetDifferentialPointCloudMap::Request>();
  request->area = area_info;
  request->cached_ids = {in_a};
  auto response = std::make_shared<GetDifferentialPointCloudMap::Response>();

  EXPECT_TRUE(module.create_response(request, response));

  EXPECT_EQ(loaded_ids(response), (std::vector<std::string>{in_b}));
  EXPECT_TRUE(response->ids_to_remove.empty());
  EXPECT_EQ(response->header.frame_id, "map");
}

TEST(DifferentialMapLoader, CachedOutOfAreaAndUnknownIdsAreRemoved)
{
  const std::string in_a = "/tmp/test_differential_map_loader_remove_in_a.pcd";
  const std::string out_a = "/tmp/test_differential_map_loader_remove_out_a.pcd";
  const std::string unknown = "/tmp/test_differential_map_loader_remove_unknown.pcd";
  write_dummy_pcd(in_a);

  autoware_map_msgs::msg::AreaInfo area_info;
  area_info.center_x = 0.0;
  area_info.center_y = 0.0;
  area_info.radius = 2.0;

  const std::map<std::string, PCDFileMetadata> metadata_dict = {
    {in_a, make_metadata(-1.0F, -1.0F, -1.0F, 1.0F, 1.0F, 1.0F)},
    {out_a, make_metadata(5.0F, 5.0F, -1.0F, 6.0F, 6.0F, 1.0F)},
  };

  DifferentialMapLoaderModule module(metadata_dict);

  auto request = std::make_shared<GetDifferentialPointCloudMap::Request>();
  request->area = area_info;
  request->cached_ids = {out_a, unknown};
  auto response = std::make_shared<GetDifferentialPointCloudMap::Response>();

  EXPECT_TRUE(module.create_response(request, response));

  EXPECT_EQ(loaded_ids(response), (std::vector<std::string>{in_a}));
  EXPECT_EQ(response->ids_to_remove, (std::vector<std::string>{out_a, unknown}));
  EXPECT_EQ(response->header.frame_id, "map");
}
}  // namespace
}  // namespace autoware::map_loader

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}