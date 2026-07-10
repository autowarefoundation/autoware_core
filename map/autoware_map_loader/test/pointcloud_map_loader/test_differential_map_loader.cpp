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

#include <map>
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

TEST(DifferentialMapLoader, EmptyCacheLoadsAllCellsInArea)
{
  autoware_map_msgs::msg::AreaInfo area_info;
  area_info.center_x = 0.0;
  area_info.center_y = 0.0;
  area_info.radius = 2.0;

  const std::map<std::string, PCDFileMetadata> metadata_dict = {
    {"in_a.pcd", make_metadata(-1.0F, -1.0F, -1.0F, 1.0F, 1.0F, 1.0F)},
    {"in_b.pcd", make_metadata(1.0F, 0.0F, -1.0F, 3.0F, 1.0F, 1.0F)},
    {"out_a.pcd", make_metadata(5.0F, 5.0F, -1.0F, 6.0F, 6.0F, 1.0F)},
  };

  const std::vector<std::string> cached_ids;
  const auto plan = create_differential_map_load_plan(area_info, cached_ids, metadata_dict);

  EXPECT_EQ(plan.map_ids_to_load, (std::vector<std::string>{"in_a.pcd", "in_b.pcd"}));
  EXPECT_TRUE(plan.ids_to_remove.empty());
}

TEST(DifferentialMapLoader, CachedInAreaIsKeptAndNotReloaded)
{
  autoware_map_msgs::msg::AreaInfo area_info;
  area_info.center_x = 0.0;
  area_info.center_y = 0.0;
  area_info.radius = 2.0;

  const std::map<std::string, PCDFileMetadata> metadata_dict = {
    {"in_a.pcd", make_metadata(-1.0F, -1.0F, -1.0F, 1.0F, 1.0F, 1.0F)},
    {"in_b.pcd", make_metadata(1.0F, 0.0F, -1.0F, 3.0F, 1.0F, 1.0F)},
  };

  const std::vector<std::string> cached_ids = {"in_a.pcd"};
  const auto plan = create_differential_map_load_plan(area_info, cached_ids, metadata_dict);

  EXPECT_EQ(plan.map_ids_to_load, (std::vector<std::string>{"in_b.pcd"}));
  EXPECT_TRUE(plan.ids_to_remove.empty());
}

TEST(DifferentialMapLoader, CachedOutOfAreaAndUnknownIdsAreRemoved)
{
  autoware_map_msgs::msg::AreaInfo area_info;
  area_info.center_x = 0.0;
  area_info.center_y = 0.0;
  area_info.radius = 2.0;

  const std::map<std::string, PCDFileMetadata> metadata_dict = {
    {"in_a.pcd", make_metadata(-1.0F, -1.0F, -1.0F, 1.0F, 1.0F, 1.0F)},
    {"out_a.pcd", make_metadata(5.0F, 5.0F, -1.0F, 6.0F, 6.0F, 1.0F)},
  };

  const std::vector<std::string> cached_ids = {"out_a.pcd", "unknown.pcd"};
  const auto plan = create_differential_map_load_plan(area_info, cached_ids, metadata_dict);

  EXPECT_EQ(plan.map_ids_to_load, (std::vector<std::string>{"in_a.pcd"}));
  EXPECT_EQ(plan.ids_to_remove, cached_ids);
}
}  // namespace
}  // namespace autoware::map_loader

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}