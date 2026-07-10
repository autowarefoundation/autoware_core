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

#include "../src/pointcloud_map_loader/selected_map_loader.hpp"

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

TEST(SelectedMapLoaderCore, ReturnsLoadAndMissingIdsInRequestOrder)
{
  const std::map<std::string, PCDFileMetadata> metadata_dict = {
    {"a.pcd", make_metadata(-1.0F, -1.0F, -1.0F, 1.0F, 1.0F, 1.0F)},
    {"b.pcd", make_metadata(-2.0F, -2.0F, -1.0F, 2.0F, 2.0F, 1.0F)},
  };

  const std::vector<std::string> request_ids = {
    "missing_0.pcd",
    "b.pcd",
    "missing_1.pcd",
    "a.pcd",
  };

  const auto plan = create_selected_map_load_plan(request_ids, metadata_dict);

  EXPECT_EQ(plan.map_ids_to_load, (std::vector<std::string>{"b.pcd", "a.pcd"}));
  EXPECT_EQ(plan.missing_ids, (std::vector<std::string>{"missing_0.pcd", "missing_1.pcd"}));
}

TEST(SelectedMapLoaderCore, EmptyRequestProducesEmptyPlan)
{
  const std::map<std::string, PCDFileMetadata> metadata_dict = {
    {"a.pcd", make_metadata(-1.0F, -1.0F, -1.0F, 1.0F, 1.0F, 1.0F)},
  };

  const std::vector<std::string> request_ids;
  const auto plan = create_selected_map_load_plan(request_ids, metadata_dict);

  EXPECT_TRUE(plan.map_ids_to_load.empty());
  EXPECT_TRUE(plan.missing_ids.empty());
}

TEST(SelectedMapLoaderCore, DuplicateRequestsArePreserved)
{
  const std::map<std::string, PCDFileMetadata> metadata_dict = {
    {"a.pcd", make_metadata(-1.0F, -1.0F, -1.0F, 1.0F, 1.0F, 1.0F)},
  };

  const std::vector<std::string> request_ids = {
    "a.pcd",
    "a.pcd",
    "missing.pcd",
    "missing.pcd",
  };

  const auto plan = create_selected_map_load_plan(request_ids, metadata_dict);

  EXPECT_EQ(plan.map_ids_to_load, (std::vector<std::string>{"a.pcd", "a.pcd"}));
  EXPECT_EQ(plan.missing_ids, (std::vector<std::string>{"missing.pcd", "missing.pcd"}));
}

TEST(SelectedMapLoaderCore, CreateMetadataSetsMapFrameAndCellBounds)
{
  const std::map<std::string, PCDFileMetadata> metadata_dict = {
    {"a.pcd", make_metadata(0.0F, 1.0F, 2.0F, 10.0F, 11.0F, 12.0F)},
    {"b.pcd", make_metadata(-5.0F, -6.0F, -7.0F, 5.0F, 6.0F, 7.0F)},
  };

  const auto msg = create_metadata(metadata_dict);

  EXPECT_EQ(msg.header.frame_id, "map");
  ASSERT_EQ(msg.metadata_list.size(), 2U);

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
}  // namespace
}  // namespace autoware::map_loader

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
