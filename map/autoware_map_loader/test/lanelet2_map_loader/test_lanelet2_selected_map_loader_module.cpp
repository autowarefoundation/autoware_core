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

#include "../src/lanelet2_map_loader/lanelet2_selected_map_loader_module.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp>
#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/map_loader/lanelet2_map_loader_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_meta_data.hpp>
#include <autoware_map_msgs/srv/get_selected_lanelet2_map.hpp>

#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

using autoware::map_loader::Lanelet2FileMetaData;
using autoware::map_loader::Lanelet2SelectedMapLoaderModule;

namespace
{
// Floating point tolerance at EXPECT_NEAR and similar checks
constexpr float near_tol = 1e-4F;
}  // namespace

class TestLanelet2SelectedMapLoaderModule : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Dummy metadata for testing
    Lanelet2FileMetaData meta1;
    meta1.id = "cell_1";
    meta1.min_x = 0.0;
    meta1.min_y = 0.0;
    meta1.max_x = 10.0;
    meta1.max_y = 10.0;

    Lanelet2FileMetaData meta2;
    meta2.id = "cell_2";
    meta2.min_x = 10.0;
    meta2.min_y = 10.0;
    meta2.max_x = 20.0;
    meta2.max_y = 20.0;

    cell_metadata_dict_["cell_1"] = meta1;
    cell_metadata_dict_["cell_2"] = meta2;

    // Use local projector
    projector_info_.projector_type = autoware_map_msgs::msg::MapProjectorInfo::LOCAL;

    module_ = std::make_unique<Lanelet2SelectedMapLoaderModule>(
      cell_metadata_dict_, projector_info_, 0.2, false);
  }

  std::map<std::string, Lanelet2FileMetaData> cell_metadata_dict_;
  autoware_map_msgs::msg::MapProjectorInfo projector_info_;
  std::unique_ptr<Lanelet2SelectedMapLoaderModule> module_;
};

// TEST 1. Confirms metadata msg is correctly built from provided cell metadata.
// Expects:
// - Number of metadata entries matches input
// - Bounding boxes are correct.
TEST_F(TestLanelet2SelectedMapLoaderModule, TestCreateMetadataMsg)
{
  const auto msg = module_->build_metadata_msg();

  EXPECT_EQ(msg.metadata_list.size(), 2u);

  bool found_cell_1 = false;
  for (const auto & cell : msg.metadata_list) {
    if (cell.cell_id == "cell_1") {
      found_cell_1 = true;
      EXPECT_NEAR(cell.min_x, 0.0, near_tol);
      EXPECT_NEAR(cell.max_x, 10.0, near_tol);
    }
  }
  EXPECT_TRUE(found_cell_1);
}

// TEST 2. Confirms when invalid cell IDs are provided, module returns an empty LaneletMapBin.
TEST_F(TestLanelet2SelectedMapLoaderModule, TestExecuteWithInvalidCells)
{
  std::vector<std::string> invalid_cells = {"missing_cell_1", "missing_cell_2"};
  std::vector<std::string> warnings;  // Here I just use dummy warnings so the test still compiles
  const auto result_bin = module_->execute(invalid_cells, warnings);

  EXPECT_TRUE(result_bin.data.empty());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
