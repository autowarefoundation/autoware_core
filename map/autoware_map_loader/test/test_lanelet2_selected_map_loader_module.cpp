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

#include "../src/lanelet2_map_loader/lanelet2_selected_map_loader_module.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/map_loader/lanelet2_map_loader_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_meta_data.hpp>
#include <autoware_map_msgs/srv/get_selected_lanelet2_map.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <utility>

using autoware::map_loader::Lanelet2FileMetaData;
using autoware::map_loader::Lanelet2SelectedMapLoaderModule;
using autoware_map_msgs::srv::GetSelectedLanelet2Map;

namespace
{
std::string test_map_path()
{
  return ament_index_cpp::get_package_share_directory("autoware_map_loader") +
         "/test/data/test_map.osm";
}

autoware_map_msgs::msg::MapProjectorInfo mgrs_projector_info()
{
  autoware_map_msgs::msg::MapProjectorInfo info;
  info.projector_type = autoware_map_msgs::msg::MapProjectorInfo::MGRS;
  info.mgrs_grid = "54SUE";
  return info;
}
}  // namespace

class TestLanelet2SelectedMapLoaderModule : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_lanelet2_selected_map_loader_module");

    const std::string path = test_map_path();
    const auto projector_info = mgrs_projector_info();

    // Load the map once to compute bounding-box metadata.
    auto map = Lanelet2MapLoaderNode::load_map(path, projector_info);
    ASSERT_NE(map, nullptr) << "Failed to load test map: " << path;

    Lanelet2FileMetaData meta;
    meta.id = path;
    meta.min_x = meta.min_y = meta.max_x = meta.max_y = 0.0;
    for (const auto & pt : map->pointLayer) {
      meta.min_x = std::min(meta.min_x, pt.x());
      meta.min_y = std::min(meta.min_y, pt.y());
      meta.max_x = std::max(meta.max_x, pt.x());
      meta.max_y = std::max(meta.max_y, pt.y());
    }

    std::map<std::string, Lanelet2FileMetaData> metadata_dict{{path, meta}};
    module_ = std::make_shared<Lanelet2SelectedMapLoaderModule>(
      node_.get(), std::move(metadata_dict), projector_info,
      /*center_line_resolution=*/5.0, /*use_waypoints=*/true);

    client_ = node_->create_client<GetSelectedLanelet2Map>("service/get_selected_lanelet2_map");
  }

  void TearDown() override { rclcpp::shutdown(); }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Lanelet2SelectedMapLoaderModule> module_;
  rclcpp::Client<GetSelectedLanelet2Map>::SharedPtr client_;
};

TEST_F(TestLanelet2SelectedMapLoaderModule, ServiceIsAvailable)
{
  ASSERT_TRUE(client_->wait_for_service(std::chrono::seconds(3)));
}

TEST_F(TestLanelet2SelectedMapLoaderModule, LoadKnownCellReturnsNonEmptyMap)
{
  ASSERT_TRUE(client_->wait_for_service(std::chrono::seconds(3)));

  auto request = std::make_shared<GetSelectedLanelet2Map::Request>();
  request->cell_ids = {test_map_path()};

  auto future = client_->async_send_request(request);
  ASSERT_EQ(rclcpp::spin_until_future_complete(node_, future), rclcpp::FutureReturnCode::SUCCESS);

  const auto response = future.get();
  EXPECT_EQ(response->header.frame_id, "map");
  EXPECT_FALSE(response->lanelet2_cells.data.empty())
    << "Expected non-empty LaneletMapBin for a valid cell";
}

TEST_F(TestLanelet2SelectedMapLoaderModule, LoadUnknownCellReturnsFalse)
{
  ASSERT_TRUE(client_->wait_for_service(std::chrono::seconds(3)));

  auto request = std::make_shared<GetSelectedLanelet2Map::Request>();
  request->cell_ids = {"/nonexistent/path/map.osm"};

  auto future = client_->async_send_request(request);
  ASSERT_EQ(rclcpp::spin_until_future_complete(node_, future), rclcpp::FutureReturnCode::SUCCESS);

  // Service returns false for unknown IDs; lanelet2_cells should be empty.
  const auto response = future.get();
  EXPECT_TRUE(response->lanelet2_cells.data.empty());
}

TEST_F(TestLanelet2SelectedMapLoaderModule, MetadataTopicPublished)
{
  bool received = false;
  auto sub = node_->create_subscription<autoware_map_msgs::msg::LaneletMapMetaData>(
    "output/lanelet2_map_metadata", rclcpp::QoS{1}.transient_local(),
    [&received, this](const autoware_map_msgs::msg::LaneletMapMetaData::SharedPtr msg) {
      EXPECT_EQ(msg->header.frame_id, "map");
      EXPECT_EQ(msg->metadata_list.size(), 1u);
      EXPECT_EQ(msg->metadata_list[0].cell_id, test_map_path());
      received = true;
    });

  rclcpp::spin_some(node_);
  EXPECT_TRUE(received) << "LaneletMapMetaData was not received on output/lanelet2_map_metadata";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
