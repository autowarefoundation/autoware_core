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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/map_loader/lanelet2_map_loader_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_map_msgs/msg/map_projector_info.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <chrono>
#include <memory>
#include <string>

class CharacterizationTestLanelet2MapLoaderNode : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    test_node_ = std::make_shared<rclcpp::Node>("test_lanelet2_map_loader_client");

    // Params setting
    rclcpp::NodeOptions options;
    const std::string map_path =
      ament_index_cpp::get_package_share_directory("autoware_map_loader") +
      "/test/data/test_map.osm";

    options.append_parameter_override("lanelet2_map_path", map_path);
    options.append_parameter_override("center_line_resolution", 5.0);
    options.append_parameter_override("use_waypoints", true);
    options.append_parameter_override("allow_unsupported_version", true);
    options.append_parameter_override("enable_selected_map_loading", false);
    options.append_parameter_override("lanelet2_map_metadata_path", "");

    // Init node
    target_node_ = std::make_shared<autoware::map_loader::Lanelet2MapLoaderNode>(options);

    // Pub/sub
    projector_pub_ = test_node_->create_publisher<autoware_map_msgs::msg::MapProjectorInfo>(
      "input/map_projector_info", rclcpp::QoS{1}.transient_local());
  }

  void TearDown() override { rclcpp::shutdown(); }

  rclcpp::Node::SharedPtr test_node_;
  std::shared_ptr<autoware::map_loader::Lanelet2MapLoaderNode> target_node_;
  rclcpp::Publisher<autoware_map_msgs::msg::MapProjectorInfo>::SharedPtr projector_pub_;
};

// TEST 1. Main integration test to see if Lanelet2MapLoaderNode can load test map and work properly
// as expected. Spins up pub/sub and checks normal behaviors including proper ROS metadata, lanelet
// count, and centerline injection. Expects:
// - Node publishes LaneletMapBin message (with 3 secs)
// - LaneletMapBin message has proper ROS metadata (frame_id, version_map_format, version_map)
// - LaneletMapBin message decodes to a LaneletMap with 4 lanelets
// - Each lanelet has a custom centerline injected via waypoints
TEST_F(CharacterizationTestLanelet2MapLoaderNode, VerifiesEndToEndMapLoadingAndCenterlineInjection)
{
  // Prepare to catch map
  std::promise<autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr> map_promise;
  auto map_future = map_promise.get_future();

  auto map_sub = test_node_->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "/map/vector_map", rclcpp::QoS{1}.transient_local(),
    [&map_promise](const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg) {
      map_promise.set_value(msg);
    });

  autoware_map_msgs::msg::MapProjectorInfo projector_info;
  projector_info.projector_type = autoware_map_msgs::msg::MapProjectorInfo::MGRS;
  projector_info.mgrs_grid = "54SUE";
  projector_pub_->publish(projector_info);

  // Spin both nodes to process callback
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(test_node_);
  executor.add_node(target_node_);

  auto status = executor.spin_until_future_complete(map_future, std::chrono::seconds(3));

  // Expected message published
  ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS)
    << "Target node failed to publish LaneletMapBin within timeout.";
  auto map_bin_msg = map_future.get();
  ASSERT_NE(map_bin_msg, nullptr);

  // Expected proper ROS metadata
  EXPECT_EQ(map_bin_msg->header.frame_id, "map");
  EXPECT_EQ(map_bin_msg->version_map_format, "1");
  EXPECT_EQ(map_bin_msg->version_map, "2");

  auto decoded_map = autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*map_bin_msg);

  // Expected 4 lanelets
  ASSERT_EQ(decoded_map->laneletLayer.size(), 4U);

  // Expected centerline injected via waypoints
  for (const auto & lanelet : decoded_map->laneletLayer) {
    EXPECT_TRUE(lanelet.hasCustomCenterline())
      << "Centerline generation failed to apply to lanelet " << lanelet.id();
  }
}
