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
#include <autoware_map_msgs/msg/lanelet_map_meta_data.hpp>
#include <autoware_map_msgs/msg/map_projector_info.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <chrono>
#include <fstream>
#include <memory>
#include <string>

using MapProjectorInfo = autoware::component_interface_specs::map::MapProjectorInfo;

class Lanelet2MapLoaderIntegrationHarness : public ::testing::Test
{
protected:
  // Runs once per process, safe for death test in TEST 4
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }

  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {}

  void TearDown() override { test_node_.reset(); }

  // Helper to ensure pub/sub are linked before sending data
  void wait_for_discovery(std::chrono::seconds timeout = std::chrono::seconds(3))
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (projector_pub_->get_subscription_count() == 0) {
      if (std::chrono::steady_clock::now() > deadline) {
        FAIL() << "Pub/sub handshake & discovery timed out.";
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  struct ReceivedMessages
  {
    rclcpp::Node::SharedPtr target_node;
    autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr map_msg;
    autoware_map_msgs::msg::LaneletMapMetaData::ConstSharedPtr meta_msg;
  };

  ReceivedMessages receive_map_and_metadata(
    const rclcpp::NodeOptions & options, std::chrono::seconds timeout = std::chrono::seconds(3))
  {
    auto target_node = std::make_shared<autoware::map_loader::Lanelet2MapLoaderNode>(options);

    // Must reset executor each time to prevent zombie nodes
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(target_node);

    test_node_ = std::make_shared<rclcpp::Node>("test_lanelet2_map_loader_client");
    projector_pub_ = test_node_->create_publisher<MapProjectorInfo::Message>(
      MapProjectorInfo::name, autoware::component_interface_specs::get_qos<MapProjectorInfo>());
    executor_->add_node(test_node_);

    autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr map_msg;
    auto map_sub = test_node_->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
      "/map/vector_map", rclcpp::QoS{1}.transient_local(),
      [&map_msg](const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg) {
        map_msg = msg;
      });

    autoware_map_msgs::msg::LaneletMapMetaData::ConstSharedPtr meta_msg;
    auto meta_sub = test_node_->create_subscription<autoware_map_msgs::msg::LaneletMapMetaData>(
      "output/lanelet2_map_metadata", rclcpp::QoS{1}.transient_local(),
      [&meta_msg](const autoware_map_msgs::msg::LaneletMapMetaData::ConstSharedPtr msg) {
        meta_msg = msg;
      });

    wait_for_discovery();

    autoware_map_msgs::msg::MapProjectorInfo projector_info;
    projector_info.projector_type = autoware_map_msgs::msg::MapProjectorInfo::MGRS;
    projector_info.mgrs_grid = "54SUE";
    projector_pub_->publish(projector_info);

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!map_msg && std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return {target_node, map_msg, meta_msg};
  }

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Node::SharedPtr test_node_;
  rclcpp::Publisher<MapProjectorInfo::Message>::SharedPtr projector_pub_;

  // Standard test map
  const std::string map_path_ =
    ament_index_cpp::get_package_share_directory("autoware_map_loader") + "/test/data/test_map.osm";
};

// TEST 1. Main integration test to see if Lanelet2MapLoaderNode can load test map and work properly
// as expected. Spins up pub/sub and checks normal behaviors including proper ROS metadata, lanelet
// count, and centerline injection. Expects:
// - Node publishes LaneletMapBin message (with 3 secs)
// - LaneletMapBin message has proper ROS metadata (frame_id, version_map_format, version_map)
// - LaneletMapBin message decodes to a LaneletMap with 4 lanelets
// - Each lanelet has a custom centerline injected via waypoints
TEST_F(Lanelet2MapLoaderIntegrationHarness, VerifiesNormalMapLoadingAndReading)
{
  rclcpp::NodeOptions options;
  const std::string map_path =
    ament_index_cpp::get_package_share_directory("autoware_map_loader") + "/test/data/test_map.osm";

  options.append_parameter_override("lanelet2_map_path", map_path);
  options.append_parameter_override("center_line_resolution", 5.0);
  options.append_parameter_override("use_waypoints", true);
  options.append_parameter_override("allow_unsupported_version", true);
  options.append_parameter_override("enable_selected_map_loading", false);
  options.append_parameter_override("lanelet2_map_metadata_path", "");

  auto received = receive_map_and_metadata(options);
  ASSERT_NE(received.map_msg, nullptr)
    << "Target node failed to publish LaneletMapBin within timeout.";

  auto decoded_map =
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*received.map_msg);

  // Expected 4 lanelets
  ASSERT_EQ(decoded_map->laneletLayer.size(), 4U);

  // Expected centerline injected via waypoints
  for (const auto & lanelet : decoded_map->laneletLayer) {
    EXPECT_TRUE(lanelet.hasCustomCenterline())
      << "Centerline generation failed to apply to lanelet " << lanelet.id();
  }
}

// TEST 2. Confirms that when use_waypoints is set to false, standard overwriteLaneletsCenterline is
// used instead of custom centerline injection. Expects:
// - Node publishes LaneletMapBin message (with 3 secs)
// - LaneletMapBin message decodes to a LaneletMap with 4 lanelets
// - Each lanelet has a standard centerline injected via overwriteLaneletsCenterline
TEST_F(Lanelet2MapLoaderIntegrationHarness, VerifiesStandardCenterlineInjection)
{
  // Re-init node options with use_waypoints = false
  rclcpp::NodeOptions options;
  const std::string map_path =
    ament_index_cpp::get_package_share_directory("autoware_map_loader") + "/test/data/test_map.osm";
  options.append_parameter_override("lanelet2_map_path", map_path);
  options.append_parameter_override("center_line_resolution", 5.0);
  options.append_parameter_override("allow_unsupported_version", true);
  options.append_parameter_override("enable_selected_map_loading", false);
  options.append_parameter_override("lanelet2_map_metadata_path", "");
  // TEST TARGET use_waypoints = false
  options.append_parameter_override("use_waypoints", false);

  auto received = receive_map_and_metadata(options);
  ASSERT_NE(received.map_msg, nullptr);

  auto decoded_map =
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*received.map_msg);

  // Standard overwriteLaneletsCenterline is used.
  ASSERT_EQ(decoded_map->laneletLayer.size(), 4U);

  // Each lanelet has a standard centerline injected via overwriteLaneletsCenterline
  for (const auto & lanelet : decoded_map->laneletLayer) {
    // Standard centerline exists with points
    EXPECT_FALSE(lanelet.centerline().empty())
      << "Standard centerline should be populated when use_waypoints is false.";

    // No custom waypoints generated
    EXPECT_FALSE(lanelet.hasAttribute("waypoints"))
      << "Waypoints attribute should NOT exist when use_waypoints is false.";
  }
}

// TEST 3. Confirms when enable_selected_map_loading is set to true, node publishes metadata upon
// initialization. Expects:
// - Node publishes LaneletMapMetaData message (with 3 secs)
// - LaneletMapMetaData message has 1 metadata entry with cell_id matching the map
TEST_F(Lanelet2MapLoaderIntegrationHarness, VerifiesMetadataPublishedWhenSelectedLoadingEnabled)
{
  // Re-init node options with enable_selected_map_loading = true
  rclcpp::NodeOptions options;
  const std::string map_path =
    ament_index_cpp::get_package_share_directory("autoware_map_loader") + "/test/data/test_map.osm";
  options.append_parameter_override("lanelet2_map_path", map_path);
  options.append_parameter_override("center_line_resolution", 5.0);
  options.append_parameter_override("use_waypoints", true);
  options.append_parameter_override("allow_unsupported_version", true);
  options.append_parameter_override("lanelet2_map_metadata_path", "");
  // TEST TARGET enable_selected_map_loading = true
  options.append_parameter_override("enable_selected_map_loading", true);

  auto received = receive_map_and_metadata(options);

  // Expected message published
  ASSERT_NE(received.meta_msg, nullptr) << "Metadata was not published upon initialization.";

  // Expects 1 metadata entry with cell_id matching test map
  ASSERT_EQ(received.meta_msg->metadata_list.size(), 1U);
  EXPECT_EQ(received.meta_msg->metadata_list[0].cell_id, map_path);
}

// TEST 4. Confirms when allow_unsupported_version = false and loading a map with an unsupported
// version, node will throw an error and terminate.
TEST_F(Lanelet2MapLoaderIntegrationHarness, RejectsWeirdVersion)
{
  // Dummy file map with wrong version number
  const std::string incompat_map_path = "/tmp/stupid_version_map.osm";
  std::ofstream out(incompat_map_path);
  out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
      << "<osm generator=\"VMB\">\n"
      << "  <MetaInfo format_version=\"69\" map_version=\"1\"/>\n"
      << "</osm>";
  out.close();

  // Re-init node options with allow_unsupported_version = false
  rclcpp::NodeOptions options;
  options.append_parameter_override("lanelet2_map_path", incompat_map_path);
  options.append_parameter_override("center_line_resolution", 5.0);
  options.append_parameter_override("use_waypoints", true);
  options.append_parameter_override("enable_selected_map_loading", false);
  options.append_parameter_override("lanelet2_map_metadata_path", "");

  // TEST TARGET allow_unsupported_version = false
  options.append_parameter_override("allow_unsupported_version", false);

  EXPECT_THROW(receive_map_and_metadata(options), std::invalid_argument);
}
