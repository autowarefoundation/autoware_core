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

#include "../src/lanelet2_map_visualization_node.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <memory>
#include <string>
#include <vector>

using autoware::lanelet2_map_visualizer::Lanelet2MapVisualizationNode;

class TestLanelet2MapVisualizationNode : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize ROS node
    rclcpp::init(0, nullptr);

    // Create the node
    node_ = std::make_shared<rclcpp::Node>("test_lanelet2_map_visualization_node");

    // Create the visualization node
    visualization_node_ = std::make_shared<Lanelet2MapVisualizationNode>(rclcpp::NodeOptions());

    // Create publisher for map bin messages
    map_bin_pub_ = node_->create_publisher<autoware_map_msgs::msg::LaneletMapBin>(
      "input/lanelet2_map", rclcpp::QoS{1}.transient_local());

    // Create subscriber for marker array messages
    marker_sub_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
      "output/lanelet2_map_marker", rclcpp::QoS{1}.transient_local(),
      [this](const visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
        received_markers_ = msg;
        marker_received_ = true;
      });
  }

  void TearDown() override
  {
    visualization_node_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  // Helper function to create a simple lanelet map
  lanelet::LaneletMapPtr createSimpleLaneletMap()
  {
    lanelet::LaneletMapPtr map = std::make_shared<lanelet::LaneletMap>();

    // Create points for a simple rectangular lanelet
    lanelet::Point3d p1{1, 0, 0, 0};
    lanelet::Point3d p2{2, 10, 0, 0};
    lanelet::Point3d p3{3, 0, 5, 0};
    lanelet::Point3d p4{4, 10, 5, 0};

    // Create linestrings for left and right bounds
    lanelet::LineString3d left_ls(5, {p1, p2});
    lanelet::LineString3d right_ls(6, {p3, p4});

    // Create a lanelet
    lanelet::Lanelet ll(7, left_ls, right_ls);
    ll.attributes()["subtype"] = "road";

    // Add the lanelet to the map
    map->add(ll);

    return map;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Lanelet2MapVisualizationNode> visualization_node_;
  rclcpp::Publisher<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr map_bin_pub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
  visualization_msgs::msg::MarkerArray::ConstSharedPtr received_markers_;
  bool marker_received_ = false;
};

TEST_F(TestLanelet2MapVisualizationNode, VisualizeLaneletMap)
{
  // Create a simple lanelet map
  auto lanelet_map = createSimpleLaneletMap();

  // Convert lanelet map to binary message
  autoware_map_msgs::msg::LaneletMapBin map_bin_msg;
  lanelet::utils::conversion::toBinMsg(lanelet_map, &map_bin_msg);

  // Publish the map
  map_bin_pub_->publish(map_bin_msg);

  // Wait for marker array to be received
  auto start_time = node_->now();
  while (!marker_received_ && (node_->now() - start_time).seconds() < 5.0) {
    rclcpp::spin_some(node_);
    rclcpp::spin_some(visualization_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Check that markers were received
  ASSERT_TRUE(marker_received_);
  ASSERT_GT(received_markers_->markers.size(), 0);

  // Check that at least one marker has the correct frame_id
  bool found_marker_with_map_frame = false;
  for (const auto & marker : received_markers_->markers) {
    if (marker.header.frame_id == "map") {
      found_marker_with_map_frame = true;
      break;
    }
  }
  EXPECT_TRUE(found_marker_with_map_frame);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
