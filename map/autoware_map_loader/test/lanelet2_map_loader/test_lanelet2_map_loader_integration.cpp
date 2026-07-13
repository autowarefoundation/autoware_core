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
#include <autoware/map_loader/lanelet2_map_loader_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_map_msgs/msg/map_projector_info.hpp>

#include <gtest/gtest.h>

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
