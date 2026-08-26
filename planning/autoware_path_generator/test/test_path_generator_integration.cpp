// Copyright 2025 TIER IV, Inc.
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

#include "autoware/path_generator/node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_test_utils/mock_data_parser.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::path_generator {

class PathGeneratorIntegrationHarness : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    // Load params
    const auto autoware_test_utils_dir = ament_index_cpp::get_package_share_directory("autoware_test_utils");
    const auto path_generator_dir = ament_index_cpp::get_package_share_directory("autoware_path_generator");
    const auto node_options = rclcpp::NodeOptions{}.arguments(
      {"--ros-args", 
       "--params-file", autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml", 
       "--params-file", autoware_test_utils_dir + "/config/test_nearest_search.param.yaml", 
       "--params-file", path_generator_dir + "/config/path_generator.param.yaml"});

    // Init nodes
    node_ = std::make_shared<PathGenerator>(node_options);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    // Setup I/O harness
    harness_node_ = std::make_shared<rclcpp::Node>("path_generator_harness");
    executor_->add_node(harness_node_);

    // Pubs
    pub_map_ = harness_node_->create_publisher<autoware_map_msgs::msg::LaneletMapBin>("~/input/vector_map", 1);
    pub_odom_ = harness_node_->create_publisher<nav_msgs::msg::Odometry>("~/input/odometry", 1);
    pub_route_ = harness_node_->create_publisher<autoware_planning_msgs::msg::LaneletRoute>("~/input/route", 1);

    // Subs
    sub_path_ = harness_node_->create_subscription<autoware_internal_planning_msgs::msg::PathWithLaneId>(
      "~/output/path", 1, [this](const autoware_internal_planning_msgs::msg::PathWithLaneId::ConstSharedPtr msg) {
        latest_path_ = msg;
      });
    sub_turn_ = harness_node_->create_subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>(
      "~/output/turn_indicators_cmd", 1, [this](const autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg) {
        latest_turn_ = msg;
      });
    sub_hazard_ = harness_node_->create_subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>(
      "~/output/hazard_lights_cmd", 1, [this](const autoware_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg) {
        latest_hazard_ = msg;
      });
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  void spin_executor_for(std::chrono::milliseconds duration)
  {
    const auto end_time = std::chrono::steady_clock::now() + duration;

    while (std::chrono::steady_clock::now() < end_time && rclcpp::ok()) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  void load_and_publish_map(const std::string & package_name, const std::string & map_filename)
  {
    const auto map_path = autoware::test_utils::get_absolute_path_to_lanelet_map(package_name, map_filename);
    auto map_msg = autoware::test_utils::make_map_bin_msg(map_path);
    
    pub_map_->publish(map_msg);
  }

  static autoware_planning_msgs::msg::LaneletRoute load_route(const std::string & package_name, const std::string & route_filename)
  {
    const auto route_path = autoware::test_utils::get_absolute_path_to_route(package_name, route_filename);
    auto route_opt = autoware::test_utils::parse<std::optional<autoware_planning_msgs::msg::LaneletRoute>>(route_path);
    
    if (!route_opt) {
      throw std::runtime_error("Failed to parse mock route.");
    }

    return route_opt.value();
  }

  // Harness pointers
  std::shared_ptr<PathGenerator> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<rclcpp::Node> harness_node_;

  // Pubs
  rclcpp::Publisher<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr pub_map_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr pub_route_;

  // Output storages
  autoware_internal_planning_msgs::msg::PathWithLaneId::ConstSharedPtr latest_path_{nullptr};
  autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr latest_turn_{nullptr};
  autoware_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr latest_hazard_{nullptr};

  // Subs
  rclcpp::Subscription<autoware_internal_planning_msgs::msg::PathWithLaneId>::SharedPtr sub_path_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr sub_turn_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr sub_hazard_;
};

} // namespace autoware::path_generator