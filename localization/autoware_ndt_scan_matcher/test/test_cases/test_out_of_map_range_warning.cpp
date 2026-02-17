// Copyright 2023 Autoware Foundation
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

#include "../test_fixture.hpp"
#include "../test_util.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/localization_util/smart_pose_buffer.hpp>
#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <gtest/gtest.h>
#include <rcl_yaml_param_parser/parser.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

TEST_F(TestNDTScanMatcher, test_out_of_map_range_warning)  // NOLINT
{
  //---------//
  // Arrange //
  //---------//
  // Set up subscriber to verify diagnostic messages
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> received_diagnostics;
  auto test_node = rclcpp::Node::make_shared("test_diagnostics_subscriber");
  auto diagnostics_sub = test_node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10,
    [&received_diagnostics](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
      for (const auto & status : msg->status) {
        if (status.name.find("scan_matching_status") != std::string::npos) {
          received_diagnostics.push_back(status);
        }
      }
    });

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node_);
  exec.add_node(test_node);
  exec.add_node(pcd_loader_);
  std::thread t1([&]() { exec.spin(); });

  // Wait for publishers and subscribers to connect
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Wait for map to be loaded
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // Activate node (set is_activated_ to true)
  EXPECT_TRUE(trigger_node_client_->send_trigger_node(true));

  // Set initial position to trigger map update
  // Publish message to ekf_pose_with_covariance topic to set latest_ekf_position_
  // Set position to (100.0, 100.0) which is the center of the map
  auto ekf_pose_pub = test_node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ekf_pose_with_covariance", 10);
  geometry_msgs::msg::PoseWithCovarianceStamped ekf_pose =
    make_pose(/* x = */ 100.0, /* y = */ 100.0);
  ekf_pose.header.stamp = node_->now();
  ekf_pose.header.frame_id = "map";
  ekf_pose_pub->publish(ekf_pose);

  // Wait for map update timer to execute (timer runs every 1 second)
  // Wait until map update is executed and last_update_position_ is set
  std::this_thread::sleep_for(std::chrono::milliseconds(2500));

  //-----//
  // Act //
  //-----//
  // Create InterpolateResult with position outside map range
  autoware::localization_util::SmartPoseBuffer::InterpolateResult interpolation_result;

  // Create interpolated_pose with position outside map range
  geometry_msgs::msg::PoseWithCovarianceStamped interpolated_pose =
    make_pose(/* x = */ -100.0, /* y = */ -100.0);
  interpolated_pose.header.stamp = node_->now();
  interpolated_pose.header.frame_id = "map";
  interpolation_result.interpolated_pose = interpolated_pose;

  // Set old_pose and new_pose as well (not used in function but required by struct)
  interpolation_result.old_pose = interpolated_pose;
  interpolation_result.new_pose = interpolated_pose;

  // Clear any previous diagnostics
  received_diagnostics.clear();

  // Call check_out_of_map_range_warning function directly
  // Verify that warning message is output via diagnostics and RCLCPP_WARN_STREAM_THROTTLE
  check_out_of_map_range_warning(interpolation_result);

  // Publish diagnostics explicitly (normally done in callback_sensor_points)
  publish_diagnostics(node_->now());

  // Wait for diagnostics to be received by subscriber
  // Poll with timeout to wait for diagnostics message
  const auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (received_diagnostics.empty() && std::chrono::steady_clock::now() < timeout) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  //--------//
  // Assert //
  //--------//
  // Verify that diagnostics warning message was output
  EXPECT_FALSE(received_diagnostics.empty())
    << "Diagnostics message should be published when lidar is out of map range.";
  
  if (!received_diagnostics.empty()) {
    const auto & latest_diag = received_diagnostics.back();
    EXPECT_EQ(latest_diag.level, diagnostic_msgs::msg::DiagnosticStatus::WARN)
      << "Diagnostics level should be WARN when lidar is out of map range.";
    EXPECT_TRUE(latest_diag.message.find("out of the map range") != std::string::npos)
      << "Diagnostics message should contain 'out of the map range'.";
  }

  // Stop executor to stop threads
  exec.cancel();
  t1.join();
}

TEST_F(
  TestNDTScanMatcher, test_out_of_map_range_warning_when_last_update_position_is_null)  // NOLINT
{
  //---------//
  // Arrange //
  //---------//
  // Set up subscriber to verify diagnostic messages
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> received_diagnostics;
  auto test_node = rclcpp::Node::make_shared("test_diagnostics_subscriber");
  auto diagnostics_sub = test_node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10,
    [&received_diagnostics](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
      for (const auto & status : msg->status) {
        if (status.name.find("scan_matching_status") != std::string::npos) {
          received_diagnostics.push_back(status);
        }
      }
    });

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node_);
  exec.add_node(test_node);
  exec.add_node(pcd_loader_);
  std::thread t1([&]() { exec.spin(); });

  // Wait for publishers and subscribers to connect
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Run test before map is loaded (last_update_position_ is null state)
  // Do not wait for map loading

  //-----//
  // Act //
  //-----//
  // Create InterpolateResult with arbitrary position
  autoware::localization_util::SmartPoseBuffer::InterpolateResult interpolation_result;

  // Create interpolated_pose with arbitrary position
  geometry_msgs::msg::PoseWithCovarianceStamped interpolated_pose =
    make_pose(/* x = */ 0.0, /* y = */ 0.0);
  interpolated_pose.header.stamp = node_->now();
  interpolated_pose.header.frame_id = "map";
  interpolation_result.interpolated_pose = interpolated_pose;

  // Set old_pose and new_pose as well (not used in function but required by struct)
  interpolation_result.old_pose = interpolated_pose;
  interpolation_result.new_pose = interpolated_pose;

  // Clear any previous diagnostics
  received_diagnostics.clear();

  // Call check_out_of_map_range_warning function directly
  // Verify that warning message is output via both RCLCPP_WARN_STREAM_THROTTLE and diagnostics
  check_out_of_map_range_warning(interpolation_result);

  // Publish diagnostics explicitly (normally done in callback_sensor_points)
  publish_diagnostics(node_->now());

  // Wait for diagnostics to be received by subscriber
  // Poll with timeout to wait for diagnostics message
  const auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (received_diagnostics.empty() && std::chrono::steady_clock::now() < timeout) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  //--------//
  // Assert //
  //--------//
  // Verify that diagnostics warning message was output when last_update_position_ is null
  EXPECT_FALSE(received_diagnostics.empty())
    << "Diagnostics message should be published when last_update_position_ is null.";
  
  if (!received_diagnostics.empty()) {
    const auto & latest_diag = received_diagnostics.back();
    EXPECT_EQ(latest_diag.level, diagnostic_msgs::msg::DiagnosticStatus::WARN)
      << "Diagnostics level should be WARN when last_update_position_ is null.";
    EXPECT_TRUE(latest_diag.message.find("Map information is not available") != std::string::npos)
      << "Diagnostics message should contain 'Map information is not available'.";
  }

  // Stop executor to stop threads
  exec.cancel();
  t1.join();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
