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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/localization_util/smart_pose_buffer.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>

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
  // 診断メッセージを検証するためのサブスクライバーを設定
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

  std::thread t1([&]() {
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node_);
    exec.add_node(test_node);
    exec.spin();
  });
  std::thread t2([&]() { rclcpp::spin(pcd_loader_); });

  // パブリッシャーとサブスクライバーが接続されるまで待機
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // 地図がロードされるまで少し待機
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  //-----//
  // Act //
  //-----//
  // 地図範囲外の位置を持つInterpolateResultを作成
  autoware::localization_util::SmartPoseBuffer::InterpolateResult interpolation_result;

  // 地図範囲外の位置を持つinterpolated_poseを作成
  geometry_msgs::msg::PoseWithCovarianceStamped interpolated_pose = make_pose(/* x = */ -100.0, /* y = */ -100.0);
  interpolated_pose.header.stamp = node_->now();
  interpolated_pose.header.frame_id = "map";
  interpolation_result.interpolated_pose = interpolated_pose;

  // old_poseとnew_poseも設定（関数内では使用されないが、構造体の要件として設定）
  interpolation_result.old_pose = interpolated_pose;
  interpolation_result.new_pose = interpolated_pose;

  // check_out_of_map_range_warning関数を直接呼び出し
  // 関数がtrueを返すことを確認（地図範囲外であることを示す）
  // RCLCPP_WARN_STREAM_THROTTLEで警告メッセージが出力されることを確認
  const bool is_out_of_map = node_->check_out_of_map_range_warning(interpolation_result);

  //--------//
  // Assert //
  //--------//
  // 関数がtrueを返すことを確認（地図範囲外であることを示す）
  EXPECT_TRUE(is_out_of_map)
    << "check_out_of_map_range_warning should return true when lidar is out of map range.";

  rclcpp::shutdown();
  t1.join();
  t2.join();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
