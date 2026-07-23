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

#ifndef TEST_CASES__STANDARD_SEQUENCE_FOR_INITIAL_POSE_ESTIMATION_HPP_
#define TEST_CASES__STANDARD_SEQUENCE_FOR_INITIAL_POSE_ESTIMATION_HPP_

#include "../test_fixture.hpp"

#ifdef NDT_USE_RUST
#include "autoware_ndt_scan_matcher_rs.h"
#endif

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <rcl_yaml_param_parser/parser.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

TEST_F(TestNDTScanMatcher, standard_sequence_for_initial_pose_estimation)  // NOLINT
{
  //---------//
  // Arrange //
  //---------//
  std::thread t1([&]() {
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node_);
    exec.spin();
  });
  std::thread t2([&]() { rclcpp::spin(pcd_loader_); });
#ifdef NDT_USE_RUST
  std::array<AwNdtAlignServiceTraceEvent, 4> trace_events{};
  AwNdtAlignServiceTrace trace{};
  trace.events = trace_events.data();
  trace.capacity = trace_events.size();
  trace.len = 0U;
  trace.overflowed = 0U;
  node_->set_align_service_trace(&trace);
#endif

  //-----//
  // Act //
  //-----//
  // (1) trigger initial pose estimation
  EXPECT_TRUE(trigger_node_client_->send_trigger_node(true));

  // (2) publish LiDAR point cloud
  const sensor_msgs::msg::PointCloud2 input_cloud = make_default_sensor_pcd();
  RCLCPP_INFO_STREAM(node_->get_logger(), "sensor cloud size: " << input_cloud.width);
  sensor_pcd_publisher_->publish_pcd(input_cloud);

  // (3) send initial pose
  const geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_msg =
    make_pose(/* x = */ 100.0, /* y = */ 100.0);
  const geometry_msgs::msg::Pose result_pose =
    initialpose_client_->send_initialpose(initial_pose_msg).pose.pose;

  //--------//
  // Assert //
  //--------//
  RCLCPP_INFO_STREAM(
    node_->get_logger(), std::fixed << "result_pose: " << result_pose.position.x << ", "
                                    << result_pose.position.y << ", " << result_pose.position.z);
  EXPECT_NEAR(result_pose.position.x, 100.0, 2.0);
  EXPECT_NEAR(result_pose.position.y, 100.0, 2.0);
  EXPECT_NEAR(result_pose.position.z, 0.0, 2.0);

#ifdef NDT_USE_RUST
  ASSERT_EQ(trace.len, 4U);
  EXPECT_EQ(trace.overflowed, 0U);
  EXPECT_EQ(trace_events[0].kind, NDT_ALIGN_TRACE_EVENT_DECISION);
  EXPECT_EQ(trace_events[0].status, NDT_ALIGN_SERVICE_STATUS_READY_TO_ALIGN);
  EXPECT_EQ(trace_events[0].should_align, 1U);
  EXPECT_EQ(trace_events[1].kind, NDT_ALIGN_TRACE_EVENT_SEARCH_SUMMARY);
  EXPECT_EQ(trace_events[1].status, NDT_ALIGN_SERVICE_STATUS_ALIGNED);
  EXPECT_EQ(trace_events[1].particles_requested, 100);
  EXPECT_EQ(trace_events[1].particles_evaluated, 100);
  EXPECT_EQ(trace_events[1].marker_publish_count, 20);
  EXPECT_EQ(trace_events[1].cloud_publish_count, 100);
  EXPECT_GE(trace_events[1].best_iteration, 0);
  EXPECT_TRUE(std::isfinite(trace_events[1].best_score));
  EXPECT_EQ(trace_events[2].kind, NDT_ALIGN_TRACE_EVENT_DECISION);
  EXPECT_EQ(trace_events[2].status, NDT_ALIGN_SERVICE_STATUS_ALIGNED);
  EXPECT_EQ(trace_events[2].success, 1U);
  EXPECT_EQ(trace_events[3].kind, NDT_ALIGN_TRACE_EVENT_RESPONSE);
  EXPECT_EQ(trace_events[3].status, NDT_ALIGN_SERVICE_STATUS_ALIGNED);
  EXPECT_EQ(trace_events[3].success, 1U);
  EXPECT_EQ(trace_events[3].response_stamp_ns, 0);
  EXPECT_EQ(trace_events[3].response_position[0], result_pose.position.x);
  EXPECT_EQ(trace_events[3].response_position[1], result_pose.position.y);
  EXPECT_EQ(trace_events[3].response_position[2], result_pose.position.z);
  EXPECT_EQ(trace_events[3].response_orientation[0], result_pose.orientation.x);
  EXPECT_EQ(trace_events[3].response_orientation[1], result_pose.orientation.y);
  EXPECT_EQ(trace_events[3].response_orientation[2], result_pose.orientation.z);
  EXPECT_EQ(trace_events[3].response_orientation[3], result_pose.orientation.w);
  EXPECT_EQ(trace_events[3].response_covariance[0], initial_pose_msg.pose.covariance[0]);
  EXPECT_EQ(trace_events[3].response_covariance[7], initial_pose_msg.pose.covariance[7]);
  EXPECT_EQ(trace_events[3].response_covariance[14], initial_pose_msg.pose.covariance[14]);
  EXPECT_EQ(trace_events[3].response_covariance[21], initial_pose_msg.pose.covariance[21]);
  EXPECT_EQ(trace_events[3].response_covariance[28], initial_pose_msg.pose.covariance[28]);
  EXPECT_EQ(trace_events[3].response_covariance[35], initial_pose_msg.pose.covariance[35]);
  node_->set_align_service_trace(nullptr);
#endif

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

#endif  // TEST_CASES__STANDARD_SEQUENCE_FOR_INITIAL_POSE_ESTIMATION_HPP_
