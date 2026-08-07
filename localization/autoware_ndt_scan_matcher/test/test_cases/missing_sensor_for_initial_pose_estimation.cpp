// Copyright 2026 Autoware Foundation
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

#ifndef TEST_CASES__MISSING_SENSOR_FOR_INITIAL_POSE_ESTIMATION_HPP_
#define TEST_CASES__MISSING_SENSOR_FOR_INITIAL_POSE_ESTIMATION_HPP_

#include "../test_fixture.hpp"

#ifdef NDT_USE_RUST
#include "autoware_ndt_scan_matcher_rs.h"
#endif

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <array>
#include <memory>
#include <thread>

TEST_F(TestNDTScanMatcher, missing_sensor_for_initial_pose_estimation)  // NOLINT
{
  std::thread t1([&]() {
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node_);
    exec.spin();
  });
  std::thread t2([&]() { rclcpp::spin(pcd_loader_); });

#ifdef NDT_USE_RUST
  std::array<AwNdtAlignServiceTraceEvent, 1> trace_events{};
  AwNdtAlignServiceTrace trace{};
  trace.events = trace_events.data();
  trace.capacity = trace_events.size();
  trace.len = 0U;
  trace.overflowed = 0U;
  node_->set_align_service_trace(&trace);
#endif

  EXPECT_TRUE(trigger_node_client_->send_trigger_node(true));

  const geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_msg =
    make_pose(/* x = */ 100.0, /* y = */ 100.0);
  static_cast<void>(initialpose_client_->send_initialpose(initial_pose_msg));

#ifdef NDT_USE_RUST
  ASSERT_EQ(trace.len, 1U);
  EXPECT_EQ(trace.overflowed, 0U);
  EXPECT_EQ(trace_events[0].kind, NDT_ALIGN_TRACE_EVENT_DECISION);
  EXPECT_EQ(trace_events[0].status, NDT_ALIGN_SERVICE_STATUS_SENSOR_UNAVAILABLE);
  EXPECT_EQ(trace_events[0].success, 0U);
  EXPECT_EQ(trace_events[0].should_align, 0U);
  EXPECT_EQ(trace_events[0].diagnostic_level, NDT_ALIGN_SERVICE_DIAGNOSTIC_WARN);
  EXPECT_EQ(trace_events[0].message_kind, NDT_ALIGN_SERVICE_MESSAGE_SENSOR_UNAVAILABLE);
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

#endif  // TEST_CASES__MISSING_SENSOR_FOR_INITIAL_POSE_ESTIMATION_HPP_
