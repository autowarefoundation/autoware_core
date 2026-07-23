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

#ifndef TEST_CASES__ALIGN_SERVICE_TPE_BASELINE_HPP_
#define TEST_CASES__ALIGN_SERVICE_TPE_BASELINE_HPP_

#include "../test_fixture.hpp"
#include "../test_util.hpp"

#ifdef NDT_USE_RUST
#include "autoware_ndt_scan_matcher_rs.h"
#endif

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#ifdef NDT_USE_RUST
#include <limits>
#endif
#include <memory>
#include <thread>
#include <vector>

namespace
{
constexpr std::size_t kBaselineRunCount = 5U;
constexpr double kExpectedX = 100.0;
constexpr double kExpectedY = 100.0;
constexpr double kExpectedZ = 0.0;
constexpr double kPoseTolerance = 2.0;
constexpr double kPositionSpreadTolerance = 0.25;
#ifdef NDT_USE_RUST
constexpr double kScoreSpreadTolerance = 0.05;
#endif

struct BaselineSample
{
  geometry_msgs::msg::Pose pose{};
#ifdef NDT_USE_RUST
  double best_score{std::numeric_limits<double>::quiet_NaN()};
#endif
};

double spread(const std::vector<double> & values)
{
  const auto [min_it, max_it] = std::minmax_element(values.begin(), values.end());
  return *max_it - *min_it;
}

#ifdef NDT_USE_RUST
AwNdtAlignServiceTrace make_trace(std::array<AwNdtAlignServiceTraceEvent, 4> & events)
{
  AwNdtAlignServiceTrace trace{};
  trace.events = events.data();
  trace.capacity = events.size();
  trace.len = 0U;
  trace.overflowed = 0U;
  return trace;
}

void expect_align_trace(
  const std::array<AwNdtAlignServiceTraceEvent, 4> & events, const AwNdtAlignServiceTrace & trace,
  const geometry_msgs::msg::Pose & result_pose)
{
  ASSERT_EQ(trace.len, 4U);
  EXPECT_EQ(trace.overflowed, 0U);
  EXPECT_EQ(events[0].kind, NDT_ALIGN_TRACE_EVENT_DECISION);
  EXPECT_EQ(events[0].status, NDT_ALIGN_SERVICE_STATUS_READY_TO_ALIGN);
  EXPECT_EQ(events[0].should_align, 1U);
  EXPECT_EQ(events[1].kind, NDT_ALIGN_TRACE_EVENT_SEARCH_SUMMARY);
  EXPECT_EQ(events[1].status, NDT_ALIGN_SERVICE_STATUS_ALIGNED);
  EXPECT_EQ(events[1].particles_requested, 100);
  EXPECT_EQ(events[1].particles_evaluated, 100);
  EXPECT_EQ(events[1].marker_publish_count, 20);
  EXPECT_EQ(events[1].cloud_publish_count, 100);
  EXPECT_GE(events[1].best_iteration, 0);
  EXPECT_TRUE(std::isfinite(events[1].best_score));
  EXPECT_EQ(events[2].kind, NDT_ALIGN_TRACE_EVENT_DECISION);
  EXPECT_EQ(events[2].status, NDT_ALIGN_SERVICE_STATUS_ALIGNED);
  EXPECT_EQ(events[2].success, 1U);
  EXPECT_EQ(events[3].kind, NDT_ALIGN_TRACE_EVENT_RESPONSE);
  EXPECT_EQ(events[3].status, NDT_ALIGN_SERVICE_STATUS_ALIGNED);
  EXPECT_EQ(events[3].success, 1U);
  EXPECT_EQ(events[3].response_position[0], result_pose.position.x);
  EXPECT_EQ(events[3].response_position[1], result_pose.position.y);
  EXPECT_EQ(events[3].response_position[2], result_pose.position.z);
}
#endif
}  // namespace

TEST_F(TestNDTScanMatcher, align_service_tpe_baseline_repeated_runs)  // NOLINT
{
  std::thread t1([&]() {
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node_);
    exec.spin();
  });
  std::thread t2([&]() { rclcpp::spin(pcd_loader_); });

  std::vector<BaselineSample> samples;
  samples.reserve(kBaselineRunCount);

  for (std::size_t i = 0; i < kBaselineRunCount; ++i) {
#ifdef NDT_USE_RUST
    std::array<AwNdtAlignServiceTraceEvent, 4> trace_events{};
    AwNdtAlignServiceTrace trace = make_trace(trace_events);
    node_->set_align_service_trace(&trace);
#endif

    EXPECT_TRUE(trigger_node_client_->send_trigger_node(true));

    const sensor_msgs::msg::PointCloud2 input_cloud = make_default_sensor_pcd();
    sensor_pcd_publisher_->publish_pcd(input_cloud);

    const geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_msg =
      make_pose(/* x = */ 100.0, /* y = */ 100.0);
    const geometry_msgs::msg::Pose result_pose =
      initialpose_client_->send_initialpose(initial_pose_msg).pose.pose;

    EXPECT_NEAR(result_pose.position.x, kExpectedX, kPoseTolerance);
    EXPECT_NEAR(result_pose.position.y, kExpectedY, kPoseTolerance);
    EXPECT_NEAR(result_pose.position.z, kExpectedZ, kPoseTolerance);

    BaselineSample sample{};
    sample.pose = result_pose;
#ifdef NDT_USE_RUST
    expect_align_trace(trace_events, trace, result_pose);
    sample.best_score = trace_events[1].best_score;
    node_->set_align_service_trace(nullptr);
#endif
    samples.push_back(sample);

#ifdef NDT_USE_RUST
    RCLCPP_INFO_STREAM(
      node_->get_logger(), "align-service baseline run "
                             << i << ": pose=(" << result_pose.position.x << ", "
                             << result_pose.position.y << ", " << result_pose.position.z << ")"
                             << ", best_score=" << sample.best_score);
#else
    RCLCPP_INFO_STREAM(
      node_->get_logger(), "align-service baseline run "
                             << i << ": pose=(" << result_pose.position.x << ", "
                             << result_pose.position.y << ", " << result_pose.position.z << ")");
#endif
  }

  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> zs;
#ifdef NDT_USE_RUST
  std::vector<double> scores;
#endif
  xs.reserve(samples.size());
  ys.reserve(samples.size());
  zs.reserve(samples.size());
#ifdef NDT_USE_RUST
  scores.reserve(samples.size());
#endif
  for (const auto & sample : samples) {
    xs.push_back(sample.pose.position.x);
    ys.push_back(sample.pose.position.y);
    zs.push_back(sample.pose.position.z);
#ifdef NDT_USE_RUST
    scores.push_back(sample.best_score);
#endif
  }

  const double x_spread = spread(xs);
  const double y_spread = spread(ys);
  const double z_spread = spread(zs);
  RCLCPP_INFO_STREAM(
    node_->get_logger(), "align-service baseline position spread: x="
                           << x_spread << ", y=" << y_spread << ", z=" << z_spread);
  EXPECT_LE(x_spread, kPositionSpreadTolerance);
  EXPECT_LE(y_spread, kPositionSpreadTolerance);
  EXPECT_LE(z_spread, kPositionSpreadTolerance);

#ifdef NDT_USE_RUST
  const double score_spread = spread(scores);
  RCLCPP_INFO_STREAM(node_->get_logger(), "align-service baseline score spread: " << score_spread);
  EXPECT_LE(score_spread, kScoreSpreadTolerance);
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

#endif  // TEST_CASES__ALIGN_SERVICE_TPE_BASELINE_HPP_
