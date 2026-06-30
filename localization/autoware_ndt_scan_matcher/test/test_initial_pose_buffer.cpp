// Copyright 2024 Autoware Foundation
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

// Differential test (Phase 1 slice B): the Rust-owned initial-pose buffer (pushed through
// on_initial_pose after activation, queried through initial_pose_interpolate) must reproduce the C++
// SmartPoseBuffer's interpolation bit-close over many random sequences — with the real (finite)
// initial-pose tolerances. Also pins activation gating: a pose pushed while deactivated is dropped.

#include "autoware/ndt_scan_matcher/ndt_scan_matcher_rs.hpp"

#include <autoware/localization_util/smart_pose_buffer.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace autoware::ndt_scan_matcher
{
namespace
{
constexpr double kTimeoutSec = 10.0;
constexpr double kDistanceTolM = 10.0;

extern "C" void nd_noop_clear(void *) {}
extern "C" void nd_noop_bool(void *, const std::uint8_t *, std::size_t, bool) {}
extern "C" void nd_noop_i64(void *, const std::uint8_t *, std::size_t, std::int64_t) {}
extern "C" void nd_noop_f64(void *, const std::uint8_t *, std::size_t, double) {}
extern "C" void nd_noop_str(
  void *, const std::uint8_t *, std::size_t, const std::uint8_t *, std::size_t)
{
}
extern "C" void nd_noop_level(void *, std::int8_t, const std::uint8_t *, std::size_t) {}
extern "C" void nd_noop_publish(void *, std::int64_t) {}
AwDiagnostics noop_diag()
{
  return AwDiagnostics{
    nullptr,     nd_noop_clear, nd_noop_bool,    nd_noop_i64,
    nd_noop_f64, nd_noop_str,   nd_noop_level, nd_noop_publish};
}

AwNdtScanMatcher * make_handle(const std::string & map_frame)
{
  AwNdtParams p{};
  p.resolution = 1.0;
  p.min_points = 6;
  p.eig_mult = 0.01;
  p.max_iterations = 1;
  p.outlier_ratio = 0.55;
  p.num_threads = 1;
  p.covariance_scale_factor = 1.0;
  p.covariance_temperature = 1.0;
  p.regularization_enable = false;
  p.regularization_pose_timeout_sec = 1000.0;
  p.regularization_pose_distance_tolerance_m = 1000.0;
  p.map_frame = reinterpret_cast<const std::uint8_t *>(map_frame.data());
  p.map_frame_len = map_frame.size();
  p.initial_pose_timeout_sec = kTimeoutSec;
  p.initial_pose_distance_tolerance_m = kDistanceTolM;
  return autoware_ndt_scan_matcher_rs_new(&p);
}

geometry_msgs::msg::PoseWithCovarianceStamped make_msg(
  std::int64_t stamp_ns, double x, double y, double z, double roll, double pitch, double yaw)
{
  geometry_msgs::msg::PoseWithCovarianceStamped m;
  m.header.stamp = rclcpp::Time(stamp_ns);
  m.header.frame_id = "map";
  m.pose.pose.position.x = x;
  m.pose.pose.position.y = y;
  m.pose.pose.position.z = z;
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  m.pose.pose.orientation = tf2::toMsg(q);
  return m;
}
}  // namespace

TEST(InitialPoseBuffer, MatchesCppOnRandomSequences)  // NOLINT
{
  std::uint64_t seed = 2'463'534'242ULL;  // xorshift, deterministic
  auto rnd = [&seed]() {
    seed ^= seed << 13;
    seed ^= seed >> 7;
    seed ^= seed << 17;
    return static_cast<double>(seed % 1000000U) / 1000000.0;
  };

  const AwDiagnostics diag = noop_diag();
  for (int scenario = 0; scenario < 50; ++scenario) {
    const int n = 3 + static_cast<int>(rnd() * 5.0);
    std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> seq;
    std::int64_t t = 1'000'000'000;
    for (int i = 0; i < n; ++i) {
      t += static_cast<std::int64_t>(1e8 + rnd() * 1e8);  // 0.1..0.2 s
      seq.push_back(make_msg(
        t, rnd() * 2.0, rnd() * 2.0, rnd() * 0.2, (rnd() - 0.5) * 0.2, (rnd() - 0.5) * 0.2,
        (rnd() - 0.5) * 2.0));
    }
    const std::int64_t first = rclcpp::Time(seq.front().header.stamp).nanoseconds();
    const std::int64_t last = rclcpp::Time(seq.back().header.stamp).nanoseconds();
    const std::int64_t query =
      first + static_cast<std::int64_t>(rnd() * static_cast<double>(last - first));

    // C++ reference.
    autoware::localization_util::SmartPoseBuffer cpp_buf(
      rclcpp::get_logger("diff"), kTimeoutSec, kDistanceTolM);
    for (const auto & m : seq) {
      cpp_buf.push_back(std::make_shared<const geometry_msgs::msg::PoseWithCovarianceStamped>(m));
    }
    const auto cpp = cpp_buf.interpolate(rclcpp::Time(query, RCL_ROS_TIME));
    ASSERT_TRUE(cpp.has_value()) << "scenario " << scenario;

    // Rust under test (activate, push through on_initial_pose, query through the FFI).
    AwNdtScanMatcher * h = make_handle("map");
    ASSERT_NE(h, nullptr);
    autoware_ndt_scan_matcher_rs_node_on_trigger(h, &diag, true, 0);
    for (const auto & m : seq) {
      const AwPoseWithCovarianceStampedView v = make_pose_with_cov_view(m);
      autoware_ndt_scan_matcher_rs_node_on_initial_pose(h, &diag, &v);
    }
    AwInitialPoseInterpolation interp{};
    const bool ok = autoware_ndt_scan_matcher_rs_initial_pose_interpolate(h, query, &interp);
    ASSERT_TRUE(ok) << "scenario " << scenario;

    const auto & ip = cpp->interpolated_pose.pose.pose;
    EXPECT_NEAR(interp.interpolated_position[0], ip.position.x, 1e-6) << "scenario " << scenario;
    EXPECT_NEAR(interp.interpolated_position[1], ip.position.y, 1e-6) << "scenario " << scenario;
    EXPECT_NEAR(interp.interpolated_position[2], ip.position.z, 1e-6) << "scenario " << scenario;
    const double dot = interp.interpolated_orientation[0] * ip.orientation.x +
                       interp.interpolated_orientation[1] * ip.orientation.y +
                       interp.interpolated_orientation[2] * ip.orientation.z +
                       interp.interpolated_orientation[3] * ip.orientation.w;
    EXPECT_NEAR(std::abs(dot), 1.0, 1e-6) << "scenario " << scenario;

    // The bracket positions (old/new) feed publish_initial_to_result.
    EXPECT_NEAR(interp.old_position[0], cpp->old_pose.pose.pose.position.x, 1e-9)
      << "scenario " << scenario;
    EXPECT_NEAR(interp.new_position[0], cpp->new_pose.pose.pose.position.x, 1e-9)
      << "scenario " << scenario;

    autoware_ndt_scan_matcher_rs_free(h);
  }
}

// A pose pushed while the node is deactivated is dropped (the activation gate), so the buffer cannot
// interpolate afterwards.
TEST(InitialPoseBuffer, PoseDroppedWhenNotActivated)  // NOLINT
{
  const AwDiagnostics diag = noop_diag();
  AwNdtScanMatcher * h = make_handle("map");
  ASSERT_NE(h, nullptr);
  // Not activated: two pushes are rejected by the gate.
  const auto m0 = make_msg(1'000'000'000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  const auto m1 = make_msg(1'100'000'000, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  const AwPoseWithCovarianceStampedView v0 = make_pose_with_cov_view(m0);
  const AwPoseWithCovarianceStampedView v1 = make_pose_with_cov_view(m1);
  autoware_ndt_scan_matcher_rs_node_on_initial_pose(h, &diag, &v0);
  autoware_ndt_scan_matcher_rs_node_on_initial_pose(h, &diag, &v1);
  AwInitialPoseInterpolation interp{};
  EXPECT_FALSE(autoware_ndt_scan_matcher_rs_initial_pose_interpolate(h, 1'050'000'000, &interp));
  autoware_ndt_scan_matcher_rs_free(h);
}

}  // namespace autoware::ndt_scan_matcher
