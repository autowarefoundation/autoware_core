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

// Differential test: the Rust-owned regularization pose buffer (driven through the
// FFI — push via on_regularization_pose, query via regularization_interpolate) must reproduce the C++
// SmartPoseBuffer's interpolation bit-close, over many random pose sequences. This pins the ported
// twist/linear-RPY interpolation math against the original.

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
#include <vector>

namespace autoware::ndt_scan_matcher
{
namespace
{
// No-op diagnostics vtable (the callback emits diagnostics we don't assert here).
extern "C" void nd_clear(void *) {}
extern "C" void nd_bool(void *, const std::uint8_t *, std::size_t, bool) {}
extern "C" void nd_i64(void *, const std::uint8_t *, std::size_t, std::int64_t) {}
extern "C" void nd_f64(void *, const std::uint8_t *, std::size_t, double) {}
extern "C" void nd_str(void *, const std::uint8_t *, std::size_t, const std::uint8_t *, std::size_t) {}
extern "C" void nd_level(void *, std::int8_t, const std::uint8_t *, std::size_t) {}
extern "C" void nd_publish(void *, std::int64_t) {}
AwDiagnostics noop_diag()
{
  return AwDiagnostics{nullptr, nd_clear, nd_bool, nd_i64, nd_f64, nd_str, nd_level, nd_publish};
}

AwNdtParams reg_params()
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
  p.regularization_enable = true;
  p.regularization_pose_timeout_sec = 1000.0;
  p.regularization_pose_distance_tolerance_m = 1000.0;
  return p;
}

geometry_msgs::msg::PoseWithCovarianceStamped make_msg(
  std::int64_t stamp_ns, double x, double y, double z, double roll, double pitch, double yaw)
{
  geometry_msgs::msg::PoseWithCovarianceStamped m;
  m.header.stamp = rclcpp::Time(stamp_ns);
  m.pose.pose.position.x = x;
  m.pose.pose.position.y = y;
  m.pose.pose.position.z = z;
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  m.pose.pose.orientation = tf2::toMsg(q);
  return m;
}

// The Rust port must match the C++ SmartPoseBuffer interpolation on many random, non-degenerate
// (bounded roll/pitch — no gimbal lock) sequences.
TEST(RegularizationBuffer, MatchesCppOnRandomSequences)  // NOLINT
{
  std::uint64_t seed = 88172645463325252ULL;  // xorshift, deterministic
  auto rnd = [&seed]() {
    seed ^= seed << 13;
    seed ^= seed >> 7;
    seed ^= seed << 17;
    return static_cast<double>(seed % 1000000U) / 1000000.0;
  };

  for (int scenario = 0; scenario < 50; ++scenario) {
    const int n = 3 + static_cast<int>(rnd() * 5.0);  // 3..7 entries
    std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> seq;
    std::int64_t t = 1'000'000'000;  // 1 s base (nonzero — avoids the zero-stamp guard)
    for (int i = 0; i < n; ++i) {
      t += static_cast<std::int64_t>(1e8 + rnd() * 1e8);  // 0.1..0.2 s steps
      seq.push_back(make_msg(
        t, rnd() * 2.0, rnd() * 2.0, rnd() * 0.2, (rnd() - 0.5) * 0.2, (rnd() - 0.5) * 0.2,
        (rnd() - 0.5) * 2.0));
    }
    const std::int64_t first = rclcpp::Time(seq.front().header.stamp).nanoseconds();
    const std::int64_t last = rclcpp::Time(seq.back().header.stamp).nanoseconds();
    const std::int64_t query = first + static_cast<std::int64_t>(rnd() * static_cast<double>(last - first));

    // C++ reference.
    autoware::localization_util::SmartPoseBuffer cpp_buf(rclcpp::get_logger("diff"), 1000.0, 1000.0);
    for (const auto & m : seq) {
      cpp_buf.push_back(std::make_shared<const geometry_msgs::msg::PoseWithCovarianceStamped>(m));
    }
    // The buffer's stamps are RCL_ROS_TIME (from the message Time); match the query clock source.
    const auto cpp = cpp_buf.interpolate(rclcpp::Time(query, RCL_ROS_TIME));
    ASSERT_TRUE(cpp.has_value()) << "scenario " << scenario;

    // Rust under test (push through the callback, query through the FFI).
    const AwNdtParams p = reg_params();
    AwNdtScanMatcher * h = autoware_ndt_scan_matcher_rs_new(&p);
    ASSERT_NE(h, nullptr);
    const AwDiagnostics d = noop_diag();
    for (const auto & m : seq) {
      const AwPoseWithCovarianceStampedView v = make_pose_with_cov_view(m);
      autoware_ndt_scan_matcher_rs_node_on_regularization_pose(h, &d, &v);
    }
    AwInterpolatedPose out{};
    const bool ok = autoware_ndt_scan_matcher_rs_regularization_interpolate(h, query, &out);
    ASSERT_TRUE(ok) << "scenario " << scenario;

    const auto & cp = cpp->interpolated_pose.pose.pose.position;
    EXPECT_NEAR(out.position[0], cp.x, 1e-6) << "scenario " << scenario;
    EXPECT_NEAR(out.position[1], cp.y, 1e-6) << "scenario " << scenario;
    EXPECT_NEAR(out.position[2], cp.z, 1e-6) << "scenario " << scenario;

    // Orientation: same rotation up to quaternion sign → |dot| ~ 1.
    const auto & co = cpp->interpolated_pose.pose.pose.orientation;
    const double dot = out.orientation[0] * co.x + out.orientation[1] * co.y +
                       out.orientation[2] * co.z + out.orientation[3] * co.w;
    EXPECT_NEAR(std::abs(dot), 1.0, 1e-6) << "scenario " << scenario;

    autoware_ndt_scan_matcher_rs_free(h);
  }
}

}  // namespace
}  // namespace autoware::ndt_scan_matcher
