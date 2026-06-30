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

// Test (Phase 1 slice B): the trigger + initial-pose callbacks run entirely in Rust, driving the
// node's Rust-owned state on the opaque handle (activation, the initial-pose buffer, latest-EKF
// position) and emitting /diagnostics through the AwDiagnostics vtable. We build a real handle via the
// FFI + a MOCK AwDiagnostics that records the ordered events, then assert each gate's status, the
// observable state (via the is_activated / latest_ekf_position read-FFIs), and the exact diagnostics
// sequence (key order + values + WARN/ERROR text). The host vtable is gone (state is Rust-owned now).

#include "autoware_ndt_scan_matcher_rs.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

namespace
{
// A live handle with the given expected initial-pose frame. Regularization off; 1000/1000 initial-pose
// tolerances (validation effectively off, as for the regularization buffer). `map_frame` must outlive
// the `_new` call (Rust copies it).
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
  p.initial_pose_timeout_sec = 1000.0;
  p.initial_pose_distance_tolerance_m = 1000.0;
  return autoware_ndt_scan_matcher_rs_new(&p);
}

AwPoseWithCovarianceStampedView make_view(
  std::int64_t stamp_ns, const std::string & frame_id, const double (&pos)[3])
{
  AwPoseWithCovarianceStampedView v{};
  v.stamp_ns = stamp_ns;
  v.position[0] = pos[0];
  v.position[1] = pos[1];
  v.position[2] = pos[2];
  v.orientation[3] = 1.0;
  v.frame_id = reinterpret_cast<const std::uint8_t *>(frame_id.data());
  v.frame_id_len = frame_id.size();
  return v;
}

// Mock diagnostics: each vtable op appends a human-readable event so the callback's full diagnostics
// sequence (order + keys + values) can be asserted.
struct DiagRec
{
  std::vector<std::string> events;
};
std::string key_str(const std::uint8_t * p, std::size_t len)
{
  return std::string(reinterpret_cast<const char *>(p), len);
}
extern "C" void d_clear(void * d) { static_cast<DiagRec *>(d)->events.emplace_back("clear"); }
extern "C" void d_bool(void * d, const std::uint8_t * k, std::size_t kl, bool v)
{
  static_cast<DiagRec *>(d)->events.push_back("bool " + key_str(k, kl) + "=" + (v ? "true" : "false"));
}
extern "C" void d_i64(void * d, const std::uint8_t * k, std::size_t kl, std::int64_t v)
{
  static_cast<DiagRec *>(d)->events.push_back("i64 " + key_str(k, kl) + "=" + std::to_string(v));
}
extern "C" void d_f64(void * d, const std::uint8_t * k, std::size_t kl, double v)
{
  static_cast<DiagRec *>(d)->events.push_back("f64 " + key_str(k, kl) + "=" + std::to_string(v));
}
extern "C" void d_str(
  void * d, const std::uint8_t * k, std::size_t kl, const std::uint8_t * val, std::size_t vl)
{
  static_cast<DiagRec *>(d)->events.push_back("str " + key_str(k, kl) + "=" + key_str(val, vl));
}
extern "C" void d_level(void * d, std::int8_t level, const std::uint8_t * msg, std::size_t ml)
{
  static_cast<DiagRec *>(d)->events.push_back(
    "level " + std::to_string(static_cast<int>(level)) + " " + key_str(msg, ml));
}
extern "C" void d_publish(void * d, std::int64_t stamp)
{
  static_cast<DiagRec *>(d)->events.push_back("publish " + std::to_string(stamp));
}
AwDiagnostics mock_diag(DiagRec & d)
{
  return AwDiagnostics{&d, d_clear, d_bool, d_i64, d_f64, d_str, d_level, d_publish};
}

// Activate the handle via the trigger callback (discarding its diagnostics).
void activate(AwNdtScanMatcher * h)
{
  DiagRec dr;
  const AwDiagnostics diag = mock_diag(dr);
  autoware_ndt_scan_matcher_rs_node_on_trigger(h, &diag, true, 0);
}

// Discriminants mirrored from the Rust INITIAL_POSE_* codes (also documented in the header).
constexpr int kAccepted = 0;
constexpr int kNotActivated = 1;
constexpr int kWrongFrame = 2;
}  // namespace

TEST(NodePoseCallbacks, TriggerSetsActivationAndEmitsDiagnostics)  // NOLINT
{
  AwNdtScanMatcher * h = make_handle("map");
  ASSERT_NE(h, nullptr);
  DiagRec dr;
  const AwDiagnostics diag = mock_diag(dr);

  EXPECT_TRUE(autoware_ndt_scan_matcher_rs_node_on_trigger(h, &diag, true, 123));
  EXPECT_TRUE(autoware_ndt_scan_matcher_rs_is_activated(h));
  EXPECT_EQ(
    dr.events, (std::vector<std::string>{
                 "clear", "i64 service_call_time_stamp=123", "bool is_activated=true",
                 "bool is_succeed_service=true", "publish 123"}));

  dr.events.clear();
  EXPECT_TRUE(autoware_ndt_scan_matcher_rs_node_on_trigger(h, &diag, false, 456));
  EXPECT_FALSE(autoware_ndt_scan_matcher_rs_is_activated(h));
  autoware_ndt_scan_matcher_rs_free(h);
}

TEST(NodePoseCallbacks, InitialPoseRejectedWhenNotActivated)  // NOLINT
{
  AwNdtScanMatcher * h = make_handle("map");  // not activated
  ASSERT_NE(h, nullptr);
  DiagRec dr;
  const AwDiagnostics diag = mock_diag(dr);
  const double pos[3] = {1.0, 2.0, 3.0};
  const std::string frame_id = "map";  // must outlive the call (the view borrows its bytes)
  const AwPoseWithCovarianceStampedView view = make_view(100, frame_id, pos);
  EXPECT_EQ(
    autoware_ndt_scan_matcher_rs_node_on_initial_pose(h, &diag, &view), kNotActivated);
  double xyz[3] = {0.0, 0.0, 0.0};
  EXPECT_FALSE(autoware_ndt_scan_matcher_rs_latest_ekf_position(h, xyz));
  EXPECT_EQ(
    dr.events, (std::vector<std::string>{
                 "clear", "i64 topic_time_stamp=100", "bool is_activated=false",
                 "level 1 Node is not activated.", "publish 100"}));
  autoware_ndt_scan_matcher_rs_free(h);
}

TEST(NodePoseCallbacks, InitialPoseRejectedWhenFrameMismatch)  // NOLINT
{
  AwNdtScanMatcher * h = make_handle("map");
  ASSERT_NE(h, nullptr);
  activate(h);
  DiagRec dr;
  const AwDiagnostics diag = mock_diag(dr);
  const double pos[3] = {1.0, 2.0, 3.0};
  const std::string frame_id = "lidar";  // must outlive the call (the view borrows its bytes)
  const AwPoseWithCovarianceStampedView view = make_view(100, frame_id, pos);
  EXPECT_EQ(autoware_ndt_scan_matcher_rs_node_on_initial_pose(h, &diag, &view), kWrongFrame);
  double xyz[3] = {0.0, 0.0, 0.0};
  EXPECT_FALSE(autoware_ndt_scan_matcher_rs_latest_ekf_position(h, xyz));
  EXPECT_EQ(
    dr.events,
    (std::vector<std::string>{
      "clear", "i64 topic_time_stamp=100", "bool is_activated=true",
      "bool is_expected_frame_id=false",
      "level 2 Received initial pose message with frame_id lidar, but expected map. Please check "
      "the frame_id in the input topic and ensure it is correct.",
      "publish 100"}));
  autoware_ndt_scan_matcher_rs_free(h);
}

TEST(NodePoseCallbacks, InitialPoseAcceptedSetsLatestEkfPosition)  // NOLINT
{
  AwNdtScanMatcher * h = make_handle("map");
  ASSERT_NE(h, nullptr);
  activate(h);
  DiagRec dr;
  const AwDiagnostics diag = mock_diag(dr);
  const double pos[3] = {1.5, -2.5, 0.25};
  const std::string frame_id = "map";  // must outlive the call (the view borrows its bytes)
  const AwPoseWithCovarianceStampedView view = make_view(100, frame_id, pos);
  EXPECT_EQ(autoware_ndt_scan_matcher_rs_node_on_initial_pose(h, &diag, &view), kAccepted);
  double xyz[3] = {0.0, 0.0, 0.0};
  ASSERT_TRUE(autoware_ndt_scan_matcher_rs_latest_ekf_position(h, xyz));
  EXPECT_DOUBLE_EQ(xyz[0], 1.5);
  EXPECT_DOUBLE_EQ(xyz[1], -2.5);
  EXPECT_DOUBLE_EQ(xyz[2], 0.25);
  EXPECT_EQ(
    dr.events, (std::vector<std::string>{
                 "clear", "i64 topic_time_stamp=100", "bool is_activated=true",
                 "bool is_expected_frame_id=true", "publish 100"}));
  autoware_ndt_scan_matcher_rs_free(h);
}

TEST(NodePoseCallbacks, InitialPoseNullHandleIsNotActivated)  // NOLINT
{
  DiagRec dr;
  const AwDiagnostics diag = mock_diag(dr);
  const double pos[3] = {0.0, 0.0, 0.0};
  const std::string frame_id = "map";  // must outlive the call (the view borrows its bytes)
  const AwPoseWithCovarianceStampedView view = make_view(0, frame_id, pos);
  EXPECT_EQ(
    autoware_ndt_scan_matcher_rs_node_on_initial_pose(nullptr, &diag, &view), kNotActivated);
  EXPECT_TRUE(dr.events.empty());  // null handle → no diagnostics emitted
}
