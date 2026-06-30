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

// Test (node port N2 → callback-level slice 2): the pose callbacks run entirely in Rust, driving node
// state through the host vtable AND the node's /diagnostics through the AwDiagnostics vtable. We supply
// MOCK AwNdtHost + AwDiagnostics whose trampolines record side effects + the ordered diagnostics events
// into local recorders (mirroring the real ctx == NDTScanMatcher* / diag == DiagnosticsInterface*
// pattern), then assert each gate produces the right status, buffer-push/latest-position effects, AND
// the exact diagnostics sequence (key order + values + WARN/ERROR text). The opaque `msg` token is
// passed straight through and never dereferenced by Rust.

#include "autoware_ndt_scan_matcher_rs.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

namespace
{
// Records what the migrated Rust callbacks ask the host to do.
struct Recorder
{
  bool activated = false;
  bool is_activated_ret = false;
  int initial_pushes = 0;
  const void * last_msg = nullptr;
  std::optional<std::tuple<double, double, double>> position;
};

extern "C" void rec_set_activated(void * ctx, bool a) { static_cast<Recorder *>(ctx)->activated = a; }
extern "C" void rec_clear(void * /*ctx*/) {}
extern "C" bool rec_is_activated(void * ctx) { return static_cast<Recorder *>(ctx)->is_activated_ret; }
extern "C" void rec_push_initial(void * ctx, const void * msg)
{
  auto * r = static_cast<Recorder *>(ctx);
  ++r->initial_pushes;
  r->last_msg = msg;
}
extern "C" void rec_set_position(void * ctx, double x, double y, double z)
{
  static_cast<Recorder *>(ctx)->position = std::make_tuple(x, y, z);
}

AwNdtHost mock_host(Recorder & r)
{
  return AwNdtHost{
    &r,           rec_set_activated, rec_clear, rec_is_activated,
    rec_push_initial, rec_set_position};
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

int call_initial(
  const AwNdtHost & host, const AwDiagnostics & diag, const std::string & frame_id,
  const std::string & map_frame, const double (&pos)[3], const void * msg)
{
  return autoware_ndt_scan_matcher_rs_node_on_initial_pose(
    &host, &diag, reinterpret_cast<const uint8_t *>(frame_id.data()), frame_id.size(),
    reinterpret_cast<const uint8_t *>(map_frame.data()), map_frame.size(), pos, msg, 100);
}

// Discriminants mirrored from the Rust INITIAL_POSE_* codes (also documented in the header).
constexpr int kAccepted = 0;
constexpr int kNotActivated = 1;
constexpr int kWrongFrame = 2;
}  // namespace

TEST(NodePoseCallbacks, InitialPoseRejectedWhenNotActivated)  // NOLINT
{
  Recorder r;
  r.is_activated_ret = false;
  DiagRec dr;
  const AwNdtHost host = mock_host(r);
  const AwDiagnostics diag = mock_diag(dr);
  const double pos[3] = {1.0, 2.0, 3.0};
  int dummy = 0;
  EXPECT_EQ(call_initial(host, diag, "map", "map", pos, &dummy), kNotActivated);
  EXPECT_EQ(r.initial_pushes, 0);
  EXPECT_FALSE(r.position.has_value());
  EXPECT_EQ(
    dr.events, (std::vector<std::string>{
                 "clear", "i64 topic_time_stamp=100", "bool is_activated=false",
                 "level 1 Node is not activated.", "publish 100"}));
}

TEST(NodePoseCallbacks, InitialPoseRejectedWhenFrameMismatch)  // NOLINT
{
  Recorder r;
  r.is_activated_ret = true;
  DiagRec dr;
  const AwNdtHost host = mock_host(r);
  const AwDiagnostics diag = mock_diag(dr);
  const double pos[3] = {1.0, 2.0, 3.0};
  int dummy = 0;
  EXPECT_EQ(call_initial(host, diag, "lidar", "map", pos, &dummy), kWrongFrame);
  EXPECT_EQ(r.initial_pushes, 0);
  EXPECT_FALSE(r.position.has_value());
  EXPECT_EQ(
    dr.events,
    (std::vector<std::string>{
      "clear", "i64 topic_time_stamp=100", "bool is_activated=true",
      "bool is_expected_frame_id=false",
      "level 2 Received initial pose message with frame_id lidar, but expected map. Please check "
      "the frame_id in the input topic and ensure it is correct.",
      "publish 100"}));
}

TEST(NodePoseCallbacks, InitialPoseAcceptedPushesAndSetsPosition)  // NOLINT
{
  Recorder r;
  r.is_activated_ret = true;
  DiagRec dr;
  const AwNdtHost host = mock_host(r);
  const AwDiagnostics diag = mock_diag(dr);
  const double pos[3] = {1.5, -2.5, 0.25};
  int dummy = 0;
  EXPECT_EQ(call_initial(host, diag, "map", "map", pos, &dummy), kAccepted);
  EXPECT_EQ(r.initial_pushes, 1);
  EXPECT_EQ(r.last_msg, &dummy);  // the opaque token was forwarded unchanged
  ASSERT_TRUE(r.position.has_value());
  EXPECT_EQ(*r.position, std::make_tuple(1.5, -2.5, 0.25));
  EXPECT_EQ(
    dr.events, (std::vector<std::string>{
                 "clear", "i64 topic_time_stamp=100", "bool is_activated=true",
                 "bool is_expected_frame_id=true", "publish 100"}));
}

TEST(NodePoseCallbacks, InitialPoseNullHostIsNotActivated)  // NOLINT
{
  DiagRec dr;
  const AwDiagnostics diag = mock_diag(dr);
  const double pos[3] = {0.0, 0.0, 0.0};
  const std::string frame = "map";
  int dummy = 0;
  EXPECT_EQ(
    autoware_ndt_scan_matcher_rs_node_on_initial_pose(
      nullptr, &diag, reinterpret_cast<const uint8_t *>(frame.data()), frame.size(),
      reinterpret_cast<const uint8_t *>(frame.data()), frame.size(), pos, &dummy, 0),
    kNotActivated);
  EXPECT_TRUE(dr.events.empty());  // null host → no diagnostics emitted
}

// The regularization callback moved to the Rust-owned buffer on the node handle (Phase 1 slice A);
// it no longer uses the host vtable. Its diagnostics + buffer behavior is covered by the Rust unit
// tests and the differential test (test_regularization_buffer.cpp).
