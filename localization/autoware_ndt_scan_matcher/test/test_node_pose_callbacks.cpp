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

// Test (node port N2): the thin pose callbacks migrated to Rust drive node state through the host
// vtable. We supply a MOCK AwNdtHost whose trampolines record side effects into a local Recorder
// (mirroring the real ctx == NDTScanMatcher* pattern), then assert that the FFI entry points return
// the right status and produce the right buffer-push / latest-position effects for each gate
// combination. The opaque `msg` token is passed straight through and never dereferenced by Rust.

#include "autoware_ndt_scan_matcher_rs.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <string>
#include <tuple>

namespace
{
// Records what the migrated Rust callbacks ask the host to do.
struct Recorder
{
  bool activated = false;
  bool is_activated_ret = false;
  int initial_pushes = 0;
  int reg_pushes = 0;
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
extern "C" void rec_push_reg(void * ctx, const void * msg)
{
  auto * r = static_cast<Recorder *>(ctx);
  ++r->reg_pushes;
  r->last_msg = msg;
}
extern "C" void rec_set_position(void * ctx, double x, double y, double z)
{
  static_cast<Recorder *>(ctx)->position = std::make_tuple(x, y, z);
}

AwNdtHost mock_host(Recorder & r)
{
  return AwNdtHost{
    &r,          rec_set_activated, rec_clear,       rec_is_activated,
    rec_push_initial, rec_push_reg, rec_set_position};
}

int call_initial(
  const AwNdtHost & host, const std::string & frame_id, const std::string & map_frame,
  const double (&pos)[3], const void * msg)
{
  return autoware_ndt_scan_matcher_rs_node_on_initial_pose(
    &host, reinterpret_cast<const uint8_t *>(frame_id.data()), frame_id.size(),
    reinterpret_cast<const uint8_t *>(map_frame.data()), map_frame.size(), pos, msg);
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
  const AwNdtHost host = mock_host(r);
  const double pos[3] = {1.0, 2.0, 3.0};
  int dummy = 0;
  EXPECT_EQ(call_initial(host, "map", "map", pos, &dummy), kNotActivated);
  EXPECT_EQ(r.initial_pushes, 0);
  EXPECT_FALSE(r.position.has_value());
}

TEST(NodePoseCallbacks, InitialPoseRejectedWhenFrameMismatch)  // NOLINT
{
  Recorder r;
  r.is_activated_ret = true;
  const AwNdtHost host = mock_host(r);
  const double pos[3] = {1.0, 2.0, 3.0};
  int dummy = 0;
  EXPECT_EQ(call_initial(host, "lidar", "map", pos, &dummy), kWrongFrame);
  EXPECT_EQ(r.initial_pushes, 0);
  EXPECT_FALSE(r.position.has_value());
}

TEST(NodePoseCallbacks, InitialPoseAcceptedPushesAndSetsPosition)  // NOLINT
{
  Recorder r;
  r.is_activated_ret = true;
  const AwNdtHost host = mock_host(r);
  const double pos[3] = {1.5, -2.5, 0.25};
  int dummy = 0;
  EXPECT_EQ(call_initial(host, "map", "map", pos, &dummy), kAccepted);
  EXPECT_EQ(r.initial_pushes, 1);
  EXPECT_EQ(r.last_msg, &dummy);  // the opaque token was forwarded unchanged
  ASSERT_TRUE(r.position.has_value());
  EXPECT_EQ(*r.position, std::make_tuple(1.5, -2.5, 0.25));
}

TEST(NodePoseCallbacks, InitialPoseNullHostIsNotActivated)  // NOLINT
{
  const double pos[3] = {0.0, 0.0, 0.0};
  const std::string frame = "map";
  int dummy = 0;
  EXPECT_EQ(
    autoware_ndt_scan_matcher_rs_node_on_initial_pose(
      nullptr, reinterpret_cast<const uint8_t *>(frame.data()), frame.size(),
      reinterpret_cast<const uint8_t *>(frame.data()), frame.size(), pos, &dummy),
    kNotActivated);
}

TEST(NodePoseCallbacks, RegularizationPushesMsgAndNullIsNoop)  // NOLINT
{
  Recorder r;
  const AwNdtHost host = mock_host(r);
  int dummy = 0;
  autoware_ndt_scan_matcher_rs_node_on_regularization_pose(&host, &dummy);
  EXPECT_EQ(r.reg_pushes, 1);
  EXPECT_EQ(r.last_msg, &dummy);

  // null msg → no extra push.
  autoware_ndt_scan_matcher_rs_node_on_regularization_pose(&host, nullptr);
  EXPECT_EQ(r.reg_pushes, 1);
}
