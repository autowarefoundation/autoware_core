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

// Test: the map-update decision state (last-update position + need-rebuild) is Rust-owned
// on the node handle. Drive the `..._map_update_*` FFIs through a handle and assert the state
// machine: first update forces a rebuild, a successful record clears it, an in-range move triggers
// no update, and an out-of-keep-up move latches the rebuild. (The pure distance math is also
// covered by test_map_update_verdict.)

#include "autoware_ndt_scan_matcher_rs.h"

#include <gtest/gtest.h>

namespace
{
// radii mirroring a typical config: lidar 50 m, map 150 m, update every 20 m.
constexpr double kLidarR = 50.0;
constexpr double kMapR = 150.0;
constexpr double kUpdateDist = 20.0;

AwNdtScanMatcher * make_handle()
{
  AwNdtParams p{};
  p.resolution = 1.0;
  p.min_points = 6;
  p.num_threads = 1;
  return autoware_ndt_scan_matcher_rs_new(&p);
}
}  // namespace

TEST(MapUpdateState, FirstUpdateForcesRebuildThenRecordClears)  // NOLINT
{
  AwNdtScanMatcher * h = make_handle();
  ASSERT_NE(h, nullptr);

  // No prior update → first-update verdict: rebuild + should_update.
  AwMapUpdateDecision d0{};
  ASSERT_TRUE(autoware_ndt_scan_matcher_rs_map_update_evaluate(
    h, 0.0, 0.0, kLidarR, kMapR, kUpdateDist, &d0));
  EXPECT_TRUE(d0.is_first_update);
  EXPECT_TRUE(d0.should_update);
  EXPECT_TRUE(d0.need_rebuild);
  EXPECT_TRUE(autoware_ndt_scan_matcher_rs_map_update_need_rebuild(h));

  // A successful update at the origin clears the rebuild latch + records the position.
  autoware_ndt_scan_matcher_rs_map_update_record(h, 0.0, 0.0, true);
  EXPECT_FALSE(autoware_ndt_scan_matcher_rs_map_update_need_rebuild(h));

  // A small move (3-4-5 → 5 m < 20 m) → no update, still keeping up.
  AwMapUpdateDecision d1{};
  ASSERT_TRUE(autoware_ndt_scan_matcher_rs_map_update_evaluate(
    h, 3.0, 4.0, kLidarR, kMapR, kUpdateDist, &d1));
  EXPECT_FALSE(d1.is_first_update);
  EXPECT_FALSE(d1.should_update);
  EXPECT_FALSE(d1.out_of_keep_up);
  EXPECT_NEAR(d1.distance, 5.0, 1e-9);
  EXPECT_FALSE(autoware_ndt_scan_matcher_rs_map_update_out_of_range(h, 3.0, 4.0, kLidarR, kMapR));

  autoware_ndt_scan_matcher_rs_free(h);
}

TEST(MapUpdateState, OutOfKeepUpLatchesRebuild)  // NOLINT
{
  AwNdtScanMatcher * h = make_handle();
  ASSERT_NE(h, nullptr);
  autoware_ndt_scan_matcher_rs_map_update_record(h, 0.0, 0.0, true);  // last=origin, rebuild=false

  // 120 m move: 120 + lidar(50) = 170 > map(150) → out_of_keep_up; 120 > 20 → should_update.
  AwMapUpdateDecision d{};
  ASSERT_TRUE(autoware_ndt_scan_matcher_rs_map_update_evaluate(
    h, 120.0, 0.0, kLidarR, kMapR, kUpdateDist, &d));
  EXPECT_TRUE(d.out_of_keep_up);
  EXPECT_TRUE(d.should_update);
  EXPECT_TRUE(d.need_rebuild);
  EXPECT_TRUE(autoware_ndt_scan_matcher_rs_map_update_need_rebuild(h));  // latched
  EXPECT_TRUE(autoware_ndt_scan_matcher_rs_map_update_out_of_range(h, 120.0, 0.0, kLidarR, kMapR));

  // A failed update advances the position but does NOT clear the latch.
  autoware_ndt_scan_matcher_rs_map_update_record(h, 120.0, 0.0, false);
  EXPECT_TRUE(autoware_ndt_scan_matcher_rs_map_update_need_rebuild(h));

  autoware_ndt_scan_matcher_rs_free(h);
}

TEST(MapUpdateState, OutOfRangeTrueBeforeFirstUpdate)  // NOLINT
{
  AwNdtScanMatcher * h = make_handle();
  ASSERT_NE(h, nullptr);
  EXPECT_TRUE(autoware_ndt_scan_matcher_rs_map_update_out_of_range(h, 0.0, 0.0, kLidarR, kMapR));
  // Null handle is treated as out-of-range (no state).
  EXPECT_TRUE(
    autoware_ndt_scan_matcher_rs_map_update_out_of_range(nullptr, 0.0, 0.0, kLidarR, kMapR));
  autoware_ndt_scan_matcher_rs_free(h);
}
