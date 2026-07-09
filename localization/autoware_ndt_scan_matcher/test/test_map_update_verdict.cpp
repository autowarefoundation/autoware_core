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

// Differential test: the dynamic-map-update distance decision migrated to Rust
// (autoware_ndt_scan_matcher_rs_node_evaluate_map_update) must match the original C++ logic from
// MapUpdateModule::should_update_map / out_of_map_range. Both call libm hypot and then do threshold
// comparisons, so the verdict (distance + the two flags) is bit-exact. The C++ reference below is a
// verbatim transcription of that distance math (the nullopt short-circuits stay C++-side and are
// not exercised here).

#include "autoware_ndt_scan_matcher_rs.h"

#include <gtest/gtest.h>

#include <cmath>

namespace
{
// Verbatim transcription of the C++ keep-up / update-distance math (has-last branch).
AwMapUpdateVerdict cpp_reference(const AwMapUpdateInput & in)
{
  const double dx = in.current_x - in.last_update_x;
  const double dy = in.current_y - in.last_update_y;
  const double distance = std::hypot(dx, dy);
  AwMapUpdateVerdict v{};
  v.distance = distance;
  v.out_of_keep_up = (distance + in.lidar_radius > in.map_radius);
  v.should_update = (distance > in.update_distance);
  return v;
}

void expect_match(const AwMapUpdateInput & in)
{
  const AwMapUpdateVerdict expected = cpp_reference(in);
  AwMapUpdateVerdict got{};
  autoware_ndt_scan_matcher_rs_node_evaluate_map_update(&in, &got);
  EXPECT_DOUBLE_EQ(got.distance, expected.distance);
  EXPECT_EQ(got.out_of_keep_up, expected.out_of_keep_up);
  EXPECT_EQ(got.should_update, expected.should_update);
}
}  // namespace

// Grid over current positions (incl. the keep-up boundary at distance 100 and the update-distance
// boundary at 20) and a couple of radii configurations.
TEST(MapUpdateVerdict, MatchesCppAcrossGrid)  // NOLINT
{
  const double xs[] = {0.0, 3.0, 19.999, 20.0, 20.001, 99.999, 100.0, 100.001, 200.0};
  const double ys[] = {0.0, 4.0, -30.0};
  for (const double x : xs) {
    for (const double y : ys) {
      AwMapUpdateInput in{};
      in.current_x = x;
      in.current_y = y;
      in.last_update_x = 0.0;
      in.last_update_y = 0.0;
      in.lidar_radius = 50.0;
      in.map_radius = 150.0;
      in.update_distance = 20.0;
      expect_match(in);
    }
  }
}

// A non-origin last-update position, to confirm the delta (not the absolute) drives the distance.
TEST(MapUpdateVerdict, MatchesCppWithOffsetLastUpdate)  // NOLINT
{
  AwMapUpdateInput in{};
  in.current_x = 13.0;
  in.current_y = 9.0;
  in.last_update_x = 10.0;
  in.last_update_y = 5.0;  // delta (3,4) -> distance 5
  in.lidar_radius = 50.0;
  in.map_radius = 150.0;
  in.update_distance = 20.0;
  expect_match(in);
  AwMapUpdateVerdict got{};
  autoware_ndt_scan_matcher_rs_node_evaluate_map_update(&in, &got);
  EXPECT_DOUBLE_EQ(got.distance, 5.0);
}

// A null output pointer must be a no-op (no crash, no write).
TEST(MapUpdateVerdict, NullOutputIsNoop)  // NOLINT
{
  AwMapUpdateInput in{};
  in.map_radius = 150.0;
  autoware_ndt_scan_matcher_rs_node_evaluate_map_update(&in, nullptr);
  autoware_ndt_scan_matcher_rs_node_evaluate_map_update(nullptr, nullptr);
  SUCCEED();
}
