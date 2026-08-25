// Copyright 2026 TIER IV, Inc.
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

#include "autoware/behavior_velocity_planner_common/utilization/util.hpp"

#include <autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <vector>

namespace
{
using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_internal_planning_msgs::msg::PathWithLaneId;

PathWithLaneId makePathWithLaneIds(const std::vector<std::vector<int64_t>> & per_point_lane_ids)
{
  PathWithLaneId path;
  for (const auto & lane_ids : per_point_lane_ids) {
    PathPointWithLaneId point;
    point.lane_ids = lane_ids;
    path.points.push_back(point);
  }
  return path;
}
}  // namespace

// ---------------------------------------------------------------------------
// getSortedLaneIdsFromPath: preserves first-seen order, deduplicates
// ---------------------------------------------------------------------------
TEST(PlanningUtilsPure, getSortedLaneIdsFromPathPreservesOrderAndDeduplicates)
{
  using autoware::behavior_velocity_planner::planning_utils::getSortedLaneIdsFromPath;

  // Lane ids appear in first-seen order; duplicates within and across points are removed.
  const auto path = makePathWithLaneIds({{10, 20}, {20, 30}, {10}, {40}});
  const auto result = getSortedLaneIdsFromPath(path);

  const std::vector<int64_t> expected{10, 20, 30, 40};
  EXPECT_EQ(result, expected);
}

TEST(PlanningUtilsPure, getSortedLaneIdsFromPathEmpty)
{
  using autoware::behavior_velocity_planner::planning_utils::getSortedLaneIdsFromPath;

  const auto path = makePathWithLaneIds({});
  EXPECT_TRUE(getSortedLaneIdsFromPath(path).empty());
}

// ---------------------------------------------------------------------------
// getSubsequentLaneIdsSetOnPath: returns the tail starting at base_lane_id
// ---------------------------------------------------------------------------
TEST(PlanningUtilsPure, getSubsequentLaneIdsSetOnPathFromMiddle)
{
  using autoware::behavior_velocity_planner::planning_utils::getSubsequentLaneIdsSetOnPath;

  const auto path = makePathWithLaneIds({{10}, {20}, {30}, {40}});
  const auto result = getSubsequentLaneIdsSetOnPath(path, 20);

  const std::vector<int64_t> expected{20, 30, 40};
  EXPECT_EQ(result, expected);
}

TEST(PlanningUtilsPure, getSubsequentLaneIdsSetOnPathFromFirst)
{
  using autoware::behavior_velocity_planner::planning_utils::getSubsequentLaneIdsSetOnPath;

  const auto path = makePathWithLaneIds({{10}, {20}, {30}});
  const auto result = getSubsequentLaneIdsSetOnPath(path, 10);

  const std::vector<int64_t> expected{10, 20, 30};
  EXPECT_EQ(result, expected);
}

TEST(PlanningUtilsPure, getSubsequentLaneIdsSetOnPathNotFoundReturnsEmpty)
{
  using autoware::behavior_velocity_planner::planning_utils::getSubsequentLaneIdsSetOnPath;

  const auto path = makePathWithLaneIds({{10}, {20}, {30}});
  // base_lane_id 999 is not on the path -> empty result.
  EXPECT_TRUE(getSubsequentLaneIdsSetOnPath(path, 999).empty());
}

// ---------------------------------------------------------------------------
// findReachTime: throw on invalid search range, solve on valid range
// ---------------------------------------------------------------------------
TEST(PlanningUtilsPure, findReachTimeThrowsWhenRangeInvalid)
{
  using autoware::behavior_velocity_planner::planning_utils::findReachTime;

  // f(t) = j t^3/6 + a t^2/2 + v t - d.
  // With j=0, a=0, v=1, d=10: f(t) = t - 10, so f(t_min)=f(0)=-10 (<=0) and
  // f(t_max)=f(5)=-5 (<0) -> the target distance is never reached in [0, 5].
  // findReachTime requires f(min) <= 0 and f(max) >= 0, so this must throw.
  EXPECT_THROW(
    { findReachTime(/*jerk=*/0.0, /*accel=*/0.0, /*velocity=*/1.0, /*distance=*/10.0, 0.0, 5.0); },
    std::logic_error);

  // f(min) > 0 also violates the bracket precondition: j=0, a=0, v=1, d=-1 -> f(0)=1 > 0.
  EXPECT_THROW(
    { findReachTime(/*jerk=*/0.0, /*accel=*/0.0, /*velocity=*/1.0, /*distance=*/-1.0, 0.0, 5.0); },
    std::logic_error);
}

TEST(PlanningUtilsPure, findReachTimeSolvesLinearCase)
{
  using autoware::behavior_velocity_planner::planning_utils::findReachTime;

  // j=0, a=0, v=2, d=6 -> f(t) = 2 t - 6, root at t = 3 within [0, 10].
  const double t =
    findReachTime(/*jerk=*/0.0, /*accel=*/0.0, /*velocity=*/2.0, /*distance=*/6.0, 0.0, 10.0);
  EXPECT_NEAR(t, 3.0, 1e-4);
}

TEST(PlanningUtilsPure, findReachTimeSolvesConstantAccelCase)
{
  using autoware::behavior_velocity_planner::planning_utils::findReachTime;

  // j=0, a=2, v=0, d=4 -> f(t) = t^2 - 4, root at t = 2 within [0, 10].
  const double t =
    findReachTime(/*jerk=*/0.0, /*accel=*/2.0, /*velocity=*/0.0, /*distance=*/4.0, 0.0, 10.0);
  EXPECT_NEAR(t, 2.0, 1e-4);
}

// ---------------------------------------------------------------------------
// calcDecelerationVelocityFromDistanceToTarget: throw + edge branches
// ---------------------------------------------------------------------------
TEST(PlanningUtilsPure, calcDecelerationVelocityThrowsOnNonNegativeJerk)
{
  using autoware::behavior_velocity_planner::planning_utils::
    calcDecelerationVelocityFromDistanceToTarget;

  // jerk/accel must be negative; a non-negative jerk (i.e. >= 0) must throw.
  // A strictly-positive jerk must throw.
  EXPECT_THROW(
    {
      calcDecelerationVelocityFromDistanceToTarget(
        /*max_slowdown_jerk=*/0.5, /*max_slowdown_accel=*/-2.0, /*current_accel=*/0.0,
        /*current_velocity=*/5.0, /*distance_to_target=*/10.0);
    },
    std::logic_error);

  // The zero boundary must throw too: jerk == 0 would otherwise divide by j_max == 0.
  EXPECT_THROW(
    {
      calcDecelerationVelocityFromDistanceToTarget(
        /*max_slowdown_jerk=*/0.0, /*max_slowdown_accel=*/-2.0, /*current_accel=*/0.0,
        /*current_velocity=*/5.0, /*distance_to_target=*/10.0);
    },
    std::logic_error);
}

TEST(PlanningUtilsPure, calcDecelerationVelocityThrowsOnNonNegativeAccel)
{
  using autoware::behavior_velocity_planner::planning_utils::
    calcDecelerationVelocityFromDistanceToTarget;

  // A non-negative accel (i.e. >= 0) must also throw.
  // A strictly-positive accel must throw.
  EXPECT_THROW(
    {
      calcDecelerationVelocityFromDistanceToTarget(
        /*max_slowdown_jerk=*/-1.0, /*max_slowdown_accel=*/0.5, /*current_accel=*/0.0,
        /*current_velocity=*/5.0, /*distance_to_target=*/10.0);
    },
    std::logic_error);

  // The zero boundary must throw too: accel == 0 would otherwise divide by a_max == 0.
  EXPECT_THROW(
    {
      calcDecelerationVelocityFromDistanceToTarget(
        /*max_slowdown_jerk=*/-1.0, /*max_slowdown_accel=*/0.0, /*current_accel=*/0.0,
        /*current_velocity=*/5.0, /*distance_to_target=*/10.0);
    },
    std::logic_error);
}

TEST(PlanningUtilsPure, calcDecelerationVelocityBehindEgoReturnsCurrentVelocity)
{
  using autoware::behavior_velocity_planner::planning_utils::
    calcDecelerationVelocityFromDistanceToTarget;

  // distance_to_target <= 0 -> target is behind ego -> current velocity is returned unchanged.
  const double v = calcDecelerationVelocityFromDistanceToTarget(
    /*max_slowdown_jerk=*/-1.0, /*max_slowdown_accel=*/-2.0, /*current_accel=*/1.0,
    /*current_velocity=*/5.0, /*distance_to_target=*/-3.0);
  EXPECT_DOUBLE_EQ(v, 5.0);
}

TEST(PlanningUtilsPure, calcDecelerationVelocityConstJerkBranch)
{
  using autoware::behavior_velocity_planner::planning_utils::
    calcDecelerationVelocityFromDistanceToTarget;

  // Matches the verified constant-jerk case from test_path_utilization.cpp:
  // jerk=-1, accel=-2, current_accel=1, current_velocity=5, distance=8 -> v ~= 5.380.
  // This exercises the d_const_acc_stop < 0 branch that calls findReachTime.
  const double v = calcDecelerationVelocityFromDistanceToTarget(
    /*max_slowdown_jerk=*/-1.0, /*max_slowdown_accel=*/-2.0, /*current_accel=*/1.0,
    /*current_velocity=*/5.0, /*distance_to_target=*/8.0);
  EXPECT_NEAR(v, 5.380, 1e-3);
}

TEST(PlanningUtilsPure, calcDecelerationVelocityAlreadyStoppingBranch)
{
  using autoware::behavior_velocity_planner::planning_utils::
    calcDecelerationVelocityFromDistanceToTarget;

  // Matches the verified "after stop" case from test_path_utilization.cpp:
  // distance=24 is farther than the achievable stop distance -> discriminant_of_stop <= 0 -> 0.
  const double v = calcDecelerationVelocityFromDistanceToTarget(
    /*max_slowdown_jerk=*/-1.0, /*max_slowdown_accel=*/-2.0, /*current_accel=*/1.0,
    /*current_velocity=*/5.0, /*distance_to_target=*/24.0);
  EXPECT_DOUBLE_EQ(v, 0.0);
}

TEST(PlanningUtilsPure, calcDecelerationVelocityConstAccelBranch)
{
  using autoware::behavior_velocity_planner::planning_utils::
    calcDecelerationVelocityFromDistanceToTarget;

  // Matches the verified constant-accel case from test_path_utilization.cpp:
  // distance=16 -> v ~= 2.872.
  const double v = calcDecelerationVelocityFromDistanceToTarget(
    /*max_slowdown_jerk=*/-1.0, /*max_slowdown_accel=*/-2.0, /*current_accel=*/1.0,
    /*current_velocity=*/5.0, /*distance_to_target=*/16.0);
  EXPECT_NEAR(v, 2.872, 1e-3);
}
