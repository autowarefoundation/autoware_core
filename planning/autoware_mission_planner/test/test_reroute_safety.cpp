// Copyright 2025 The Autoware Contributors
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

#include "../src/mission_planner/reroute_safety.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <memory>
#include <vector>

namespace
{
using autoware::mission_planner::check_reroute_safety;
using autoware_planning_msgs::msg::LaneletPrimitive;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;
using geometry_msgs::msg::Pose;

// Build a straight, axis-aligned lanelet occupying x in [x_start, x_end], y in [-1, 1]. Its
// centerline therefore runs along y = 0 and has a 2D length of (x_end - x_start).
lanelet::Lanelet make_straight_lanelet(
  const lanelet::Id id, const double x_start, const double x_end)
{
  lanelet::LineString3d left_bound(lanelet::utils::getId());
  left_bound.push_back(lanelet::Point3d{lanelet::utils::getId(), x_start, 1.0, 0.0});
  left_bound.push_back(lanelet::Point3d{lanelet::utils::getId(), x_end, 1.0, 0.0});
  lanelet::LineString3d right_bound(lanelet::utils::getId());
  right_bound.push_back(lanelet::Point3d{lanelet::utils::getId(), x_start, -1.0, 0.0});
  right_bound.push_back(lanelet::Point3d{lanelet::utils::getId(), x_end, -1.0, 0.0});
  return lanelet::Lanelet(id, left_bound, right_bound);
}

Pose make_pose(const double x, const double y = 0.0)
{
  Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.0;
  pose.orientation.w = 1.0;
  return pose;
}

LaneletSegment make_segment(const std::vector<lanelet::Id> & ids)
{
  LaneletSegment segment;
  for (const auto id : ids) {
    LaneletPrimitive primitive;
    primitive.id = id;
    primitive.primitive_type = "lane";
    segment.primitives.push_back(primitive);
    segment.preferred_primitive.id = id;
  }
  return segment;
}

rclcpp::Logger test_logger()
{
  return rclcpp::get_logger("test_reroute_safety");
}

}  // namespace

class CheckRerouteSafetyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Three colinear lanelets, each 100 m long, covering x in [0, 300].
    map_ = std::make_shared<lanelet::LaneletMap>();
    map_->add(make_straight_lanelet(1, 0.0, 100.0));
    map_->add(make_straight_lanelet(2, 100.0, 200.0));
    map_->add(make_straight_lanelet(3, 200.0, 300.0));
  }

  // A nominal route: segments {1}, {2}, {3}, start at x=10, goal at x=290.
  static LaneletRoute make_nominal_route()
  {
    LaneletRoute route;
    route.start_pose = make_pose(10.0);
    route.goal_pose = make_pose(290.0);
    route.segments = {make_segment({1}), make_segment({2}), make_segment({3})};
    return route;
  }

  lanelet::LaneletMapPtr map_;
  static constexpr double kRerouteTimeThreshold = 10.0;
  static constexpr double kMinimumRerouteLength = 30.0;
};

// ---------------------------------------------------------------------------------------------
// Early-return branches.
// ---------------------------------------------------------------------------------------------

TEST_F(CheckRerouteSafetyTest, ReturnsFalseWhenOriginalRouteEmpty)
{
  const auto target = make_nominal_route();
  LaneletRoute original;  // empty segments
  EXPECT_FALSE(check_reroute_safety(
    original, target, map_, 5.0, kRerouteTimeThreshold, kMinimumRerouteLength, test_logger()));
}

TEST_F(CheckRerouteSafetyTest, ReturnsFalseWhenTargetRouteEmpty)
{
  const auto original = make_nominal_route();
  LaneletRoute target;  // empty segments
  EXPECT_FALSE(check_reroute_safety(
    original, target, map_, 5.0, kRerouteTimeThreshold, kMinimumRerouteLength, test_logger()));
}

TEST_F(CheckRerouteSafetyTest, ReturnsFalseWhenMapIsNull)
{
  const auto original = make_nominal_route();
  const auto target = make_nominal_route();
  const lanelet::LaneletMapConstPtr null_map = nullptr;
  EXPECT_FALSE(check_reroute_safety(
    original, target, null_map, 5.0, kRerouteTimeThreshold, kMinimumRerouteLength, test_logger()));
}

TEST_F(CheckRerouteSafetyTest, ReturnsTrueWhenVehicleStopped)
{
  // current_velocity < 0.01 short-circuits to true before any geometric check, so even a
  // target route that shares no segment with the original is accepted.
  const auto original = make_nominal_route();
  LaneletRoute target;
  target.start_pose = make_pose(10.0);
  target.goal_pose = make_pose(290.0);
  target.segments = {make_segment({99})};  // unknown id, no common segment
  EXPECT_TRUE(check_reroute_safety(
    original, target, map_, 0.0, kRerouteTimeThreshold, kMinimumRerouteLength, test_logger()));
}

TEST_F(CheckRerouteSafetyTest, ReturnsFalseWhenNoCommonSegment)
{
  const auto original = make_nominal_route();
  LaneletRoute target;
  target.start_pose = make_pose(10.0);
  target.goal_pose = make_pose(290.0);
  target.segments = {make_segment({99})};  // no overlap with original {1,2,3}
  EXPECT_FALSE(check_reroute_safety(
    original, target, map_, 5.0, kRerouteTimeThreshold, kMinimumRerouteLength, test_logger()));
}

TEST_F(CheckRerouteSafetyTest, ReturnsFalseWhenEgoNotOnFirstTargetSection)
{
  // Common segment exists, but the target route start_pose is placed far outside lanelet 1,
  // so ego is not inside the first target section.
  const auto original = make_nominal_route();
  LaneletRoute target;
  target.start_pose = make_pose(10.0, 50.0);  // y far outside the [-1, 1] band of any lanelet
  target.goal_pose = make_pose(290.0);
  target.segments = {make_segment({1}), make_segment({2}), make_segment({3})};
  EXPECT_FALSE(check_reroute_safety(
    original, target, map_, 5.0, kRerouteTimeThreshold, kMinimumRerouteLength, test_logger()));
}

// ---------------------------------------------------------------------------------------------
// Final safety-length comparison branches.
// ---------------------------------------------------------------------------------------------

TEST_F(CheckRerouteSafetyTest, ReturnsTrueWhenAccumulatedLengthExceedsSafetyLength)
{
  // Ego at x=10 on lanelet 1; remaining length to the end of the shared route up to the goal at
  // x=290 is ~280 m, far above the safety length max(5 * 10, 30) = 50 m.
  const auto original = make_nominal_route();
  const auto target = make_nominal_route();
  EXPECT_TRUE(check_reroute_safety(
    original, target, map_, 5.0, kRerouteTimeThreshold, kMinimumRerouteLength, test_logger()));
}

TEST_F(CheckRerouteSafetyTest, ReturnsFalseWhenAccumulatedLengthBelowSafetyLength)
{
  // Single shared lanelet 1 (length 100). Ego at x=80 and goal at x=85 leave only ~5 m of shared
  // route ahead, which is below the safety length max(5 * 10, 30) = 50 m.
  LaneletRoute original;
  original.start_pose = make_pose(80.0);
  original.goal_pose = make_pose(85.0);
  original.segments = {make_segment({1})};

  LaneletRoute target;
  target.start_pose = make_pose(80.0);
  target.goal_pose = make_pose(85.0);
  target.segments = {make_segment({1})};

  EXPECT_FALSE(check_reroute_safety(
    original, target, map_, 5.0, kRerouteTimeThreshold, kMinimumRerouteLength, test_logger()));
}

TEST_F(CheckRerouteSafetyTest, SafetyLengthScalesWithVelocity)
{
  // Shared route: lanelets 1, 2, 3. Ego at x=10, goal at x=290, accumulated length ~280 m.
  // At velocity 30, safety length = max(30 * 10, 30) = 300 m > 280 m -> unsafe.
  // The same geometry at velocity 5 (safety length 50 m) was accepted by the test above, so this
  // pins the velocity dependence of the decision.
  const auto original = make_nominal_route();
  const auto target = make_nominal_route();
  EXPECT_FALSE(check_reroute_safety(
    original, target, map_, 30.0, kRerouteTimeThreshold, kMinimumRerouteLength, test_logger()));
}

// ---------------------------------------------------------------------------------------------
// Start-segment selection branch.
//
// The common-segment search returns the FIRST (original_idx, target_idx) pair whose primitives
// match. When that match is NOT at the front of the target route (start_idx_target != 0) and lands
// beyond the second original segment (start_idx_original > 1), the accumulation must start from the
// segment immediately BEFORE the common segment (start_idx_original - 1), because the front of the
// target route is then conflicting with original_route.segments[start_idx_original - 1]. This pins
// that `start_idx_original - 1` branch, which every preceding test leaves unexercised (they all
// build target.front() == original.front(), forcing start_idx_target == 0 and the `else` branch).
// ---------------------------------------------------------------------------------------------

TEST_F(CheckRerouteSafetyTest, AccumulatesFromSegmentBeforeCommonWhenTargetStartsMidRoute)
{
  // Add lanelet 4: a "conflicting" lane occupying the same x in [100, 200] region as lanelet 2. Ego
  // placed at x=190 is inside lanelet 4 (the target front) and inside lanelet 2, while sitting just
  // before the start of lanelet 3.
  map_->add(make_straight_lanelet(4, 100.0, 200.0));

  // original: {1}, {2}, {3}   target: {4}, {3}
  // The common-segment search (i over original, j over target) finds its FIRST match at original
  // {3}
  // == target {3}: i=2, j=1. Hence start_idx_original = 2 (> 1) and start_idx_target = 1 (!= 0), so
  // the `start_idx_original - 1` branch must select original segment 1 == lanelet 2 [100, 200] as
  // the start segment (the lane the target front conflicts with), not segment 2 == lanelet 3.
  LaneletRoute original;
  original.start_pose = make_pose(190.0);
  original.goal_pose = make_pose(290.0, 50.0);
  original.segments = {make_segment({1}), make_segment({2}), make_segment({3})};

  LaneletRoute target;
  target.start_pose = make_pose(190.0);       // inside lanelet 4 (target front) and lanelet 2
  target.goal_pose = make_pose(290.0, 50.0);  // y=50 lies outside every lanelet -> no goal subtract
  target.segments = {make_segment({4}), make_segment({3})};

  // The extension loop body never runs (end_idx == start_idx), and the goal lies outside lanelet 3,
  // so accumulated == the remaining arc length of the start segment alone:
  //   `-1` branch (correct): start = lanelet 2 [100, 200], ego x=190 -> remaining 10 m.
  //   regression to start_idx_original: start = lanelet 3 [200, 300], ego x=190 projects onto its
  //     start (clamped) -> remaining 100 m.
  // safety_length = max(5 * 10, 30) = 50. The correct branch (10 m) is below it -> false; the
  // regression (100 m) would be above it -> true. Asserting false is RED against a regression that
  // drops the `- 1` and always uses start_idx_original.
  EXPECT_FALSE(check_reroute_safety(
    original, target, map_, 5.0, kRerouteTimeThreshold, kMinimumRerouteLength, test_logger()));
}

// ---------------------------------------------------------------------------------------------
// Degenerate intermediate segment (empty primitives) inside the accumulation loop.
//
// The accumulation loop over (start_idx_original, end_idx_original] breaks on the first segment
// with no primitives. This pins that break: an empty intermediate segment stops accumulation
// cleanly (no crash, deterministic decision) instead of running std::min_element over an empty
// vector.
// ---------------------------------------------------------------------------------------------

TEST_F(CheckRerouteSafetyTest, StopsAccumulationOnEmptyIntermediateSegment)
{
  // original == target == {1}, {}(empty), {3}. The match is at the front (i=0, j=0), so the start
  // segment is lanelet 1 and the accumulation loop runs over segments 1 and 2. Segment 1 has empty
  // primitives, so the loop breaks immediately, leaving accumulated = start-segment length only.
  LaneletRoute route;
  route.start_pose = make_pose(10.0);  // inside lanelet 1 (route front)
  route.goal_pose = make_pose(250.0);  // inside lanelet 3 (terminal)
  route.segments = {make_segment({1}), LaneletSegment{}, make_segment({3})};

  // start segment = lanelet 1 [0, 100]; ego at x=10 leaves 90 m. The empty segment 1 breaks the
  // accumulation, so segment 3 is NOT added. Goal at x=250 is 50 m into lanelet 3: but the terminal
  // (end_idx_target == 2 -> lanelet 3) goal subtraction applies, accumulated = max(90 - 50, 0)
  // = 40. safety_length = max(5 * 10, 30) = 50; 40 <= 50 -> unsafe -> false, and the call does not
  // crash.
  EXPECT_FALSE(check_reroute_safety(
    route, route, map_, 5.0, kRerouteTimeThreshold, kMinimumRerouteLength, test_logger()));
}
