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
