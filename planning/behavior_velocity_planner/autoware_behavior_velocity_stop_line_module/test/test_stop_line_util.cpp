// Copyright 2025 Tier IV, Inc.
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

#include "../src/stop_line_util.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>

#include <autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/utility/Utilities.h>

#include <optional>
#include <vector>

namespace
{
using autoware::behavior_velocity_planner::stop_line_utils::advance_state;
using autoware::behavior_velocity_planner::stop_line_utils::compute_ego_and_stop_point;
using autoware::behavior_velocity_planner::stop_line_utils::has_intersection;
using autoware::behavior_velocity_planner::stop_line_utils::State;
using autoware::behavior_velocity_planner::stop_line_utils::StateTransition;
using autoware::behavior_velocity_planner::stop_line_utils::Trajectory;
using autoware_utils_geometry::create_point;

autoware_internal_planning_msgs::msg::PathPointWithLaneId make_path_point(
  const double x, const double y, const int64_t lane_id)
{
  autoware_internal_planning_msgs::msg::PathPointWithLaneId point;
  point.point.pose.position = create_point(x, y, 0.0);
  point.lane_ids = {lane_id};
  return point;
}

struct StopLineFixture
{
  StopLineFixture()
  {
    points = {
      make_path_point(0.0, 0.0, 0), make_path_point(1.0, 0.0, 0), make_path_point(2.0, 0.0, 0),
      make_path_point(3.0, 0.0, 0), make_path_point(4.0, 0.0, 0), make_path_point(5.0, 0.0, 0),
      make_path_point(6.0, 0.0, 0), make_path_point(7.0, 0.0, 0), make_path_point(8.0, 0.0, 0),
      make_path_point(9.0, 0.0, 0), make_path_point(10.0, 0.0, 0)};
    left_bound = {create_point(0.0, 1.0, 0.0), create_point(10.0, 1.0, 0.0)};
    right_bound = {create_point(0.0, -1.0, 0.0), create_point(10.0, -1.0, 0.0)};

    // Stop line crossing the path at x = 7.0.
    stop_line = lanelet::ConstLineString3d(
      lanelet::utils::getId(), {lanelet::Point3d(lanelet::utils::getId(), 7.0, -1.0, 0.0),
                                lanelet::Point3d(lanelet::utils::getId(), 7.0, 1.0, 0.0)});

    trajectory = *Trajectory::Builder{}.build(points);
  }

  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> points;
  std::vector<geometry_msgs::msg::Point> left_bound;
  std::vector<geometry_msgs::msg::Point> right_bound;
  lanelet::ConstLineString3d stop_line;
  Trajectory trajectory;
};

geometry_msgs::msg::Pose make_ego_pose(const double x, const double y)
{
  geometry_msgs::msg::Pose pose;
  pose.position = create_point(x, y, 0.0);
  return pose;
}
}  // namespace

// --- has_intersection -------------------------------------------------------

TEST(StopLineUtilHasIntersection, ReturnsTrueWhenShared)
{
  EXPECT_TRUE(has_intersection(lanelet::Ids{1, 2, 3}, std::vector<int64_t>{3, 4}));
  EXPECT_TRUE(has_intersection(lanelet::Ids{5}, std::vector<int64_t>{5}));
}

TEST(StopLineUtilHasIntersection, ReturnsFalseWhenDisjointOrEmpty)
{
  EXPECT_FALSE(has_intersection(lanelet::Ids{1, 2, 3}, std::vector<int64_t>{4, 5}));
  EXPECT_FALSE(has_intersection(lanelet::Ids{}, std::vector<int64_t>{1}));
  EXPECT_FALSE(has_intersection(lanelet::Ids{1}, std::vector<int64_t>{}));
}

// --- compute_ego_and_stop_point ---------------------------------------------

TEST(StopLineUtilComputeEgoAndStopPoint, ApproachPositiveStopPoint)
{
  const StopLineFixture f;
  const auto [ego_s, stop_point_s] = compute_ego_and_stop_point(
    f.trajectory, f.left_bound, f.right_bound, f.stop_line, make_ego_pose(5.0, 0.0),
    State::APPROACH, lanelet::Ids{0}, /*base_link2front=*/1.0, /*stop_margin=*/0.5);

  EXPECT_DOUBLE_EQ(ego_s, 5.0);
  ASSERT_TRUE(stop_point_s.has_value());
  EXPECT_DOUBLE_EQ(stop_point_s.value(), 7.0 - 0.5 - 1.0);
}

TEST(StopLineUtilComputeEgoAndStopPoint, ApproachNoIntersectionReturnsNullopt)
{
  const StopLineFixture f;
  // Connected lanelet ids do not overlap the path lane id (0), so the stop-line constraint
  // never matches and no intersection is found.
  const auto [ego_s, stop_point_s] = compute_ego_and_stop_point(
    f.trajectory, f.left_bound, f.right_bound, f.stop_line, make_ego_pose(5.0, 0.0),
    State::APPROACH, lanelet::Ids{42}, /*base_link2front=*/1.0, /*stop_margin=*/0.5);

  EXPECT_DOUBLE_EQ(ego_s, 5.0);
  EXPECT_FALSE(stop_point_s.has_value());
}

TEST(StopLineUtilComputeEgoAndStopPoint, ApproachNegativeStopPointReturnsNullopt)
{
  const StopLineFixture f;
  // base_link2front + stop_margin pushes the stop point behind ego (< 0).
  const auto [ego_s, stop_point_s] = compute_ego_and_stop_point(
    f.trajectory, f.left_bound, f.right_bound, f.stop_line, make_ego_pose(5.0, 0.0),
    State::APPROACH, lanelet::Ids{0}, /*base_link2front=*/100.0, /*stop_margin=*/0.5);

  EXPECT_DOUBLE_EQ(ego_s, 5.0);
  EXPECT_FALSE(stop_point_s.has_value());
}

TEST(StopLineUtilComputeEgoAndStopPoint, StoppedReturnsEgoS)
{
  const StopLineFixture f;
  const auto [ego_s, stop_point_s] = compute_ego_and_stop_point(
    f.trajectory, f.left_bound, f.right_bound, f.stop_line, make_ego_pose(5.0, 0.0), State::STOPPED,
    lanelet::Ids{0}, 1.0, 0.5);

  EXPECT_DOUBLE_EQ(ego_s, 5.0);
  ASSERT_TRUE(stop_point_s.has_value());
  EXPECT_DOUBLE_EQ(stop_point_s.value(), 5.0);
}

TEST(StopLineUtilComputeEgoAndStopPoint, StartReturnsNullopt)
{
  const StopLineFixture f;
  const auto [ego_s, stop_point_s] = compute_ego_and_stop_point(
    f.trajectory, f.left_bound, f.right_bound, f.stop_line, make_ego_pose(5.0, 0.0), State::START,
    lanelet::Ids{0}, 1.0, 0.5);

  EXPECT_DOUBLE_EQ(ego_s, 5.0);
  EXPECT_FALSE(stop_point_s.has_value());
}

// --- advance_state ----------------------------------------------------------

class AdvanceStateTest : public ::testing::Test
{
protected:
  static constexpr double kHoldStopMarginDistance = 0.5;
  static constexpr double kRequiredStopDurationSec = 2.0;
  rclcpp::Clock clock_{RCL_ROS_TIME};
};

TEST_F(AdvanceStateTest, ApproachToStopped)
{
  State state = State::APPROACH;
  std::optional<rclcpp::Time> stopped_time;
  const auto now = clock_.now();

  const auto result = advance_state(
    state, stopped_time, now, /*distance=*/0.1, /*is_stopped=*/true, kHoldStopMarginDistance,
    kRequiredStopDurationSec);

  EXPECT_EQ(result.transition, StateTransition::APPROACH_TO_STOPPED);
  EXPECT_EQ(state, State::STOPPED);
  ASSERT_TRUE(stopped_time.has_value());
  EXPECT_EQ(stopped_time.value(), now);
}

TEST_F(AdvanceStateTest, ApproachStaysWhenNotStopped)
{
  State state = State::APPROACH;
  std::optional<rclcpp::Time> stopped_time;

  const auto result = advance_state(
    state, stopped_time, clock_.now(), /*distance=*/0.1, /*is_stopped=*/false,
    kHoldStopMarginDistance, kRequiredStopDurationSec);

  EXPECT_EQ(result.transition, StateTransition::APPROACH_STAY);
  EXPECT_EQ(state, State::APPROACH);
  EXPECT_FALSE(stopped_time.has_value());
}

TEST_F(AdvanceStateTest, ApproachStaysWhenTooFar)
{
  State state = State::APPROACH;
  std::optional<rclcpp::Time> stopped_time;

  const auto result = advance_state(
    state, stopped_time, clock_.now(), /*distance=*/2.0, /*is_stopped=*/true,
    kHoldStopMarginDistance, kRequiredStopDurationSec);

  EXPECT_EQ(result.transition, StateTransition::APPROACH_STAY);
  EXPECT_EQ(state, State::APPROACH);
  EXPECT_FALSE(stopped_time.has_value());
}

TEST_F(AdvanceStateTest, StoppedTimeRecoveryWhenNoValue)
{
  State state = State::STOPPED;
  std::optional<rclcpp::Time> stopped_time;  // no value
  const auto now = clock_.now();

  const auto result = advance_state(
    state, stopped_time, now, /*distance=*/0.1, /*is_stopped=*/true, kHoldStopMarginDistance,
    kRequiredStopDurationSec);

  EXPECT_EQ(result.transition, StateTransition::STOPPED_TIME_RECOVERY);
  EXPECT_EQ(state, State::STOPPED);
  ASSERT_TRUE(stopped_time.has_value());
  EXPECT_EQ(stopped_time.value(), now);
}

TEST_F(AdvanceStateTest, StoppedStaysWhenDurationNotReached)
{
  State state = State::STOPPED;
  const auto start = clock_.now();
  std::optional<rclcpp::Time> stopped_time = start;

  const auto result = advance_state(
    state, stopped_time, start + rclcpp::Duration::from_seconds(1.0), /*distance=*/0.1,
    /*is_stopped=*/false, kHoldStopMarginDistance, kRequiredStopDurationSec);

  EXPECT_EQ(result.transition, StateTransition::STOPPED_STAY);
  EXPECT_EQ(state, State::STOPPED);
  ASSERT_TRUE(stopped_time.has_value());
  EXPECT_DOUBLE_EQ(result.stop_duration, 1.0);
}

TEST_F(AdvanceStateTest, StoppedToStartAfterDuration)
{
  State state = State::STOPPED;
  const auto start = clock_.now();
  std::optional<rclcpp::Time> stopped_time = start;

  const auto result = advance_state(
    state, stopped_time, start + rclcpp::Duration::from_seconds(3.0), /*distance=*/0.1,
    /*is_stopped=*/true, kHoldStopMarginDistance, kRequiredStopDurationSec);

  EXPECT_EQ(result.transition, StateTransition::STOPPED_TO_START);
  EXPECT_EQ(state, State::START);
  EXPECT_FALSE(stopped_time.has_value());
  EXPECT_DOUBLE_EQ(result.stop_duration, 3.0);
}

TEST_F(AdvanceStateTest, StartStaysStart)
{
  State state = State::START;
  std::optional<rclcpp::Time> stopped_time;

  const auto result = advance_state(
    state, stopped_time, clock_.now(), /*distance=*/0.1, /*is_stopped=*/true,
    kHoldStopMarginDistance, kRequiredStopDurationSec);

  EXPECT_EQ(result.transition, StateTransition::START);
  EXPECT_EQ(state, State::START);
}
