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

#include "../src/parameter_helper.hpp"
#include "../src/route_builder.hpp"
#include "../src/routing_state_machine.hpp"

#include <autoware_adapi_v1_msgs/srv/set_route_points.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::adapi_adaptors
{

using SetRoutePoints = autoware_adapi_v1_msgs::srv::SetRoutePoints;
using PoseStamped = geometry_msgs::msg::PoseStamped;

namespace
{

PoseStamped make_pose(const std::string & frame_id, double x)
{
  PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.pose.position.x = x;
  return pose;
}

}  // namespace

// --- vector_to_array ----------------------------------------------------------------------------

TEST(VectorToArray, CopiesWhenSizeMatches)
{
  std::vector<double> values(36);
  for (size_t i = 0; i < values.size(); ++i) {
    values[i] = static_cast<double>(i);
  }
  const auto array = vector_to_array<double, 36>(values);
  ASSERT_EQ(array.size(), 36u);
  EXPECT_DOUBLE_EQ(array[0], 0.0);
  EXPECT_DOUBLE_EQ(array[35], 35.0);
}

TEST(VectorToArray, ThrowsWhenTooSmall)
{
  const std::vector<double> values(35, 1.0);
  EXPECT_THROW((vector_to_array<double, 36>(values)), std::invalid_argument);
}

TEST(VectorToArray, ThrowsWhenTooLarge)
{
  const std::vector<double> values(37, 1.0);
  EXPECT_THROW((vector_to_array<double, 36>(values)), std::invalid_argument);
}

TEST(VectorToArray, ThrowsWhenEmpty)
{
  const std::vector<double> values;
  EXPECT_THROW((vector_to_array<double, 36>(values)), std::invalid_argument);
}

// --- decide_routing_action ----------------------------------------------------------------------

TEST(DecideRoutingAction, IdleCounterStaysIdleAndDoesNothing)
{
  // number_of_requests_for_timing_control_ == 0 means no pending request; nothing happens.
  const auto decision = decide_routing_action(0, false, true);
  EXPECT_EQ(decision.action, RoutingAction::None);
  EXPECT_EQ(decision.number_of_requests_for_timing_control, 0);
}

TEST(DecideRoutingAction, CounterIncrementsUntilDelayCount)
{
  // 1 -> 2: still within the merge window, no action yet.
  auto decision = decide_routing_action(1, false, true);
  EXPECT_EQ(decision.action, RoutingAction::None);
  EXPECT_EQ(decision.number_of_requests_for_timing_control, 2);

  // 2 -> 3 (== delay_count): fires the route call and resets the counter.
  decision = decide_routing_action(2, false, true);
  EXPECT_EQ(decision.action, RoutingAction::CallRoute);
  EXPECT_EQ(decision.number_of_requests_for_timing_control, 0);
}

TEST(DecideRoutingAction, ReachesDelayCountAndCallsRouteWhenUnset)
{
  // At delay_count with state UNSET: send the SetRoutePoints request, reset counter.
  const auto decision = decide_routing_action(g_routing_request_delay_count, false, true);
  EXPECT_EQ(decision.action, RoutingAction::CallRoute);
  EXPECT_EQ(decision.number_of_requests_for_timing_control, 0);
}

TEST(DecideRoutingAction, ReachesDelayCountAndCallsClearWhenNotUnset)
{
  // At delay_count with state not UNSET: must clear the existing route first.
  // The counter is NOT reset so the next tick retries until the route is cleared.
  const auto decision = decide_routing_action(g_routing_request_delay_count, false, false);
  EXPECT_EQ(decision.action, RoutingAction::CallClear);
  EXPECT_EQ(decision.number_of_requests_for_timing_control, g_routing_request_delay_count);
}

TEST(DecideRoutingAction, DoesNothingWhileServiceCallInFlight)
{
  // calling_service_ == true gates both the clear and route calls.
  auto decision = decide_routing_action(g_routing_request_delay_count, true, true);
  EXPECT_EQ(decision.action, RoutingAction::None);
  EXPECT_EQ(decision.number_of_requests_for_timing_control, g_routing_request_delay_count);

  decision = decide_routing_action(g_routing_request_delay_count, true, false);
  EXPECT_EQ(decision.action, RoutingAction::None);
  EXPECT_EQ(decision.number_of_requests_for_timing_control, g_routing_request_delay_count);
}

// --- set_goal -----------------------------------------------------------------------------------

TEST(SetGoal, CopiesHeaderGoalAndClearsWaypoints)
{
  SetRoutePoints::Request route;
  route.waypoints.resize(2);  // pre-existing waypoints must be cleared.
  const auto pose = make_pose("map", 1.5);

  set_goal(route, pose, false);

  EXPECT_EQ(route.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(route.goal.position.x, 1.5);
  EXPECT_TRUE(route.waypoints.empty());
  EXPECT_FALSE(route.option.allow_goal_modification);
}

TEST(SetGoal, SetsAllowGoalModificationFlag)
{
  SetRoutePoints::Request route;
  const auto pose = make_pose("map", 0.0);

  set_goal(route, pose, true);

  EXPECT_TRUE(route.option.allow_goal_modification);
}

// --- append_waypoint ----------------------------------------------------------------------------

TEST(AppendWaypoint, AppendsWhenFrameMatches)
{
  SetRoutePoints::Request route;
  set_goal(route, make_pose("map", 0.0), false);

  const bool appended = append_waypoint(route, make_pose("map", 2.0));

  EXPECT_TRUE(appended);
  ASSERT_EQ(route.waypoints.size(), 1u);
  EXPECT_DOUBLE_EQ(route.waypoints[0].position.x, 2.0);
}

TEST(AppendWaypoint, RejectsWhenFrameMismatchesAndLeavesRouteUnchanged)
{
  SetRoutePoints::Request route;
  set_goal(route, make_pose("map", 0.0), false);

  const bool appended = append_waypoint(route, make_pose("base_link", 2.0));

  EXPECT_FALSE(appended);
  EXPECT_TRUE(route.waypoints.empty());
}

}  // namespace autoware::adapi_adaptors

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
