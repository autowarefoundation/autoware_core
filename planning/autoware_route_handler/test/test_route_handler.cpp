// Copyright 2024 TIER IV
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

#include "test_route_handler.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <vector>

namespace autoware::route_handler::test
{
TEST_F(TestRouteHandler, isRouteHandlerReadyTest)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());
}

TEST_F(TestRouteHandler, checkIfIDReturned)
{
  const auto lanelet1 = route_handler_->getLaneletsFromId(4785);
  const auto is_lanelet1_in_goal_route_section = route_handler_->isInGoalRouteSection(lanelet1);
  ASSERT_TRUE(is_lanelet1_in_goal_route_section);

  const auto lanelet2 = route_handler_->getLaneletsFromId(4780);
  const auto is_lanelet2_in_goal_route_section = route_handler_->isInGoalRouteSection(lanelet2);
  ASSERT_FALSE(is_lanelet2_in_goal_route_section);
}

TEST_F(TestRouteHandler, getGoalLaneId)
{
  lanelet::ConstLanelet goal_lane;

  const auto goal_lane_obtained = route_handler_->getGoalLanelet(&goal_lane);
  ASSERT_TRUE(goal_lane_obtained);
  ASSERT_EQ(goal_lane.id(), 5088);
}

TEST_F(TestRouteHandler, getLaneletSequenceWhenOverlappingRoute)
{
  set_route_handler("overlap_map.osm");
  ASSERT_FALSE(route_handler_->isHandlerReady());

  geometry_msgs::msg::Pose start_pose;
  geometry_msgs::msg::Pose goal_pose;
  start_pose.position = autoware_utils_geometry::create_point(3728.870361, 73739.281250, 0);
  start_pose.orientation = autoware_utils_geometry::create_quaternion(0, 0, -0.513117, 0.858319);
  goal_pose.position = autoware_utils_geometry::create_point(3729.961182, 73727.328125, 0);
  goal_pose.orientation = autoware_utils_geometry::create_quaternion(0, 0, 0.234831, 0.972036);

  lanelet::ConstLanelets path_lanelets;
  ASSERT_TRUE(
    route_handler_->planPathLaneletsBetweenCheckpoints(start_pose, goal_pose, &path_lanelets));
  ASSERT_EQ(path_lanelets.size(), 12);
  ASSERT_EQ(path_lanelets.front().id(), 168);
  ASSERT_EQ(path_lanelets.back().id(), 345);

  route_handler_->setRouteLanelets(path_lanelets);
  ASSERT_TRUE(route_handler_->isHandlerReady());

  auto lanelet_sequence = route_handler_->getLaneletSequence(path_lanelets.back());
  ASSERT_EQ(lanelet_sequence.size(), 12);
  ASSERT_EQ(lanelet_sequence.front().id(), 168);
  ASSERT_EQ(lanelet_sequence.back().id(), 345);
}

TEST_F(TestRouteHandler, getClosestRouteLaneletFromLaneletWhenOverlappingRoute)
{
  set_route_handler("overlap_map.osm");
  set_test_route("overlap_test_route.yaml");
  ASSERT_TRUE(route_handler_->isHandlerReady());

  geometry_msgs::msg::Pose reference_pose;
  geometry_msgs::msg::Pose search_pose;

  lanelet::ConstLanelet reference_lanelet;
  reference_pose.position = autoware_utils_geometry::create_point(3730.88, 73735.3, 0);
  reference_pose.orientation =
    autoware_utils_geometry::create_quaternion(0, 0, -0.504626, 0.863338);
  const auto found_reference_lanelet =
    route_handler_->getClosestLaneletWithinRoute(reference_pose, &reference_lanelet);
  ASSERT_TRUE(found_reference_lanelet);
  EXPECT_EQ(reference_lanelet.id(), 168);

  lanelet::ConstLanelet closest_lanelet;
  search_pose.position = autoware_utils_geometry::create_point(3736.89, 73730.8, 0);
  search_pose.orientation = autoware_utils_geometry::create_quaternion(0, 0, 0.223244, 0.974763);
  bool found_lanelet = route_handler_->getClosestLaneletWithinRoute(search_pose, &closest_lanelet);
  ASSERT_TRUE(found_lanelet);
  EXPECT_EQ(closest_lanelet.id(), 277);

  found_lanelet = route_handler_->getClosestRouteLaneletFromLanelet(
    search_pose, reference_lanelet, &closest_lanelet, dist_threshold, yaw_threshold);
  ASSERT_TRUE(found_lanelet);
  EXPECT_EQ(closest_lanelet.id(), 277);
}

TEST_F(TestRouteHandler, CheckLaneIsInGoalRouteSection)
{
  const auto lane = route_handler_->getLaneletsFromId(4785);
  const auto is_lane_in_goal_route_section = route_handler_->isInGoalRouteSection(lane);
  ASSERT_TRUE(is_lane_in_goal_route_section);
}

TEST_F(TestRouteHandler, CheckLaneIsNotInGoalRouteSection)
{
  const auto lane = route_handler_->getLaneletsFromId(4780);
  const auto is_lane_in_goal_route_section = route_handler_->isInGoalRouteSection(lane);
  ASSERT_FALSE(is_lane_in_goal_route_section);
}

TEST_F(TestRouteHandler, checkGetLaneletSequence)
{
  const auto current_pose = autoware::test_utils::createPose(-50.0, 1.75, 0.0, 0.0, 0.0, 0.0);

  lanelet::ConstLanelet closest_lanelet;
  const auto found_closest_lanelet = route_handler_->getClosestLaneletWithConstrainsWithinRoute(
    current_pose, &closest_lanelet, dist_threshold, yaw_threshold);
  ASSERT_TRUE(found_closest_lanelet);
  ASSERT_EQ(closest_lanelet.id(), 4765ul);

  const auto current_lanes = route_handler_->getLaneletSequence(
    closest_lanelet, current_pose, backward_path_length, forward_path_length);

  ASSERT_EQ(current_lanes.size(), 6ul);
  ASSERT_EQ(current_lanes.at(0).id(), 4765ul);
  ASSERT_EQ(current_lanes.at(1).id(), 4770ul);
  ASSERT_EQ(current_lanes.at(2).id(), 4775ul);
  ASSERT_EQ(current_lanes.at(3).id(), 4424ul);
  ASSERT_EQ(current_lanes.at(4).id(), 4780ul);
  ASSERT_EQ(current_lanes.at(5).id(), 4785ul);
}

TEST_F(TestRouteHandler, checkLateralIntervalToPreferredLaneWhenLaneChangeToRight)
{
  const auto current_lanes = get_current_lanes();

  // this lane is of preferred lane type
  std::for_each(current_lanes.begin(), current_lanes.begin() + 3, [&](const auto & lane) {
    const auto result = route_handler_->getLateralIntervalsToPreferredLane(lane, Direction::RIGHT);
    ASSERT_EQ(result.size(), 0ul);
  });

  // this alternative lane is a subset of preferred lane route section
  std::for_each(current_lanes.begin() + 3, current_lanes.end(), [&](const auto & lane) {
    const auto result = route_handler_->getLateralIntervalsToPreferredLane(lane, Direction::RIGHT);
    ASSERT_EQ(result.size(), 1ul);
    EXPECT_DOUBLE_EQ(result.at(0), -3.5);
  });

  // Although Direction::NONE, the function should still return result similar to
  std::for_each(current_lanes.begin(), current_lanes.begin() + 3, [&](const auto & lane) {
    const auto result = route_handler_->getLateralIntervalsToPreferredLane(lane, Direction::NONE);
    ASSERT_EQ(result.size(), 0ul);
  });

  // Although Direction::NONE is provided, the function should behave similarly to
  std::for_each(current_lanes.begin() + 3, current_lanes.end(), [&](const auto & lane) {
    const auto result = route_handler_->getLateralIntervalsToPreferredLane(lane, Direction::NONE);
    ASSERT_EQ(result.size(), 1ul);
    EXPECT_DOUBLE_EQ(result.at(0), -3.5);
  });
}

TEST_F(TestRouteHandler, checkLateralIntervalToPreferredLaneUsingUnexpectedResults)
{
  const auto current_lanes = get_current_lanes();

  std::for_each(current_lanes.begin(), current_lanes.end(), [&](const auto & lane) {
    const auto result = route_handler_->getLateralIntervalsToPreferredLane(lane, Direction::LEFT);
    ASSERT_EQ(result.size(), 0ul);
  });
}

TEST_F(TestRouteHandler, testGetCenterLinePath)
{
  const auto current_lanes = route_handler_->getLaneletsFromIds({4424, 4780, 4785});
  {
    const auto center_line_path = route_handler_->getCenterLinePath(current_lanes, 0.0, 50.0);
    ASSERT_EQ(center_line_path.points.size(), 51);  // 26 + 26 - 1(overlapped)
    ASSERT_EQ(center_line_path.points.back().lane_ids.size(), 2);
    ASSERT_EQ(center_line_path.points.back().lane_ids.at(0), 4780);
    ASSERT_EQ(center_line_path.points.back().lane_ids.at(1), 4785);
  }
  {
    const auto center_line_path = route_handler_->getCenterLinePath(current_lanes, 14.5, 60.5);
    ASSERT_EQ(center_line_path.points.size(), 48);
    ASSERT_EQ(center_line_path.points.front().lane_ids.size(), 1);
    ASSERT_EQ(center_line_path.points.back().lane_ids.size(), 1);
    ASSERT_EQ(center_line_path.points.front().lane_ids.at(0), 4424);
    ASSERT_EQ(center_line_path.points.back().lane_ids.at(0), 4785);
  }
  {
    const auto center_line_path = route_handler_->getCenterLinePath(current_lanes, -1.0, 200.0);
    ASSERT_EQ(center_line_path.points.size(), 76);  // 26 + 26 + 26 - 2(overlapped)
    ASSERT_EQ(center_line_path.points.front().lane_ids.size(), 1);
    ASSERT_EQ(center_line_path.points.front().lane_ids.at(0), 4424);
    ASSERT_EQ(center_line_path.points.back().lane_ids.size(), 1);
    ASSERT_EQ(center_line_path.points.back().lane_ids.at(0), 4785);
  }
}
TEST_F(TestRouteHandler, DISABLED_testGetCenterLinePathWhenLanesIsNotConnected)
{
  const auto current_lanes = route_handler_->getLaneletsFromIds({4424, 4780, 4785});

  const auto center_line_path = route_handler_->getCenterLinePath(current_lanes, 0.0, 75.0);
  ASSERT_EQ(center_line_path.points.size(), 26);  // 26 + 26 + 26 - 2(overlapped)
  ASSERT_EQ(center_line_path.points.front().lane_ids.size(), 1);
  ASSERT_EQ(center_line_path.points.front().lane_ids.at(0), 4424);
  ASSERT_EQ(center_line_path.points.back().lane_ids.size(), 1);
  ASSERT_EQ(center_line_path.points.back().lane_ids.at(0), 4424);
}

TEST_F(TestRouteHandler, getClosestLaneletWithinRouteWhenPointsInRoute)
{
  auto get_closest_lanelet_within_route =
    [&](double x, double y, double z) -> std::optional<lanelet::Id> {
    const auto pose = autoware::test_utils::createPose(x, y, z, 0.0, 0.0, 0.0);
    lanelet::ConstLanelet closest_lanelet;
    const auto closest_lane_obtained =
      route_handler_->getClosestLaneletWithinRoute(pose, &closest_lanelet);
    if (!closest_lane_obtained) {
      return std::nullopt;
    }
    return closest_lanelet.id();
  };

  ASSERT_TRUE(get_closest_lanelet_within_route(-0.5, 1.75, 0).has_value());
  ASSERT_EQ(get_closest_lanelet_within_route(-0.5, 1.75, 0).value(), 4775ul);

  ASSERT_TRUE(get_closest_lanelet_within_route(-0.01, 1.75, 0).has_value());
  ASSERT_EQ(get_closest_lanelet_within_route(-0.01, 1.75, 0).value(), 4775ul);

  ASSERT_TRUE(get_closest_lanelet_within_route(0.0, 1.75, 0).has_value());
  ASSERT_EQ(get_closest_lanelet_within_route(0.0, 1.75, 0).value(), 4775ul);

  ASSERT_TRUE(get_closest_lanelet_within_route(0.01, 1.75, 0).has_value());
  ASSERT_EQ(get_closest_lanelet_within_route(0.01, 1.75, 0).value(), 4424ul);

  ASSERT_TRUE(get_closest_lanelet_within_route(0.5, 1.75, 0).has_value());
  ASSERT_EQ(get_closest_lanelet_within_route(0.5, 1.75, 0).value(), 4424ul);
}

TEST_F(TestRouteHandler, testGetLaneChangeTargetLanes)
{
  {
    const auto current_lanes = route_handler_->getLaneletsFromIds({4770, 4775});
    const auto lane_change_lane =
      route_handler_->getLaneChangeTarget(current_lanes, Direction::RIGHT);
    ASSERT_FALSE(lane_change_lane.has_value());
  }

  {
    const auto current_lanes = route_handler_->getLaneletsFromIds({4775, 4424});
    const auto lane_change_lane =
      route_handler_->getLaneChangeTarget(current_lanes, Direction::RIGHT);
    EXPECT_TRUE(lane_change_lane.has_value());
    ASSERT_EQ(lane_change_lane.value().id(), 9598ul);
  }

  {
    // There is a lane-changing lane. Within the maximum current lanes, there is an alternative lane
    const auto current_lanes = get_current_lanes();
    const auto lane_change_lane = route_handler_->getLaneChangeTarget(current_lanes);
    ASSERT_TRUE(lane_change_lane.has_value());
    ASSERT_EQ(lane_change_lane.value().id(), 9598ul);
  }
}

TEST_F(TestRouteHandler, testGetShoulderLaneletsAtPose)
{
  set_route_handler("overlap_map.osm");

  geometry_msgs::msg::Pose pose;
  pose.position.x = 3719.5;
  pose.position.y = 73765.6;
  auto shoulder_lanelets = route_handler_->getShoulderLaneletsAtPose(pose);
  ASSERT_FALSE(shoulder_lanelets.empty());
  ASSERT_EQ(shoulder_lanelets.front().id(), 359ul);

  pose.position.y = 73768.6;
  shoulder_lanelets = route_handler_->getShoulderLaneletsAtPose(pose);
  ASSERT_TRUE(shoulder_lanelets.empty());
}

// In overlap_map.osm the lanelets 282 -> 287 -> ... -> 345 -> 282 form a cycle. A route that
// contains the whole cycle without using it as its start or goal (a full lap of a loop course)
// makes the forward traversal come back to an already visited lanelet. It must stop there, so that
TEST_F(TestRouteHandler, getLaneletSequenceVisitsEachLaneletAtMostOnceOnCyclicRoute)
{
  set_route_handler("overlap_map.osm");

  const lanelet::Ids expected_ids{277, 282, 287, 292, 297, 302, 307, 312, 317, 322, 345};
  autoware_planning_msgs::msg::LaneletRoute route;
  for (const auto & id : lanelet::Ids{277, 282, 287, 292, 297, 302, 307, 312, 317, 322, 345, 168}) {
    autoware_planning_msgs::msg::LaneletPrimitive primitive;
    primitive.id = id;
    primitive.primitive_type = "lane";
    autoware_planning_msgs::msg::LaneletSegment segment;
    segment.preferred_primitive = primitive;
    segment.primitives.push_back(primitive);
    route.segments.push_back(segment);
  }
  route_handler_->setRoute(route);
  ASSERT_TRUE(route_handler_->isHandlerReady());

  constexpr double forward_distance_longer_than_the_cycle = 1.0e6;
  const auto lanelet_sequence = route_handler_->getLaneletSequence(
    route_handler_->getLaneletsFromId(277), 0.0, forward_distance_longer_than_the_cycle);

  lanelet::Ids ids;
  ids.reserve(lanelet_sequence.size());
  for (const auto & lanelet : lanelet_sequence) ids.push_back(lanelet.id());
  EXPECT_EQ(ids, expected_ids);
}

// The only in-route predecessor of 282 is the goal lanelet 345, which is skipped, so the backward
TEST_F(TestRouteHandler, getLaneletSequenceStopsWhenNoNewPreviousLaneletIsFound)
{
  set_route_handler("overlap_map.osm");
  const auto route_lanelets = route_handler_->getLaneletsFromIds({287, 282, 345});
  route_handler_->setRouteLanelets(route_lanelets);

  const auto lanelet_sequence =
    route_handler_->getLaneletSequence(route_handler_->getLaneletsFromId(282), 100.0, 0.0);

  ASSERT_EQ(lanelet_sequence.size(), 1ul);
  EXPECT_EQ(lanelet_sequence.front().id(), 282ul);

  // Sanity check that traversal above stopped because its only candidate was goal lanelet:
  route_handler_->setRouteLanelets(route_handler_->getLaneletsFromIds({287, 277, 282, 345}));
  const auto sequence_with_predecessor =
    route_handler_->getLaneletSequence(route_handler_->getLaneletsFromId(282), 100.0, 0.0);
  EXPECT_GT(sequence_with_predecessor.size(), 1ul);
}

// - Validates legacy logic's ability to calculate metric distances (-3.5m)
// - Ensures that requesting a lane change target from  a set of preferred lanes
//   tests for ManeuverTargetingAndIntervals
TEST_F(
  TestRouteHandler, getLateralIntervalsToPreferredLaneReturnsExpectedIntervalsWhenLaneChangeToRight)
{
  const auto current_lanes = get_current_lanes();

  const auto intervals =
    route_handler_->getLateralIntervalsToPreferredLane(current_lanes.back(), Direction::RIGHT);
  ASSERT_EQ(intervals.size(), 1UL);
  EXPECT_DOUBLE_EQ(intervals.front(), -3.5);
}

// Expects getting lane change target returns nullopt when targeting preferred lanes.
TEST_F(TestRouteHandler, getLaneChangeTargetReturnsNulloptWhenTargetingPreferredLanes)
{
  const auto safe_lanes = route_handler_->getLaneletsFromIds({4770, 4775});
  const auto target_lane = route_handler_->getLaneChangeTarget(safe_lanes, Direction::RIGHT);
  EXPECT_FALSE(target_lane.has_value());
}

// Bicycle lanes and opposite lanes
TEST_F(TestRouteHandler, getLeftBicycleLaneletReturnsNulloptOnStandardMap)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());
  const auto ref_lane = route_handler_->getLaneletsFromId(4765);

  const auto left_bike = route_handler_->getLeftBicycleLanelet(ref_lane);
  EXPECT_FALSE(left_bike.has_value());
}

// Expects getting right bicycle lanelet returns nullopt on standard map.
TEST_F(TestRouteHandler, getRightBicycleLaneletReturnsNulloptOnStandardMap)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());
  const auto ref_lane = route_handler_->getLaneletsFromId(4765);

  const auto right_bike = route_handler_->getRightBicycleLanelet(ref_lane);
  EXPECT_FALSE(right_bike.has_value());
}

// Expects getting left opposite lanelets returns empty on standard map.
TEST_F(TestRouteHandler, getLeftOppositeLaneletsReturnsEmptyOnStandardMap)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());
  const auto ref_lane = route_handler_->getLaneletsFromId(4765);

  const auto left_opp = route_handler_->getLeftOppositeLanelets(ref_lane);
  EXPECT_TRUE(left_opp.empty());
}

// Expects getting right opposite lanelets returns empty on standard map.
TEST_F(TestRouteHandler, getRightOppositeLaneletsReturnsEmptyOnStandardMap)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());
  const auto ref_lane = route_handler_->getLaneletsFromId(4765);

  const auto right_opp = route_handler_->getRightOppositeLanelets(ref_lane);
  EXPECT_TRUE(right_opp.empty());
}

// Maneuver specifics
TEST_F(TestRouteHandler, getPullOverTargetReturnsNulloptOnContinuousLanes)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());

  const auto goal_pose = route_handler_->getGoalPose();
  const auto pull_over_target = route_handler_->getPullOverTarget(goal_pose);
  EXPECT_FALSE(pull_over_target.has_value());
}

// Expects getting pull out start lane returns nullopt on continuous lanes.
TEST_F(TestRouteHandler, getPullOutStartLaneReturnsNulloptOnContinuousLanes)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());

  const auto start_pose = route_handler_->getStartPose();
  const auto pull_out_start = route_handler_->getPullOutStartLane(start_pose, 2.0);
  EXPECT_FALSE(pull_out_start.has_value());
}

// Is dead end lanelet returns false on continuous lanes.
TEST_F(TestRouteHandler, isDeadEndLaneletReturnsFalseOnContinuousLanes)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());

  const auto ref_lane = route_handler_->getLaneletsFromId(4765);
  EXPECT_FALSE(route_handler_->isDeadEndLanelet(ref_lane));
}

/*
 * Coverage for findDrivableLanePathIncludingAreas using custom-built map
 * Illustration of this map with parallel drivable and non-drivable lanes
 *
 *       y (m)
 *
 *  30 ┤  exit_lane (id=300)
 *     │  ↑
 *  25 ┼  │   G (0.5, 25.0)
 *     │  │
 *  20 ┤──┴──────────────────
 *     │ drivable │ non-drivable
 *     │ (id=201) │ (id=200)
 *     │  ↑       │  ↑ (preferred but "no_drivable_lane")
 *  10 ┤──┴───────┴──────────
 *     │  entry_lane (id=100)
 *     │  ↑
 *   5 ┼  │   S (0.5, 5.0)
 *     │  │
 *   0 ┼─────────────────── x (m)
 *       -2       0    1
 */
TEST_F(
  TestRouteHandler,
  findDrivableLanePathIncludingAreasFindsFallbackPathWhenShortestPathContainsNonDrivableLane)
{
  // Entry lanelet
  const lanelet::Point3d entry_left_0(1, 0.0, 0.0, 0.0);
  const lanelet::Point3d entry_left_1(2, 0.0, 10.0, 0.0);
  const lanelet::Point3d entry_right_0(3, 1.0, 0.0, 0.0);
  const lanelet::Point3d entry_right_1(4, 1.0, 10.0, 0.0);
  const lanelet::LineString3d entry_left(10, {entry_left_0, entry_left_1});
  const lanelet::LineString3d entry_right(11, {entry_right_0, entry_right_1});
  lanelet::Lanelet entry_lane(100, entry_left, entry_right);
  entry_lane.attributes()["subtype"] = "road";

  // Exit lanelet
  const lanelet::Point3d exit_left_0(5, 0.0, 20.0, 0.0);
  const lanelet::Point3d exit_left_1(6, 0.0, 30.0, 0.0);
  const lanelet::Point3d exit_right_0(7, 1.0, 20.0, 0.0);
  const lanelet::Point3d exit_right_1(8, 1.0, 30.0, 0.0);
  const lanelet::LineString3d exit_left(12, {exit_left_0, exit_left_1});
  const lanelet::LineString3d exit_right(13, {exit_right_0, exit_right_1});
  lanelet::Lanelet exit_lane(300, exit_left, exit_right);
  exit_lane.attributes()["subtype"] = "road";

  // Middle lanelets (left drivable, right non-drivable)
  const lanelet::Point3d mid_left_0(9, 0.0, 10.0, 0.0);
  const lanelet::Point3d mid_left_1(10, 0.0, 20.0, 0.0);
  const lanelet::Point3d mid_right_0(11, 1.0, 10.0, 0.0);
  const lanelet::Point3d mid_right_1(12, 1.0, 20.0, 0.0);
  const lanelet::LineString3d mid_left(14, {entry_left_1, mid_left_0, mid_left_1, exit_left_0});
  const lanelet::LineString3d mid_right(
    15, {entry_right_1, mid_right_0, mid_right_1, exit_right_0});

  lanelet::Lanelet non_drivable_lane(200, mid_left, mid_right);
  non_drivable_lane.attributes()["subtype"] = "road";
  non_drivable_lane.attributes()["no_drivable_lane"] = "yes";

  const lanelet::Point3d alt_left_0(13, -2.0, 10.0, 0.0);
  const lanelet::Point3d alt_left_1(14, -2.0, 20.0, 0.0);
  const lanelet::Point3d alt_right_0(15, -1.0, 10.0, 0.0);
  const lanelet::Point3d alt_right_1(16, -1.0, 20.0, 0.0);
  const lanelet::LineString3d alt_left(16, {entry_left_1, alt_left_0, alt_left_1, exit_left_0});
  const lanelet::LineString3d alt_right(
    17, {entry_right_1, alt_right_0, alt_right_1, exit_right_0});

  lanelet::Lanelet drivable_lane(201, alt_left, alt_right);
  drivable_lane.attributes()["subtype"] = "road";

  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(entry_lane);
  map->add(non_drivable_lane);
  map->add(drivable_lane);
  map->add(exit_lane);

  const auto map_bin = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>(map_bin);

  autoware_planning_msgs::msg::LaneletRoute route;
  route.header.frame_id = "map";
  route.start_pose = autoware::test_utils::createPose(0.5, 5.0, 0.0, 0.0, 0.0, 0.0);
  route.goal_pose = autoware::test_utils::createPose(0.5, 25.0, 0.0, 0.0, 0.0, 0.0);

  autoware_planning_msgs::msg::LaneletPrimitive primitive1;
  primitive1.id = 100;
  primitive1.primitive_type = "lane";
  autoware_planning_msgs::msg::LaneletSegment segment1;
  segment1.preferred_primitive = primitive1;
  segment1.primitives.push_back(primitive1);

  autoware_planning_msgs::msg::LaneletPrimitive primitive2;
  primitive2.id = 200;
  primitive2.primitive_type = "lane";
  autoware_planning_msgs::msg::LaneletSegment segment2;
  segment2.preferred_primitive = primitive2;
  segment2.primitives.push_back(primitive2);

  autoware_planning_msgs::msg::LaneletPrimitive primitive3;
  primitive3.id = 300;
  primitive3.primitive_type = "lane";
  autoware_planning_msgs::msg::LaneletSegment segment3;
  segment3.preferred_primitive = primitive3;
  segment3.primitives.push_back(primitive3);

  route.segments.push_back(segment1);
  route.segments.push_back(segment2);
  route.segments.push_back(segment3);

  route_handler->setRoute(route);
  ASSERT_TRUE(route_handler->isHandlerReady());

  lanelet::ConstLanelets path_lanelets;
  const auto success = route_handler->planPathLaneletsBetweenCheckpoints(
    route.start_pose, route.goal_pose, &path_lanelets, true);

  ASSERT_TRUE(success);
  EXPECT_FALSE(path_lanelets.empty());

  bool used_drivable_lane = false;
  for (const auto & lane : path_lanelets) {
    if (lane.id() == 201) used_drivable_lane = true;
  }
  EXPECT_TRUE(used_drivable_lane);
}

// Expects test clear route to reset handler readiness when called.
TEST_F(TestRouteHandler, clearRouteResetsHandlerReadinessWhenCalled)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());

  route_handler_->clearRoute();
  EXPECT_FALSE(route_handler_->isHandlerReady());
}

// Expects getting area from id throws exception when id is invalid.
TEST_F(TestRouteHandler, getAreaFromIdThrowsExceptionWhenIdIsInvalid)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());

  EXPECT_THROW(route_handler_->getAreaFromId(9999999), lanelet::NoSuchPrimitiveError);
}

// Expects getting routing graph ptr returns valid pointer when ready.
TEST_F(TestRouteHandler, getRoutingGraphPtrReturnsValidPointerWhenReady)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());

  EXPECT_NE(route_handler_->getRoutingGraphPtr(), nullptr);
  EXPECT_NE(route_handler_->getTrafficRulesPtr(), nullptr);
  EXPECT_NE(route_handler_->getOverallGraphPtr(), nullptr);
  EXPECT_NE(route_handler_->getLaneletMapPtr(), nullptr);
}

// Expects getting pose from 2d arc length returns expected pose.
TEST_F(TestRouteHandler, getPoseFrom2dArcLengthReturnsExpectedPose)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());

  const auto ref_lane = route_handler_->getLaneletsFromId(4765);
  lanelet::ConstLanelets lane_vector = {ref_lane};

  auto pose = route_handler_->get_pose_from_2d_arc_length(lane_vector, 5.0);
  EXPECT_NEAR(pose.position.x, -70.0, 1e-1);
}

// Expects routing cost drivable calculates cost succeeding and cost lane change.
TEST_F(TestRouteHandler, routingCostDrivableCalculatesCostSucceedingAndCostLaneChange)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());

  const auto ref_lane = route_handler_->getLaneletsFromId(4765);
  lanelet::ConstLanelets lane_vector = {ref_lane};

  autoware::route_handler::RoutingCostDrivable cost;
  auto tr = route_handler_->getTrafficRulesPtr();

  double succeeding_cost = cost.getCostSucceeding(*tr, ref_lane, ref_lane);
  EXPECT_GT(succeeding_cost, 0.0);

  double lane_change_cost = cost.getCostLaneChange(*tr, lane_vector, lane_vector);
  EXPECT_GT(lane_change_cost, 0.0);
}

// Expects starting road lanelets for checkpoint filters non road lanelets when start pose is
// outside lanelets.
TEST_F(
  TestRouteHandler,
  getStartRoadLaneletsForCheckpointFiltersNonRoadLaneletsWhenStartPoseIsOutsideLanelets)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());

  auto start_pose = route_handler_->getStartPose();
  const auto goal_pose = route_handler_->getGoalPose();

  start_pose.position.x += 10.0;
  start_pose.position.y -= 10.0;

  lanelet::ConstLanelets path_lanelets;
  const auto success = route_handler_->planPathLaneletsBetweenCheckpoints(
    start_pose, goal_pose, &path_lanelets, false);

  EXPECT_TRUE(success);
  ASSERT_FALSE(path_lanelets.empty());
  EXPECT_EQ(path_lanelets.front().id(), 5072);
}

// Test create map segments from lanelet or area path returns valid segment when given area.
TEST_F(TestRouteHandler, createMapSegmentsFromLaneletOrAreaPathReturnsValidSegmentWhenGivenArea)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());

  lanelet::Point3d p1(lanelet::utils::getId(), 0, 0, 0);
  lanelet::Point3d p2(lanelet::utils::getId(), 1, 0, 0);
  lanelet::Point3d p3(lanelet::utils::getId(), 1, 1, 0);
  lanelet::LineString3d ls1(lanelet::utils::getId(), {p1, p2, p3});
  lanelet::Area area(lanelet::utils::getId(), {ls1});
  lanelet::ConstArea const_area = area;

  std::vector<lanelet::ConstLaneletOrArea> path_areas;
  path_areas.emplace_back(const_area);

  auto segments = route_handler_->createMapSegmentsFromLaneletOrAreaPath(path_areas);
  ASSERT_FALSE(segments.empty());
  EXPECT_EQ(segments.front().preferred_primitive.primitive_type, "area");
}

// Expects getting shoulder lanelet sequence returns expected sequence when on shoulder lane.
TEST_F(TestRouteHandler, getShoulderLaneletSequenceReturnsExpectedSequenceWhenOnShoulderLane)
{
  set_route_handler("overlap_map.osm");

  geometry_msgs::msg::Pose pose;
  pose.position.x = 3719.5;
  pose.position.y = 73765.6;
  const auto shoulder_lanelets = route_handler_->getShoulderLaneletsAtPose(pose);
  ASSERT_FALSE(shoulder_lanelets.empty());

  const auto & shoulder_lane = shoulder_lanelets.front();

  auto seq = route_handler_->getShoulderLaneletSequence(shoulder_lane, pose, 10.0, 10.0);
  ASSERT_FALSE(seq.empty());
  EXPECT_EQ(seq.front().id(), 359);
}

// Expects getting right lanelet returns expected lanelet when neighbor exists.
TEST_F(TestRouteHandler, getRightLaneletReturnsExpectedLaneletWhenNeighborExists)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());

  const auto ref_lane = route_handler_->getLaneletsFromId(4765);
  const auto right_lane = route_handler_->getRightLanelet(ref_lane, false, false);
  ASSERT_TRUE(right_lane.has_value());
  EXPECT_EQ(right_lane->id(), 9590);
}

// Expects getting most left and right lanelet returns extremity lanes.
TEST_F(TestRouteHandler, getMostLeftAndRightLaneletReturnsExtremityLanes)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());

  const auto ref_lane = route_handler_->getLaneletsFromId(4765);
  EXPECT_EQ(route_handler_->getMostLeftLanelet(ref_lane, false, false).id(), 4765);
  EXPECT_EQ(route_handler_->getMostRightLanelet(ref_lane, false, false).id(), 9590);
}

// Is route lanelet returns true for route lanes.
TEST_F(TestRouteHandler, isRouteLaneletReturnsTrueForRouteLanes)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());

  const auto ref_lane = route_handler_->getLaneletsFromId(4765);
  EXPECT_TRUE(route_handler_->isRouteLanelet(ref_lane));
}

// Expects getting all left shared linestring lanelets returns valid lanelets on standard map.
TEST_F(TestRouteHandler, getAllLeftSharedLinestringLaneletsReturnsValidLaneletsOnStandardMap)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());
  const auto ref_lane = route_handler_->getLaneletsFromId(4765);
  auto left_shared_include =
    route_handler_->getAllLeftSharedLinestringLanelets(ref_lane, true, true);
  auto left_shared_exclude =
    route_handler_->getAllLeftSharedLinestringLanelets(ref_lane, false, false);
  EXPECT_TRUE(left_shared_include.empty());
  EXPECT_TRUE(left_shared_exclude.empty());
}

// Expects getting all right shared linestring lanelets returns valid lanelets on standard map.
TEST_F(TestRouteHandler, getAllRightSharedLinestringLaneletsReturnsValidLaneletsOnStandardMap)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());
  const auto ref_lane = route_handler_->getLaneletsFromId(4765);
  auto right_shared_include =
    route_handler_->getAllRightSharedLinestringLanelets(ref_lane, true, true);
  auto right_shared_exclude =
    route_handler_->getAllRightSharedLinestringLanelets(ref_lane, false, false);
  ASSERT_FALSE(right_shared_include.empty());
  EXPECT_EQ(right_shared_include.front().id(), 9590);
  ASSERT_FALSE(right_shared_exclude.empty());
  EXPECT_EQ(right_shared_exclude.front().id(), 9590);
}

// Expects getting preceding lanelet sequence returns valid sequence for reference lane.
TEST_F(TestRouteHandler, getPrecedingLaneletSequenceReturnsValidSequenceForReferenceLane)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());
  const auto ref_lane = route_handler_->getLaneletsFromId(4765);
  auto sequence = route_handler_->getPrecedingLaneletSequence(ref_lane, 50.0);
  ASSERT_FALSE(sequence.empty());
  EXPECT_EQ(sequence.front().front().id(), 4755);
}

// Expects getting previous lanelets returns expected previous lanes.
TEST_F(TestRouteHandler, getPreviousLaneletsReturnsExpectedPreviousLanes)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());
  const auto ref_lane = route_handler_->getLaneletsFromId(4765);
  auto prev_lanes = route_handler_->getPreviousLanelets(ref_lane);
  ASSERT_FALSE(prev_lanes.empty());
  EXPECT_EQ(prev_lanes.front().id(), 4760);
}

// Expects getting next lanelets returns expected next lanes.
TEST_F(TestRouteHandler, getNextLaneletsReturnsExpectedNextLanes)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());
  const auto ref_lane = route_handler_->getLaneletsFromId(4765);
  auto next_lanes = route_handler_->getNextLanelets(ref_lane);
  ASSERT_FALSE(next_lanes.empty());
  EXPECT_EQ(next_lanes.front().id(), 4770);
}

// Expects getting lane change target except preferred lane returns valid target.
TEST_F(TestRouteHandler, getLaneChangeTargetExceptPreferredLaneReturnsValidTarget)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());
  const auto ref_lane = route_handler_->getLaneletsFromId(4765);
  lanelet::ConstLanelets lane_vector = {ref_lane};
  auto right_target =
    route_handler_->getLaneChangeTargetExceptPreferredLane(lane_vector, Direction::RIGHT);
  auto left_target =
    route_handler_->getLaneChangeTargetExceptPreferredLane(lane_vector, Direction::LEFT);
  ASSERT_TRUE(right_target.has_value());
  EXPECT_EQ(right_target.value().id(), 9590);
  EXPECT_FALSE(left_target.has_value());
}

// Expects getting lanes after goal returns expected lanelets when goal is set.
TEST_F(TestRouteHandler, getLanesAfterGoalReturnsExpectedLaneletsWhenGoalIsSet)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());
  auto lanes_after = route_handler_->getLanesAfterGoal(10.0);
  ASSERT_FALSE(lanes_after.empty());
  EXPECT_EQ(lanes_after.front().id(), 5092);
}

// Expects getting preferred lanelets returns configured preferred lanes.
TEST_F(TestRouteHandler, getPreferredLaneletsReturnsConfiguredPreferredLanes)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());
  auto pref_lanes = route_handler_->getPreferredLanelets();
  ASSERT_FALSE(pref_lanes.empty());
  EXPECT_EQ(pref_lanes.front().id(), 4765);
}

// Expects getting route header and uuid returns valid metadata.
TEST_F(TestRouteHandler, getRouteHeaderAndUuidReturnsValidMetadata)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());
  auto header = route_handler_->getRouteHeader();
  auto uuid = route_handler_->getRouteUuid();
  EXPECT_TRUE(header.frame_id.empty());
  EXPECT_GT(uuid.uuid.size(), 0);
}

// Expects getting original start and goal pose returns valid poses.
TEST_F(TestRouteHandler, getOriginalStartAndGoalPoseReturnsValidPoses)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());
  auto orig_start = route_handler_->getOriginalStartPose();
  auto orig_goal = route_handler_->getOriginalGoalPose();
  EXPECT_NE(orig_start.position.x, 0.0);
  EXPECT_NE(orig_goal.position.x, 0.0);
}

// Is allowed goal modification returns expected boolean.
TEST_F(TestRouteHandler, isAllowedGoalModificationReturnsExpectedBoolean)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());
  EXPECT_FALSE(route_handler_->isAllowedGoalModification());
}

// Is map msg ready returns true when map is set.
TEST_F(TestRouteHandler, isMapMsgReadyReturnsTrueWhenMapIsSet)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());
  EXPECT_TRUE(route_handler_->isMapMsgReady());
}

// Expects getting left and right shoulder lanelet returns nullopt for standard lane.
TEST_F(TestRouteHandler, getLeftAndRightShoulderLaneletReturnsNulloptForStandardLane)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());
  const auto ref_lane = route_handler_->getLaneletsFromId(4765);
  EXPECT_FALSE(route_handler_->getLeftShoulderLanelet(ref_lane).has_value());
  EXPECT_FALSE(route_handler_->getRightShoulderLanelet(ref_lane).has_value());
}

// Expects getting lane changeable neighbors returns expected neighbors.
TEST_F(TestRouteHandler, getLaneChangeableNeighborsReturnsExpectedNeighbors)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());
  const auto ref_lane = route_handler_->getLaneletsFromId(4765);
  auto neighbors = route_handler_->getLaneChangeableNeighbors(ref_lane);
  ASSERT_FALSE(neighbors.empty());
  EXPECT_EQ(neighbors.front().id(), 4765);
}

// Test create map segments returns valid segments from path lanelets.
TEST_F(TestRouteHandler, createMapSegmentsReturnsValidSegmentsFromPathLanelets)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());
  const auto ref_lane = route_handler_->getLaneletsFromId(4765);
  lanelet::ConstLanelets lane_vector = {ref_lane};
  auto segments = route_handler_->createMapSegments(lane_vector);
  ASSERT_FALSE(segments.empty());
  EXPECT_EQ(segments.front().preferred_primitive.id, 4765);
}

// Test plan path lanelets between checkpoints with area overload returns valid path.
TEST_F(TestRouteHandler, planPathLaneletsBetweenCheckpointsWithAreaOverloadReturnsValidPath)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());

  const auto start_pose = route_handler_->getStartPose();
  const auto goal_pose = route_handler_->getGoalPose();

  std::vector<lanelet::ConstLaneletOrArea> path_areas;
  const auto success =
    route_handler_->planPathLaneletsBetweenCheckpoints(start_pose, goal_pose, &path_areas, false);

  ASSERT_TRUE(success);
  ASSERT_FALSE(path_areas.empty());
  EXPECT_EQ(path_areas.front().id(), 4765);
}

// Expects getting shoulder lanelet sequence 2 returns expected sequence when on shoulder lane.
TEST_F(TestRouteHandler, getShoulderLaneletSequence2ReturnsExpectedSequenceWhenOnShoulderLane)
{
  set_route_handler("overlap_map.osm");

  geometry_msgs::msg::Pose pose;
  pose.position.x = 3719.5;
  pose.position.y = 73765.6;
  const auto shoulder_lanelets = route_handler_->getShoulderLaneletsAtPose(pose);
  ASSERT_FALSE(shoulder_lanelets.empty());

  const auto & shoulder_lane = shoulder_lanelets.front();

  auto seq = route_handler_->get_shoulder_lanelet_sequence(shoulder_lane, 10.0, 10.0);
  ASSERT_FALSE(seq.empty());
  EXPECT_EQ(seq.front().id(), 359);
}
}  // namespace autoware::route_handler::test
