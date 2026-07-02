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

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Area.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <memory>

namespace autoware::route_handler::test
{
namespace
{
using autoware::experimental::lanelet2_utils::to_autoware_map_msgs;
using autoware_planning_msgs::msg::LaneletPrimitive;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;

constexpr lanelet::Id kEntryLaneId = 100;
constexpr lanelet::Id kAreaId = 200;
constexpr lanelet::Id kExitLaneId = 300;

lanelet::Lanelet makeRoadLanelet(
  const lanelet::Id id, const lanelet::LineString3d & left, const lanelet::LineString3d & right)
{
  lanelet::Lanelet lanelet(id, left, right);
  lanelet.attributes()["subtype"] = "road";
  lanelet.attributes()["participant:vehicle"] = "yes";
  return lanelet;
}

lanelet::LaneletMapPtr makeLaneAreaLaneMap()
{
  const lanelet::Point3d entry_left_0(1, 0.0, 0.0, 0.0);
  const lanelet::Point3d entry_left_1(2, 0.0, 10.0, 0.0);
  const lanelet::Point3d entry_right_0(3, 1.0, 0.0, 0.0);
  const lanelet::Point3d entry_right_1(4, 1.0, 10.0, 0.0);
  const lanelet::LineString3d entry_left(10, {entry_left_0, entry_left_1});
  const lanelet::LineString3d entry_right(11, {entry_right_0, entry_right_1});

  const lanelet::Point3d exit_left_0(5, 0.0, 20.0, 0.0);
  const lanelet::Point3d exit_left_1(6, 0.0, 30.0, 0.0);
  const lanelet::Point3d exit_right_0(7, 1.0, 20.0, 0.0);
  const lanelet::Point3d exit_right_1(8, 1.0, 30.0, 0.0);
  const lanelet::LineString3d exit_left(12, {exit_left_0, exit_left_1});
  const lanelet::LineString3d exit_right(13, {exit_right_0, exit_right_1});

  const lanelet::Point3d area_p1(21, 0.0, 10.0, 0.0);
  const lanelet::Point3d area_p2(22, 2.0, 10.0, 0.0);
  const lanelet::Point3d area_p3(23, 2.0, 20.0, 0.0);
  const lanelet::Point3d area_p4(24, 0.0, 20.0, 0.0);
  const lanelet::LineString3d area_ring(20, {area_p1, area_p2, area_p3, area_p4, area_p1});
  const lanelet::Area area(kAreaId, lanelet::LineStrings3d{area_ring});

  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(makeRoadLanelet(kEntryLaneId, entry_left, entry_right));
  map->add(area);
  map->add(makeRoadLanelet(kExitLaneId, exit_left, exit_right));
  return map;
}

LaneletSegment makeLaneSegment(const lanelet::Id id)
{
  LaneletPrimitive primitive;
  primitive.id = id;
  primitive.primitive_type = "lane";

  LaneletSegment segment;
  segment.preferred_primitive = primitive;
  segment.preferred_primitive.primitive_type = "";
  segment.primitives.push_back(primitive);
  return segment;
}

LaneletSegment makeAreaSegment(const lanelet::Id id)
{
  LaneletPrimitive primitive;
  primitive.id = id;
  primitive.primitive_type = "area";

  LaneletSegment segment;
  segment.preferred_primitive = primitive;
  segment.primitives.push_back(primitive);
  return segment;
}

LaneletRoute makeLaneAreaLaneRoute()
{
  LaneletRoute route;
  route.header.frame_id = "map";
  route.start_pose = autoware::test_utils::createPose(0.5, 5.0, 0.0, 0.0, 0.0, 0.0);
  route.goal_pose = autoware::test_utils::createPose(0.5, 25.0, 0.0, 0.0, 0.0, 0.0);
  route.segments.push_back(makeLaneSegment(kEntryLaneId));
  route.segments.push_back(makeAreaSegment(kAreaId));
  route.segments.push_back(makeLaneSegment(kExitLaneId));
  return route;
}

std::shared_ptr<RouteHandler> makeRouteHandler(const bool allow_area)
{
  const auto map = makeLaneAreaLaneMap();
  const auto map_bin = to_autoware_map_msgs(map);
  auto route_handler = std::make_shared<RouteHandler>(map_bin);
  route_handler->setAllowArea(allow_area);
  route_handler->setRoute(makeLaneAreaLaneRoute());
  return route_handler;
}

bool hasDirectFollowingInRoute(
  const RouteHandler & route_handler, const lanelet::ConstLanelet & lanelet)
{
  const auto routing_graph = route_handler.getRoutingGraphPtr();
  const auto following = routing_graph->following(lanelet);
  for (const auto & candidate : following) {
    if (route_handler.isRouteLanelet(candidate)) {
      return true;
    }
  }
  return false;
}

bool hasDirectPreviousInRoute(
  const RouteHandler & route_handler, const lanelet::ConstLanelet & lanelet)
{
  const auto routing_graph = route_handler.getRoutingGraphPtr();
  const auto previous = routing_graph->previous(lanelet);
  for (const auto & candidate : previous) {
    if (route_handler.isRouteLanelet(candidate)) {
      return true;
    }
  }
  return false;
}
}  // namespace

TEST(AllowAreaRouteHandler, setAllowAreaDefaultIsFalse)
{
  const auto map = makeLaneAreaLaneMap();
  const RouteHandler route_handler(to_autoware_map_msgs(map));
  EXPECT_FALSE(route_handler.allowArea());
}

TEST(AllowAreaRouteHandler, setAllowAreaUpdatesValue)
{
  const auto map = makeLaneAreaLaneMap();
  RouteHandler route_handler(to_autoware_map_msgs(map));

  route_handler.setAllowArea(true);
  EXPECT_TRUE(route_handler.allowArea());

  route_handler.setAllowArea(false);
  EXPECT_FALSE(route_handler.allowArea());
}

TEST(AllowAreaRouteHandler, routeWithAreaRejectedWhenAllowAreaDisabled)
{
  const auto route_handler = makeRouteHandler(false);
  EXPECT_FALSE(route_handler->isHandlerReady());
}

TEST(AllowAreaRouteHandler, routeWithAreaAcceptedWhenAllowAreaEnabled)
{
  const auto route_handler = makeRouteHandler(true);
  EXPECT_TRUE(route_handler->isHandlerReady());
}

TEST(AllowAreaRouteHandler, getNextLaneletsWithinRouteCrossesAreaBoundary)
{
  const auto route_handler = makeRouteHandler(true);
  ASSERT_TRUE(route_handler->isHandlerReady());

  const auto entry_lanelet = route_handler->getLaneletsFromId(kEntryLaneId);
  ASSERT_FALSE(hasDirectFollowingInRoute(*route_handler, entry_lanelet));

  lanelet::ConstLanelets next_lanelets;
  ASSERT_TRUE(route_handler->getNextLaneletsWithinRoute(entry_lanelet, &next_lanelets));
  ASSERT_EQ(next_lanelets.size(), 1u);
  EXPECT_EQ(next_lanelets.front().id(), kExitLaneId);
}

TEST(AllowAreaRouteHandler, getPreviousLaneletsWithinRouteCrossesAreaBoundary)
{
  const auto route_handler = makeRouteHandler(true);
  ASSERT_TRUE(route_handler->isHandlerReady());

  const auto exit_lanelet = route_handler->getLaneletsFromId(kExitLaneId);
  ASSERT_FALSE(hasDirectPreviousInRoute(*route_handler, exit_lanelet));

  lanelet::ConstLanelets previous_lanelets;
  ASSERT_TRUE(route_handler->getPreviousLaneletsWithinRoute(exit_lanelet, &previous_lanelets));
  ASSERT_EQ(previous_lanelets.size(), 1u);
  EXPECT_EQ(previous_lanelets.front().id(), kEntryLaneId);
}

TEST(AllowAreaRouteHandler, getNextLaneletsWithinRouteReturnsFalseAtRouteEnd)
{
  const auto route_handler = makeRouteHandler(true);
  ASSERT_TRUE(route_handler->isHandlerReady());

  const auto exit_lanelet = route_handler->getLaneletsFromId(kExitLaneId);
  lanelet::ConstLanelets next_lanelets;
  EXPECT_FALSE(route_handler->getNextLaneletsWithinRoute(exit_lanelet, &next_lanelets));
  EXPECT_TRUE(next_lanelets.empty());
}

TEST(AllowAreaRouteHandler, getPreviousLaneletsWithinRouteReturnsFalseAtRouteStart)
{
  const auto route_handler = makeRouteHandler(true);
  ASSERT_TRUE(route_handler->isHandlerReady());

  const auto entry_lanelet = route_handler->getLaneletsFromId(kEntryLaneId);
  lanelet::ConstLanelets previous_lanelets;
  EXPECT_FALSE(route_handler->getPreviousLaneletsWithinRoute(entry_lanelet, &previous_lanelets));
  EXPECT_TRUE(previous_lanelets.empty());
}

}  // namespace autoware::route_handler::test
