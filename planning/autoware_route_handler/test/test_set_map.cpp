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

// Characterization tests for RouteHandler::setMap(). They pin the observable
// graph state exposed through getRoutingGraphPtr() / getOverallGraphPtr() so a
// performance refactor of setMap() (eliminating the duplicate whole-map routing
// graph build and the dead all_lanelets query) stays behavior-preserving.

#include "test_route_handler.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <algorithm>
#include <filesystem>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

namespace autoware::route_handler::test
{

namespace
{
// Collect, for every lanelet in the map, the sorted list of ids of its
// "following" lanelets according to the given routing graph. This fully
// captures the directed reachability encoded by the graph, which is the
// observable property a graph reuse must not change.
std::vector<std::pair<lanelet::Id, std::vector<lanelet::Id>>> collect_following_ids(
  const lanelet::LaneletMapPtr & map, const lanelet::routing::RoutingGraph & graph)
{
  std::vector<std::pair<lanelet::Id, std::vector<lanelet::Id>>> result;
  for (const auto & lanelet : map->laneletLayer) {
    std::vector<lanelet::Id> following_ids;
    for (const auto & next : graph.following(lanelet)) {
      following_ids.push_back(next.id());
    }
    std::sort(following_ids.begin(), following_ids.end());
    result.emplace_back(lanelet.id(), std::move(following_ids));
  }
  std::sort(
    result.begin(), result.end(), [](const auto & a, const auto & b) { return a.first < b.first; });
  return result;
}

lanelet::LaneletMapConstPtr load_straight_sample_map()
{
  using autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map;
  const auto map_abs_path =
    fs::path(ament_index_cpp::get_package_share_directory("autoware_lanelet2_utils")) /
    "sample_map" / "vm_01_10-12" / "straight_waypoint" / "lanelet2_map.osm";
  return load_mgrs_coordinate_map(map_abs_path);
}
}  // namespace

// The overall-graph container must always expose exactly two graphs (vehicle,
// pedestrian), both non-null. This holds for both setMap() overloads.
TEST_F(TestRouteHandler, setMapFromMsgPopulatesTwoNonNullOverallGraphs)
{
  // The base fixture builds the handler via the LaneletMapBin (msg) overload.
  const auto overall_graphs = route_handler_->getOverallGraphPtr();
  ASSERT_NE(overall_graphs, nullptr);
  ASSERT_EQ(overall_graphs->routingGraphs().size(), 2u);
  EXPECT_NE(overall_graphs->routingGraphs().at(0), nullptr);
  EXPECT_NE(overall_graphs->routingGraphs().at(1), nullptr);
}

TEST(RouteHandlerSetMap, ptrOverloadPopulatesTwoNonNullOverallGraphs)
{
  const auto map = load_straight_sample_map();
  ASSERT_NE(map, nullptr);
  RouteHandler route_handler(map);

  const auto overall_graphs = route_handler.getOverallGraphPtr();
  ASSERT_NE(overall_graphs, nullptr);
  ASSERT_EQ(overall_graphs->routingGraphs().size(), 2u);
  EXPECT_NE(overall_graphs->routingGraphs().at(0), nullptr);
  EXPECT_NE(overall_graphs->routingGraphs().at(1), nullptr);
}

// For the LaneletMapConstPtr overload, the refactor reuses the already-built
// routing_graph_ptr_ as the vehicle slot of the overall-graph container. That
// reuse is only valid if the reused graph is equivalent to one built
// independently with Germany/Vehicle traffic rules. We therefore compare the
// production routing graph against a SEPARATE reference graph built here (the
// way the pre-refactor code built the vehicle slot), so the oracle does not
// depend on the object under test.
TEST(RouteHandlerSetMap, ptrOverloadVehicleGraphMatchesRoutingGraph)
{
  const auto map = load_straight_sample_map();
  ASSERT_NE(map, nullptr);
  RouteHandler route_handler(map);

  const auto lanelet_map = route_handler.getLaneletMapPtr();
  ASSERT_NE(lanelet_map, nullptr);
  ASSERT_GT(lanelet_map->laneletLayer.size(), 0u);

  const auto routing_graph = route_handler.getRoutingGraphPtr();
  ASSERT_NE(routing_graph, nullptr);

  // Independent reference: build a fresh Germany/Vehicle routing graph from the
  // same map, exactly as the original (pre-refactor) code constructed the
  // vehicle slot. This is the oracle the reused production graph must match.
  const auto ref_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  const auto ref_vehicle_graph = lanelet::routing::RoutingGraph::build(*lanelet_map, *ref_rules);
  ASSERT_NE(ref_vehicle_graph, nullptr);

  const auto routing_following = collect_following_ids(lanelet_map, *routing_graph);
  const auto ref_following = collect_following_ids(lanelet_map, *ref_vehicle_graph);

  ASSERT_EQ(routing_following.size(), ref_following.size());
  EXPECT_EQ(routing_following, ref_following);

  // Sanity: at least one lanelet has a successor, so the equality above is not
  // a comparison of two empty relation sets.
  const bool any_following = std::any_of(
    routing_following.begin(), routing_following.end(),
    [](const auto & entry) { return !entry.second.empty(); });
  EXPECT_TRUE(any_following);

  // Document the intended pointer identity (the vehicle slot reuses
  // routing_graph_ptr_). This is in addition to, not a substitute for, the
  // independent-reference equivalence check above.
  const auto overall_graphs = route_handler.getOverallGraphPtr();
  ASSERT_NE(overall_graphs, nullptr);
  ASSERT_EQ(overall_graphs->routingGraphs().size(), 2u);
  EXPECT_EQ(overall_graphs->routingGraphs().at(0).get(), routing_graph.get());
}

// Reconstructing the handler from the same map must yield byte-for-byte
// identical reachability in both the primary routing graph and the overall
// vehicle graph. Pins determinism of setMap() across the refactor.
TEST(RouteHandlerSetMap, ptrOverloadGraphsAreDeterministicAcrossReconstruction)
{
  const auto map = load_straight_sample_map();
  ASSERT_NE(map, nullptr);

  RouteHandler handler_a(map);
  RouteHandler handler_b(map);

  const auto map_a = handler_a.getLaneletMapPtr();
  const auto map_b = handler_b.getLaneletMapPtr();
  ASSERT_NE(map_a, nullptr);
  ASSERT_NE(map_b, nullptr);

  const auto routing_graph_a = handler_a.getRoutingGraphPtr();
  const auto routing_graph_b = handler_b.getRoutingGraphPtr();
  ASSERT_NE(routing_graph_a, nullptr);
  ASSERT_NE(routing_graph_b, nullptr);
  const auto routing_a = collect_following_ids(map_a, *routing_graph_a);
  const auto routing_b = collect_following_ids(map_b, *routing_graph_b);
  EXPECT_EQ(routing_a, routing_b);

  const auto overall_a = handler_a.getOverallGraphPtr();
  const auto overall_b = handler_b.getOverallGraphPtr();
  ASSERT_NE(overall_a, nullptr);
  ASSERT_NE(overall_b, nullptr);
  ASSERT_EQ(overall_a->routingGraphs().size(), 2u);
  ASSERT_EQ(overall_b->routingGraphs().size(), 2u);
  const auto vehicle_graph_a = overall_a->routingGraphs().at(0);
  const auto vehicle_graph_b = overall_b->routingGraphs().at(0);
  ASSERT_NE(vehicle_graph_a, nullptr);
  ASSERT_NE(vehicle_graph_b, nullptr);
  const auto vehicle_a = collect_following_ids(map_a, *vehicle_graph_a);
  const auto vehicle_b = collect_following_ids(map_b, *vehicle_graph_b);
  EXPECT_EQ(vehicle_a, vehicle_b);
}

}  // namespace autoware::route_handler::test
