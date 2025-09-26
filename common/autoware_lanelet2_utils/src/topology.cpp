// Copyright 2025 TIER IV, Inc.
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

#include <autoware/lanelet2_utils/topology.hpp>
#include <range/v3/all.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <vector>

namespace autoware::experimental::lanelet2_utils
{

std::optional<lanelet::ConstLanelet> left_opposite_lanelet(
  const lanelet::ConstLanelet & lanelet, const lanelet::LaneletMapConstPtr lanelet_map)
{
  for (const auto & opposite_candidate :
       lanelet_map->laneletLayer.findUsages(lanelet.leftBound().invert())) {
    return opposite_candidate;
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> right_opposite_lanelet(
  const lanelet::ConstLanelet & lanelet, const lanelet::LaneletMapConstPtr lanelet_map)
{
  for (const auto & opposite_candidate :
       lanelet_map->laneletLayer.findUsages(lanelet.rightBound().invert())) {
    return opposite_candidate;
  }
  return std::nullopt;
}

lanelet::ConstLanelets following_lanelets(
  const lanelet::ConstLanelet & lanelet, const lanelet::routing::RoutingGraphConstPtr routing_graph)
{
  return routing_graph->following(lanelet);
}

lanelet::ConstLanelets previous_lanelets(
  const lanelet::ConstLanelet & lanelet, const lanelet::routing::RoutingGraphConstPtr routing_graph)
{
  return routing_graph->previous(lanelet);
}

lanelet::ConstLanelets sibling_lanelets(
  const lanelet::ConstLanelet & lanelet, const lanelet::routing::RoutingGraphConstPtr routing_graph)
{
  lanelet::ConstLanelets siblings;
  for (const auto & previous : previous_lanelets(lanelet, routing_graph)) {
    for (const auto & following : following_lanelets(previous, routing_graph)) {
      if (following.id() != lanelet.id()) {
        siblings.push_back(following);
      }
    }
  }
  return siblings;
}

lanelet::ConstLanelets from_ids(
  const lanelet::LaneletMapConstPtr lanelet_map, const std::vector<lanelet::Id> & ids)
{
  return ids | ranges::views::transform([&](const auto id) {
           return lanelet_map->laneletLayer.get(id);
         }) |
         ranges::to<std::vector>();
}
}  // namespace autoware::experimental::lanelet2_utils
