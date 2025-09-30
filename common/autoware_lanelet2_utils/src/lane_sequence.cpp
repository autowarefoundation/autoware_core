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

#include <autoware/lanelet2_utils/lane_sequence.hpp>
#include <autoware/lanelet2_utils/topology.hpp>
#include <range/v3/all.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::experimental::lanelet2_utils
{

std::optional<LaneSequence> LaneSequence::create(
  const lanelet::ConstLanelets & lanelets, lanelet::routing::RoutingGraphConstPtr routing_graph)
{
  for (const auto lane1_2 : ranges::views::zip(lanelets, lanelets | ranges::views::drop(1))) {
    const auto lane1 = std::get<0>(lane1_2);
    const auto lane2 = std::get<1>(lane1_2);
    if (const auto nexts = following_lanelets(lane1, routing_graph);
        std::find_if(nexts.begin(), nexts.end(), [&](const auto & lane) {
          return lane.id() == lane2.id();
        }) == nexts.end()) {
      return std::nullopt;
    }
  }
  return LaneSequence(lanelet::ConstLanelets(lanelets));
}

LaneSequence::LaneSequence(const lanelet::ConstLanelet & lanelet)
: lanelets_(lanelet::ConstLanelets{lanelet})
{
}

LaneSequence::LaneSequence(lanelet::ConstLanelets && lanelets) : lanelets_(std::move(lanelets))
{
}

}  // namespace autoware::experimental::lanelet2_utils
