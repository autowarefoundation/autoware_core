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

#ifndef AUTOWARE__LANELET2_UTILS__LANE_SEQUENCE_HPP_
#define AUTOWARE__LANELET2_UTILS__LANE_SEQUENCE_HPP_

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/Forward.h>

#include <optional>
#include <vector>

namespace autoware::experimental::lanelet2_utils
{

class LaneSequence
{
public:
  explicit LaneSequence(const lanelet::ConstLanelet & lanelet);

  static std::optional<LaneSequence> create(
    const lanelet::ConstLanelets & lanelets, lanelet::routing::RoutingGraphConstPtr routing_graph);

  const lanelet::ConstLanelets & as_lanelets() const { return lanelets_; }

private:
  LaneSequence(lanelet::ConstLanelets && lanelets, std::vector<double> && distance);

  lanelet::ConstLanelets lanelets_;
  std::vector<double> distance_;
};

}  // namespace autoware::experimental::lanelet2_utils

#endif  // AUTOWARE__LANELET2_UTILS__LANE_SEQUENCE_HPP_
