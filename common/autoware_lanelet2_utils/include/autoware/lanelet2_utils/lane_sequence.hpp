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

/**
 * @brief a class for representing consecutive sequence of Lanelet on the road topology
 * @invariant as_lanelets() returns a list of Lanelet along driving direction, and they are
 * consecutive
 */
class LaneSequence
{
public:
  explicit LaneSequence(const lanelet::ConstLanelet & lanelet);

  /**
   * @brief create LaneSequence object
   * @return if the inputs does not satisfy the invariance, return nullopt
   */
  static std::optional<LaneSequence> create(
    const lanelet::ConstLanelets & lanelets, lanelet::routing::RoutingGraphConstPtr routing_graph);

  const lanelet::ConstLanelets & as_lanelets() const { return lanelets_; }

private:
  explicit LaneSequence(lanelet::ConstLanelets && lanelets);

  lanelet::ConstLanelets lanelets_;
};

}  // namespace autoware::experimental::lanelet2_utils

#endif  // AUTOWARE__LANELET2_UTILS__LANE_SEQUENCE_HPP_
