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

#ifndef AUTOWARE__LANELET2_UTILS__CONVERSION_HPP_
#define AUTOWARE__LANELET2_UTILS__CONVERSION_HPP_

#include <lanelet2_core/Forward.h>
#include <lanelet2_routing/Forward.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <string>
#include <utility>

namespace autoware::experimental::lanelet2_utils
{

/**
 * @brief load a map file from the given path and return LaneletMap object
 */
lanelet::LaneletMapConstPtr load_mgrs_coordinate_map(
  const std::string & path, const double centerline_resolution = 5.0);

/**
 * @brief instantiate RoutingGraph from given LaneletMap only from "road" lanes
 * @param location [in, opt, lanelet::Locations::Germany] location value
 * @param participant [in, opt, lanelet::Participants::Vehicle] participant value
 * @return RoutingGraph object without road_shoulder and bicycle_lane, and traffic rule object
 */
std::pair<lanelet::routing::RoutingGraphConstPtr, lanelet::traffic_rules::TrafficRulesPtr>
instantiate_routing_graph_and_traffic_rules(
  lanelet::LaneletMapConstPtr lanelet_map, const char * location = lanelet::Locations::Germany,
  const char * participant = lanelet::Participants::Vehicle);

}  // namespace autoware::experimental::lanelet2_utils
#endif  // AUTOWARE__LANELET2_UTILS__CONVERSION_HPP_
