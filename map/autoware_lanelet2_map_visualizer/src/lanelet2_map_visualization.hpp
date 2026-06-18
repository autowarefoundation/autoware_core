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

#ifndef LANELET2_MAP_VISUALIZATION_HPP_
#define LANELET2_MAP_VISUALIZATION_HPP_

#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/Forward.h>

namespace autoware::lanelet2_map_visualizer
{
/// \brief Build the RViz MarkerArray that visualizes every supported element category of a
/// Lanelet2 map.
///
/// This is the pure, ROS-publish-free core of the visualization node: given a deserialized
/// Lanelet2 map it queries each element category (lanelets, crosswalks, traffic lights, stop
/// lines, parking, curbstones, bicycle lanes, etc.) and assembles the corresponding markers into
/// a single MarkerArray. It performs no subscription/publication and depends only on the map and
/// the centerline flag, which makes it directly unit testable.
///
/// \param viz_lanelet_map deserialized Lanelet2 map to visualize.
/// \param viz_centerline  when true, lanelet boundary markers also include the centerline.
/// \return the assembled MarkerArray (empty when the map contains no elements to visualize).
visualization_msgs::msg::MarkerArray create_lanelet_map_marker_array(
  const lanelet::LaneletMapConstPtr & viz_lanelet_map, bool viz_centerline);
}  // namespace autoware::lanelet2_map_visualizer

#endif  // LANELET2_MAP_VISUALIZATION_HPP_
