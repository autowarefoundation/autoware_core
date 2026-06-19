// Copyright 2021 Tier IV, Inc.
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

#ifndef LANELET2_MAP_VISUALIZATION_NODE_HPP_
#define LANELET2_MAP_VISUALIZATION_NODE_HPP_

#include <autoware/agnocast_wrapper/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

namespace autoware::lanelet2_map_visualizer
{
class Lanelet2MapVisualizationNode : public autoware::agnocast_wrapper::Node
{
public:
  explicit Lanelet2MapVisualizationNode(const rclcpp::NodeOptions & options);

private:
  AUTOWARE_SUBSCRIPTION_PTR(autoware_map_msgs::msg::LaneletMapBin) sub_map_bin_;
  AUTOWARE_PUBLISHER_PTR(visualization_msgs::msg::MarkerArray) pub_marker_;

  bool viz_lanelets_centerline_;

  void on_map_bin(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(autoware_map_msgs::msg::LaneletMapBin) &
                    msg);
};
}  // namespace autoware::lanelet2_map_visualizer

#endif  // LANELET2_MAP_VISUALIZATION_NODE_HPP_
