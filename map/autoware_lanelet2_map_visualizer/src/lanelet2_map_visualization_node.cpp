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

/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Simon Thompson, Ryohsuke Mitsudome
 *
 */

#include "lanelet2_map_visualization_node.hpp"

#include "lanelet2_map_visualization.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_projection/UTM.h>

#include <functional>

namespace autoware::lanelet2_map_visualizer
{
Lanelet2MapVisualizationNode::Lanelet2MapVisualizationNode(const rclcpp::NodeOptions & options)
: Node("lanelet2_map_visualization", options)
{
  using std::placeholders::_1;

  viz_lanelets_centerline_ = true;

  sub_map_bin_ = this->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "input/lanelet2_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&Lanelet2MapVisualizationNode::on_map_bin, this, _1));

  pub_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "output/lanelet2_map_marker", rclcpp::QoS{1}.transient_local());
}

void Lanelet2MapVisualizationNode::on_map_bin(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
{
  lanelet::LaneletMapConstPtr viz_lanelet_map =
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*msg);
  RCLCPP_INFO(this->get_logger(), "Map is loaded\n");

  const visualization_msgs::msg::MarkerArray map_marker_array =
    create_lanelet_map_marker_array(viz_lanelet_map, viz_lanelets_centerline_);

  pub_marker_->publish(map_marker_array);
}
}  // namespace autoware::lanelet2_map_visualizer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::lanelet2_map_visualizer::Lanelet2MapVisualizationNode)
