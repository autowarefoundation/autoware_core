// Copyright 2023 TIER IV, Inc.
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

#include "path_to_trajectory_node.hpp"

#include "path_to_trajectory.hpp"

#include <string>

namespace autoware::planning_topic_converter
{

PathToTrajectoryNode::PathToTrajectoryNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("path_to_trajectory_converter", options),
  sub_(
    create_subscription<Path>(
      declare_parameter<std::string>("input_topic"), 1,
      std::bind(&PathToTrajectoryNode::process, this, std::placeholders::_1))),
  pub_(create_publisher<Trajectory>(declare_parameter<std::string>("output_topic"), 1))
{
}

void PathToTrajectoryNode::process(const Path::ConstSharedPtr msg)
{
  pub_->publish(path_to_trajectory::convert(*msg));
}

}  // namespace autoware::planning_topic_converter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::planning_topic_converter::PathToTrajectoryNode)
