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

#include "map_projection_loader_node.hpp"

#include "autoware/map_projection_loader/map_projection_loader.hpp"

#include <autoware_map_msgs/msg/map_projector_info.hpp>

#include <string>
#include <utility>

namespace autoware::map_projection_loader
{
MapProjectionLoader::MapProjectionLoader(const rclcpp::NodeOptions & options)
: Node("map_projection_loader", options)
{
  const std::string yaml_filename = this->declare_parameter<std::string>("map_projector_info_path");
  const std::string lanelet2_map_filename =
    this->declare_parameter<std::string>("lanelet2_map_path");

  RCLCPP_INFO_STREAM(get_logger(), "map_projector_info_path: " << yaml_filename);
  RCLCPP_INFO_STREAM(get_logger(), "lanelet2_map_path: " << lanelet2_map_filename);

  const autoware_map_msgs::msg::MapProjectorInfo msg =
    load_map_projector_info(yaml_filename, lanelet2_map_filename);

  RCLCPP_INFO_STREAM(get_logger(), "Loaded map projector info");

  // Publish the message
  publisher_ = this->create_publisher<MapProjectorInfo::Message>(
    MapProjectorInfo::name, autoware::component_interface_specs::get_qos<MapProjectorInfo>());
  auto output = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(publisher_);
  *output = msg;
  publisher_->publish(std::move(output));
}
}  // namespace autoware::map_projection_loader

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::map_projection_loader::MapProjectionLoader)
