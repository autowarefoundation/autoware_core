// Copyright 2021 TierIV
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

#include "autoware/map_loader/lanelet2_map_loader_node.hpp"  // NOLINT(build/include)

#include "lanelet2_map_loader.hpp"
#include "utils/lanelet2_selected_map_loader_module.hpp"

#include <memory>
#include <string>
#include <vector>

namespace autoware::map_loader
{

using autoware_map_msgs::msg::MapProjectorInfo;

Lanelet2MapLoaderNode::Lanelet2MapLoaderNode(const rclcpp::NodeOptions & options)
: Node("lanelet2_map_loader", options)
{
  // subscription
  sub_map_projector_info_ = this->create_subscription<MapProjectorInfo::Message>(
    MapProjectorInfo::name, autoware::component_interface_specs::get_qos<MapProjectorInfo>(),
    [this](const MapProjectorInfo::Message::ConstSharedPtr msg) { on_map_projector_info(msg); });

  declare_parameter<bool>("allow_unsupported_version");
  declare_parameter<std::string>("lanelet2_map_path");
  declare_parameter<double>("center_line_resolution");
  declare_parameter<bool>("use_waypoints");
  declare_parameter<bool>("enable_selected_map_loading");
  declare_parameter<std::string>("lanelet2_map_metadata_path");
}

// Defined here so the compiler sees the full type of Lanelet2SelectedMapLoaderModule
// when generating the unique_ptr destructor.
Lanelet2MapLoaderNode::~Lanelet2MapLoaderNode() = default;

void Lanelet2MapLoaderNode::on_map_projector_info(
  const MapProjectorInfo::Message::ConstSharedPtr msg)
{
  Lanelet2MapLoaderParams params;
  params.lanelet2_map_path = get_parameter("lanelet2_map_path").as_string();
  params.lanelet2_map_metadata_path = get_parameter("lanelet2_map_metadata_path").as_string();
  params.center_line_resolution = get_parameter("center_line_resolution").as_double();
  params.use_waypoints = get_parameter("use_waypoints").as_bool();
  params.allow_unsupported_version = get_parameter("allow_unsupported_version").as_bool();

  Lanelet2MapLoader core_logic(params);
  Lanelet2MapLoaderResult result;

  try {
    result = core_logic.execute(*msg, now());
  } catch (const MapLoadException & e) {
    // Here recoverable, so I log and wait for valid input
    for (const auto & err : e.errors) RCLCPP_ERROR_STREAM(get_logger(), err);
    return;
  } catch (const std::invalid_argument & e) {
    // Here UNrecoverable, so I kill this node for users to fix their inputs
    RCLCPP_WARN(get_logger(), "%s", e.what());
    throw;
  } catch (const std::exception & e) {
    // Here unknown error, so I log and still stay alive to avoid crash loops
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    return;
  }

  for (const auto & warn : result.warnings) RCLCPP_WARN(get_logger(), "%s", warn.c_str());
  for (const auto & info : result.infos) RCLCPP_INFO(get_logger(), "%s", info.c_str());

  if (get_parameter("enable_selected_map_loading").as_bool()) {
    selected_map_loader_module_ = std::make_unique<Lanelet2SelectedMapLoaderModule>(
      result.cell_metadata_dict, *msg, params.center_line_resolution, params.use_waypoints);

    pub_metadata_ = create_publisher<autoware_map_msgs::msg::LaneletMapMetaData>(
      "output/lanelet2_map_metadata", rclcpp::QoS{1}.transient_local());

    auto meta_msg = selected_map_loader_module_->build_metadata_msg();
    meta_msg.header.stamp = now();
    meta_msg.header.frame_id = "map";
    pub_metadata_->publish(meta_msg);

    srv_get_selected_lanelet2_map_ = create_service<autoware_map_msgs::srv::GetSelectedLanelet2Map>(
      "service/get_selected_lanelet2_map", std::bind(
                                             &Lanelet2MapLoaderNode::on_get_selected_lanelet2_map,
                                             this, std::placeholders::_1, std::placeholders::_2));
  }

  pub_map_bin_ =
    create_publisher<VectorMap::Message>(VectorMap::name, rclcpp::QoS{1}.transient_local());
  pub_map_bin_->publish(result.map_bin_msg);

  RCLCPP_INFO(get_logger(), "Succeeded to load lanelet2_map. Map is published.");
}

void Lanelet2MapLoaderNode::on_get_selected_lanelet2_map(
  const autoware_map_msgs::srv::GetSelectedLanelet2Map::Request::ConstSharedPtr req,
  const autoware_map_msgs::srv::GetSelectedLanelet2Map::Response::SharedPtr res)
{
  if (!selected_map_loader_module_) {
    return;
  }

  const auto current_time = now();

  // Retrieve payload from pure C++ core
  res->lanelet2_cells = selected_map_loader_module_->execute(req->cell_ids);

  if (res->lanelet2_cells.data.empty()) {
    RCLCPP_ERROR(get_logger(), "Failed to load selected cells or map is empty.");
    return;
  }

  // Explicitly apply outer header
  res->header.stamp = current_time;
  res->header.frame_id = "map";

  // Explicitly apply inner header
  res->lanelet2_cells.header.stamp = current_time;
  res->lanelet2_cells.header.frame_id = "map";
}

lanelet::LaneletMapPtr Lanelet2MapLoaderNode::load_map(
  const std::string & lanelet2_filename,
  const autoware_map_msgs::msg::MapProjectorInfo & projector_info)
{
  try {
    std::vector<std::string> warnings;

    auto map = Lanelet2MapLoader::load_map(lanelet2_filename, projector_info, warnings);
    for (const auto & w : warnings) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_loader"), w);
    }

    return map;
  } catch (const MapLoadException & e) {
    for (const auto & err : e.errors) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_loader"), err);
    }
    return nullptr;
  }
}

component_interface_specs::map::VectorMap::Message Lanelet2MapLoaderNode::create_map_bin_msg(
  const lanelet::LaneletMapPtr map, const std::string & lanelet2_filename, const rclcpp::Time & now)
{
  auto msg = Lanelet2MapLoader::create_map_bin_msg(map, lanelet2_filename);
  msg.header.stamp = now;
  msg.header.frame_id = "map";

  return msg;
}

}  // namespace autoware::map_loader

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::map_loader::Lanelet2MapLoaderNode)
