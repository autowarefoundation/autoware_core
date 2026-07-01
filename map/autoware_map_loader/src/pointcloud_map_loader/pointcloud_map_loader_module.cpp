// Copyright 2022 The Autoware Contributors
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

#include "pointcloud_map_loader_module.hpp"

#include "pointcloud_map_loader.hpp"

#include <string>
#include <vector>

namespace autoware::map_loader
{
PointcloudMapLoaderModule::PointcloudMapLoaderModule(
  rclcpp::Node * node, const std::vector<std::string> & pcd_paths,
  const std::string & publisher_name, const bool use_downsample)
: logger_(node->get_logger())
{
  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();
  pub_pointcloud_map_ =
    node->create_publisher<sensor_msgs::msg::PointCloud2>(publisher_name, durable_qos);

  sensor_msgs::msg::PointCloud2 pcd;
  if (use_downsample) {
    const auto leaf_size =
      boost::make_optional(static_cast<float>(node->declare_parameter<float>("leaf_size")));
    pcd = load_pointcloud_map(
      pcd_paths, leaf_size,
      [this](const std::string & msg) { RCLCPP_DEBUG_STREAM(logger_, msg); },
      [this](const std::string & msg) { RCLCPP_ERROR_STREAM(logger_, msg); });
  } else {
    pcd = load_pointcloud_map(
      pcd_paths, boost::none,
      [this](const std::string & msg) { RCLCPP_DEBUG_STREAM(logger_, msg); },
      [this](const std::string & msg) { RCLCPP_ERROR_STREAM(logger_, msg); });
  }

  if (pcd.width == 0) {
    RCLCPP_ERROR(logger_, "No PCD was loaded: pcd_paths.size() = %zu", pcd_paths.size());
    return;
  }

  pub_pointcloud_map_->publish(pcd);
}
}  // namespace autoware::map_loader
