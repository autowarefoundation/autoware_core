// Copyright 2024 TIER IV, Inc.
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

#include "voxel_grid_downsample_filter_node.hpp"

#include <memory>
#include <string>
#include <utility>

namespace autoware::downsample_filters
{
VoxelGridDownsampleFilterNode::VoxelGridDownsampleFilterNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("VoxelGridDownsampleFilter", options),
  filter_core_(
    VoxelGridDownsampleFilter::Parameters{
      static_cast<float>(declare_parameter<double>("voxel_size_x")),
      static_cast<float>(declare_parameter<double>("voxel_size_y")),
      static_cast<float>(declare_parameter<double>("voxel_size_z"))}),
  max_queue_size_(static_cast<std::size_t>(declare_parameter<int64_t>("max_queue_size")))
{
  // Set publishers
  {
    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    pub_output_ = this->create_publisher<PointCloud2>(
      "output", rclcpp::SensorDataQoS().keep_last(max_queue_size_), pub_options);

    published_time_publisher_ =
      std::make_unique<autoware_utils_debug::PublishedTimePublisher>(this);
  }

  // Set subscribers
  {
    sub_input_ = create_subscription<PointCloud2>(
      "input", rclcpp::SensorDataQoS().keep_last(max_queue_size_),
      std::bind(&VoxelGridDownsampleFilterNode::input_callback, this, std::placeholders::_1));
  }

  RCLCPP_DEBUG(this->get_logger(), "[Filter Constructor] successfully created.");
}

void VoxelGridDownsampleFilterNode::input_callback(const PointCloud2ConstPtr cloud)
{
  RCLCPP_DEBUG(
    this->get_logger(),
    "[input_callback] PointCloud with %d data points and frame %s on input topic "
    "received.",
    cloud->width * cloud->height, cloud->header.frame_id.c_str());

  auto result = filter_core_.filter(cloud);
  if (!result) {
    RCLCPP_ERROR(this->get_logger(), "[input_callback] %s", result.error().c_str());
    return;
  }

  pub_output_->publish(std::make_unique<PointCloud2>(std::move(result.value())));
  published_time_publisher_->publish_if_subscribed(pub_output_, cloud->header.stamp);
}
}  // namespace autoware::downsample_filters

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::downsample_filters::VoxelGridDownsampleFilterNode)
