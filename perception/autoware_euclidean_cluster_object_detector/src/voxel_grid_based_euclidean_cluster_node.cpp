// Copyright 2020 Tier IV, Inc.
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

#include "voxel_grid_based_euclidean_cluster_node.hpp"

#include <memory>
#include <string>

namespace autoware::euclidean_cluster
{
VoxelGridBasedEuclideanClusterNode::VoxelGridBasedEuclideanClusterNode(
  const rclcpp::NodeOptions & options)
: Node("voxel_grid_based_euclidean_cluster_node", options)
{
  EuclideanClusterParams param;
  param.use_height = declare_parameter<bool>("use_height", false);
  param.min_cluster_size = static_cast<int>(declare_parameter<int64_t>("min_cluster_size", 1));
  param.max_cluster_size = static_cast<int>(declare_parameter<int64_t>("max_cluster_size", 500));
  param.tolerance = static_cast<float>(declare_parameter<double>("tolerance", 1.0));
  param.voxel_leaf_size = static_cast<float>(declare_parameter<double>("voxel_leaf_size", 0.5));
  param.min_points_number_per_voxel =
    static_cast<int>(declare_parameter<int64_t>("min_points_number_per_voxel", 3));

  detector_ = std::make_shared<EuclideanClusterObjectDetector>(param);

  // Pass the diagnostics interface pointer from the node to the cluster
  diagnostics_interface_ptr_ =
    std::make_unique<autoware_utils_diagnostics::DiagnosticsInterface>(this, "euclidean_cluster");

  using std::placeholders::_1;
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&VoxelGridBasedEuclideanClusterNode::on_point_cloud, this, _1));

  cluster_pub_ = this->create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
    "output", rclcpp::QoS{1});
  debug_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("debug/clusters", 1);

  stop_watch_ptr_ = std::make_unique<autoware_utils_system::StopWatch<std::chrono::milliseconds>>();
  debug_publisher_ = std::make_unique<autoware_utils_debug::DebugPublisher>(
    this, "voxel_grid_based_euclidean_cluster");

  stop_watch_ptr_->tic("cyclic_time");
  stop_watch_ptr_->tic("processing_time");
}

void VoxelGridBasedEuclideanClusterNode::on_point_cloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
{
  stop_watch_ptr_->toc("processing_time", true);

  // convert ros to pcl
  if (input_msg->data.empty()) {
    // NOTE: prevent pcl log spam
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Empty sensor points!");
    // Publish empty DetectedObjects and return early
    autoware_perception_msgs::msg::DetectedObjects output;
    output.header = input_msg->header;
    cluster_pub_->publish(output);
    return;
  }

  const auto result = detector_->cluster(*input_msg);
  cluster_pub_->publish(result.cluster_message);

  if (debug_pub_->get_subscription_count() >= 1) {
    debug_pub_->publish(result.debug_message);
  }

  diagnostics_interface_ptr_->clear();
  if (result.skipped_cluster_count > 0) {
    const int64_t max_size = this->get_parameter("max_cluster_size").as_int();
    std::string summary =
      std::to_string(result.skipped_cluster_count) +
      " clusters skipped because cluster point size exceeds the maximum allowed " +
      std::to_string(max_size);
    diagnostics_interface_ptr_->add_key_value("is_cluster_data_size_within_range", false);
    diagnostics_interface_ptr_->update_level_and_message(
      static_cast<int8_t>(diagnostic_msgs::msg::DiagnosticStatus::WARN), summary);
  } else {
    diagnostics_interface_ptr_->add_key_value("is_cluster_data_size_within_range", true);
    diagnostics_interface_ptr_->update_level_and_message(
      static_cast<int8_t>(diagnostic_msgs::msg::DiagnosticStatus::OK), "OK");
  }
  diagnostics_interface_ptr_->publish(input_msg->header.stamp);

  if (debug_publisher_) {
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds(
          (this->get_clock()->now() - result.cluster_message.header.stamp).nanoseconds()))
        .count();

    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms);
  }
}
}  // namespace autoware::euclidean_cluster

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::euclidean_cluster::VoxelGridBasedEuclideanClusterNode)
