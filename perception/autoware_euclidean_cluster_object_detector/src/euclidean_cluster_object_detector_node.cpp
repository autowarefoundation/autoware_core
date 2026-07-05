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

#include "euclidean_cluster_object_detector_node.hpp"

#include "../lib/ros_conversions.hpp"
#include "parameters.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <functional>
#include <memory>
#include <string>

namespace autoware::euclidean_cluster
{

// Constructor
EuclideanClusterObjectDetectorNode::EuclideanClusterObjectDetectorNode(
  const rclcpp::NodeOptions & options)
: Node("euclidean_cluster_object_detector", options)
{
  // Fetch params once
  EuclideanClusterParams param;
  param.use_height = declare_parameter<bool>("use_height", false);
  param.min_cluster_size = static_cast<int>(declare_parameter<int64_t>("min_cluster_size", 3));
  param.max_cluster_size = static_cast<int>(declare_parameter<int64_t>("max_cluster_size", 200));
  param.tolerance = static_cast<float>(declare_parameter<double>("tolerance", 1.0));
  param.voxel_leaf_size = static_cast<float>(declare_parameter<double>("voxel_leaf_size", 0.0));
  param.min_points_number_per_voxel =
    static_cast<int>(declare_parameter<int64_t>("min_points_number_per_voxel", 1));

  detector_ = std::make_unique<EuclideanClusterObjectDetector>(param);

  // Setup pub/sub
  using std::placeholders::_1;
  sub_pointcloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&EuclideanClusterObjectDetectorNode::on_point_cloud, this, _1));

  pub_clusters_ =
    create_publisher<autoware_perception_msgs::msg::DetectedObjects>("output", rclcpp::QoS{1});
  pub_debug_ = create_publisher<sensor_msgs::msg::PointCloud2>("debug/clusters", 1);

  // Diagnostics & metrics
  stop_watch_ = std::make_unique<autoware_utils_system::StopWatch<std::chrono::milliseconds>>();
  debug_publisher_ =
    std::make_unique<autoware_utils_debug::DebugPublisher>(this, "euclidean_cluster");
  diagnostics_interface_ptr_ =
    std::make_unique<autoware_utils_diagnostics::DiagnosticsInterface>(this, "euclidean_cluster");

  stop_watch_->tic("cyclic_time");
  stop_watch_->tic("processing_time");
}

/**
 * @brief Callback function for point cloud subscription
 * @param input_msg Point cloud message
 */
void EuclideanClusterObjectDetectorNode::on_point_cloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
{
  stop_watch_->toc("processing_time", true);

  // Handle empty input
  if (input_msg->data.empty() || input_msg->width * input_msg->height == 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Received empty point cloud, bypassing detector.");
    autoware_perception_msgs::msg::DetectedObjects empty_output;
    empty_output.header = input_msg->header;
    pub_clusters_->publish(empty_output);
    return;
  }

  // Receive point cloud and PCL conversion
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input_msg, *raw_cloud);

  // Core logic called
  auto result = detector_->cluster(raw_cloud);

  // Legacy diagnostic logic
  // (similar to what used to be publishDiagnosticSummary in legacy code)
  diagnostics_interface_ptr_->clear();
  if (result.skipped_cluster_count > 0) {
    const int max_size = this->get_parameter("max_cluster_size").as_int();
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

  // Publish to ROS
  autoware_perception_msgs::msg::DetectedObjects output_msg;
  convertClustersToDetectedObjects(input_msg->header, result.clusters, output_msg);
  pub_clusters_->publish(output_msg);

  // Publish visualization if RViz is on
  if (pub_debug_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 debug_msg;
    convertClustersToDebugPointCloud(input_msg->header, result.clusters, debug_msg);
    pub_debug_->publish(debug_msg);
  }

  // Publish diagnostic latencies if required
  if (debug_publisher_) {
    const double cyclic_time_ms = stop_watch_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_->toc("processing_time", true);
    const double pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds((get_clock()->now() - output_msg.header.stamp).nanoseconds()))
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

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::euclidean_cluster::EuclideanClusterObjectDetectorNode)
