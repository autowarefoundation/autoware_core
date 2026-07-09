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

#include "euclidean_cluster_node.hpp"

#include "../lib/ros_conversions.hpp"
#include "parameters.hpp"

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace autoware::euclidean_cluster
{
EuclideanClusterNode::EuclideanClusterNode(const rclcpp::NodeOptions & options)
: Node("euclidean_cluster_node", options)
{
  EuclideanClusterParams param;
  param.use_height = declare_parameter<bool>("use_height", false);
  param.min_cluster_size = static_cast<int>(declare_parameter<int64_t>("min_cluster_size", 3));
  param.max_cluster_size = static_cast<int>(declare_parameter<int64_t>("max_cluster_size", 200));
  param.tolerance = static_cast<float>(declare_parameter<double>("tolerance", 1.0));

  // cppcheck-suppress useInitializationList
  detector_ = std::make_unique<EuclideanClusterObjectDetector>(param);

  using std::placeholders::_1;
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&EuclideanClusterNode::on_point_cloud, this, _1));

  cluster_pub_ = this->create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
    "output", rclcpp::QoS{1});
  debug_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("debug/clusters", 1);
  stop_watch_ptr_ = std::make_unique<autoware_utils_system::StopWatch<std::chrono::milliseconds>>();
  debug_publisher_ =
    std::make_unique<autoware_utils_debug::DebugPublisher>(this, "euclidean_cluster");

  stop_watch_ptr_->tic("cyclic_time");
  stop_watch_ptr_->tic("processing_time");
}

void EuclideanClusterNode::on_point_cloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
{
  stop_watch_ptr_->toc("processing_time", true);

  // Check for empty point cloud
  if (input_msg->data.empty() || input_msg->width * input_msg->height == 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Received empty point cloud, skipping processing.");
    // Publish empty DetectedObjects
    autoware_perception_msgs::msg::DetectedObjects output;
    output.header = input_msg->header;
    cluster_pub_->publish(output);
    return;
  }

  // Now passing to isolated core logic
  const auto result = detector_->cluster(*input_msg);

  cluster_pub_->publish(result.cluster_message);

  // build debug msg
  if (debug_pub_->get_subscription_count() >= 1) {
    debug_pub_->publish(result.debug_message);
  }
  if (debug_publisher_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
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

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::euclidean_cluster::EuclideanClusterNode)
