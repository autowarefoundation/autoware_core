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

#include <memory>

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
  param.min_cluster_size = declare_parameter<int>("min_cluster_size", 3);
  param.max_cluster_size = declare_parameter<int>("max_cluster_size", 200);
  param.tolerance = declare_parameter<float>("tolerance", 1.0f);
  param.voxel_leaf_size = declare_parameter<float>("voxel_leaf_size", 0.0f);
  param.min_points_number_per_voxel = declare_parameter<int>("min_points_number_per_voxel", 1);

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

  stop_watch_->tic("cyclic_time");
  stop_watch_->tic("processing_time");
}

}  // namespace autoware::euclidean_cluster
