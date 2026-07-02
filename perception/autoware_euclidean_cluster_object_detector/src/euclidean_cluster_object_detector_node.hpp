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

#ifndef EUCLIDEAN_CLUSTER_OBJECT_DETECTOR_NODE_HPP_
#define EUCLIDEAN_CLUSTER_OBJECT_DETECTOR_NODE_HPP_

#include "euclidean_cluster_object_detector.hpp"

#include <autoware_utils_debug/debug_publisher.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>

namespace autoware::euclidean_cluster
{

class EuclideanClusterObjectDetectorNode : public rclcpp::Node
{
public:
  explicit EuclideanClusterObjectDetectorNode(const rclcpp::NodeOptions & options);

private:
  /**
   * @brief Callback func for point cloud subscription
   * @param input_msg Point cloud message
   */
  void on_point_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_;
  rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr pub_clusters_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_debug_;

  std::unique_ptr<EuclideanClusterObjectDetector> detector_;
  std::unique_ptr<autoware_utils_system::StopWatch<std::chrono::milliseconds>> stop_watch_;
  std::unique_ptr<autoware_utils_debug::DebugPublisher> debug_publisher_;
};

}  // namespace autoware::euclidean_cluster

#endif  // EUCLIDEAN_CLUSTER_OBJECT_DETECTOR_NODE_HPP_
