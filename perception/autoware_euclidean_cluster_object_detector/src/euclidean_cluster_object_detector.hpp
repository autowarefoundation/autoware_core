// Copyright 2020 TIER IV, Inc.
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

#pragma once

#include "parameters.hpp"

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::euclidean_cluster
{
struct ClusterFeatureResult
{
  autoware_perception_msgs::msg::DetectedObjects cluster_message{};
  sensor_msgs::msg::PointCloud2 debug_message{};
  size_t skipped_cluster_count{0};
};

class EuclideanClusterObjectDetector
{
public:
  explicit EuclideanClusterObjectDetector(const EuclideanClusterParams & param);
  [[nodiscard]] ClusterFeatureResult cluster(const sensor_msgs::msg::PointCloud2 & input_msg) const;

private:
  EuclideanClusterParams param_;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_;

  // Addressing Akamine-san's concern, here I make the package decides cluster strategy only once at
  // startup
  using ClusterStrategy =
    std::function<std::pair<std::vector<pcl::PointCloud<pcl::PointXYZ>>, size_t>(
      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &)>;
  ClusterStrategy strategy_;

  [[nodiscard]] std::pair<std::vector<pcl::PointCloud<pcl::PointXYZ>>, size_t> cluster_standard(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input_cloud) const;

  [[nodiscard]] std::pair<std::vector<pcl::PointCloud<pcl::PointXYZ>>, size_t> cluster_voxel_grid(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input_cloud) const;
};
}  // namespace autoware::euclidean_cluster
