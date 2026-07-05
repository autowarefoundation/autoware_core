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

#ifndef EUCLIDEAN_CLUSTER_OBJECT_DETECTOR_HPP_
#define EUCLIDEAN_CLUSTER_OBJECT_DETECTOR_HPP_

#include "parameters.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>

namespace autoware::euclidean_cluster
{

// Struct to bubble up diagnostic telemetry without ROS dependency
struct ClusterFeatureResult
{
  std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
  size_t skipped_cluster_count{0};
};

class EuclideanClusterObjectDetector
{
public:
  explicit EuclideanClusterObjectDetector(const EuclideanClusterParams & param);

  // Single public interface for everything
  ClusterFeatureResult cluster(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input_cloud) const;

private:
  EuclideanClusterParams param_;

  // Helper func to perform standard Euclidean clustering on a point cloud
  [[nodiscard]] ClusterFeatureResult cluster_standard(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input_cloud) const;

  // Helper func to perform voxel grid downsampling on a point cloud, then perform Euclidean
  [[nodiscard]] ClusterFeatureResult cluster_voxel_grid(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input_cloud) const;
};

}  // namespace autoware::euclidean_cluster

#endif  // EUCLIDEAN_CLUSTER_OBJECT_DETECTOR_HPP_
