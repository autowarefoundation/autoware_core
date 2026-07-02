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

#include "euclidean_cluster_object_detector.hpp"

#include "parameters.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <string>
#include <vector>

namespace autoware::euclidean_cluster
{

EuclideanClusterObjectDetector::EuclideanClusterObjectDetector(const EuclideanClusterParams & param)
: param_(param)
{
}

/**
 * @brief Public interface to perform Euclidean clustering on a point cloud. This function decides
 * whether to use standard clustering or voxel grid downsampling based on given params.
 *
 * @param input_cloud Input point cloud to cluster.
 *
 * @return tl::expected<std::vector<pcl::PointCloud<pcl::PointXYZ>>, std::string> A vector of point
 * clouds, each representing a cluster.
 */
tl::expected<std::vector<pcl::PointCloud<pcl::PointXYZ>>, std::string>
EuclideanClusterObjectDetector::cluster(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input_cloud) const
{
  if (!input_cloud || input_cloud->empty()) {
    return std::vector<pcl::PointCloud<pcl::PointXYZ>>{};
  }

  if (param_.voxel_leaf_size > 0.0f) {
    return cluster_voxel_grid(input_cloud);
  }

  return cluster_standard(input_cloud);
}

}  // namespace autoware::euclidean_cluster
