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

#ifndef EUCLIDEAN_CLUSTER_OBJECT_DETECTOR_HPP_
#define EUCLIDEAN_CLUSTER_OBJECT_DETECTOR_HPP_

#include "parameters.hpp"

#include <tl_expected/expected.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>

namespace autoware::euclidean_cluster
{

class EuclideanClusterObjectDetector
{
public:
  explicit EuclideanClusterObjectDetector(const EuclideanClusterParams & param);

  // Single public interface for everything
  tl::expected<std::vector<pcl::PointCloud<pcl::PointXYZ>>, std::string> cluster(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input_cloud) const;

private:
  EuclideanClusterParams param_;

  /**
   * @brief Helper func to perform standard Euclidean clustering on a point cloud.
   * Basically this is the brute-force approach without downsampling, a.k.a slower but accurate (I
   * guess?). This algorithm loads point cloud into 3D KD-Tree for O(logN) nearest neighbor search,
   * then performs clustering based on distance tolerance and min/max cluster size.
   *
   * @param input_cloud Input point cloud to cluster.
   *
   * @return tl::expected<std::vector<pcl::PointCloud<pcl::PointXYZ>>, std::string> A vector of
   * point clouds, each representing a cluster.
   */
  [[nodiscard]] tl::expected<std::vector<pcl::PointCloud<pcl::PointXYZ>>, std::string>
  cluster_standard(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input_cloud) const;

  /**
   * @brief Helper func to perform voxel grid downsampling on a point cloud, then perform Euclidean
   * clustering. This is a faster but less accurate approach, as downsampling may remove points
   * important for clustering. Still, it's useful for large point clouds where speed is more
   * important than accuracy.
   *
   * @param input_cloud Input point cloud to cluster.
   *
   * @return tl::expected<std::vector<pcl::PointCloud<pcl::PointXYZ>>, std::string> A vector of
   * point clouds, each representing a cluster.
   */
  [[nodiscard]] tl::expected<std::vector<pcl::PointCloud<pcl::PointXYZ>>, std::string>
  cluster_voxel_grid(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input_cloud) const;
};

}  // namespace autoware::euclidean_cluster

#endif  // EUCLIDEAN_CLUSTER_OBJECT_DETECTOR_HPP_
