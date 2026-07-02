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
tl::expected<std::vector<pcl::PointCloud<pcl::PointXYZ>>, std::string>
EuclideanClusterObjectDetector::cluster_standard(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input_cloud) const
{
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr search_cloud = input_cloud;

  // 2D (BEV) flatten/projection to XY plan if height is not used for clustering
  if (!param_.use_height) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_2d->points.reserve(input_cloud->points.size());
    for (const auto & point : input_cloud->points) {
      cloud_2d->push_back(pcl::PointXYZ(point.x, point.y, 0.0f));
    }
    search_cloud = cloud_2d;
  }

  // Build KD-Tree for nearest neighbor search
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(search_cloud);

  // Perform Euclidean clustering
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(param_.tolerance);
  ec.setMinClusterSize(param_.min_cluster_size);
  ec.setMaxClusterSize(param_.max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(search_cloud);
  ec.extract(cluster_indices);

  // Build pure C++ output structures
  std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
  clusters.reserve(cluster_indices.size());

  for (const auto & indices : cluster_indices) {
    auto & cloud_cluster = clusters.emplace_back();
    cloud_cluster.points.reserve(indices.indices.size());

    // Map back to original input_cloud to preserve physical Z vals
    for (const auto & idx : indices.indices) {
      cloud_cluster.points.push_back(input_cloud->points[idx]);
    }
    cloud_cluster.width = cloud_cluster.points.size();
    cloud_cluster.height = 1;
    cloud_cluster.is_dense = false;
  }

  return clusters;
}

}  // namespace autoware::euclidean_cluster
