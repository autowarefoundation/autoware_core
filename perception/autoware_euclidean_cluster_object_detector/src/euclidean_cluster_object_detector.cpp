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

#include "euclidean_cluster_object_detector.hpp"

#include "parameters.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <string>
#include <unordered_map>
#include <utility>
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
ClusterFeatureResult EuclideanClusterObjectDetector::cluster(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input_cloud) const
{
  if (!input_cloud || input_cloud->empty()) {
    return ClusterFeatureResult{};
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
ClusterFeatureResult EuclideanClusterObjectDetector::cluster_standard(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input_cloud) const
{
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr search_cloud = input_cloud;

  // If not use_height, flatten to 2D BEV (adhering to legacy logic)
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

  return ClusterFeatureResult{std::move(clusters), 0};
}

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
ClusterFeatureResult EuclideanClusterObjectDetector::cluster_voxel_grid(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input_cloud) const
{
  // 1. Downsample with voxel grid
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_centroids(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setLeafSize(param_.voxel_leaf_size, param_.voxel_leaf_size, 100000.0f);
  voxel_grid.setMinimumPointsNumberPerVoxel(param_.min_points_number_per_voxel);
  voxel_grid.setInputCloud(input_cloud);
  voxel_grid.setSaveLeafLayout(true);
  voxel_grid.filter(*voxel_centroids);

  if (voxel_centroids->empty()) {
    return ClusterFeatureResult{};
  }

  // 2. Create KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(voxel_centroids);

  // 3. Euclidean clustering on voxel centroids
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(param_.tolerance);
  ec.setMinClusterSize(1);
  ec.setMaxClusterSize(param_.max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(voxel_centroids);
  ec.extract(cluster_indices);

  // 4. Create map to search cluster index from voxel grid index
  std::unordered_map<int, size_t> voxel_to_cluster_map;
  voxel_to_cluster_map.reserve(voxel_centroids->points.size());

  for (size_t cluster_id = 0; cluster_id < cluster_indices.size(); ++cluster_id) {
    for (const auto & centroid_idx : cluster_indices[cluster_id].indices) {
      const auto & p = voxel_centroids->points[centroid_idx];

// Temporarily disable array-bounds warning for this specific PCL function call
// This is a known issue with PCL 1.14 and GCC 13 due to Eigen alignment
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
      int voxel_1d_idx =
        voxel_grid.getCentroidIndexAt(voxel_grid.getGridCoordinates(p.x, p.y, p.z));
#pragma GCC diagnostic pop

      voxel_to_cluster_map[voxel_1d_idx] = cluster_id;
    }
  }

  // 5. Stream raw input cloud & bucket points into their clusters
  std::vector<pcl::PointCloud<pcl::PointXYZ>> temp_clusters(cluster_indices.size());

  for (const auto & point : input_cloud->points) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
    int voxel_1d_idx =
      voxel_grid.getCentroidIndexAt(voxel_grid.getGridCoordinates(point.x, point.y, point.z));
#pragma GCC diagnostic pop

    auto map_it = voxel_to_cluster_map.find(voxel_1d_idx);
    if (map_it != voxel_to_cluster_map.end()) {
      size_t target_cluster_id = map_it->second;

      // Here I try to follow EXACTLY the legacy code's logic, although it's a bit funny:
      // In legacy code it was like this:
      // if (
      //     cluster_data_size >
      //     static_cast<std::size_t>(max_cluster_size_) * static_cast<std::size_t>(point_step)
      // ) { continue; }
      // Seems like authors intentionally allowed this cluster to exceed max size by 1 point so
      // later it could trigger the skip with this:
      // if (cluster_size > max_cluster_size_) {
      //     skipped_cluster_count++;
      //     continue;
      // }
      // I'm gonna do the same logic, but cleaner.
      if (
        temp_clusters[target_cluster_id].points.size() <=
        static_cast<size_t>(param_.max_cluster_size)) {
        temp_clusters[target_cluster_id].points.push_back(point);
      }
    }
  }

  // 6. Filter final clusters by size constraints
  std::vector<pcl::PointCloud<pcl::PointXYZ>> valid_clusters;
  valid_clusters.reserve(temp_clusters.size());
  size_t skipped_cluster_count = 0;

  // 7. Build final output
  for (auto & cloud_cluster : temp_clusters) {
    size_t cluster_size = cloud_cluster.points.size();

    // Ignore small noises, log skipped big cluster
    if (cluster_size < static_cast<size_t>(param_.min_cluster_size)) {
      continue;
    }
    if (cluster_size > static_cast<size_t>(param_.max_cluster_size)) {
      skipped_cluster_count++;
      continue;
    }

    cloud_cluster.width = cluster_size;
    cloud_cluster.height = 1;
    cloud_cluster.is_dense = false;
    valid_clusters.push_back(std::move(cloud_cluster));
  }

  return ClusterFeatureResult{std::move(valid_clusters), skipped_cluster_count};
}

}  // namespace autoware::euclidean_cluster
