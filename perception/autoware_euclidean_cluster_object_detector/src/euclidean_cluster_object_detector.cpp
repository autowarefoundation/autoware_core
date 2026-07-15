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

#include "../lib/ros_conversions.hpp"

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include <functional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::euclidean_cluster
{

// ========================= STANDARD CLUSTERING ========================= //

EuclideanClusterObjectDetector::EuclideanClusterObjectDetector(const EuclideanClusterParams & param)
: param_(param)
{
}

ClusterFeatureResult EuclideanClusterObjectDetector::cluster(
  const sensor_msgs::msg::PointCloud2 & input_msg) const
{
  ClusterFeatureResult result;

  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(input_msg, *raw_cloud);

  if (raw_cloud->empty()) {
    result.cluster_message.header = input_msg.header;
    result.debug_message.header = input_msg.header;
    return result;
  }

  auto [valid_clusters, skipped_count] = cluster_standard(raw_cloud);

  result.skipped_cluster_count = skipped_count;
  convert_clusters_to_detected_objects(input_msg.header, valid_clusters, result.cluster_message);
  convert_clusters_to_debug_point_cloud(input_msg.header, valid_clusters, result.debug_message);

  return result;
}

std::pair<std::vector<pcl::PointCloud<pcl::PointXYZ>>, size_t>
EuclideanClusterObjectDetector::cluster_standard(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input_cloud) const
{
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  if (!param_.use_height) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_2d_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pointcloud_2d_ptr->points.reserve(input_cloud->points.size());

    for (const auto & point : input_cloud->points) {
      pcl::PointXYZ point2d;
      point2d.x = point.x;
      point2d.y = point.y;
      point2d.z = 0.0;
      pointcloud_2d_ptr->push_back(point2d);
    }
    pointcloud_ptr = pointcloud_2d_ptr;

  } else {
    pointcloud_ptr = input_cloud;
  }

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(pointcloud_ptr);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> pcl_euclidean_cluster;
  pcl_euclidean_cluster.setClusterTolerance(param_.tolerance);
  pcl_euclidean_cluster.setMinClusterSize(1);
  pcl_euclidean_cluster.setMaxClusterSize(param_.max_cluster_size);
  pcl_euclidean_cluster.setSearchMethod(tree);
  pcl_euclidean_cluster.setInputCloud(pointcloud_ptr);
  pcl_euclidean_cluster.extract(cluster_indices);

  std::vector<pcl::PointCloud<pcl::PointXYZ>> valid_clusters;
  size_t skipped_count = 0;

  valid_clusters.reserve(cluster_indices.size());
  for (const auto & cluster_item : cluster_indices) {
    if (static_cast<int>(cluster_item.indices.size()) < param_.min_cluster_size) {
      continue;
    }
    if (static_cast<int>(cluster_item.indices.size()) > param_.max_cluster_size) {
      skipped_count++;
      continue;
    }

    auto & cloud_cluster = valid_clusters.emplace_back();
    cloud_cluster.points.reserve(cluster_item.indices.size());
    for (const auto & point_idx : cluster_item.indices) {
      cloud_cluster.points.push_back(input_cloud->points[point_idx]);
    }
    cloud_cluster.width = cloud_cluster.points.size();
    cloud_cluster.height = 1;
    cloud_cluster.is_dense = false;
  }

  return std::make_pair(std::move(valid_clusters), skipped_count);
}

// ========================= VOXEL-GRID-BASED CLUSTERING ========================= //

VoxelGridBasedEuclideanClusterDetector::VoxelGridBasedEuclideanClusterDetector(
  const EuclideanClusterParams & param)
: param_(param)
{
}

ClusterFeatureResult VoxelGridBasedEuclideanClusterDetector::cluster(
  const sensor_msgs::msg::PointCloud2 & input_msg) const
{
  ClusterFeatureResult result;
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(input_msg, *raw_cloud);

  if (raw_cloud->empty()) {
    result.cluster_message.header = input_msg.header;
    result.debug_message.header = input_msg.header;
    return result;
  }

  auto [valid_clusters, skipped_count] = cluster_voxel_grid(raw_cloud);

  result.skipped_cluster_count = skipped_count;
  convert_clusters_to_detected_objects(input_msg.header, valid_clusters, result.cluster_message);
  convert_clusters_to_debug_point_cloud(input_msg.header, valid_clusters, result.debug_message);

  return result;
}

std::pair<std::vector<pcl::PointCloud<pcl::PointXYZ>>, size_t>
VoxelGridBasedEuclideanClusterDetector::cluster_voxel_grid(
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
    return std::pair<std::vector<pcl::PointCloud<pcl::PointXYZ>>, size_t>{};
  }

  // In legacy code, the previous authors unconditionally flatten centroids to 2D for clustering.
  // This sounds weird, but right now as this is a refactoring effort, I will keep as it is.
  // Thus here I will match legacy code's behavior which bypassed `param_.use_height`.
  pcl::PointCloud<pcl::PointXYZ>::Ptr flattened_centroids(new pcl::PointCloud<pcl::PointXYZ>);
  flattened_centroids->points.reserve(voxel_centroids->points.size());
  for (const auto & point : voxel_centroids->points) {
    flattened_centroids->points.emplace_back(point.x, point.y, 0.0f);
  }

  // 2. Create KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(flattened_centroids);

  // 3. Euclidean clustering on voxel centroids
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(param_.tolerance);
  ec.setMinClusterSize(1);
  ec.setMaxClusterSize(param_.max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(flattened_centroids);
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

  return std::make_pair(std::move(valid_clusters), skipped_cluster_count);
}
}  // namespace autoware::euclidean_cluster
