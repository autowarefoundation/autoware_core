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
#include "parameters.hpp"

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

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

ClusterFeatureResult EuclideanClusterObjectDetector::cluster(
  const sensor_msgs::msg::PointCloud2 & input_msg) const
{
  ClusterFeatureResult result{};

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
  for (const auto & cluster : cluster_indices) {
    if (static_cast<int>(cluster.indices.size()) < param_.min_cluster_size) {
      continue;
    }
    if (static_cast<int>(cluster.indices.size()) > param_.max_cluster_size) {
      skipped_count++;
      continue;
    }

    auto & cloud_cluster = valid_clusters.emplace_back();
    cloud_cluster.points.reserve(cluster.indices.size());
    for (const auto & point_idx : cluster.indices) {
      cloud_cluster.points.push_back(input_cloud->points[point_idx]);
    }
    cloud_cluster.width = cloud_cluster.points.size();
    cloud_cluster.height = 1;
    cloud_cluster.is_dense = false;
  }

  return std::make_pair(std::move(valid_clusters), skipped_count);
}
}  // namespace autoware::euclidean_cluster
