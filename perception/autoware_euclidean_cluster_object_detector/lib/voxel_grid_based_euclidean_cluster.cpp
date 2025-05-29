// Copyright 2020 Tier IV, Inc.
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

#include <autoware/euclidean_cluster_object_detector/voxel_grid_based_euclidean_cluster.hpp>
#include <rclcpp/node.hpp>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::euclidean_cluster
{
VoxelGridBasedEuclideanCluster::VoxelGridBasedEuclideanCluster()
{
}

VoxelGridBasedEuclideanCluster::VoxelGridBasedEuclideanCluster(
  bool use_height, int min_cluster_size, int max_cluster_size)
: EuclideanClusterInterface(use_height, min_cluster_size, max_cluster_size)
{
}

VoxelGridBasedEuclideanCluster::VoxelGridBasedEuclideanCluster(
  bool use_height, int min_cluster_size, int max_cluster_size, float tolerance,
  float voxel_leaf_size, int min_points_number_per_voxel)
: EuclideanClusterInterface(use_height, min_cluster_size, max_cluster_size),
  tolerance_(tolerance),
  voxel_leaf_size_(voxel_leaf_size),
  min_points_number_per_voxel_(min_points_number_per_voxel)
{
}

// After processing all clusters, publish a summary of diagnostics.
void VoxelGridBasedEuclideanCluster::publishDiagnosticsSummary(
  size_t skipped_cluster_count,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_msg)
{
  if (!diagnostics_interface_ptr_) {
    return;
  }
  diagnostics_interface_ptr_->clear();
  std::string summary;
  if (skipped_cluster_count > 0) {
    summary = std::to_string(skipped_cluster_count) +
              " clusters skipped because cluster point size exceeds the maximum allowed " +
              std::to_string(max_cluster_size_);
    diagnostics_interface_ptr_->add_key_value("is_cluster_data_size_within_range", false);
  } else {
    diagnostics_interface_ptr_->add_key_value("is_cluster_data_size_within_range", true);
  }
  diagnostics_interface_ptr_->update_level_and_message(
    skipped_cluster_count > 0 ? static_cast<int8_t>(diagnostic_msgs::msg::DiagnosticStatus::WARN)
                              : static_cast<int8_t>(diagnostic_msgs::msg::DiagnosticStatus::OK),
    summary);
  diagnostics_interface_ptr_->publish(pointcloud_msg->header.stamp);
}

// TODO(badai-nguyen): remove this function when field copying also implemented for
// euclidean_cluster.cpp
bool VoxelGridBasedEuclideanCluster::cluster(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & pointcloud,
  std::vector<pcl::PointCloud<pcl::PointXYZ>> & clusters)
{
  (void)pointcloud;
  (void)clusters;
  return false;
}

bool VoxelGridBasedEuclideanCluster::cluster(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_msg,
  autoware_perception_msgs::msg::DetectedObjects & output_clusters)
{
  (void)input_msg;
  (void)output_clusters;
  return false;
}

bool VoxelGridBasedEuclideanCluster::cluster(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_msg,
  autoware_perception_msgs::msg::DetectedObjects & objects,
  std::vector<pcl::PointCloud<pcl::PointXYZ>> & clusters)
{
  // TODO(Saito) implement use_height is false version

  // create voxel
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
  int point_step = pointcloud_msg->point_step;
  pcl::fromROSMsg(*pointcloud_msg, *pointcloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_map_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, 100000.0);
  voxel_grid_.setMinimumPointsNumberPerVoxel(min_points_number_per_voxel_);
  voxel_grid_.setInputCloud(pointcloud);
  voxel_grid_.setSaveLeafLayout(true);
  voxel_grid_.filter(*voxel_map_ptr);

  // voxel is pressed 2d
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_2d_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto & point : voxel_map_ptr->points) {
    pcl::PointXYZ point2d;
    point2d.x = point.x;
    point2d.y = point.y;
    point2d.z = 0.0;
    pointcloud_2d_ptr->push_back(point2d);
  }

  // create tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(pointcloud_2d_ptr);

  // clustering
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> pcl_euclidean_cluster;
  pcl_euclidean_cluster.setClusterTolerance(tolerance_);
  pcl_euclidean_cluster.setMinClusterSize(1);
  pcl_euclidean_cluster.setMaxClusterSize(max_cluster_size_);
  pcl_euclidean_cluster.setSearchMethod(tree);
  pcl_euclidean_cluster.setInputCloud(pointcloud_2d_ptr);
  pcl_euclidean_cluster.extract(cluster_indices);

  // create map to search cluster index from voxel grid index
  std::unordered_map</* voxel grid index */ int, /* cluster index */ int> map;
  std::vector<sensor_msgs::msg::PointCloud2> temporary_clusters;  // no check about cluster size
  std::vector<size_t> clusters_data_size;
  temporary_clusters.resize(cluster_indices.size());
  size_t clusters_size = cluster_indices.size();
  for (size_t cluster_idx = 0; cluster_idx < clusters_size; ++cluster_idx) {
    const auto & cluster = cluster_indices.at(cluster_idx);
    auto & temporary_cluster = temporary_clusters.at(cluster_idx);
    for (const auto & point_idx : cluster.indices) {
      map[point_idx] = cluster_idx;
    }
    temporary_cluster.height = pointcloud_msg->height;
    temporary_cluster.fields = pointcloud_msg->fields;
    temporary_cluster.point_step = point_step;
    temporary_cluster.data.resize(cluster.indices.size() * point_step);
    clusters_data_size.push_back(0);
  }

  // create vector of point cloud cluster. vector index is voxel grid index.
  size_t pointcloud_size = pointcloud->points.size();
  for (size_t i = 0; i < pointcloud_size; ++i) {
    const auto & point = pointcloud->points.at(i);
    // Temporarily disable array-bounds warning for this specific PCL function call
    // This is a known issue with PCL 1.14 and GCC 13 due to Eigen alignment
#pragma GCC diagnostic push
// cspell: ignore Warray
#pragma GCC diagnostic ignored "-Warray-bounds"
    const int index =
      voxel_grid_.getCentroidIndexAt(voxel_grid_.getGridCoordinates(point.x, point.y, point.z));
#pragma GCC diagnostic pop
    if (map.find(index) != map.end()) {
      auto & cluster_data_size = clusters_data_size.at(map[index]);
      if (
        cluster_data_size >
        static_cast<std::size_t>(max_cluster_size_) * static_cast<std::size_t>(point_step)) {
        continue;
      }
      std::memcpy(
        &temporary_clusters.at(map[index]).data[cluster_data_size],
        &pointcloud_msg->data[i * point_step], point_step);
      cluster_data_size += point_step;
      if (cluster_data_size == temporary_clusters.at(map[index]).data.size()) {
        temporary_clusters.at(map[index])
          .data.resize(temporary_clusters.at(map[index]).data.size() * 2);
      }
    }
  }

  // build output and check cluster size
  {
    size_t skipped_cluster_count = 0;  // Count the skipped clusters
    size_t temporary_clusters_size = temporary_clusters.size();
    for (size_t i = 0; i < temporary_clusters_size; ++i) {
      auto & i_cluster_data_size = clusters_data_size.at(i);
      int cluster_size = static_cast<int>(i_cluster_data_size / point_step);
      if (cluster_size < min_cluster_size_) {
        // Cluster size is below the minimum threshold; skip without messaging.
        continue;
      }
      if (cluster_size > max_cluster_size_) {
        // Cluster size exceeds the maximum threshold; log a warning.
        skipped_cluster_count++;
        continue;
      }
      const auto & cluster = temporary_clusters.at(i);

      // Create a temporary cluster with correct size for getting the centroid
      sensor_msgs::msg::PointCloud2 temp_cluster = cluster;
      temp_cluster.data.resize(i_cluster_data_size);
      // temp_cluster.header = pointcloud_msg->header;
      // temp_cluster.is_bigendian = pointcloud_msg->is_bigendian;
      // temp_cluster.is_dense = pointcloud_msg->is_dense;
      // temp_cluster.point_step = point_step;
      // temp_cluster.row_step = i_cluster_data_size / pointcloud_msg->height;
      // temp_cluster.width = i_cluster_data_size / point_step / pointcloud_msg->height;

      autoware_perception_msgs::msg::DetectedObject object;
      object.kinematics.pose_with_covariance.pose.position = getCentroid(temp_cluster);

      autoware_perception_msgs::msg::ObjectClassification classification;
      classification.label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
      classification.probability = 1.0f;
      object.classification.emplace_back(classification);

      objects.objects.push_back(object);

      pcl::PointCloud<pcl::PointXYZ> cluster_point_cloud;

      for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cluster, "x"), iter_y(cluster, "y"),
           iter_z(cluster, "z");
           iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        pcl::PointXYZ point;
        point.x = *iter_x;
        point.y = *iter_y;
        point.z = *iter_z;
        cluster_point_cloud.push_back(point);
      }
      clusters.push_back(cluster_point_cloud);
    }
    objects.header = pointcloud_msg->header;
    // Publish the diagnostics summary.
    publishDiagnosticsSummary(skipped_cluster_count, pointcloud_msg);
  }

  return true;
}

}  // namespace autoware::euclidean_cluster
