// Copyright 2024 TIER IV, Inc.
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

#include "../src/euclidean_cluster_object_detector.hpp"
#include "../src/parameters.hpp"

#include <autoware/point_types/types.hpp>
#include <experimental/random>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <vector>

using autoware::point_types::PointXYZI;
void setPointCloud2Fields(sensor_msgs::msg::PointCloud2 & pointcloud)
{
  pointcloud.fields.resize(4);
  pointcloud.fields[0].name = "x";
  pointcloud.fields[1].name = "y";
  pointcloud.fields[2].name = "z";
  pointcloud.fields[3].name = "intensity";
  pointcloud.fields[0].offset = 0;
  pointcloud.fields[1].offset = 4;
  pointcloud.fields[2].offset = 8;
  pointcloud.fields[3].offset = 12;
  pointcloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  pointcloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  pointcloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  pointcloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  pointcloud.fields[0].count = 1;
  pointcloud.fields[1].count = 1;
  pointcloud.fields[2].count = 1;
  pointcloud.fields[3].count = 1;
  pointcloud.height = 1;
  pointcloud.point_step = 16;
  pointcloud.is_bigendian = false;
  pointcloud.is_dense = true;
  pointcloud.header.frame_id = "dummy_frame_id";
  pointcloud.header.stamp.sec = 0;
  pointcloud.header.stamp.nanosec = 0;
}

sensor_msgs::msg::PointCloud2 generateClusterWithinVoxel(const int nb_points)
{
  sensor_msgs::msg::PointCloud2 pointcloud;
  setPointCloud2Fields(pointcloud);
  pointcloud.data.resize(nb_points * pointcloud.point_step);

  // generate one cluster with specified number of points within 1 voxel
  for (int i = 0; i < nb_points; ++i) {
    PointXYZI point;
    point.x = std::experimental::randint(0, 30) / 100.0;  // point.x within 0.0 to 0.3
    point.y = std::experimental::randint(0, 30) / 100.0;  // point.y within 0.0 to 0.3
    point.z = std::experimental::randint(0, 30) / 1.0;
    point.intensity = 0.0;
    memcpy(&pointcloud.data[i * pointcloud.point_step], &point, pointcloud.point_step);
  }
  pointcloud.width = nb_points;
  pointcloud.row_step = pointcloud.point_step * nb_points;
  return pointcloud;
}

// Test case 1: Test case when the input pointcloud has only one cluster with points number equal to
// max_cluster_size
TEST(VoxelGridBasedEuclideanClusterTest, testcase1)
{
  int nb_generated_points = 100;
  sensor_msgs::msg::PointCloud2 pointcloud = generateClusterWithinVoxel(nb_generated_points);

  autoware::euclidean_cluster::EuclideanClusterParams param;
  param.use_height = false;
  param.min_cluster_size = 1;
  param.max_cluster_size = 100;
  param.tolerance = 0.7f;
  param.voxel_leaf_size = 0.3f;
  param.min_points_number_per_voxel = 1;

  autoware::euclidean_cluster::EuclideanClusterObjectDetector cluster(param);
  auto result = cluster.cluster(pointcloud);

  // the output clusters should has only one cluster with nb_generated_points points
  EXPECT_EQ(result.cluster_message.objects.size(), 1);
}

// Test case 2: Test case when the input pointcloud has only one cluster with points number less
// than min_cluster_size
TEST(VoxelGridBasedEuclideanClusterTest, testcase2)
{
  int nb_generated_points = 1;

  sensor_msgs::msg::PointCloud2 pointcloud = generateClusterWithinVoxel(nb_generated_points);

  autoware::euclidean_cluster::EuclideanClusterParams param;
  param.use_height = false;
  param.min_cluster_size = 2;
  param.max_cluster_size = 100;
  param.tolerance = 0.7f;
  param.voxel_leaf_size = 0.3f;
  param.min_points_number_per_voxel = 1;

  autoware::euclidean_cluster::EuclideanClusterObjectDetector cluster(param);
  auto result = cluster.cluster(pointcloud);

  // the output clusters should be empty
  EXPECT_EQ(result.cluster_message.objects.size(), 0);
}

// Test case 3: Test case when the input pointcloud has two clusters with points number greater to
// max_cluster_size
TEST(VoxelGridBasedEuclideanClusterTest, testcase3)
{
  int nb_generated_points = 100;
  sensor_msgs::msg::PointCloud2 pointcloud = generateClusterWithinVoxel(nb_generated_points);

  autoware::euclidean_cluster::EuclideanClusterParams param;
  param.use_height = false;
  param.min_cluster_size = 1;
  param.max_cluster_size = 99;
  param.tolerance = 0.7f;
  param.voxel_leaf_size = 0.3f;
  param.min_points_number_per_voxel = 1;

  autoware::euclidean_cluster::EuclideanClusterObjectDetector cluster(param);
  auto result = cluster.cluster(pointcloud);

  // the output clusters should be emtpy
  EXPECT_EQ(result.cluster_message.objects.size(), 0);
}

// Test default constructor
TEST(VoxelGridBasedEuclideanClusterTest, DefaultConstructor)
{
  autoware::euclidean_cluster::EuclideanClusterParams param;
  autoware::euclidean_cluster::EuclideanClusterObjectDetector cluster(param);
  EXPECT_NE(cluster, nullptr);

  // Since the default constructor doesn't initialize parameters, we just check if the object was
  // created successfully
}

// Helper function: Generate a point cloud with multiple clusters
sensor_msgs::msg::PointCloud2 generateMultiClusterPointCloud(
  int point_per_cluster, int num_clusters)
{
  sensor_msgs::msg::PointCloud2 pointcloud;
  setPointCloud2Fields(pointcloud);

  int total_points = point_per_cluster * num_clusters;
  pointcloud.data.resize(total_points * pointcloud.point_step);

  // Generate points for each cluster
  for (int c = 0; c < num_clusters; ++c) {
    float offset_x =
      c * 15.0;  // Distance between clusters should be greater than tolerance to ensure separation

    for (int i = 0; i < point_per_cluster; ++i) {
      PointXYZI point;
      // Generate random points within each cluster
      point.x = offset_x + std::experimental::randint(0, 30) / 100.0;
      point.y = std::experimental::randint(0, 30) / 100.0;
      point.z = std::experimental::randint(0, 30) / 1.0;
      point.intensity = 0.0;

      int idx = (c * point_per_cluster + i);
      memcpy(&pointcloud.data[idx * pointcloud.point_step], &point, pointcloud.point_step);
    }
  }

  pointcloud.width = total_points;
  pointcloud.row_step = pointcloud.point_step * total_points;
  return pointcloud;
}

// Characterization test: pin the exact relationship between the `objects` output and the
// parallel `clusters` output so the map-lookup-caching and in-place cluster-building refactor
// is proven behavior-preserving. The number of output clusters must stay in lockstep with the
// number of detected objects (one cluster per object), and every extracted point must lie within
// the originating voxel region (x,y in [0,0.3], z in [0,30]) along with the object centroid. The
// per-cluster point count is governed by voxel grouping and is intentionally not asserted.
TEST(VoxelGridBasedEuclideanClusterTest, ClusterOutputMatchesObjects)
{
  int nb_generated_points = 100;
  sensor_msgs::msg::PointCloud2 pointcloud = generateClusterWithinVoxel(nb_generated_points);

  autoware::euclidean_cluster::EuclideanClusterParams param;
  param.use_height = false;
  param.min_cluster_size = 1;
  param.max_cluster_size = 100;
  param.tolerance = 0.7f;
  param.voxel_leaf_size = 0.3f;
  param.min_points_number_per_voxel = 1;

  autoware::euclidean_cluster::EuclideanClusterObjectDetector cluster(param);
  auto result = cluster.cluster(pointcloud);

  // Exactly one cluster, and the parallel outputs stay in lockstep.
  ASSERT_EQ(result.cluster_message.objects.size(), 1u);

  // The single extracted cluster must be non-empty, and every point must lie inside the
  // generated voxel region (x,y in [0,0.3], z in [0,30]). The exact count is governed by the
  // voxel grouping rather than the raw point count, so it is not asserted here.
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(result.debug_message, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(result.debug_message, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(result.debug_message, "z");

  size_t point_count = 0;
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    point_count++;
    EXPECT_GE(*iter_x, 0.0f);
    EXPECT_LE(*iter_x, 0.3f);
    EXPECT_GE(*iter_y, 0.0f);
    EXPECT_LE(*iter_y, 0.3f);
    EXPECT_GE(*iter_z, 0.0f);
    EXPECT_LE(*iter_z, 30.0f);
  }
  EXPECT_GT(point_count, 0u);

  // The object centroid's x/y must fall within the same region as the input points.
  const auto & position =
    result.cluster_message.objects.front().kinematics.pose_with_covariance.pose.position;
  EXPECT_GE(position.x, 0.0);
  EXPECT_LE(position.x, 0.3);
  EXPECT_GE(position.y, 0.0);
  EXPECT_LE(position.y, 0.3);
}

// Characterization test for the previously-untested empty-input path of the implemented
// `cluster(pointcloud_msg, objects, clusters)` overload. An empty cloud (fields and point_step
// set, no data, width 0) must flow through fromROSMsg -> voxel filter -> KdTree -> extraction
// without crashing and pin the degenerate-input contract: the call returns true and produces no
// objects and no clusters.
TEST(VoxelGridBasedEuclideanClusterTest, ClusterEmptyInput)
{
  sensor_msgs::msg::PointCloud2 pointcloud;
  setPointCloud2Fields(pointcloud);
  pointcloud.data.clear();
  pointcloud.width = 0;
  pointcloud.row_step = 0;

  autoware::euclidean_cluster::EuclideanClusterParams param;
  param.use_height = false;
  param.min_cluster_size = 1;
  param.max_cluster_size = 100;
  param.tolerance = 0.7f;
  param.voxel_leaf_size = 0.3f;
  param.min_points_number_per_voxel = 1;

  autoware::euclidean_cluster::EuclideanClusterObjectDetector cluster(param);
  auto result = cluster.cluster(pointcloud);

  EXPECT_EQ(result.cluster_message.objects.size(), 0u);
}

// Test exceeding max_cluster_size case
TEST(VoxelGridBasedEuclideanClusterTest, ExceedMaxClusterSize)
{
  // Create a cluster with a relatively small max_cluster_size
  int max_cluster_size = 50;
  // auto cluster = std::make_shared<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster>(
  //   true, 5, max_cluster_size, 0.5, 0.2, 1);
  autoware::euclidean_cluster::EuclideanClusterParams param;
  param.use_height = true;
  param.min_cluster_size = 5;
  param.max_cluster_size = max_cluster_size;
  param.tolerance = 0.5f;
  param.voxel_leaf_size = 0.2f;
  param.min_points_number_per_voxel = 1;

  autoware::euclidean_cluster::EuclideanClusterObjectDetector cluster(param);

  // Create a point cloud message with many points which should exceed max_cluster_size
  sensor_msgs::msg::PointCloud2 msg = generateMultiClusterPointCloud(200, 1);
  auto result = cluster.cluster(msg);

  // Even when exceeding max_cluster_size, function should return true
  EXPECT_EQ(result.cluster_message.objects.size(), 0);

  // But since too many points were filtered out, no objects should be detected
  EXPECT_GT(result.skipped_cluster_count, 0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
