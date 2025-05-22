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

#include <autoware/euclidean_cluster_object_detector/voxel_grid_based_euclidean_cluster.hpp>
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

  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_msg =
    std::make_shared<sensor_msgs::msg::PointCloud2>(pointcloud);
  autoware_perception_msgs::msg::DetectedObjects output;
  std::shared_ptr<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster> cluster_;
  float tolerance = 0.7;
  float voxel_leaf_size = 0.3;
  int min_points_number_per_voxel = 1;
  int min_cluster_size = 1;
  int max_cluster_size = 100;
  bool use_height = false;
  cluster_ = std::make_shared<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster>(
    use_height, min_cluster_size, max_cluster_size, tolerance, voxel_leaf_size,
    min_points_number_per_voxel);
  std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
  if (cluster_->cluster(pointcloud_msg, output, clusters)) {
    std::cout << "cluster success" << std::endl;
  } else {
    std::cout << "cluster failed" << std::endl;
  }
  std::cout << "number of output objects " << output.objects.size() << std::endl;

  // the output clusters should has only one cluster with nb_generated_points points
  EXPECT_EQ(output.objects.size(), 1);
}

// Test case 2: Test case when the input pointcloud has only one cluster with points number less
// than min_cluster_size
TEST(VoxelGridBasedEuclideanClusterTest, testcase2)
{
  int nb_generated_points = 1;

  sensor_msgs::msg::PointCloud2 pointcloud = generateClusterWithinVoxel(nb_generated_points);

  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_msg =
    std::make_shared<sensor_msgs::msg::PointCloud2>(pointcloud);
  autoware_perception_msgs::msg::DetectedObjects output;
  std::shared_ptr<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster> cluster_;
  float tolerance = 0.7;
  float voxel_leaf_size = 0.3;
  int min_points_number_per_voxel = 1;
  int min_cluster_size = 2;
  int max_cluster_size = 100;
  bool use_height = false;
  cluster_ = std::make_shared<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster>(
    use_height, min_cluster_size, max_cluster_size, tolerance, voxel_leaf_size,
    min_points_number_per_voxel);
  std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
  if (cluster_->cluster(pointcloud_msg, output, clusters)) {
    std::cout << "cluster success" << std::endl;
  } else {
    std::cout << "cluster failed" << std::endl;
  }
  std::cout << "number of output clusters " << output.objects.size() << std::endl;
  // the output clusters should be empty
  EXPECT_EQ(output.objects.size(), 0);
}

// Test case 3: Test case when the input pointcloud has two clusters with points number greater to
// max_cluster_size
TEST(VoxelGridBasedEuclideanClusterTest, testcase3)
{
  int nb_generated_points = 100;
  sensor_msgs::msg::PointCloud2 pointcloud = generateClusterWithinVoxel(nb_generated_points);

  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_msg =
    std::make_shared<sensor_msgs::msg::PointCloud2>(pointcloud);
  autoware_perception_msgs::msg::DetectedObjects output;
  std::shared_ptr<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster> cluster_;
  float tolerance = 0.7;
  float voxel_leaf_size = 0.3;
  int min_points_number_per_voxel = 1;
  int min_cluster_size = 1;
  int max_cluster_size = 99;  // max_cluster_size is less than nb_generated_points
  bool use_height = false;
  cluster_ = std::make_shared<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster>(
    use_height, min_cluster_size, max_cluster_size, tolerance, voxel_leaf_size,
    min_points_number_per_voxel);
  std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
  if (cluster_->cluster(pointcloud_msg, output, clusters)) {
    std::cout << "cluster success" << std::endl;
  } else {
    std::cout << "cluster failed" << std::endl;
  }
  std::cout << "number of output clusters " << output.objects.size() << std::endl;
  // the output clusters should be emtpy
  EXPECT_EQ(output.objects.size(), 0);
}

// Test default constructor
TEST(VoxelGridBasedEuclideanClusterTest, DefaultConstructor)
{
  auto cluster = std::make_shared<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster>();
  EXPECT_NE(cluster, nullptr);

  // Since the default constructor doesn't initialize parameters, we just check if the object was
  // created successfully
}

// Test three-parameter constructor
TEST(VoxelGridBasedEuclideanClusterTest, ThreeParamConstructor)
{
  bool use_height = true;
  int min_cluster_size = 5;
  int max_cluster_size = 100;

  auto cluster = std::make_shared<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster>(
    use_height, min_cluster_size, max_cluster_size);
  EXPECT_NE(cluster, nullptr);

  // Indirectly test if parameters were set correctly by calling other methods
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr empty_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;

  // This method should return false (unimplemented)
  EXPECT_FALSE(cluster->cluster(empty_cloud, clusters));
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

// Test unimplemented cluster functions
TEST(VoxelGridBasedEuclideanClusterTest, UnimplementedClusterMethod)
{
  auto cluster = std::make_shared<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster>();

  // Test first unimplemented cluster method
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr empty_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
  EXPECT_FALSE(cluster->cluster(empty_cloud, clusters));

  // Test second unimplemented cluster method
  sensor_msgs::msg::PointCloud2::ConstSharedPtr msg =
    std::make_shared<sensor_msgs::msg::PointCloud2>(generateMultiClusterPointCloud(5, 2));
  autoware_perception_msgs::msg::DetectedObjects objects;
  EXPECT_FALSE(cluster->cluster(msg, objects));
}

// Test diagnostics interface (indirectly)
TEST(VoxelGridBasedEuclideanClusterTest, DiagnosticsInterface)
{
  // Create cluster
  auto cluster = std::make_shared<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster>(
    true, 5, 100, 0.5, 0.2, 1);

  // Create a point cloud message
  sensor_msgs::msg::PointCloud2::ConstSharedPtr msg =
    std::make_shared<sensor_msgs::msg::PointCloud2>(generateMultiClusterPointCloud(5, 2));

  // Indirectly test the diagnostics functionality by calling cluster
  // At this point, the diagnostics interface is nullptr, which shouldn't affect functionality
  autoware_perception_msgs::msg::DetectedObjects objects;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;

  // Indirect call to diagnostics functionality shouldn't crash
  EXPECT_TRUE(cluster->cluster(msg, objects, clusters));
}

// Test exceeding max_cluster_size case
TEST(VoxelGridBasedEuclideanClusterTest, ExceedMaxClusterSize)
{
  // Create a cluster with a relatively small max_cluster_size
  int max_cluster_size = 50;
  auto cluster = std::make_shared<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster>(
    true, 5, max_cluster_size, 0.5, 0.2, 1);

  // Create a point cloud message with many points which should exceed max_cluster_size
  sensor_msgs::msg::PointCloud2::ConstSharedPtr msg =
    std::make_shared<sensor_msgs::msg::PointCloud2>(generateMultiClusterPointCloud(200, 1));

  autoware_perception_msgs::msg::DetectedObjects objects;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;

  // Even when exceeding max_cluster_size, function should return true
  EXPECT_TRUE(cluster->cluster(msg, objects, clusters));

  // But since too many points were filtered out, no objects should be detected
  EXPECT_EQ(objects.objects.size(), 0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
