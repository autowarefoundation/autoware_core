// Copyright 2025 TIER IV, Inc.
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

#include <autoware/euclidean_cluster_object_detector/euclidean_cluster.hpp>
#include <autoware/point_types/types.hpp>

#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <vector>

using autoware::point_types::PointXYZI;

class EuclideanClusterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create a test point cloud with 10 points in 3D space
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = 10;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    // Add points that form a single cluster (close to each other)
    for (size_t i = 0; i < 5; ++i) {
      cloud->points[i].x = 0.1 * static_cast<float>(i);
      cloud->points[i].y = 0.1 * static_cast<float>(i);
      cloud->points[i].z = 0.1 * static_cast<float>(i);
    }

    // Add points that form another cluster (far from the first cluster)
    for (size_t i = 5; i < 10; ++i) {
      cloud->points[i].x = 10.0 + 0.1 * static_cast<float>(i);
      cloud->points[i].y = 10.0 + 0.1 * static_cast<float>(i);
      cloud->points[i].z = 10.0 + 0.1 * static_cast<float>(i);
    }

    test_cloud_ = cloud;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud_;
};

TEST_F(EuclideanClusterTest, TestClusteringWithDefaultParams)
{
  // Create cluster with default parameters
  autoware::euclidean_cluster::EuclideanCluster cluster(true, 1, 100);
  cluster.setTolerance(0.5);  // Set tolerance to 0.5 meters

  // Perform clustering
  std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
  bool result = cluster.cluster(test_cloud_, clusters);

  // Verify the result
  EXPECT_TRUE(result);
  EXPECT_EQ(clusters.size(), 2);  // Should detect two clusters

  // Check the size of each cluster
  EXPECT_EQ(clusters[0].points.size(), 5);
  EXPECT_EQ(clusters[1].points.size(), 5);
}

TEST_F(EuclideanClusterTest, TestClusteringWithCustomParams)
{
  // Create cluster with custom parameters
  autoware::euclidean_cluster::EuclideanCluster cluster(true, 3, 100, 0.5);

  // Perform clustering
  std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
  bool result = cluster.cluster(test_cloud_, clusters);

  // Verify the result
  EXPECT_TRUE(result);
  EXPECT_EQ(clusters.size(), 2);  // Should detect two clusters
}

TEST_F(EuclideanClusterTest, TestClusteringWithoutHeight)
{
  // Create cluster with height disabled
  autoware::euclidean_cluster::EuclideanCluster cluster(false, 1, 100, 0.5);

  // Perform clustering
  std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
  bool result = cluster.cluster(test_cloud_, clusters);

  // Verify the result
  EXPECT_TRUE(result);
  EXPECT_EQ(clusters.size(), 2);  // Should still detect two clusters

  // When use_height is false, we're flattening points for clustering, but original z-values
  // are preserved in the output. So we expect to still see the original z values.
  bool found_non_zero_z = false;
  for (const auto & cluster_cloud : clusters) {
    for (const auto & point : cluster_cloud.points) {
      if (point.z != 0.0) {
        found_non_zero_z = true;
        break;
      }
    }
    if (found_non_zero_z) break;
  }
  EXPECT_TRUE(found_non_zero_z) << "Expected at least some points to have non-zero z values";
}

TEST_F(EuclideanClusterTest, TestClusteringWithMinSizeFilter)
{
  // Create cluster with higher min_cluster_size to filter out small clusters
  autoware::euclidean_cluster::EuclideanCluster cluster(true, 6, 100, 0.5);

  // Perform clustering
  std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
  bool result = cluster.cluster(test_cloud_, clusters);

  // Verify the result
  EXPECT_TRUE(result);
  EXPECT_EQ(clusters.size(), 0);  // No clusters should pass the size filter
}

TEST_F(EuclideanClusterTest, TestUnimplementedMethods)
{
  autoware::euclidean_cluster::EuclideanCluster cluster(true, 1, 100, 0.5);

  // Test unimplemented method 1
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg =
    std::make_shared<sensor_msgs::msg::PointCloud2>();
  autoware_perception_msgs::msg::DetectedObjects objects;

  bool result1 = cluster.cluster(cloud_msg, objects);
  EXPECT_FALSE(result1);  // Should return false as method is not implemented

  // Test unimplemented method 2
  std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
  bool result2 = cluster.cluster(cloud_msg, objects, clusters);
  EXPECT_FALSE(result2);  // Should return false as method is not implemented
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
