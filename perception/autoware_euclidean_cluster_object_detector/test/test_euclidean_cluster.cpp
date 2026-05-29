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

#include "euclidean_cluster.hpp"

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

// Characterization test: pin exact cluster membership and per-point coordinates so the
// in-place output-building refactor (no 'new', no deep copy, reserve) is proven equivalent.
TEST_F(EuclideanClusterTest, TestClusterMembershipAndPointValues)
{
  autoware::euclidean_cluster::EuclideanCluster cluster(true, 1, 100, 0.5);

  std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
  ASSERT_TRUE(cluster.cluster(test_cloud_, clusters));
  ASSERT_EQ(clusters.size(), 2u);

  // Each output cluster must carry exactly 5 points and have the PointCloud2-style metadata
  // (width == point count, height == 1, is_dense == false) set on it.
  for (const auto & c : clusters) {
    EXPECT_EQ(c.points.size(), 5u);
    EXPECT_EQ(c.width, 5u);
    EXPECT_EQ(c.height, 1u);
    EXPECT_FALSE(c.is_dense);
  }

  // The two ground-truth clusters from SetUp(): near-origin and around (10,10,10).
  // Collect every output point and verify each belongs to exactly one expected group,
  // and that all 10 input points are reproduced verbatim (coordinates preserved).
  size_t near_origin_count = 0;
  size_t far_count = 0;
  for (const auto & c : clusters) {
    for (const auto & point : c.points) {
      if (point.x < 1.0f) {
        ++near_origin_count;
        // near-origin points are i*0.1 for i in [0,5)
        EXPECT_FLOAT_EQ(point.x, point.y);
        EXPECT_FLOAT_EQ(point.y, point.z);
      } else {
        ++far_count;
        // far points are 10 + i*0.1 for i in [5,10)
        EXPECT_GE(point.x, 10.0f);
        EXPECT_FLOAT_EQ(point.x, point.y);
        EXPECT_FLOAT_EQ(point.y, point.z);
      }
    }
  }
  EXPECT_EQ(near_origin_count, 5u);
  EXPECT_EQ(far_count, 5u);
}

// Characterization test for the previously-untested empty-input path of the implemented overload.
TEST_F(EuclideanClusterTest, TestClusteringEmptyInput)
{
  autoware::euclidean_cluster::EuclideanCluster cluster(true, 1, 100, 0.5);

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr empty_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;

  bool result = cluster.cluster(empty_cloud, clusters);
  EXPECT_TRUE(result);
  EXPECT_EQ(clusters.size(), 0u);
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
