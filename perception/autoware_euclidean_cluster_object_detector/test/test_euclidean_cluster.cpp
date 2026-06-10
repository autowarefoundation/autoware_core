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

#include <algorithm>
#include <array>
#include <iostream>
#include <memory>
#include <vector>

using autoware::point_types::PointXYZI;

class EuclideanClusterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Build the two ground-truth point sets that the test cloud is composed of.
    // near_points_: a tight cluster around the origin (i*0.1 for i in [0,5)).
    for (size_t i = 0; i < 5; ++i) {
      const float v = 0.1f * static_cast<float>(i);
      near_points_.push_back({v, v, v});
    }

    // far_points_: a tight cluster around (10,10,10) (10 + i*0.1 for i in [5,10)).
    for (size_t i = 5; i < 10; ++i) {
      const float v = 10.0f + 0.1f * static_cast<float>(i);
      far_points_.push_back({v, v, v});
    }

    // Build the test cloud from the two named point sets so the input is the single
    // source of truth for the expected clustering output.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto & p : near_points_) {
      cloud->points.push_back(pcl::PointXYZ(p[0], p[1], p[2]));
    }
    for (const auto & p : far_points_) {
      cloud->points.push_back(pcl::PointXYZ(p[0], p[1], p[2]));
    }
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;

    test_cloud_ = cloud;
  }

  // Copy the {x, y, z} coordinates of every point in a cluster into a sorted vector so
  // clusters can be compared as deterministic sets regardless of point ordering.
  template <typename Cluster>
  static std::vector<std::array<float, 3>> sorted_points(const Cluster & cluster)
  {
    std::vector<std::array<float, 3>> points;
    for (const auto & p : cluster.points) {
      points.push_back({p.x, p.y, p.z});
    }
    std::sort(points.begin(), points.end());
    return points;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud_;
  std::vector<std::array<float, 3>> near_points_;
  std::vector<std::array<float, 3>> far_points_;
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

  // The input point sets are the single source of truth. Sort the points within each
  // cluster and sort the clusters themselves, then compare against the expected set
  // {near_points_, far_points_} (both pre-sorted; near < far lexicographically).
  std::vector<std::vector<std::array<float, 3>>> actual;
  for (const auto & c : clusters) {
    actual.push_back(sorted_points(c));
  }
  std::sort(actual.begin(), actual.end());

  const std::vector<std::vector<std::array<float, 3>>> expected = {near_points_, far_points_};
  EXPECT_EQ(actual, expected);
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
