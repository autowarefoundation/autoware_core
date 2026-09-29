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

#include "../src/euclidean_cluster_object_detector.hpp"
#include "parameters.hpp"

#include <autoware/point_types/types.hpp>

#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <array>
#include <iostream>
#include <memory>
#include <vector>

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
      cloud->points.emplace_back(p[0], p[1], p[2]);
    }
    for (const auto & p : far_points_) {
      cloud->points.emplace_back(p[0], p[1], p[2]);
    }
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;

    // Convert to ROS PointCloud2 for new API
    pcl::toROSMsg(*cloud, ros_test_cloud_);
    ros_test_cloud_.header.frame_id = "base_link";
  }

  // Lazy init of test params/clusters
  static autoware::euclidean_cluster::EuclideanClusterParams get_default_params()
  {
    autoware::euclidean_cluster::EuclideanClusterParams param;
    param.use_height = true;
    param.min_cluster_size = 1;
    param.max_cluster_size = 100;
    param.tolerance = 0.5f;
    return param;
  }

  sensor_msgs::msg::PointCloud2 ros_test_cloud_;
  std::vector<std::array<float, 3>> near_points_;
  std::vector<std::array<float, 3>> far_points_;
};

TEST_F(EuclideanClusterTest, TestClusteringWithDefaultParams)
{
  autoware::euclidean_cluster::EuclideanClusterObjectDetector detector(get_default_params());

  auto result = detector.cluster(ros_test_cloud_);

  // Verify the result
  EXPECT_EQ(result.cluster_message.objects.size(), 2u);  // Should detect two clusters

  // Expect no points are dropped
  EXPECT_EQ(result.debug_message.width, 10u);
}

TEST_F(EuclideanClusterTest, TestClusteringWithCustomParams)
{
  auto param = get_default_params();
  param.min_cluster_size = 3;
  autoware::euclidean_cluster::EuclideanClusterObjectDetector detector(param);

  auto result = detector.cluster(ros_test_cloud_);
  EXPECT_EQ(result.cluster_message.objects.size(), 2u);  // Should detect two clusters
}

TEST_F(EuclideanClusterTest, TestClusteringWithoutHeight)
{
  auto param = get_default_params();
  param.use_height = false;
  autoware::euclidean_cluster::EuclideanClusterObjectDetector detector(param);
  auto result = detector.cluster(ros_test_cloud_);

  // Verify the result
  EXPECT_EQ(result.cluster_message.objects.size(), 2u);  // Should still detect two clusters

  // When use_height is false, we're flattening points for clustering, but original z-values
  // are preserved in the output. So we expect to still see the original z values.
  bool found_non_zero_z = false;
  for (const auto & object : result.cluster_message.objects) {
    if (object.kinematics.pose_with_covariance.pose.position.z != 0.0) {
      found_non_zero_z = true;
      break;
    }
  }
  EXPECT_TRUE(found_non_zero_z) << "Expected at least some points to have non-zero z values";
}

TEST_F(EuclideanClusterTest, TestClusteringWithMinSizeFilter)
{
  // Create cluster with higher min_cluster_size to filter out small clusters
  auto param = get_default_params();
  param.min_cluster_size = 6;  // Filter out clusters smaller than 6
  autoware::euclidean_cluster::EuclideanClusterObjectDetector detector(param);
  auto result = detector.cluster(ros_test_cloud_);

  // Verify the result
  EXPECT_EQ(result.cluster_message.objects.size(), 0u);  // No clusters should pass the size filter
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
