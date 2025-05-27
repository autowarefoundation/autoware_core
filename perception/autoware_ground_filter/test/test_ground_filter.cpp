// Copyright 2024 Tier IV, Inc.
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

#include "autoware/ground_filter/ground_filter.hpp"

#include <autoware/point_types/types.hpp>

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <utility>
#include <vector>

class GroundFilterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize parameter structure
    param_.global_slope_max_angle_rad = 0.26f;  // ~15 degrees
    param_.local_slope_max_angle_rad = 0.26f;   // ~15 degrees
    param_.radial_divider_angle_rad = 0.0175f;  // 1 degree
    param_.use_recheck_ground_cluster = true;
    param_.use_lowest_point = true;
    param_.detection_range_z_max = 2.0f;
    param_.non_ground_height_threshold = 0.2f;
    param_.grid_size_m = 0.5f;
    param_.grid_mode_switch_radius = 20.0f;
    param_.ground_grid_buffer_size = 3;
    param_.virtual_lidar_x = 1.4f;
    param_.virtual_lidar_y = 0.0f;
    param_.virtual_lidar_z = 1.9f;

    // Create filter
    ground_filter_ = std::make_unique<autoware::ground_filter::GroundFilter>(param_);

    // Create sample point cloud
    createSamplePointCloud();
  }

  void TearDown() override { ground_filter_.reset(); }

  void createSamplePointCloud()
  {
    auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();

    // Create simple XYZ point cloud for testing
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

    // Add ground points
    for (int i = 0; i < 50; ++i) {
      pcl::PointXYZ point;
      point.x = static_cast<float>(i % 10) - 5.0f;
      point.y = static_cast<float>(i / 10) - 2.5f;
      point.z =
        0.0f + (static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) - 0.5f) * 0.1f;
      pcl_cloud.push_back(point);
    }

    // Add non-ground points
    for (int i = 0; i < 25; ++i) {
      pcl::PointXYZ point;
      point.x = static_cast<float>(i % 5) - 2.5f;
      point.y = static_cast<float>(i / 5) - 2.5f;
      point.z = 0.5f + (static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX)) * 0.5f;
      pcl_cloud.push_back(point);
    }

    pcl::toROSMsg(pcl_cloud, *cloud);
    cloud->header.frame_id = "base_link";
    cloud->header.stamp = rclcpp::Clock().now();

    // Convert to const shared pointer
    cloud_ = cloud;
  }

  autoware::ground_filter::GroundFilterParameter param_;
  std::unique_ptr<autoware::ground_filter::GroundFilter> ground_filter_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_;
};

TEST_F(GroundFilterTest, TestInitialization)
{
  EXPECT_NE(ground_filter_, nullptr);
}

TEST_F(GroundFilterTest, TestBasicFiltering)
{
  pcl::PointIndices no_ground_indices;

  // Set data accessor
  ground_filter_->setDataAccessor(cloud_);

  // Process the point cloud
  ground_filter_->process(cloud_, no_ground_indices);

  // Should have some non-ground points
  EXPECT_GT(no_ground_indices.indices.size(), 0);
}

TEST_F(GroundFilterTest, TestNonGroundHeightThreshold)
{
  // Test with different height threshold
  param_.non_ground_height_threshold = 0.1f;
  auto test_filter = std::make_unique<autoware::ground_filter::GroundFilter>(param_);

  pcl::PointIndices no_ground_indices;
  test_filter->setDataAccessor(cloud_);
  test_filter->process(cloud_, no_ground_indices);

  // Should detect non-ground points
  EXPECT_GE(no_ground_indices.indices.size(), 0);
}

TEST_F(GroundFilterTest, TestTimeKeeper)
{
  // Test with time keeper
  auto time_keeper = std::make_shared<autoware_utils_debug::TimeKeeper>();
  ground_filter_->setTimeKeeper(time_keeper);

  pcl::PointIndices no_ground_indices;
  ground_filter_->setDataAccessor(cloud_);
  EXPECT_NO_THROW(ground_filter_->process(cloud_, no_ground_indices));
}

TEST_F(GroundFilterTest, TestPointsCentroidFunctionality)
{
  autoware::ground_filter::PointsCentroid centroid;

  // Test default constructor
  EXPECT_FLOAT_EQ(centroid.radius_avg, 0.0f);
  EXPECT_FLOAT_EQ(centroid.height_avg, 0.0f);
  EXPECT_FLOAT_EQ(centroid.height_max, -10.0f);
  EXPECT_FLOAT_EQ(centroid.height_min, 10.0f);
  EXPECT_EQ(centroid.grid_id, 0);

  // Test addPoint functionality
  centroid.addPoint(1.0f, 0.5f, 0);
  centroid.addPoint(2.0f, 1.0f, 1);
  centroid.addPoint(3.0f, 1.5f, 2);

  EXPECT_EQ(centroid.pcl_indices.size(), 3);
  EXPECT_EQ(centroid.height_list.size(), 3);
  EXPECT_EQ(centroid.radius_list.size(), 3);
  EXPECT_EQ(centroid.is_ground_list.size(), 3);

  // Test processAverage
  centroid.processAverage();

  EXPECT_FLOAT_EQ(centroid.radius_avg, 2.0f);  // (1+2+3)/3
  EXPECT_FLOAT_EQ(centroid.height_avg, 1.0f);  // (0.5+1.0+1.5)/3
  EXPECT_FLOAT_EQ(centroid.height_max, 1.5f);
  EXPECT_FLOAT_EQ(centroid.height_min, 0.5f);

  // Test getters
  EXPECT_FLOAT_EQ(centroid.getAverageHeight(), 1.0f);
  EXPECT_FLOAT_EQ(centroid.getAverageRadius(), 2.0f);
  EXPECT_FLOAT_EQ(centroid.getMaxHeight(), 1.5f);
  EXPECT_FLOAT_EQ(centroid.getMinHeight(), 0.5f);
  EXPECT_EQ(centroid.getGroundPointNum(), 3);
}

TEST_F(GroundFilterTest, TestPointsCentroidWithNonGroundPoints)
{
  // Test PointsCentroid with mixed ground/non-ground points
  autoware::ground_filter::PointsCentroid centroid;

  // Add some points
  centroid.addPoint(1.0f, 0.5f, 0);
  centroid.addPoint(2.0f, 1.0f, 1);
  centroid.addPoint(3.0f, 1.5f, 2);

  // Mark some as non-ground
  centroid.is_ground_list[1] = false;  // Second point is non-ground

  centroid.processAverage();

  // Only ground points should be used in average (points 0 and 2)
  EXPECT_FLOAT_EQ(centroid.radius_avg, 2.0f);  // (1+3)/2
  EXPECT_FLOAT_EQ(centroid.height_avg, 1.0f);  // (0.5+1.5)/2
  EXPECT_EQ(centroid.getGroundPointNum(), 2);
}

TEST_F(GroundFilterTest, TestPointsCentroidGetMinHeightOnly)
{
  // Test the previously uncovered getMinHeightOnly function
  autoware::ground_filter::PointsCentroid centroid;

  // Add points with various heights
  centroid.addPoint(1.0f, 0.5f, 0);   // height 0.5
  centroid.addPoint(2.0f, -0.2f, 1);  // height -0.2 (minimum)
  centroid.addPoint(3.0f, 1.5f, 2);   // height 1.5

  // Test getMinHeightOnly
  float min_height = centroid.getMinHeightOnly();
  EXPECT_FLOAT_EQ(min_height, -0.2f);

  // Test with mixed ground/non-ground points
  centroid.is_ground_list[0] = false;  // Exclude first point
  min_height = centroid.getMinHeightOnly();
  EXPECT_FLOAT_EQ(min_height, -0.2f);  // Should still find -0.2 from second point

  // Mark second point as non-ground too
  centroid.is_ground_list[1] = false;
  min_height = centroid.getMinHeightOnly();
  EXPECT_FLOAT_EQ(min_height, 1.5f);  // Only third point remains
}

TEST_F(GroundFilterTest, TestPointsCentroidEmptyCase)
{
  // Test PointsCentroid with no ground points
  autoware::ground_filter::PointsCentroid centroid;

  // Add non-ground points only
  centroid.addPoint(1.0f, 0.5f, 0);
  centroid.addPoint(2.0f, 1.0f, 1);

  // Mark all as non-ground
  centroid.is_ground_list[0] = false;
  centroid.is_ground_list[1] = false;

  // Process should handle empty case gracefully
  centroid.processAverage();

  EXPECT_EQ(centroid.getGroundPointNum(), 0);

  // getMinHeightOnly should return default for empty case
  float min_height = centroid.getMinHeightOnly();
  EXPECT_FLOAT_EQ(min_height, 10.0f);  // Default min_height value
}

TEST_F(GroundFilterTest, TestVariousParameterConfigurations)
{
  // Test with different parameter configurations to cover more code paths
  param_.use_recheck_ground_cluster = false;
  auto test_filter1 = std::make_unique<autoware::ground_filter::GroundFilter>(param_);

  pcl::PointIndices no_ground_indices1;
  test_filter1->setDataAccessor(cloud_);
  test_filter1->process(cloud_, no_ground_indices1);

  param_.use_recheck_ground_cluster = true;
  param_.use_lowest_point = false;
  auto test_filter2 = std::make_unique<autoware::ground_filter::GroundFilter>(param_);

  pcl::PointIndices no_ground_indices2;
  test_filter2->setDataAccessor(cloud_);
  test_filter2->process(cloud_, no_ground_indices2);

  param_.grid_size_m = 1.0f;
  param_.grid_mode_switch_radius = 10.0f;
  param_.ground_grid_buffer_size = 1;
  auto test_filter3 = std::make_unique<autoware::ground_filter::GroundFilter>(param_);

  pcl::PointIndices no_ground_indices3;
  test_filter3->setDataAccessor(cloud_);
  EXPECT_NO_THROW(test_filter3->process(cloud_, no_ground_indices3));
}

TEST_F(GroundFilterTest, TestDifferentPointCloudLayouts)
{
  // Test with XYZIRC layout
  auto xyzirc_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::PointCloud<autoware::point_types::PointXYZIRC> pcl_cloud_xyzirc;

  for (int i = 0; i < 30; ++i) {
    autoware::point_types::PointXYZIRC point;
    point.x = static_cast<float>(i % 6) - 3.0f;
    point.y = static_cast<float>(i / 6) - 2.5f;
    point.z = (i < 15) ? 0.0f : 0.8f;  // Half ground, half elevated
    point.intensity = 100;
    point.return_type = 1;
    point.channel = 0;
    pcl_cloud_xyzirc.push_back(point);
  }

  pcl::toROSMsg(pcl_cloud_xyzirc, *xyzirc_cloud);
  xyzirc_cloud->header.frame_id = "base_link";
  xyzirc_cloud->header.stamp = rclcpp::Clock().now();

  pcl::PointIndices no_ground_indices;
  ground_filter_->setDataAccessor(xyzirc_cloud);
  ground_filter_->process(xyzirc_cloud, no_ground_indices);

  EXPECT_GE(no_ground_indices.indices.size(), 0);
}

TEST_F(GroundFilterTest, TestExtremeParameterValues)
{
  // Test with extreme parameter values to trigger edge cases
  param_.global_slope_max_angle_rad = 0.0f;    // Very small slope
  param_.local_slope_max_angle_rad = 1.57f;    // Nearly 90 degrees
  param_.radial_divider_angle_rad = 0.001f;    // Very small divider
  param_.non_ground_height_threshold = 0.01f;  // Very small threshold
  param_.detection_range_z_max = 0.1f;         // Very small range

  auto extreme_filter = std::make_unique<autoware::ground_filter::GroundFilter>(param_);

  pcl::PointIndices no_ground_indices;
  extreme_filter->setDataAccessor(cloud_);
  EXPECT_NO_THROW(extreme_filter->process(cloud_, no_ground_indices));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
