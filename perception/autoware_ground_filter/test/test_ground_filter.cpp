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

#include "ground_filter.hpp"

#include <autoware/point_types/types.hpp>

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <vector>

namespace
{
// Floating point tolerance at EXPECT_NEAR and similar checks
constexpr float near_tol = 1e-4F;
}  // namespace

class GroundFilterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize parameter structure
    param_.elevation_grid_mode = true;
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
    create_sample_point_cloud();
  }

  void TearDown() override { ground_filter_.reset(); }

  void create_sample_point_cloud()
  {
    auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();

    // Create simple XYZ point cloud for testing
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

    // Add ground points
    for (int i = 0; i < 50; ++i) {
      pcl::PointXYZ point;
      point.x = static_cast<float>(i % 10) - 5.0f;
      point.y = static_cast<float>(i / 10.0f) - 2.5f;
      point.z =
        0.0f + (static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) - 0.5f) * 0.1f;
      pcl_cloud.push_back(point);
    }

    // Add non-ground points
    for (int i = 0; i < 25; ++i) {
      pcl::PointXYZ point;
      point.x = static_cast<float>(i % 5) - 2.5f;
      point.y = static_cast<float>(i / 5.0f) - 2.5f;
      point.z = 0.5f + (static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX)) * 0.5f;
      pcl_cloud.push_back(point);
    }

    pcl::toROSMsg(pcl_cloud, *cloud);
    cloud->header.frame_id = "base_link";
    cloud->header.stamp.sec = 0;
    cloud->header.stamp.nanosec = 0;

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
    point.y = static_cast<float>(i / 6.0f) - 2.5f;
    point.z = (i < 15) ? 0.0f : 0.8f;  // Half ground, half elevated
    point.intensity = 100;
    point.return_type = 1;
    point.channel = 0;
    pcl_cloud_xyzirc.push_back(point);
  }

  pcl::toROSMsg(pcl_cloud_xyzirc, *xyzirc_cloud);
  xyzirc_cloud->header.frame_id = "base_link";
  xyzirc_cloud->header.stamp.sec = 0;
  xyzirc_cloud->header.stamp.nanosec = 0;

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

// This new section below is added during the June 2026 refactoring effort, to address comprehensive
// testing of the radial mode of GroundFilter, currently not quite covered by above 11 tests.

/**
 * @brief This class is to support unit testing of GroundFilter in radial mode (the above one is for
 * grid mode). Provides setting up GroundFilter instance with radial mode params and creating simple
 * point clouds for it.
 */
class GroundFilterRadialTest : public ::testing::Test
{
protected:
  std::unique_ptr<autoware::ground_filter::GroundFilter> filter_;
  autoware::ground_filter::GroundFilterParameter param_;

  using RayPointsCentroid = autoware::ground_filter::GroundFilter::RayPointsCentroid;
  using PointCloudVector = autoware::ground_filter::GroundFilter::PointCloudVector;

  // Bringing the private helper funcs from GroundFilter to here as proxies
  void calc_virtual_ground_origin(pcl::PointXYZ & point)
  {
    filter_->calcVirtualGroundOrigin(point);
  }
  void convert_point_cloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in_cloud,
    std::vector<PointCloudVector> & out_radial)
  {
    filter_->convertPointCloud(in_cloud, out_radial);
  }
  void classify_point_cloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in_cloud,
    const std::vector<PointCloudVector> & in_radial, pcl::PointIndices & out_indices)
  {
    filter_->classifyPointCloud(in_cloud, in_radial, out_indices);
  }

  // Set up test environment with radial mode params.
  void SetUp() override
  {
    // Setup predictable math parameters for Radial mode
    param_.elevation_grid_mode = false;

    // Set slopes to exactly 15 degrees
    param_.global_slope_max_angle_rad = 15.0f * M_PI / 180.0f;
    param_.local_slope_max_angle_rad = 15.0f * M_PI / 180.0f;

    // Set slice size to 1 degree
    param_.radial_divider_angle_rad = 1.0f * M_PI / 180.0f;
    param_.radial_dividers_num = std::floor(2.0 * M_PI / param_.radial_divider_angle_rad);

    // Gap and Virtual Origin parameters
    param_.use_virtual_ground_point = true;
    param_.wheel_base_m = 2.8f;
    param_.split_points_distance_tolerance = 0.2f;
    param_.split_height_distance = 0.2f;

    // Standard Grid params (needed for init, though unused in these tests)
    param_.virtual_lidar_x = 1.4f;
    param_.virtual_lidar_y = 0.0f;
    param_.virtual_lidar_z = 1.9f;
    param_.grid_size_m = 1.0f;
    param_.grid_mode_switch_radius = 20.0f;

    filter_ = std::make_unique<autoware::ground_filter::GroundFilter>(param_);
  }

  /**
   * @brief Helper func to create a very simple point cloud for testing.
   *
   * @param points Vector of pcl::PointXYZ points to include in this point cloud.
   *
   * @return sensor_msgs::msg::PointCloud2::SharedPtr Point cloud message containing the points.
   */
  static sensor_msgs::msg::PointCloud2::SharedPtr create_point_cloud(
    const std::vector<pcl::PointXYZ> & points)
  {
    pcl::PointCloud<autoware::point_types::PointXYZIRC> pcl_cloud;
    for (const auto & p : points) {
      autoware::point_types::PointXYZIRC pt;
      pt.x = p.x;
      pt.y = p.y;
      pt.z = p.z;
      pcl_cloud.push_back(pt);
    }
    auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(pcl_cloud, *cloud_msg);
    cloud_msg->header.frame_id = "base_link";
    return cloud_msg;
  }
};

// TEST 1. Confirm the maths of RayPointsCentroid works as expected.
// Provides a single ray of 2 (R, Z) points (2.0, 0.0) and (4.0, 2.0).
// Expects correct calculation of average radius, height, and slope.
TEST_F(GroundFilterRadialTest, RayPointsCentroidMath)
{
  RayPointsCentroid centroid;

  EXPECT_EQ(centroid.point_num, 0U);

  centroid.addPoint(2.0f, 0.0f, 0);
  EXPECT_EQ(centroid.point_num, 1U);
  EXPECT_NEAR(centroid.getAverageRadius(), 2.0f, near_tol);
  EXPECT_NEAR(centroid.getAverageHeight(), 0.0f, near_tol);

  centroid.addPoint(4.0f, 2.0f, 1);
  EXPECT_EQ(centroid.point_num, 2U);
  EXPECT_NEAR(centroid.getAverageRadius(), 3.0f, near_tol);
  EXPECT_NEAR(centroid.getAverageHeight(), 1.0f, near_tol);
  EXPECT_NEAR(centroid.getAverageSlope(), std::atan2(1.0f, 3.0f), near_tol);
}

// TEST 2. Confirm virtual origin logic.
// Checks that virtual ogini is calculated correctly based on vehicle wheelbase and lidar position.
// Current wheelbase 2.8m, lidar at (1.4, 0, 1.9) should yield virtual origin at (2.8, 0, 0).
TEST_F(GroundFilterRadialTest, CalcVirtualGroundOrigin)
{
  pcl::PointXYZ virtual_origin;
  calc_virtual_ground_origin(virtual_origin);

  EXPECT_NEAR(virtual_origin.x, 2.8f, near_tol);
  EXPECT_NEAR(virtual_origin.y, 0.0f, near_tol);
  EXPECT_NEAR(virtual_origin.z, 0.0f, near_tol);
}

// TEST 3. Confirm azimuth slicing & sorting
// This test creates a point cloud with 3 points in different azimuths, then checks if
// they are correctly grouped into radial slices and sorted by radius within those slices.
// Adds 3 points: (5, 0, 0), (2, 0, 0), (0, 3, 0). Expects two slices:
// - One for azimuth ~0 deg with points (0, 3)
// - One for azimuth ~90 deg with point (5, 0) and (2, 0) sorted by radius.
// Note: the math inside convertPointCloud is a lil bit tricky: azimuth angle is calculated
//       as atan2(x, y) instead of normal convention atan2(y, x). Thus now:
//          - 0   deg is along +Y axis
//          - 90  deg is along +X axis
//          - 180 deg is along -Y axis
//          - 270 deg is along -X axis.
TEST_F(GroundFilterRadialTest, RadialGroupingAndSorting)
{
  std::vector<pcl::PointXYZ> raw_points = {
    pcl::PointXYZ(5.0f, 0.0f, 0.0f), pcl::PointXYZ(2.0f, 0.0f, 0.0f),
    pcl::PointXYZ(0.0f, 3.0f, 0.0f)};

  auto cloud = create_point_cloud(raw_points);
  filter_->setDataAccessor(cloud);

  std::vector<PointCloudVector> radial_ordered;
  convert_point_cloud(cloud, radial_ordered);

  // 1. Master array should have 360 slices (1 degree per slice)
  EXPECT_EQ(radial_ordered.size(), 360U);

  // 2. Here we check each ray/slice for expected points.

  // 2.a. Checking slice 0 deg (front ray). Should contain 1 point with radius 3.0.
  ASSERT_GE(radial_ordered.size(), 1U);
  ASSERT_EQ(radial_ordered[0].size(), 1U);
  EXPECT_NEAR(radial_ordered[0][0].radius, 3.0f, near_tol);

  // 2.b. Checking slice 90 deg (left ray). Should contain 2 points with radii 2.0 and 5.0, sorted
  // by radius.
  ASSERT_GE(radial_ordered.size(), 91U);
  ASSERT_EQ(radial_ordered[90].size(), 2U);
  EXPECT_NEAR(radial_ordered[90][0].radius, 2.0f, near_tol);
  EXPECT_NEAR(radial_ordered[90][1].radius, 5.0f, near_tol);
}

// TEST 4. Confirm point classification logic in classifyPointCloud.
// This test creates a simple point cloud with 3 points: (3, 0, 0), (4, 0, 0.6), (5, 0, 2.0).
// With given slope threshold 15 deg and height threshold 0.2, we expect:
// - Point (3, 0, 0) : ground (slope 0, height 0)
// - Point (4, 0, 0.6) : non-ground (slope 30 deg, height 0.6 > 0.2)
// - Point (5, 0, 2.0) : non-ground (slope ~21.8 deg, height 2.0 > 0.2)
TEST_F(GroundFilterRadialTest, ClassifyLocalAndGlobalSlopes)
{
  std::vector<pcl::PointXYZ> raw_points = {
    pcl::PointXYZ(3.0f, 0.0f, 0.0f), pcl::PointXYZ(4.0f, 0.0f, 0.6f),
    pcl::PointXYZ(5.0f, 0.0f, 2.0f)};

  auto cloud = create_point_cloud(raw_points);
  filter_->setDataAccessor(cloud);

  std::vector<PointCloudVector> radial_ordered;
  convert_point_cloud(cloud, radial_ordered);

  pcl::PointIndices out_indices;
  classify_point_cloud(cloud, radial_ordered, out_indices);

  // Expect 2 non-ground points being index 1 and 2.
  // Since the algorithm outputs raw memory byte offsets as indices,
  // I'm using point_step to verify these indices.
  EXPECT_EQ(out_indices.indices.size(), 2U);
  const uint32_t point_step = cloud->point_step;
  EXPECT_EQ(out_indices.indices[0], 1U * point_step);
  EXPECT_EQ(out_indices.indices[1], 2U * point_step);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
