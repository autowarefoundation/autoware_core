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

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

using autoware::point_types::PointXYZI;
void set_point_cloud2_fields(sensor_msgs::msg::PointCloud2 & pointcloud)
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

/// \brief Append \p points to \p cloud, which may be empty or already hold points.
void add_points(sensor_msgs::msg::PointCloud2 & cloud, const std::vector<PointXYZI> & points)
{
  if (cloud.fields.empty()) {
    set_point_cloud2_fields(cloud);
  }
  const size_t offset = cloud.data.size();
  cloud.data.resize(offset + points.size() * cloud.point_step);
  for (size_t i = 0; i < points.size(); ++i) {
    memcpy(&cloud.data[offset + i * cloud.point_step], &points[i], cloud.point_step);
  }
  cloud.width += static_cast<uint32_t>(points.size());
  cloud.row_step = cloud.point_step * cloud.width;
}

/// \brief Fill the voxel cell at \p cell_x with a uniform \p nx by \p ny by \p nz grid of points.
///
/// The grid is inset from the cell edges, so every point belongs to that one cell whatever
/// \p leaf_size is. Filling two cells a known number apart therefore puts a known distance
/// between the two groups of points.
/// \return how many points were added.
int fill_voxel_uniformly(
  sensor_msgs::msg::PointCloud2 & cloud, const float leaf_size, const int cell_x, const int nx,
  const int ny, const int nz)
{
  // Stay inside the middle 80% of the cell, so no rounding at a cell edge can move a point into
  // the neighbouring cell.
  const float margin = leaf_size * 0.1f;
  const float span = leaf_size * 0.8f;
  const auto coordinate = [margin, span](const int i, const int count) {
    if (count == 1) {
      return margin + span / 2.0f;
    }
    return margin + span * static_cast<float>(i) / static_cast<float>(count - 1);
  };
  const float cell_origin = static_cast<float>(cell_x) * leaf_size;

  std::vector<PointXYZI> points;
  points.reserve(static_cast<size_t>(nx) * ny * nz);
  for (int ix = 0; ix < nx; ++ix) {
    for (int iy = 0; iy < ny; ++iy) {
      for (int iz = 0; iz < nz; ++iz) {
        points.push_back(
          PointXYZI{
            cell_origin + coordinate(ix, nx), coordinate(iy, ny), coordinate(iz, nz), 0.0f});
      }
    }
  }
  add_points(cloud, points);
  return static_cast<int>(points.size());
}

// A cluster holding exactly `max_cluster_size` points is within the limit, so it is reported.
TEST(VoxelGridBasedEuclideanClusterTest, ClusterAtMaxSizeIsReported)
{
  autoware::euclidean_cluster::EuclideanClusterParams param;
  param.use_height = false;
  param.min_cluster_size = 1;
  param.tolerance = 0.7f;
  param.voxel_leaf_size = 0.3f;
  param.min_points_number_per_voxel = 1;

  sensor_msgs::msg::PointCloud2 cloud;
  const int points_in_cluster =
    fill_voxel_uniformly(cloud, param.voxel_leaf_size, /*cell_x=*/0, /*nx=*/3, /*ny=*/3, /*nz=*/3);
  param.max_cluster_size = points_in_cluster;

  autoware::euclidean_cluster::VoxelGridBasedEuclideanClusterDetector cluster(param);
  auto result = cluster.cluster(cloud);

  EXPECT_EQ(result.cluster_message.objects.size(), 1u);
}

// A cluster holding fewer than `min_cluster_size` points counts as noise, so it is dropped without
// being reported as skipped.
TEST(VoxelGridBasedEuclideanClusterTest, ClusterBelowMinSizeIsDropped)
{
  autoware::euclidean_cluster::EuclideanClusterParams param;
  param.use_height = false;
  param.tolerance = 0.7f;
  param.voxel_leaf_size = 0.3f;
  param.min_points_number_per_voxel = 1;

  sensor_msgs::msg::PointCloud2 cloud;
  const int points_in_cluster =
    fill_voxel_uniformly(cloud, param.voxel_leaf_size, /*cell_x=*/0, /*nx=*/1, /*ny=*/1, /*nz=*/1);
  param.min_cluster_size = points_in_cluster + 1;
  // a valid pair of limits (min <= max), so the drop can only come from the min check
  param.max_cluster_size = points_in_cluster + 1;

  autoware::euclidean_cluster::VoxelGridBasedEuclideanClusterDetector cluster(param);
  auto result = cluster.cluster(cloud);

  EXPECT_EQ(result.cluster_message.objects.size(), 0u);
  EXPECT_EQ(result.skipped_cluster_count, 0);
}

// A cluster holding more than `max_cluster_size` points is rejected and reported as skipped.
TEST(VoxelGridBasedEuclideanClusterTest, ClusterAboveMaxSizeIsSkipped)
{
  autoware::euclidean_cluster::EuclideanClusterParams param;
  param.use_height = false;
  param.min_cluster_size = 1;
  param.tolerance = 0.7f;
  param.voxel_leaf_size = 0.3f;
  param.min_points_number_per_voxel = 1;

  sensor_msgs::msg::PointCloud2 cloud;
  const int points_in_cluster =
    fill_voxel_uniformly(cloud, param.voxel_leaf_size, /*cell_x=*/0, /*nx=*/3, /*ny=*/3, /*nz=*/3);
  param.max_cluster_size = points_in_cluster - 1;

  autoware::euclidean_cluster::VoxelGridBasedEuclideanClusterDetector cluster(param);
  auto result = cluster.cluster(cloud);

  EXPECT_EQ(result.cluster_message.objects.size(), 0u);
  EXPECT_EQ(result.skipped_cluster_count, 1);
}

// Regression test: points that sit exactly on a voxel boundary must stay in their cluster.
// All 100 points form one cluster with more points than max_cluster_size, so the detector must
// reject it and return 0 objects. The centroid of the boundary voxel is a float mean. When the
// boundary voxel holds 6 identical points at x = 0.3, the mean rounds to 0.29999998f, one float
// step under the 0.3f boundary. A detector that drops these boundary points shrinks the cluster
// to fewer points than max_cluster_size and wrongly accepts it.
TEST(VoxelGridBasedEuclideanClusterTest, BoundaryVoxelPointsAreNotDropped)
{
  constexpr int nb_points = 100;
  constexpr int nb_boundary_points = 6;

  sensor_msgs::msg::PointCloud2 pointcloud;
  pointcloud.header.frame_id = "dummy_frame_id";
  sensor_msgs::PointCloud2Modifier modifier(pointcloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(nb_points);

  sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud, "z");
  for (int i = 0; i < nb_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
    // put the first `nb_boundary_points` points exactly on the voxel boundary at x = 0.3
    *iter_x = i < nb_boundary_points ? 0.3f : 0.1f;
    *iter_y = 0.1f;
    *iter_z = 0.0f;
  }

  autoware::euclidean_cluster::EuclideanClusterParams param;
  param.use_height = false;
  param.min_cluster_size = 1;
  // one point fewer than the cluster holds, so the cluster exceeds `max_cluster_size`
  param.max_cluster_size = nb_points - 1;
  param.tolerance = 0.7f;
  param.voxel_leaf_size = 0.3f;
  param.min_points_number_per_voxel = 1;

  autoware::euclidean_cluster::VoxelGridBasedEuclideanClusterDetector cluster(param);
  auto result = cluster.cluster(pointcloud);

  // all points form one cluster that exceeds max_cluster_size, so it must be rejected
  EXPECT_EQ(result.cluster_message.objects.size(), 0u);
  EXPECT_EQ(result.skipped_cluster_count, 1);
}

// The object position a cluster is reported at is the mean of the points that formed it, on all
// three axes, z included even though the grouping ignores it. Two points spell that out: written
// down, they let the expected answer be read off rather than computed.
TEST(VoxelGridBasedEuclideanClusterTest, ObjectPositionIsTheMeanOfItsPoints)
{
  autoware::euclidean_cluster::EuclideanClusterParams param;
  param.use_height = false;
  param.min_cluster_size = 1;
  param.max_cluster_size = 10;
  param.tolerance = 1.0f;
  param.voxel_leaf_size = 1.0f;
  param.min_points_number_per_voxel = 1;

  // Both points fall in the cell spanning [0, 1) on x and y, so they form one cluster.
  sensor_msgs::msg::PointCloud2 cloud;
  add_points(cloud, {PointXYZI{0.1f, 0.2f, 0.3f, 0.0f}, PointXYZI{0.9f, 0.8f, 0.7f, 0.0f}});

  autoware::euclidean_cluster::VoxelGridBasedEuclideanClusterDetector cluster(param);
  auto result = cluster.cluster(cloud);

  ASSERT_EQ(result.cluster_message.objects.size(), 1u);

  const auto & position =
    result.cluster_message.objects.front().kinematics.pose_with_covariance.pose.position;
  EXPECT_NEAR(position.x, 0.5, 1e-6);
  EXPECT_NEAR(position.y, 0.5, 1e-6);
  EXPECT_NEAR(position.z, 0.5, 1e-6);
}

// A cloud with its fields and point_step set but no data must pass through without crashing and
// produce no objects.
TEST(VoxelGridBasedEuclideanClusterTest, ClusterEmptyInput)
{
  sensor_msgs::msg::PointCloud2 pointcloud;
  set_point_cloud2_fields(pointcloud);
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

  autoware::euclidean_cluster::VoxelGridBasedEuclideanClusterDetector cluster(param);
  auto result = cluster.cluster(pointcloud);

  EXPECT_EQ(result.cluster_message.objects.size(), 0u);
}

// Groups lying farther apart than `tolerance` are reported as one object each, rather than being
// merged into a single object or split further.
TEST(VoxelGridBasedEuclideanClusterTest, SeparatedGroupsAreReportedOneObjectEach)
{
  autoware::euclidean_cluster::EuclideanClusterParams param;
  param.use_height = false;
  param.min_cluster_size = 1;
  param.tolerance = 0.7f;
  param.voxel_leaf_size = 0.3f;
  param.min_points_number_per_voxel = 1;

  constexpr int num_groups = 3;
  // Well past the cells it takes to cover `tolerance`, so no two groups can ever be linked.
  constexpr int spare_cells = 10;
  const int cells_apart =
    static_cast<int>(std::ceil(param.tolerance / param.voxel_leaf_size)) * spare_cells;

  sensor_msgs::msg::PointCloud2 cloud;
  // every group is laid out identically, so each call reports the same count
  int points_per_group = 0;
  for (int i = 0; i < num_groups; ++i) {
    points_per_group = fill_voxel_uniformly(
      cloud, param.voxel_leaf_size, i * cells_apart, /*nx=*/3, /*ny=*/3, /*nz=*/3);
  }
  param.max_cluster_size = points_per_group;

  autoware::euclidean_cluster::VoxelGridBasedEuclideanClusterDetector cluster(param);
  auto result = cluster.cluster(cloud);

  ASSERT_EQ(result.cluster_message.objects.size(), static_cast<size_t>(num_groups));
  EXPECT_EQ(result.skipped_cluster_count, 0);

  // One object per group, each standing in the cell its group was laid in. The objects come back
  // in no particular order, so sort by x before pairing them up with the cells.
  std::vector<double> reported_x;
  reported_x.reserve(result.cluster_message.objects.size());
  for (const auto & object : result.cluster_message.objects) {
    reported_x.push_back(object.kinematics.pose_with_covariance.pose.position.x);
  }
  std::sort(reported_x.begin(), reported_x.end());

  for (int i = 0; i < num_groups; ++i) {
    const double cell_lower = i * cells_apart * static_cast<double>(param.voxel_leaf_size);
    EXPECT_GT(reported_x[i], cell_lower);
    EXPECT_LT(reported_x[i], cell_lower + static_cast<double>(param.voxel_leaf_size));
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
