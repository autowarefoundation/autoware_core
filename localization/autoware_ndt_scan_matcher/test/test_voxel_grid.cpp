// Copyright 2024 Autoware Foundation
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

// Differential test: the Rust voxel_grid (autoware_ndt_scan_matcher_rs) must produce the same NDT
// leaf parameters (mean + inverse covariance) as the C++ MultiVoxelGridCovariance for the same
// cloud. C++ leaves are fetched via the public radiusSearch; Rust leaves via the FFI lookup.

#include <autoware/ndt_scan_matcher/ndt_omp/multi_voxel_grid_covariance_omp.h>

#include "autoware_ndt_scan_matcher_rs.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <random>
#include <vector>

namespace
{
using Grid = pclomp::MultiVoxelGridCovariance<pcl::PointXYZ>;

// Relative + absolute closeness (inverse covariances can be large for near-planar voxels).
void expect_close(double a, double b, double rel = 1e-3, double abs_tol = 1e-4)
{
  EXPECT_LE(std::abs(a - b), (rel * std::abs(a)) + abs_tol) << "a=" << a << " b=" << b;
}

// Build a dense isotropic cluster of `n` points around `center` into both a pcl cloud and a flat
// xyz buffer (for the Rust FFI).
void make_cluster(
  const std::array<float, 3> & center, int n, std::mt19937 & rng,
  pcl::PointCloud<pcl::PointXYZ> & cloud, std::vector<float> & flat)
{
  std::normal_distribution<float> j(0.0F, 0.1F);
  for (int k = 0; k < n; ++k) {
    pcl::PointXYZ p;
    p.x = center[0] + j(rng);
    p.y = center[1] + j(rng);
    p.z = center[2] + j(rng);
    cloud.push_back(p);
    flat.push_back(p.x);
    flat.push_back(p.y);
    flat.push_back(p.z);
  }
}
}  // namespace

TEST(VoxelGrid, MatchesCppLeafMeanAndInverseCovariance)  // NOLINT
{
  // Cluster centers sit in voxel interiors (odd coords for leaf_size 2.0, i.e. centered in
  // [2k, 2k+2)) so jittered points stay within one voxel. The (1,11,1) cluster is near-planar in z
  // to exercise the eigenvalue-regularization branch (Magnusson eq 6.11).
  const std::array<std::array<float, 4>, 4> clusters = {{
    {{1.0F, 1.0F, 1.0F, 0.10F}},    // isotropic-ish
    {{11.0F, 1.0F, 1.0F, 0.05F}},   // moderate
    {{1.0F, 11.0F, 1.0F, 0.002F}},  // near-planar in z -> regularization
    {{5.0F, 5.0F, 5.0F, 0.08F}},
  }};

  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  std::vector<float> flat;
  std::mt19937 rng(42);
  std::normal_distribution<float> xy(0.0F, 0.1F);

  for (const auto & c : clusters) {
    std::normal_distribution<float> z(0.0F, c[3]);
    for (int k = 0; k < 40; ++k) {
      pcl::PointXYZ p;
      p.x = c[0] + xy(rng);
      p.y = c[1] + xy(rng);
      p.z = c[2] + z(rng);
      cloud->push_back(p);
      flat.push_back(p.x);
      flat.push_back(p.y);
      flat.push_back(p.z);
    }
  }
  cloud->is_dense = true;

  // C++ grid (defaults: min_points_per_voxel_ = 6, min_covar_eigvalue_mult_ = 0.01).
  Grid cpp_grid;
  cpp_grid.setLeafSize(2.0, 2.0, 2.0);
  cpp_grid.setInputCloudAndFilter(cloud, "map");
  cpp_grid.createKdtree();

  // Rust grid with matching parameters.
  const std::array<double, 3> leaf_size = {2.0, 2.0, 2.0};
  AwNdtVoxelGrid * rs_grid = autoware_ndt_scan_matcher_rs_voxel_grid_build(
    flat.data(), flat.size() / 3, leaf_size.data(), 6, 0.01);
  ASSERT_NE(rs_grid, nullptr);

  for (const auto & c : clusters) {
    pcl::PointXYZ q;
    q.x = c[0];
    q.y = c[1];
    q.z = c[2];

    std::vector<Grid::LeafConstPtr> leaves;
    const int found = cpp_grid.radiusSearch(q, 1.5, leaves, 0);
    ASSERT_GT(found, 0);
    const Grid::LeafConstPtr cpp_leaf = leaves.front();

    const std::array<float, 3> qp = {c[0], c[1], c[2]};
    std::array<double, 3> mean{};
    std::array<double, 9> icov{};
    const bool hit =
      autoware_ndt_scan_matcher_rs_voxel_grid_leaf_at(rs_grid, qp.data(), mean.data(), icov.data());
    ASSERT_TRUE(hit);

    for (int d = 0; d < 3; ++d) {
      expect_close(cpp_leaf->getMean()(d), mean[d]);
    }
    const Eigen::Matrix3d & cpp_icov = cpp_leaf->getInverseCov();
    for (int r = 0; r < 3; ++r) {
      for (int col = 0; col < 3; ++col) {
        expect_close(cpp_icov(r, col), icov[(r * 3) + col]);
      }
    }
  }

  autoware_ndt_scan_matcher_rs_voxel_grid_free(rs_grid);
}

// Multi-grid map + kd-tree radiusSearch: add 3 clouds, remove one, then the Rust map must return
// the same leaves as the C++ MultiVoxelGridCovariance for the same queries.
TEST(VoxelGrid, MultiGridRadiusSearchMatchesCpp)  // NOLINT
{
  const std::array<std::array<float, 3>, 3> centers = {{{{1, 1, 1}}, {{21, 1, 1}}, {{1, 21, 1}}}};
  std::mt19937 rng(7);

  Grid cpp_grid;
  cpp_grid.setLeafSize(2.0, 2.0, 2.0);
  const std::array<double, 3> leaf_size = {2.0, 2.0, 2.0};
  AwNdtVoxelGridMap * map = autoware_ndt_scan_matcher_rs_voxel_grid_map_new(leaf_size.data(), 6, 0.01);
  ASSERT_NE(map, nullptr);

  const std::array<const char *, 3> sids = {"a", "b", "c"};
  for (size_t g = 0; g < centers.size(); ++g) {
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    std::vector<float> flat;
    make_cluster(centers[g], 40, rng, *cloud, flat);
    cloud->is_dense = true;
    cpp_grid.setInputCloudAndFilter(cloud, sids[g]);
    autoware_ndt_scan_matcher_rs_voxel_grid_map_add_target(map, flat.data(), flat.size() / 3, g);
  }

  // Remove the middle grid ("b" / id 1) from both.
  cpp_grid.removeCloud("b");
  autoware_ndt_scan_matcher_rs_voxel_grid_map_remove_target(map, 1);
  cpp_grid.createKdtree();
  autoware_ndt_scan_matcher_rs_voxel_grid_map_create_kdtree(map);

  for (size_t g = 0; g < centers.size(); ++g) {
    pcl::PointXYZ q;
    q.x = centers[g][0];
    q.y = centers[g][1];
    q.z = centers[g][2];

    std::vector<Grid::LeafConstPtr> cpp_leaves;
    const int cpp_n = cpp_grid.radiusSearch(q, 1.5, cpp_leaves, 0);

    const std::array<float, 3> qp = {centers[g][0], centers[g][1], centers[g][2]};
    std::array<uint32_t, 16> idx{};
    const uint32_t rs_n = autoware_ndt_scan_matcher_rs_voxel_grid_map_radius_search(
      map, qp.data(), 1.5, 0, idx.data(), idx.size());

    // Removed grid "b" (g==1): no leaf near its center in either.
    EXPECT_EQ(static_cast<uint32_t>(cpp_n), rs_n) << "grid " << g;
    if (g == 1) {
      EXPECT_EQ(rs_n, 0U);
      continue;
    }
    ASSERT_EQ(rs_n, 1U);
    ASSERT_EQ(cpp_n, 1);

    std::array<double, 3> mean{};
    std::array<double, 9> icov{};
    ASSERT_TRUE(autoware_ndt_scan_matcher_rs_voxel_grid_map_leaf(map, idx[0], mean.data(), icov.data()));
    for (int d = 0; d < 3; ++d) {
      expect_close(cpp_leaves.front()->getMean()(d), mean[d]);
    }
    const Eigen::Matrix3d & cpp_icov = cpp_leaves.front()->getInverseCov();
    for (int r = 0; r < 3; ++r) {
      for (int col = 0; col < 3; ++col) {
        expect_close(cpp_icov(r, col), icov[(r * 3) + col]);
      }
    }
  }

  autoware_ndt_scan_matcher_rs_voxel_grid_map_free(map);
}
