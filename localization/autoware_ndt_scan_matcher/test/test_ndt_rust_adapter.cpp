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

// Differential test (engine port E6b): the drop-in NdtRustAdapter, driven through the SAME interface
// as pclomp::MultiGridNormalDistributionsTransform (setParams / addTarget-by-string-id /
// createVoxelKdtree / align / getResult / scoring / copy), must match the C++ engine within
// tolerance. Validates the adapter surface (incl. the string-id mapping, the per-iteration score
// arrays, the per-point score cloud, and the clone) in isolation, before the node typedef swap (E6c).

#include <autoware/ndt_scan_matcher/ndt_omp/multigrid_ndt_omp.h>
#include <autoware/ndt_scan_matcher/ndt_rust_adapter.hpp>

#include <Eigen/Core>

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <random>
#include <string>
#include <vector>

namespace
{
using Cpp = pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;
using Adapter = autoware::ndt_scan_matcher::NdtRustAdapter;

void make_tile(float cx, float cy, float cz, std::mt19937 & rng, pcl::PointCloud<pcl::PointXYZ> & c)
{
  std::normal_distribution<float> jitter(0.0F, 0.3F);
  for (int k = 0; k < 60; ++k) {
    c.push_back(pcl::PointXYZ(cx + jitter(rng), cy + jitter(rng), cz + jitter(rng)));
  }
  c.is_dense = true;
}

pclomp::NdtParams make_params()
{
  pclomp::NdtParams p{};
  p.trans_epsilon = 0.01;
  p.step_size = 0.1;
  p.resolution = 2.0F;
  p.max_iterations = 30;
  p.search_method = pclomp::KDTREE;
  p.num_threads = 1;
  p.regularization_scale_factor = 0.0F;
  p.use_line_search = false;
  return p;
}
}  // namespace

// The adapter is now a thin engine handle (N4c/N4e slimmed the pclomp-shaped compute methods — those
// engine FFIs are differential-tested vs the C++ engine by test_ndt_engine / test_align /
// test_estimate_covariance_multi). This covers the remaining surface: map management + clone.
TEST(NdtRustAdapter, ThinHandleMapManagement)  // NOLINT
{
  std::mt19937 rng(13);
  auto tile0 = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto tile1 = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  make_tile(0.0F, 0.0F, 0.0F, rng, *tile0);
  make_tile(8.0F, 4.0F, 0.0F, rng, *tile1);

  auto drive = [&](auto & ndt) {
    ndt.setParams(make_params());
    ndt.addTarget(tile0, "0");
    ndt.addTarget(tile1, "1");
    ndt.createVoxelKdtree();
  };
  Cpp cpp;
  drive(cpp);
  Adapter rs;
  drive(rs);

  EXPECT_TRUE(rs.hasTarget());
  EXPECT_EQ(rs.getCurrentMapIDs(), (std::vector<std::string>{"0", "1"}));  // engine-owned, sorted
  EXPECT_EQ(rs.getMaximumIterations(), cpp.getMaximumIterations());

  // Clone (the node's map-update double-buffer) carries the engine + the cell-id map, independently.
  Adapter rs_copy = rs;
  EXPECT_TRUE(rs_copy.hasTarget());
  EXPECT_EQ(rs_copy.getCurrentMapIDs(), (std::vector<std::string>{"0", "1"}));
  rs.removeTarget("0");
  EXPECT_EQ(rs.getCurrentMapIDs(), (std::vector<std::string>{"1"}));
  EXPECT_EQ(rs_copy.getCurrentMapIDs(), (std::vector<std::string>{"0", "1"}));
}
