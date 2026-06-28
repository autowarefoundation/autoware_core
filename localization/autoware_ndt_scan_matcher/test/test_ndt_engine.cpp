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

// Differential test (engine port E6a): the persistent Rust engine handle, driven through the C ABI
// with an INCREMENTAL map (addTarget by id x2 + createVoxelKdtree), must match the C++
// MultiGridNormalDistributionsTransform built the same way, on the same guess/source/params. This
// validates the node-facing handle + the per-id add/createVoxelKdtree path (the one-shot align is
// already covered by test_align).

#include <autoware/ndt_scan_matcher/ndt_omp/multigrid_ndt_omp.h>

#include "autoware_ndt_scan_matcher_rs.h"

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
using Ndt = pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;

// One dense cluster around a center; appends to a PCL cloud and the flat xyz buffer from the same
// points.
void make_tile(
  float cx, float cy, float cz, std::mt19937 & rng, pcl::PointCloud<pcl::PointXYZ> & cloud,
  std::vector<float> & flat)
{
  std::normal_distribution<float> jitter(0.0F, 0.3F);
  for (int k = 0; k < 60; ++k) {
    pcl::PointXYZ p;
    p.x = cx + jitter(rng);
    p.y = cy + jitter(rng);
    p.z = cz + jitter(rng);
    cloud.push_back(p);
    flat.push_back(p.x);
    flat.push_back(p.y);
    flat.push_back(p.z);
  }
  cloud.is_dense = true;
}
}  // namespace

TEST(NdtEngine, IncrementalMapMatchesCppEngine)  // NOLINT
{
  std::mt19937 rng(11);
  const std::array<std::array<float, 3>, 2> centers = {{{{0.0F, 0.0F, 0.0F}}, {{8.0F, 4.0F, 0.0F}}}};

  // Two map tiles (id 0 and 1), each a PCL cloud + a flat buffer for the Rust handle.
  std::array<pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>, 2> tiles;
  std::array<std::vector<float>, 2> tiles_flat;
  for (size_t i = 0; i < centers.size(); ++i) {
    tiles[i] = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    make_tile(centers[i][0], centers[i][1], centers[i][2], rng, *tiles[i], tiles_flat[i]);
  }

  // Source = all tiles' points translated by a known offset.
  const std::array<float, 3> t_true = {0.2F, -0.15F, 0.1F};
  auto source = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  std::vector<float> source_flat;
  for (const auto & tile : tiles) {
    for (const auto & p : *tile) {
      source->push_back(pcl::PointXYZ(p.x + t_true[0], p.y + t_true[1], p.z + t_true[2]));
      source_flat.push_back(p.x + t_true[0]);
      source_flat.push_back(p.y + t_true[1]);
      source_flat.push_back(p.z + t_true[2]);
    }
  }

  pclomp::NdtParams params{};
  params.trans_epsilon = 0.01;
  params.step_size = 0.1;
  params.resolution = 2.0F;
  params.max_iterations = 30;
  params.search_method = pclomp::KDTREE;
  params.num_threads = 1;
  params.regularization_scale_factor = 0.0F;
  params.use_line_search = false;

  const Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();

  // ---- C++ engine: incremental addTarget by id ----
  Ndt ndt;
  ndt.setParams(params);
  ndt.addTarget(tiles[0], "0");
  ndt.addTarget(tiles[1], "1");
  ndt.createVoxelKdtree();
  ASSERT_TRUE(ndt.hasTarget());
  auto output = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  ndt.align(*output, guess, source);
  const pclomp::NdtResult cpp = ndt.getResult();

  // ---- Rust engine handle: incremental add_target by id via the C ABI ----
  AwNdtEngine * engine = autoware_ndt_scan_matcher_rs_ndt_engine_new(2.0, 6, 0.01);
  ASSERT_NE(engine, nullptr);
  autoware_ndt_scan_matcher_rs_ndt_engine_set_params(engine, 0.01, 0.1, 2.0, 30, 0.55, 1);
  autoware_ndt_scan_matcher_rs_ndt_engine_add_target(
    engine, tiles_flat[0].data(), tiles[0]->size(), 0);
  autoware_ndt_scan_matcher_rs_ndt_engine_add_target(
    engine, tiles_flat[1].data(), tiles[1]->size(), 1);
  autoware_ndt_scan_matcher_rs_ndt_engine_create_kdtree(engine);
  EXPECT_TRUE(autoware_ndt_scan_matcher_rs_ndt_engine_has_target(engine));

  std::array<float, 16> guess16{};
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      guess16[(r * 4) + c] = guess(r, c);
    }
  }
  autoware_ndt_scan_matcher_rs_ndt_engine_align(
    engine, guess16.data(), source_flat.data(), source->size());

  std::array<float, 16> rs_pose{};
  int32_t rs_iter = 0;
  float rs_tp = 0.0F;
  float rs_nvl = 0.0F;
  std::array<double, 36> rs_hessian{};
  AwNdtAlignOutput out{};
  out.pose = rs_pose.data();
  out.iteration_num = &rs_iter;
  out.transform_probability = &rs_tp;
  out.nearest_voxel_likelihood = &rs_nvl;
  out.hessian = rs_hessian.data();
  out.transformation_array = nullptr;
  out.transforms_cap = 0;
  out.transforms_count = nullptr;
  autoware_ndt_scan_matcher_rs_ndt_engine_get_result(engine, &out);

  // remove one tile -> still has a target; remove both -> none (mirrors removeTarget).
  autoware_ndt_scan_matcher_rs_ndt_engine_remove_target(engine, 0);
  EXPECT_TRUE(autoware_ndt_scan_matcher_rs_ndt_engine_has_target(engine));
  autoware_ndt_scan_matcher_rs_ndt_engine_remove_target(engine, 1);
  EXPECT_FALSE(autoware_ndt_scan_matcher_rs_ndt_engine_has_target(engine));

  autoware_ndt_scan_matcher_rs_ndt_engine_free(engine);

  // ---- Compare within tolerance (same as test_align) ----
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      EXPECT_NEAR(cpp.pose(r, c), rs_pose[(r * 4) + c], 2e-2) << "pose(" << r << "," << c << ")";
    }
  }
  EXPECT_NEAR(cpp.iteration_num, rs_iter, 2) << "iteration_num";
  EXPECT_NEAR(cpp.transform_probability, rs_tp, 2e-2) << "transform_probability";
  EXPECT_NEAR(cpp.nearest_voxel_transformation_likelihood, rs_nvl, 2e-2) << "nvl";
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      const double a = cpp.hessian(r, c);
      const double b = rs_hessian[(r * 6) + c];
      EXPECT_LE(std::abs(a - b), (5e-2 * std::abs(a)) + 1e-1) << "H(" << r << "," << c << ")";
    }
  }
}
