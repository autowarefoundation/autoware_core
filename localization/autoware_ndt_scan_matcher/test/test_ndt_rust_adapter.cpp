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

TEST(NdtRustAdapter, MatchesCppEngine)  // NOLINT
{
  std::mt19937 rng(13);
  auto tile0 = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto tile1 = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  make_tile(0.0F, 0.0F, 0.0F, rng, *tile0);
  make_tile(8.0F, 4.0F, 0.0F, rng, *tile1);

  const std::array<float, 3> t_true = {0.2F, -0.15F, 0.1F};
  auto source = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (const auto & tile : {tile0, tile1}) {
    for (const auto & p : *tile) {
      source->push_back(pcl::PointXYZ(p.x + t_true[0], p.y + t_true[1], p.z + t_true[2]));
    }
  }
  const Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();

  auto drive = [&](auto & ndt) {
    ndt.setParams(make_params());
    ndt.addTarget(tile0, "0");
    ndt.addTarget(tile1, "1");
    ndt.createVoxelKdtree();
    auto out = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    ndt.align(*out, guess, source);
  };

  Cpp cpp;
  drive(cpp);
  const pclomp::NdtResult cpp_r = cpp.getResult();

  Adapter rs;
  drive(rs);
  const pclomp::NdtResult rs_r = rs.getResult();

  // Map management
  EXPECT_TRUE(rs.hasTarget());
  EXPECT_EQ(rs.getCurrentMapIDs().size(), 2u);
  EXPECT_EQ(rs.getMaximumIterations(), cpp.getMaximumIterations());

  // align result
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      EXPECT_NEAR(cpp_r.pose(r, c), rs_r.pose(r, c), 2e-2) << "pose(" << r << "," << c << ")";
    }
  }
  EXPECT_NEAR(cpp_r.iteration_num, rs_r.iteration_num, 2);
  EXPECT_NEAR(cpp_r.transform_probability, rs_r.transform_probability, 2e-2);
  EXPECT_NEAR(
    cpp_r.nearest_voxel_transformation_likelihood, rs_r.nearest_voxel_transformation_likelihood,
    2e-2);
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      const double a = cpp_r.hessian(r, c);
      EXPECT_LE(std::abs(a - rs_r.hessian(r, c)), (5e-2 * std::abs(a)) + 1e-1)
        << "H(" << r << "," << c << ")";
    }
  }
  // per-iteration score traces are populated and sized like the node expects
  EXPECT_EQ(
    rs_r.transform_probability_array.size(), static_cast<size_t>(rs_r.iteration_num) + 1);
  EXPECT_EQ(
    rs_r.nearest_voxel_transformation_likelihood_array.size(),
    static_cast<size_t>(rs_r.iteration_num) + 1);

  // standalone scoring
  EXPECT_NEAR(
    cpp.calculateNearestVoxelTransformationLikelihood(*source),
    rs.calculateNearestVoxelTransformationLikelihood(*source), 2e-2);

  // per-point score cloud: same included points (those with a neighbor) + matching intensities
  const auto cpp_scores = cpp.calculateNearestVoxelScoreEachPoint(*source);
  const auto rs_scores = rs.calculateNearestVoxelScoreEachPoint(*source);
  ASSERT_EQ(cpp_scores.size(), rs_scores.size());
  for (size_t i = 0; i < cpp_scores.size(); ++i) {
    EXPECT_NEAR(cpp_scores.points[i].intensity, rs_scores.points[i].intensity, 2e-2)
      << "score point " << i;
  }

  // copy (the node's map-update double-buffer): the copy aligns to the same pose.
  Adapter rs_copy = rs;
  auto out2 = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  rs_copy.align(*out2, guess, source);
  EXPECT_TRUE(rs_copy.getResult().pose.isApprox(rs_r.pose, 1e-5F));
}
