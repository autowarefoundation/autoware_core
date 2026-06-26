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

// Differential test (engine port E4d): the Rust `align` (via FFI) must match the C++
// MultiGridNormalDistributionsTransform engine on the same target/source/guess/params, within
// tolerance (Eigen JacobiSVD + float transform vs nalgebra SVD + f32 transform). This validates the
// full optimization loop and the (pcl-form) angle-angle Hessian against the real engine.

#include <autoware/ndt_scan_matcher/ndt_omp/multigrid_ndt_omp.h>

#include "autoware_ndt_scan_matcher_rs.h"

#include <Eigen/Core>

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <random>
#include <vector>

namespace
{
using Ndt = pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;

// Target: dense clusters spread in 3D (constrains translation). Fills both a PCL cloud and the flat
// xyz buffer the Rust FFI consumes, from identical points.
void make_target(pcl::PointCloud<pcl::PointXYZ> & cloud, std::vector<float> & flat)
{
  std::mt19937 rng(7);
  std::normal_distribution<float> jitter(0.0F, 0.3F);
  const std::array<std::array<float, 3>, 5> centers = {
    {{{0.0F, 0.0F, 0.0F}},
     {{8.0F, 0.0F, 0.0F}},
     {{0.0F, 8.0F, 0.0F}},
     {{0.0F, 0.0F, 8.0F}},
     {{8.0F, 8.0F, 8.0F}}}};
  for (const auto & c : centers) {
    for (int k = 0; k < 40; ++k) {
      pcl::PointXYZ p;
      p.x = c[0] + jitter(rng);
      p.y = c[1] + jitter(rng);
      p.z = c[2] + jitter(rng);
      cloud.push_back(p);
      flat.push_back(p.x);
      flat.push_back(p.y);
      flat.push_back(p.z);
    }
  }
  cloud.is_dense = true;
}
}  // namespace

TEST(NdtAlign, MatchesCppEngine)  // NOLINT
{
  // Target + source = target translated by a known offset.
  auto target = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  std::vector<float> target_flat;
  make_target(*target, target_flat);

  const std::array<float, 3> t_true = {0.2F, -0.15F, 0.1F};
  auto source = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  std::vector<float> source_flat;
  for (const auto & p : *target) {
    pcl::PointXYZ q;
    q.x = p.x + t_true[0];
    q.y = p.y + t_true[1];
    q.z = p.z + t_true[2];
    source->push_back(q);
    source_flat.push_back(q.x);
    source_flat.push_back(q.y);
    source_flat.push_back(q.z);
  }

  pclomp::NdtParams params{};
  params.trans_epsilon = 0.01;
  params.step_size = 0.1;
  params.resolution = 2.0F;
  params.max_iterations = 30;
  params.search_method = pclomp::KDTREE;
  params.num_threads = 1;  // serial, to match the Rust serial loop
  params.regularization_scale_factor = 0.0F;
  params.use_line_search = false;

  // ---- C++ engine ----
  Ndt ndt;
  ndt.setParams(params);
  ndt.setInputTarget(target);
  auto output = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  const Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
  ndt.align(*output, guess, source);
  const pclomp::NdtResult cpp = ndt.getResult();

  // ---- Rust engine via FFI ----
  std::array<float, 16> guess16{};
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      guess16[(r * 4) + c] = guess(r, c);
    }
  }

  AwNdtAlignInput in{};
  in.target_xyz = target_flat.data();
  in.n_target = target->size();
  in.source_xyz = source_flat.data();
  in.n_source = source->size();
  in.resolution = 2.0;
  in.step_size = 0.1;
  in.trans_epsilon = 0.01;
  in.max_iterations = 30;
  in.outlier_ratio = 0.55;
  in.regularization_scale = 0.0F;
  in.regularization_pose_x = 0.0F;
  in.regularization_pose_y = 0.0F;
  in.guess = guess16.data();

  std::array<float, 16> rs_pose{};
  int32_t rs_iter = 0;
  float rs_tp = 0.0F;
  float rs_nvl = 0.0F;
  std::array<double, 36> rs_hessian{};
  constexpr uint32_t kCap = 64;
  std::array<float, kCap * 16> rs_transforms{};
  uint32_t rs_count = 0;

  AwNdtAlignOutput out{};
  out.pose = rs_pose.data();
  out.iteration_num = &rs_iter;
  out.transform_probability = &rs_tp;
  out.nearest_voxel_likelihood = &rs_nvl;
  out.hessian = rs_hessian.data();
  out.transformation_array = rs_transforms.data();
  out.transforms_cap = kCap;
  out.transforms_count = &rs_count;

  autoware_ndt_scan_matcher_rs_ndt_align(&in, &out);

  // ---- Compare within tolerance ----
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      EXPECT_NEAR(cpp.pose(r, c), rs_pose[(r * 4) + c], 2e-2) << "pose(" << r << "," << c << ")";
    }
  }
  EXPECT_NEAR(cpp.iteration_num, rs_iter, 2) << "iteration_num";
  EXPECT_NEAR(cpp.transform_probability, rs_tp, 2e-2) << "transform_probability";
  EXPECT_NEAR(cpp.nearest_voxel_transformation_likelihood, rs_nvl, 2e-2) << "nvl";

  // Hessian (validates the angle-angle pcl-form block against C++). Relative + absolute tolerance.
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      const double a = cpp.hessian(r, c);
      const double b = rs_hessian[(r * 6) + c];
      EXPECT_LE(std::abs(a - b), (5e-2 * std::abs(a)) + 1e-1)
        << "H(" << r << "," << c << ") " << a << " vs " << b;
    }
  }

  // Per-iteration trace: poses must agree up to the shorter length.
  const size_t niter = std::min<size_t>(cpp.transformation_array.size(), rs_count);
  ASSERT_GT(niter, 0u);
  for (size_t i = 0; i < niter; ++i) {
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        EXPECT_NEAR(cpp.transformation_array[i](r, c), rs_transforms[(i * 16) + (r * 4) + c], 3e-2)
          << "transformation_array[" << i << "](" << r << "," << c << ")";
      }
    }
  }
}
