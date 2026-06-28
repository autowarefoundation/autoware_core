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

// Differential test (engine port E5): the Rust multi-NDT covariance estimators (via FFI) must match
// the C++ pclomp implementation on the same target/source/main-pose/offsets/params. propose +
// multi_ndt (re-align per candidate) + multi_ndt_score (score per candidate) are all checked. The
// per-candidate align/score parity is already covered by test_align; this validates the orchestration
// (pose proposal, weighting, weighted mean/covariance, unbiased correction).

#include <autoware/ndt_scan_matcher/ndt_omp/estimate_covariance.hpp>
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

// Build the target/source clouds (+ flat buffers) and run the main C++ align; returns its NdtResult.
struct Fixture
{
  pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> target;
  pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> source;
  std::vector<float> target_flat;
  std::vector<float> source_flat;
  Ndt ndt;
  pclomp::NdtResult main_result;
};

void build_fixture(Fixture & f)
{
  f.target = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  make_target(*f.target, f.target_flat);

  const std::array<float, 3> t_true = {0.2F, -0.15F, 0.1F};
  f.source = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (const auto & p : *f.target) {
    pcl::PointXYZ q;
    q.x = p.x + t_true[0];
    q.y = p.y + t_true[1];
    q.z = p.z + t_true[2];
    f.source->push_back(q);
    f.source_flat.push_back(q.x);
    f.source_flat.push_back(q.y);
    f.source_flat.push_back(q.z);
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

  f.ndt.setParams(params);
  f.ndt.setInputTarget(f.target);
  auto output = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  f.ndt.align(*output, Eigen::Matrix4f::Identity(), f.source);
  f.main_result = f.ndt.getResult();
}

const std::vector<double> kOffsetX = {0.5, -0.5, 0.0, 0.0, 0.35, -0.35};
const std::vector<double> kOffsetY = {0.0, 0.0, 0.5, -0.5, 0.35, -0.35};

std::array<float, 16> pose16(const Eigen::Matrix4f & m)
{
  std::array<float, 16> out{};
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      out[(r * 4) + c] = m(r, c);
    }
  }
  return out;
}

AwMultiNdtCovInput make_input(const Fixture & f, const std::array<float, 16> & main_pose)
{
  AwMultiNdtCovInput in{};
  in.target_xyz = f.target_flat.data();
  in.n_target = f.target->size();
  in.source_xyz = f.source_flat.data();
  in.n_source = f.source->size();
  in.main_pose = main_pose.data();
  in.offset_x = kOffsetX.data();
  in.offset_y = kOffsetY.data();
  in.n_offsets = kOffsetX.size();
  in.resolution = 2.0;
  in.step_size = 0.1;
  in.trans_epsilon = 0.01;
  in.max_iterations = 30;
  in.outlier_ratio = 0.55;
  in.main_nvtl = f.main_result.nearest_voxel_transformation_likelihood;
  in.temperature = 0.05;
  return in;
}
}  // namespace

TEST(EstimateCovarianceMulti, ProposePosesMatchesCpp)  // NOLINT
{
  Fixture f;
  build_fixture(f);

  const std::vector<Eigen::Matrix4f> cpp =
    pclomp::propose_poses_to_search(f.main_result, kOffsetX, kOffsetY);

  const std::array<float, 16> main_pose = pose16(f.main_result.pose);
  std::vector<float> rs(kOffsetX.size() * 16, 0.0F);
  autoware_ndt_scan_matcher_rs_propose_poses_to_search(
    main_pose.data(), kOffsetX.data(), kOffsetY.data(), kOffsetX.size(), rs.data());

  ASSERT_EQ(cpp.size(), kOffsetX.size());
  for (size_t i = 0; i < cpp.size(); ++i) {
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        EXPECT_NEAR(cpp[i](r, c), rs[(i * 16) + (r * 4) + c], 1e-4)
          << "pose[" << i << "](" << r << "," << c << ")";
      }
    }
  }
}

TEST(EstimateCovarianceMulti, MultiNdtMatchesCpp)  // NOLINT
{
  Fixture f;
  build_fixture(f);

  const std::vector<Eigen::Matrix4f> poses =
    pclomp::propose_poses_to_search(f.main_result, kOffsetX, kOffsetY);
  const pclomp::ResultOfMultiNdtCovarianceEstimation cpp =
    pclomp::estimate_xy_covariance_by_multi_ndt(f.main_result, f.ndt, poses, f.source);

  const std::array<float, 16> main_pose = pose16(f.main_result.pose);
  const AwMultiNdtCovInput in = make_input(f, main_pose);
  std::array<double, 2> rs_mean{};
  std::array<double, 4> rs_cov{};
  autoware_ndt_scan_matcher_rs_estimate_cov_multi_ndt(&in, rs_mean.data(), rs_cov.data());

  EXPECT_NEAR(cpp.mean(0), rs_mean[0], 2e-2) << "mean.x";
  EXPECT_NEAR(cpp.mean(1), rs_mean[1], 2e-2) << "mean.y";
  for (int r = 0; r < 2; ++r) {
    for (int c = 0; c < 2; ++c) {
      EXPECT_LE(std::abs(cpp.covariance(r, c) - rs_cov[(r * 2) + c]), 5e-2 + (0.2 * std::abs(cpp.covariance(r, c))))
        << "cov(" << r << "," << c << ")";
    }
  }
}

TEST(EstimateCovarianceMulti, MultiNdtScoreMatchesCpp)  // NOLINT
{
  Fixture f;
  build_fixture(f);

  const std::vector<Eigen::Matrix4f> poses =
    pclomp::propose_poses_to_search(f.main_result, kOffsetX, kOffsetY);
  const double temperature = 0.05;
  const pclomp::ResultOfMultiNdtCovarianceEstimation cpp =
    pclomp::estimate_xy_covariance_by_multi_ndt_score(
      f.main_result, f.ndt, poses, f.source, temperature);

  const std::array<float, 16> main_pose = pose16(f.main_result.pose);
  const AwMultiNdtCovInput in = make_input(f, main_pose);
  std::array<double, 2> rs_mean{};
  std::array<double, 4> rs_cov{};
  autoware_ndt_scan_matcher_rs_estimate_cov_multi_ndt_score(&in, rs_mean.data(), rs_cov.data());

  EXPECT_NEAR(cpp.mean(0), rs_mean[0], 1e-2) << "mean.x";
  EXPECT_NEAR(cpp.mean(1), rs_mean[1], 1e-2) << "mean.y";
  for (int r = 0; r < 2; ++r) {
    for (int c = 0; c < 2; ++c) {
      EXPECT_LE(std::abs(cpp.covariance(r, c) - rs_cov[(r * 2) + c]), 1e-2 + (0.1 * std::abs(cpp.covariance(r, c))))
        << "cov(" << r << "," << c << ")";
    }
  }
}
