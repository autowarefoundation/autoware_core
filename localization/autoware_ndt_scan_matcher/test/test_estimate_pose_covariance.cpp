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

// Test (node port N4b): the covariance orchestrator FFI
// (autoware_ndt_scan_matcher_rs_node_estimate_pose_covariance) assembles rotate + dispatch + scale +
// adjust against the live engine map. It is checked against a hand-composition built from the same
// (still-compiled) pieces on the same NdtRustAdapter: pclomp::estimate_xy_covariance_by_multi_ndt +
// adjust_diagonal_covariance + the Rust rotate_covariance FFI. This pins the C++ marshaling
// (pose/hessian/offsets/output_cov → FFI) and the publish-kind asymmetry. The cov math itself is
// differential-tested vs pure C++ in test_estimate_covariance_multi (E5).

#include <autoware/ndt_scan_matcher/ndt_omp/estimate_covariance.hpp>
#include <autoware/ndt_scan_matcher/ndt_rust_adapter.hpp>

#include "autoware_ndt_scan_matcher_rs.h"

#include <Eigen/Core>

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <cmath>
#include <random>
#include <vector>

namespace
{
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

// Build a driven 2-tile adapter + the source cloud (shifted by a small known translation).
Adapter make_driven_adapter(pcl::PointCloud<pcl::PointXYZ>::Ptr & source_out)
{
  std::mt19937 rng(13);
  auto tile0 = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto tile1 = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  make_tile(0.0F, 0.0F, 0.0F, rng, *tile0);
  make_tile(8.0F, 4.0F, 0.0F, rng, *tile1);
  const std::array<float, 3> t_true = {0.2F, -0.15F, 0.1F};
  source_out = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (const auto & tile : {tile0, tile1}) {
    for (const auto & p : *tile) {
      source_out->push_back(pcl::PointXYZ(p.x + t_true[0], p.y + t_true[1], p.z + t_true[2]));
    }
  }
  Adapter rs;
  rs.setParams(make_params());
  rs.addTarget(tile0, "0");
  rs.addTarget(tile1, "1");
  rs.createVoxelKdtree();
  return rs;
}

std::array<double, 36> make_output_cov()
{
  std::array<double, 36> c{};
  c[0] = 0.25;
  c[7] = 0.36;
  c[14] = 0.01;
  c[21] = 0.01;
  c[28] = 0.01;
  c[35] = 0.01;
  return c;
}

const std::vector<double> kOffsetX = {0.3, -0.3, 0.0, 0.0};
const std::vector<double> kOffsetY = {0.0, 0.0, 0.3, -0.3};
const std::array<double, 9> kRotId = {1, 0, 0, 0, 1, 0, 0, 0, 1};

// Fill an AwCovEstimationInput from a driven adapter's result + the given estimation type.
AwCovEstimationInput make_input(
  const pclomp::NdtResult & ndt_result, const std::vector<float> & source_flat, int32_t type,
  const std::array<double, 36> & output_cov, double scale)
{
  AwCovEstimationInput in{};
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      in.result_pose[(r * 4) + c] = ndt_result.pose(r, c);
      in.initial_pose[(r * 4) + c] = (r == c) ? 1.0F : 0.0F;
    }
  }
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      in.hessian[(r * 6) + c] = ndt_result.hessian(r, c);
    }
  }
  std::copy(output_cov.begin(), output_cov.end(), in.output_pose_covariance);
  std::copy(kRotId.begin(), kRotId.end(), in.map_to_base_link_rot3x3);
  in.source = source_flat.data();
  in.n_source = source_flat.size() / 3;
  in.estimation_type = type;
  in.offset_x = kOffsetX.data();
  in.offset_y = kOffsetY.data();
  in.n_offsets = kOffsetX.size();
  in.scale_factor = scale;
  in.temperature = 0.1;
  in.main_nvtl = static_cast<float>(ndt_result.nearest_voxel_transformation_likelihood);
  return in;
}

std::vector<float> flatten(const pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  std::vector<float> flat;
  flat.reserve(cloud.size() * 3);
  for (const auto & p : cloud.points) {
    flat.push_back(p.x);
    flat.push_back(p.y);
    flat.push_back(p.z);
  }
  return flat;
}

// Run the FFI; returns the 36 covariance + fills the publish-kind/counts via out-params.
std::array<double, 36> run_ffi(
  Adapter & rs, const AwCovEstimationInput & in, int32_t & publish_kind, uint32_t & result_count,
  uint32_t & initial_count)
{
  const auto cap = static_cast<uint32_t>(in.n_offsets + 1);
  std::vector<float> res_buf(static_cast<size_t>(cap) * 16);
  std::vector<float> init_buf(static_cast<size_t>(cap) * 16);
  AwCovEstimationOutput out{};
  out.multi_ndt_result_poses = res_buf.data();
  out.multi_initial_poses = init_buf.data();
  out.pose_cap = cap;
  autoware_ndt_scan_matcher_rs_node_estimate_pose_covariance(rs.raw_handle(), &in, &out);
  publish_kind = out.publish_kind;
  result_count = out.multi_ndt_result_count;
  initial_count = out.multi_initial_count;
  std::array<double, 36> cov{};
  std::copy_n(out.ndt_covariance, 36, cov.begin());
  return cov;
}
}  // namespace

// FIXED_VALUE: just the rotated configured covariance (identity rotation → unchanged), no publish.
TEST(EstimatePoseCovariance, FixedValueJustRotates)  // NOLINT
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source;
  Adapter rs = make_driven_adapter(source);
  auto out_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  rs.align(*out_cloud, Eigen::Matrix4f::Identity(), source);
  const pclomp::NdtResult ndt_result = rs.getResult();

  const auto output_cov = make_output_cov();
  const auto in = make_input(ndt_result, flatten(*source), 0, output_cov, 1.0);
  int32_t kind = -1;
  uint32_t rc = 99;
  uint32_t ic = 99;
  const auto cov = run_ffi(rs, in, kind, rc, ic);

  EXPECT_EQ(kind, 0);
  // Identity rotation leaves the configured covariance unchanged.
  for (int i = 0; i < 36; ++i) {
    EXPECT_DOUBLE_EQ(cov[i], output_cov[i]) << "i=" << i;
  }
}

// MULTI_NDT: the FFI's ndt_covariance matches rotate(config) with the 2x2 block replaced by
// adjust(scale * estimate), where the estimate is the C++ templated estimator on the same adapter.
TEST(EstimatePoseCovariance, MultiNdtMatchesComposition)  // NOLINT
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source;
  Adapter rs = make_driven_adapter(source);
  auto out_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  rs.align(*out_cloud, Eigen::Matrix4f::Identity(), source);
  const pclomp::NdtResult ndt_result = rs.getResult();
  const auto output_cov = make_output_cov();
  const double scale = 2.0;

  // Run the FFI first (the node order: run_align then covariance, on the freshly-aligned engine).
  const auto in = make_input(ndt_result, flatten(*source), 2, output_cov, scale);
  int32_t kind = -1;
  uint32_t rc = 0;
  uint32_t ic = 0;
  const auto cov = run_ffi(rs, in, kind, rc, ic);

  // Reference 2x2 via the still-compiled templated estimator on the same adapter (mutates rs, but the
  // FFI result above is already captured).
  const std::vector<Eigen::Matrix4f> poses =
    pclomp::propose_poses_to_search(ndt_result, kOffsetX, kOffsetY);
  const pclomp::ResultOfMultiNdtCovarianceEstimation ref =
    pclomp::estimate_xy_covariance_by_multi_ndt(ndt_result, rs, poses, source);
  const Eigen::Matrix2d adj =
    pclomp::adjust_diagonal_covariance(ref.covariance * scale, ndt_result.pose, output_cov[0], output_cov[7]);
  std::array<double, 36> expected{};
  autoware_ndt_scan_matcher_rs_rotate_covariance(output_cov.data(), kRotId.data(), expected.data());
  expected[0] = adj(0, 0);
  expected[7] = adj(1, 1);
  expected[1] = adj(1, 0);
  expected[6] = adj(0, 1);

  EXPECT_EQ(kind, 1);
  EXPECT_EQ(rc, kOffsetX.size() + 1);  // main + per-candidate result poses
  EXPECT_EQ(ic, kOffsetX.size() + 1);  // main + per-candidate initial poses
  // The non-2x2 entries are the rotated configured covariance (identity rotation → exact). The 2x2
  // block matches within the established multi-NDT cov tolerance (test_estimate_covariance_multi,
  // E5): abs(diff) <= 5e-2 + 0.2*abs(expected). The two estimators are not bit-identical.
  for (int i = 0; i < 36; ++i) {
    const bool in_xy_block = (i == 0 || i == 1 || i == 6 || i == 7);
    if (in_xy_block) {
      EXPECT_LE(std::abs(cov[i] - expected[i]), 5e-2 + (0.2 * std::abs(expected[i]))) << "i=" << i;
    } else {
      EXPECT_DOUBLE_EQ(cov[i], expected[i]) << "i=" << i;
    }
  }
}

// MULTI_NDT_SCORE publishes only the initial poses (publish_kind 2, no result poses).
TEST(EstimatePoseCovariance, MultiNdtScorePublishesInitialOnly)  // NOLINT
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source;
  Adapter rs = make_driven_adapter(source);
  auto out_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  rs.align(*out_cloud, Eigen::Matrix4f::Identity(), source);
  const pclomp::NdtResult ndt_result = rs.getResult();

  const auto in = make_input(ndt_result, flatten(*source), 3, make_output_cov(), 1.0);
  int32_t kind = -1;
  uint32_t rc = 99;
  uint32_t ic = 0;
  run_ffi(rs, in, kind, rc, ic);

  EXPECT_EQ(kind, 2);
  EXPECT_EQ(rc, 0u);
  EXPECT_EQ(ic, kOffsetX.size() + 1);
}
