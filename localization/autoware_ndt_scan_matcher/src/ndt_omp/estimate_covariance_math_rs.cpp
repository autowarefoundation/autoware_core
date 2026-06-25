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

// FFI-adapter twin of estimate_covariance_math.cpp (the NDT_USE_RUST=ON path): the same `pclomp`
// pure-helper functions, implemented by marshaling Eigen <-> row-major arrays and calling the
// `autoware_ndt_scan_matcher_rs` crate over the C ABI.

#include <autoware/ndt_scan_matcher/ndt_omp/estimate_covariance.hpp>

#include "autoware_ndt_scan_matcher_rs.h"

#include <array>
#include <utility>
#include <vector>

namespace pclomp
{

namespace
{
// Row-major (logical) serialization, independent of Eigen's column-major storage.
std::array<double, 4> to_row_major(const Eigen::Matrix2d & m)
{
  return {m(0, 0), m(0, 1), m(1, 0), m(1, 1)};
}
Eigen::Matrix2d from_row_major(const std::array<double, 4> & a)
{
  Eigen::Matrix2d m;
  m << a[0], a[1], a[2], a[3];
  return m;
}
// Top-left 2x2 yaw block of the (float) pose, as a row-major double[4].
std::array<double, 4> rotation_row_major(const Eigen::Matrix4f & pose)
{
  const Eigen::Matrix2d rot = pose.topLeftCorner<2, 2>().cast<double>();
  return to_row_major(rot);
}
}  // namespace

Eigen::Matrix2d estimate_xy_covariance_by_laplace_approximation(
  const Eigen::Matrix<double, 6, 6> & hessian)
{
  std::array<double, 36> hessian_row_major{};
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      hessian_row_major[(r * 6) + c] = hessian(r, c);
    }
  }
  std::array<double, 4> cov_out{};
  autoware_ndt_scan_matcher_rs_laplace_xy_covariance(hessian_row_major.data(), cov_out.data());
  return from_row_major(cov_out);
}

std::vector<double> calc_weight_vec(const std::vector<double> & score_vec, double temperature)
{
  std::vector<double> weights(score_vec.size());
  autoware_ndt_scan_matcher_rs_calc_weight_vec(
    score_vec.data(), score_vec.size(), temperature, weights.data());
  return weights;
}

std::pair<Eigen::Vector2d, Eigen::Matrix2d> calculate_weighted_mean_and_cov(
  const std::vector<Eigen::Vector2d> & pose_2d_vec, const std::vector<double> & weight_vec)
{
  std::vector<double> poses_flat;
  poses_flat.reserve(pose_2d_vec.size() * 2);
  for (const Eigen::Vector2d & p : pose_2d_vec) {
    poses_flat.push_back(p.x());
    poses_flat.push_back(p.y());
  }
  std::array<double, 2> mean_out{};
  std::array<double, 4> cov_out{};
  autoware_ndt_scan_matcher_rs_calculate_weighted_mean_and_cov(
    poses_flat.data(), weight_vec.data(), weight_vec.size(), mean_out.data(), cov_out.data());
  return {Eigen::Vector2d(mean_out[0], mean_out[1]), from_row_major(cov_out)};
}

Eigen::Matrix2d rotate_covariance_to_base_link(
  const Eigen::Matrix2d & covariance, const Eigen::Matrix4f & pose)
{
  const std::array<double, 4> cov = to_row_major(covariance);
  const std::array<double, 4> rot = rotation_row_major(pose);
  std::array<double, 4> out{};
  autoware_ndt_scan_matcher_rs_rotate_covariance_to_base_link(cov.data(), rot.data(), out.data());
  return from_row_major(out);
}

Eigen::Matrix2d rotate_covariance_to_map(
  const Eigen::Matrix2d & covariance, const Eigen::Matrix4f & pose)
{
  const std::array<double, 4> cov = to_row_major(covariance);
  const std::array<double, 4> rot = rotation_row_major(pose);
  std::array<double, 4> out{};
  autoware_ndt_scan_matcher_rs_rotate_covariance_to_map(cov.data(), rot.data(), out.data());
  return from_row_major(out);
}

Eigen::Matrix2d adjust_diagonal_covariance(
  const Eigen::Matrix2d & covariance, const Eigen::Matrix4f & pose, const double fixed_cov00,
  const double fixed_cov11)
{
  const std::array<double, 4> cov = to_row_major(covariance);
  const std::array<double, 4> rot = rotation_row_major(pose);
  std::array<double, 4> out{};
  autoware_ndt_scan_matcher_rs_adjust_diagonal_covariance(
    cov.data(), rot.data(), fixed_cov00, fixed_cov11, out.data());
  return from_row_major(out);
}

}  // namespace pclomp
