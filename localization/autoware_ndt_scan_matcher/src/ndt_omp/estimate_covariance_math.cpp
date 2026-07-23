// Copyright 2023 TIER IV, Inc.
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

// Pure covariance helpers, split out from estimate_covariance.cpp. Under NDT_USE_RUST the node
// computes the covariance in the Rust orchestrator, but these C++ helpers are still built (for the
// OFF baseline and the differential tests); the equivalent Rust math is differential-tested via the
// engine FFIs. No logic changes.

#include <autoware/ndt_scan_matcher/ndt_omp/estimate_covariance.hpp>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace pclomp
{

Eigen::Matrix2d estimate_xy_covariance_by_laplace_approximation(
  const Eigen::Matrix<double, 6, 6> & hessian)
{
  const Eigen::Matrix2d hessian_xy = hessian.block<2, 2>(0, 0);
  Eigen::Matrix2d covariance_xy = -hessian_xy.inverse();
  return covariance_xy;
}

std::vector<double> calc_weight_vec(const std::vector<double> & score_vec, double temperature)
{
  const int n = static_cast<int>(score_vec.size());
  const double max_score = *std::max_element(score_vec.begin(), score_vec.end());
  std::vector<double> exp_score_vec(n);
  double exp_score_sum = 0.0;
  for (int i = 0; i < n; i++) {
    exp_score_vec[i] = std::exp((score_vec[i] - max_score) / temperature);
    exp_score_sum += exp_score_vec[i];
  }
  for (int i = 0; i < n; i++) {
    exp_score_vec[i] /= exp_score_sum;
  }
  return exp_score_vec;
}

std::pair<Eigen::Vector2d, Eigen::Matrix2d> calculate_weighted_mean_and_cov(
  const std::vector<Eigen::Vector2d> & pose_2d_vec, const std::vector<double> & weight_vec)
{
  const int n = static_cast<int>(pose_2d_vec.size());
  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  for (int i = 0; i < n; i++) {
    mean += weight_vec[i] * pose_2d_vec[i];
  }
  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  for (int i = 0; i < n; i++) {
    const Eigen::Vector2d diff = pose_2d_vec[i] - mean;
    covariance += weight_vec[i] * diff * diff.transpose();
  }
  return {mean, covariance};
}

Eigen::Matrix2d rotate_covariance_to_base_link(
  const Eigen::Matrix2d & covariance, const Eigen::Matrix4f & pose)
{
  const Eigen::Matrix2d rot = pose.topLeftCorner<2, 2>().cast<double>();
  return rot.transpose() * covariance * rot;
}

Eigen::Matrix2d rotate_covariance_to_map(
  const Eigen::Matrix2d & covariance, const Eigen::Matrix4f & pose)
{
  const Eigen::Matrix2d rot = pose.topLeftCorner<2, 2>().cast<double>();
  return rot * covariance * rot.transpose();
}

Eigen::Matrix2d adjust_diagonal_covariance(
  const Eigen::Matrix2d & covariance, const Eigen::Matrix4f & pose, const double fixed_cov00,
  const double fixed_cov11)
{
  Eigen::Matrix2d cov_base_link = rotate_covariance_to_base_link(covariance, pose);
  cov_base_link(0, 0) = std::max(cov_base_link(0, 0), fixed_cov00);
  cov_base_link(1, 1) = std::max(cov_base_link(1, 1), fixed_cov11);
  Eigen::Matrix2d cov_map = rotate_covariance_to_map(cov_base_link, pose);
  return cov_map;
}

}  // namespace pclomp
