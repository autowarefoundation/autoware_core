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

#ifndef AUTOWARE__NDT_SCAN_MATCHER__NDT_OMP__ESTIMATE_COVARIANCE_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__NDT_OMP__ESTIMATE_COVARIANCE_HPP_

#include "multigrid_ndt_omp.h"

#include <Eigen/Core>

#include <pcl/common/transforms.h>

#include <memory>
#include <utility>
#include <vector>

namespace pclomp
{

struct ResultOfMultiNdtCovarianceEstimation
{
  Eigen::Vector2d mean;
  Eigen::Matrix2d covariance;
  std::vector<Eigen::Matrix4f> ndt_initial_poses;
  std::vector<NdtResult> ndt_results;
};

/** \brief Estimate functions */
Eigen::Matrix2d estimate_xy_covariance_by_laplace_approximation(
  const Eigen::Matrix<double, 6, 6> & hessian);
// estimate_xy_covariance_by_multi_ndt[_score] are templated over the NDT type (defined below, after
// the pure-helper declarations) so both the C++ engine and the NDT_USE_RUST adapter can drive them.

/** \brief Propose poses to search.
 * (1) Compute covariance by Laplace approximation
 * (2) Find rotation matrix aligning covariance to principal axes
 * (3) Propose search points by adding offset_x and offset_y to the center_pose
 */
std::vector<Eigen::Matrix4f> propose_poses_to_search(
  const NdtResult & ndt_result, const std::vector<double> & offset_x,
  const std::vector<double> & offset_y);

/** \brief Calculate weights by exponential */
std::vector<double> calc_weight_vec(const std::vector<double> & score_vec, double temperature);

/** \brief Calculate weighted mean and covariance */
std::pair<Eigen::Vector2d, Eigen::Matrix2d> calculate_weighted_mean_and_cov(
  const std::vector<Eigen::Vector2d> & pose_2d_vec, const std::vector<double> & weight_vec);

/** \brief Rotate covariance to base_link coordinate */
Eigen::Matrix2d rotate_covariance_to_base_link(
  const Eigen::Matrix2d & covariance, const Eigen::Matrix4f & pose);

/** \brief Rotate covariance to map coordinate */
Eigen::Matrix2d rotate_covariance_to_map(
  const Eigen::Matrix2d & covariance, const Eigen::Matrix4f & pose);

/** \brief  Adjust diagonal covariance */
Eigen::Matrix2d adjust_diagonal_covariance(
  const Eigen::Matrix2d & covariance, const Eigen::Matrix4f & pose, const double fixed_cov00,
  const double fixed_cov11);

// --- Engine-driving estimators (templated over the NDT type: the C++ engine or the Rust adapter) ---

/** \brief Multi-NDT covariance: re-align from each candidate pose; uniform weights; unbiased
 * (n-1)/n covariance. */
template <class NdtT>
ResultOfMultiNdtCovarianceEstimation estimate_xy_covariance_by_multi_ndt(
  const NdtResult & ndt_result, NdtT & ndt_ref,
  const std::vector<Eigen::Matrix4f> & poses_to_search,
  const pcl::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> & source)
{
  // initialize by the main result
  const Eigen::Vector2d ndt_pose_2d(ndt_result.pose(0, 3), ndt_result.pose(1, 3));
  std::vector<Eigen::Vector2d> ndt_pose_2d_vec{ndt_pose_2d};

  // multiple searches
  std::vector<NdtResult> ndt_results;
  for (const Eigen::Matrix4f & curr_pose : poses_to_search) {
    auto sub_output_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    ndt_ref.align(*sub_output_cloud, curr_pose, source);
    const NdtResult sub_ndt_result = ndt_ref.getResult();
    ndt_results.push_back(sub_ndt_result);

    const Eigen::Vector2d sub_ndt_pose_2d = sub_ndt_result.pose.topRightCorner<2, 1>().cast<double>();
    ndt_pose_2d_vec.emplace_back(sub_ndt_pose_2d);
  }

  // calculate the weights
  const int n = static_cast<int>(ndt_results.size()) + 1;
  const std::vector<double> weight_vec(n, 1.0 / n);

  // calculate mean and covariance
  auto [mean, covariance] = calculate_weighted_mean_and_cov(ndt_pose_2d_vec, weight_vec);

  // unbiased covariance
  covariance *= static_cast<double>(n - 1) / n;

  return {mean, covariance, poses_to_search, ndt_results};
}

/** \brief Multi-NDT-score covariance: score (no re-align) each candidate; temperature softmax
 * weights. */
template <class NdtT>
ResultOfMultiNdtCovarianceEstimation estimate_xy_covariance_by_multi_ndt_score(
  const NdtResult & ndt_result, NdtT & ndt_ref,
  const std::vector<Eigen::Matrix4f> & poses_to_search,
  const pcl::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> & source, const double temperature)
{
  // initialize by the main result
  const Eigen::Vector2d ndt_pose_2d(ndt_result.pose(0, 3), ndt_result.pose(1, 3));
  std::vector<Eigen::Vector2d> ndt_pose_2d_vec{ndt_pose_2d};
  std::vector<double> score_vec{ndt_result.nearest_voxel_transformation_likelihood};

  pcl::PointCloud<pcl::PointXYZ> trans_cloud;

  // multiple searches
  std::vector<NdtResult> ndt_results;
  for (const Eigen::Matrix4f & curr_pose : poses_to_search) {
    const Eigen::Vector2d sub_ndt_pose_2d = curr_pose.topRightCorner<2, 1>().cast<double>();
    ndt_pose_2d_vec.emplace_back(sub_ndt_pose_2d);

    pcl::transformPointCloud(*source, trans_cloud, curr_pose);
    const double nvtl = ndt_ref.calculateNearestVoxelTransformationLikelihood(trans_cloud);
    score_vec.emplace_back(nvtl);

    NdtResult sub_ndt_result{};
    sub_ndt_result.pose = curr_pose;
    sub_ndt_result.iteration_num = 0;
    sub_ndt_result.nearest_voxel_transformation_likelihood = static_cast<float>(nvtl);
    ndt_results.push_back(sub_ndt_result);
  }

  // calculate the weights
  const std::vector<double> weight_vec = calc_weight_vec(score_vec, temperature);

  // calculate mean and covariance
  const auto [mean, covariance] = calculate_weighted_mean_and_cov(ndt_pose_2d_vec, weight_vec);
  return {mean, covariance, poses_to_search, ndt_results};
}

}  // namespace pclomp

#endif  // AUTOWARE__NDT_SCAN_MATCHER__NDT_OMP__ESTIMATE_COVARIANCE_HPP_
