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

#include <autoware/ndt_scan_matcher/ndt_omp/estimate_covariance.hpp>

#include <cassert>
#include <memory>
#include <utility>
#include <vector>

// NOTE: The pure helpers (calc_weight_vec, calculate_weighted_mean_and_cov,
// estimate_xy_covariance_by_laplace_approximation, rotate_covariance_to_*,
// adjust_diagonal_covariance) live in estimate_covariance_math.cpp. The engine-driving estimators
// (estimate_xy_covariance_by_multi_ndt[_score]) and propose_poses_to_search live here — they drive
// the concrete pclomp engine (NDT_USE_RUST computes the covariance in the Rust orchestrator
// instead, so the node no longer drives these under ON; they remain for the C++ baseline and the
// differential tests).

namespace pclomp
{

std::vector<Eigen::Matrix4f> propose_poses_to_search(
  const NdtResult & ndt_result, const std::vector<double> & offset_x,
  const std::vector<double> & offset_y)
{
  assert(offset_x.size() == offset_y.size());
  const Eigen::Matrix4f & center_pose = ndt_result.pose;
  const Eigen::Matrix2d rot = ndt_result.pose.topLeftCorner<2, 2>().cast<double>();
  std::vector<Eigen::Matrix4f> poses_to_search;
  for (int i = 0; i < static_cast<int>(offset_x.size()); i++) {
    const Eigen::Vector2d pose_offset(offset_x[i], offset_y[i]);
    const Eigen::Vector2d rotated_pose_offset_2d = rot * pose_offset;
    Eigen::Matrix4f curr_pose = center_pose;
    curr_pose(0, 3) += static_cast<float>(rotated_pose_offset_2d.x());
    curr_pose(1, 3) += static_cast<float>(rotated_pose_offset_2d.y());
    poses_to_search.emplace_back(curr_pose);
  }
  return poses_to_search;
}

ResultOfMultiNdtCovarianceEstimation estimate_xy_covariance_by_multi_ndt(
  const NdtResult & ndt_result,
  MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> & ndt_ref,
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

    const Eigen::Vector2d sub_ndt_pose_2d =
      sub_ndt_result.pose.topRightCorner<2, 1>().cast<double>();
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

ResultOfMultiNdtCovarianceEstimation estimate_xy_covariance_by_multi_ndt_score(
  const NdtResult & ndt_result,
  MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> & ndt_ref,
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
