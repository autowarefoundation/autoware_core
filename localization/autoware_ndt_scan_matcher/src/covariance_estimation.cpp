// Copyright 2026 Autoware Foundation
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

#include <autoware/localization_util/util_func.hpp>
#include <autoware/ndt_scan_matcher/covariance_estimation.hpp>
#include <autoware/ndt_scan_matcher/ndt_omp/estimate_covariance.hpp>

namespace autoware::ndt_scan_matcher
{

CovarianceComputationResult compute_covariance_estimate(
  const pclomp::NdtResult & ndt_result, const Eigen::Matrix4f & initial_pose_matrix,
  const HyperParameters::Covariance & param, const std::shared_ptr<NdtType> & ndt_ptr,
  const rclcpp::Time & stamp, const std::string & map_frame)
{
  CovarianceComputationResult result{};

  result.ndt_result_poses.header.stamp = stamp;
  result.ndt_result_poses.header.frame_id = map_frame;
  result.ndt_initial_poses.header.stamp = stamp;
  result.ndt_initial_poses.header.frame_id = map_frame;

  result.ndt_result_poses.poses.push_back(
    autoware::localization_util::matrix4f_to_pose(ndt_result.pose));
  result.ndt_initial_poses.poses.push_back(
    autoware::localization_util::matrix4f_to_pose(initial_pose_matrix));

  switch (param.covariance_estimation.covariance_estimation_type) {
    case CovarianceEstimationType::LAPLACE_APPROXIMATION: {
      result.covariance =
        pclomp::estimate_xy_covariance_by_laplace_approximation(ndt_result.hessian);
      break;
    }
    case CovarianceEstimationType::MULTI_NDT: {
      const std::vector<Eigen::Matrix4f> poses_to_search = pclomp::propose_poses_to_search(
        ndt_result, param.covariance_estimation.initial_pose_offset_model_x,
        param.covariance_estimation.initial_pose_offset_model_y);
      const pclomp::ResultOfMultiNdtCovarianceEstimation multi_ndt_result =
        estimate_xy_covariance_by_multi_ndt(ndt_result, ndt_ptr, poses_to_search);
      for (size_t i = 0; i < multi_ndt_result.ndt_initial_poses.size(); ++i) {
        result.ndt_result_poses.poses.push_back(
          autoware::localization_util::matrix4f_to_pose(multi_ndt_result.ndt_results[i].pose));
        result.ndt_initial_poses.poses.push_back(
          autoware::localization_util::matrix4f_to_pose(multi_ndt_result.ndt_initial_poses[i]));
      }
      result.covariance = multi_ndt_result.covariance;
      break;
    }
    case CovarianceEstimationType::MULTI_NDT_SCORE: {
      const std::vector<Eigen::Matrix4f> poses_to_search = pclomp::propose_poses_to_search(
        ndt_result, param.covariance_estimation.initial_pose_offset_model_x,
        param.covariance_estimation.initial_pose_offset_model_y);
      const auto multi_ndt_score_result = estimate_xy_covariance_by_multi_ndt_score(
        ndt_result, ndt_ptr, poses_to_search, param.covariance_estimation.temperature);
      for (const auto & sub_initial_pose_matrix : poses_to_search) {
        result.ndt_initial_poses.poses.push_back(
          autoware::localization_util::matrix4f_to_pose(sub_initial_pose_matrix));
      }
      result.covariance = multi_ndt_score_result.covariance;
      break;
    }
    case CovarianceEstimationType::FIXED_VALUE:
    default:
      result.covariance = Eigen::Matrix2d::Identity() * param.output_pose_covariance[0 + 6 * 0];
      break;
  }

  return result;
}

}  // namespace autoware::ndt_scan_matcher
