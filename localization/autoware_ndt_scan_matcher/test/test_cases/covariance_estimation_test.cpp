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

#include <autoware/ndt_scan_matcher/covariance_estimation.hpp>

#include <autoware/ndt_scan_matcher/ndt_omp/estimate_covariance.hpp>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

namespace autoware::ndt_scan_matcher
{

TEST(CovarianceEstimation, LaplaceApproximationMatchesPclomp)
{
  pclomp::NdtResult ndt_result;
  ndt_result.pose = Eigen::Matrix4f::Identity();
  ndt_result.hessian = Eigen::Matrix<double, 6, 6>::Identity();

  HyperParameters::Covariance param;
  param.covariance_estimation.covariance_estimation_type =
    CovarianceEstimationType::LAPLACE_APPROXIMATION;

  auto ndt_ptr = std::make_shared<NdtType>();
  const Eigen::Matrix4f initial_pose_matrix = Eigen::Matrix4f::Identity();
  const rclcpp::Time stamp{0, 0, RCL_ROS_TIME};

  const auto result = compute_covariance_estimate(
    ndt_result, initial_pose_matrix, param, ndt_ptr, stamp, "map");

  const auto expected = pclomp::estimate_xy_covariance_by_laplace_approximation(ndt_result.hessian);

  EXPECT_TRUE(result.covariance.isApprox(expected));
  ASSERT_EQ(1U, result.ndt_result_poses.poses.size());
  ASSERT_EQ(1U, result.ndt_initial_poses.poses.size());
}

TEST(CovarianceEstimation, FixedValueUsesConfiguredDiagonal)
{
  pclomp::NdtResult ndt_result;
  ndt_result.pose = Eigen::Matrix4f::Identity();

  HyperParameters::Covariance param;
  param.covariance_estimation.covariance_estimation_type = CovarianceEstimationType::FIXED_VALUE;
  param.output_pose_covariance.fill(0.0);
  constexpr double kDiag = 2.5;
  param.output_pose_covariance[0] = kDiag;

  auto ndt_ptr = std::make_shared<NdtType>();
  const Eigen::Matrix4f initial_pose_matrix = Eigen::Matrix4f::Identity();
  const rclcpp::Time stamp{0, 0, RCL_ROS_TIME};

  const auto result = compute_covariance_estimate(
    ndt_result, initial_pose_matrix, param, ndt_ptr, stamp, "map");

  const Eigen::Matrix2d expected = Eigen::Matrix2d::Identity() * kDiag;
  EXPECT_TRUE(result.covariance.isApprox(expected));
  ASSERT_EQ(1U, result.ndt_result_poses.poses.size());
  ASSERT_EQ(1U, result.ndt_initial_poses.poses.size());
}

}  // namespace autoware::ndt_scan_matcher
