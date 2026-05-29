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

#include <Eigen/Core>
#include <autoware/ndt_scan_matcher/ndt_omp/estimate_covariance.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

// calc_weight_vec is a temperature-scaled softmax. Equal scores yield a uniform distribution.
TEST(EstimateCovariance, CalcWeightVecUniformForEqualScores)  // NOLINT
{
  const std::vector<double> scores{1.5, 1.5, 1.5, 1.5};
  const std::vector<double> weights = pclomp::calc_weight_vec(scores, 1.0);
  ASSERT_EQ(weights.size(), scores.size());
  for (const double w : weights) {
    EXPECT_NEAR(w, 0.25, 1e-12);
  }
}

// With temperature 1.0 and scores {0, ln 3}, the softmax weights are {1/4, 3/4}.
TEST(EstimateCovariance, CalcWeightVecKnownSoftmax)  // NOLINT
{
  const std::vector<double> scores{0.0, std::log(3.0)};
  const std::vector<double> weights = pclomp::calc_weight_vec(scores, 1.0);
  ASSERT_EQ(weights.size(), 2u);
  EXPECT_NEAR(weights[0], 0.25, 1e-12);
  EXPECT_NEAR(weights[1], 0.75, 1e-12);
  EXPECT_NEAR(weights[0] + weights[1], 1.0, 1e-12);
}

// Weighted mean and covariance for two symmetric points with equal weights.
TEST(EstimateCovariance, CalculateWeightedMeanAndCovKnownValues)  // NOLINT
{
  const std::vector<Eigen::Vector2d> points{Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(2.0, 0.0)};
  const std::vector<double> weights{0.5, 0.5};
  const auto [mean, cov] = pclomp::calculate_weighted_mean_and_cov(points, weights);

  EXPECT_NEAR(mean.x(), 1.0, 1e-12);
  EXPECT_NEAR(mean.y(), 0.0, 1e-12);
  EXPECT_NEAR(cov(0, 0), 1.0, 1e-12);
  EXPECT_NEAR(cov(0, 1), 0.0, 1e-12);
  EXPECT_NEAR(cov(1, 0), 0.0, 1e-12);
  EXPECT_NEAR(cov(1, 1), 0.0, 1e-12);
}

// propose_poses_to_search rotates the (offset_x, offset_y) deltas by the pose orientation and
// adds them to the center translation.
TEST(EstimateCovariance, ProposePosesToSearchRotatesOffsets)  // NOLINT
{
  pclomp::NdtResult ndt_result{};
  ndt_result.pose = Eigen::Matrix4f::Identity();
  // 90 degrees CCW about z in the top-left 2x2 block.
  ndt_result.pose(0, 0) = 0.0f;
  ndt_result.pose(0, 1) = -1.0f;
  ndt_result.pose(1, 0) = 1.0f;
  ndt_result.pose(1, 1) = 0.0f;
  ndt_result.pose(0, 3) = 10.0f;  // center x
  ndt_result.pose(1, 3) = 20.0f;  // center y

  const std::vector<double> offset_x{1.0, 0.0};
  const std::vector<double> offset_y{0.0, 1.0};
  const auto poses = pclomp::propose_poses_to_search(ndt_result, offset_x, offset_y);

  ASSERT_EQ(poses.size(), 2u);
  // offset (1,0) rotated by +90deg -> (0,1): translation (10, 21).
  EXPECT_NEAR(poses[0](0, 3), 10.0, 1e-5);
  EXPECT_NEAR(poses[0](1, 3), 21.0, 1e-5);
  // offset (0,1) rotated by +90deg -> (-1,0): translation (9, 20).
  EXPECT_NEAR(poses[1](0, 3), 9.0, 1e-5);
  EXPECT_NEAR(poses[1](1, 3), 20.0, 1e-5);
}

// estimate_xy_covariance_by_laplace_approximation returns -inverse of the top-left 2x2 Hessian.
TEST(EstimateCovariance, LaplaceApproximationNegativeInverseHessian)  // NOLINT
{
  Eigen::Matrix<double, 6, 6> hessian = Eigen::Matrix<double, 6, 6>::Zero();
  hessian(0, 0) = -2.0;
  hessian(1, 1) = -4.0;
  // Fill the rest of the matrix to ensure only the 2x2 block matters.
  hessian(2, 2) = -100.0;
  hessian(3, 3) = 7.0;

  const Eigen::Matrix2d cov = pclomp::estimate_xy_covariance_by_laplace_approximation(hessian);
  EXPECT_NEAR(cov(0, 0), 0.5, 1e-12);
  EXPECT_NEAR(cov(1, 1), 0.25, 1e-12);
  EXPECT_NEAR(cov(0, 1), 0.0, 1e-12);
  EXPECT_NEAR(cov(1, 0), 0.0, 1e-12);
}

// rotate_covariance_to_map applies R * C * R^T using the pose rotation; verify against a directly
// computed reference, and confirm rotate_covariance_to_base_link is its inverse.
TEST(EstimateCovariance, RotateCovarianceMapAndBaseLinkAreInverses)  // NOLINT
{
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  const double theta = 0.3;  // arbitrary non-trivial yaw
  pose(0, 0) = static_cast<float>(std::cos(theta));
  pose(0, 1) = static_cast<float>(-std::sin(theta));
  pose(1, 0) = static_cast<float>(std::sin(theta));
  pose(1, 1) = static_cast<float>(std::cos(theta));

  Eigen::Matrix2d cov;
  cov << 4.0, 1.0, 1.0, 9.0;

  const Eigen::Matrix2d rot = pose.topLeftCorner<2, 2>().cast<double>();
  const Eigen::Matrix2d expected_map = rot * cov * rot.transpose();

  const Eigen::Matrix2d cov_map = pclomp::rotate_covariance_to_map(cov, pose);
  // The pose rotation is stored as float, so the achievable precision is limited to ~float eps.
  EXPECT_TRUE(cov_map.isApprox(expected_map, 1e-6));

  // Rotating back to base_link must recover the original covariance.
  const Eigen::Matrix2d round_trip = pclomp::rotate_covariance_to_base_link(cov_map, pose);
  EXPECT_TRUE(round_trip.isApprox(cov, 1e-6));
}

// adjust_diagonal_covariance clamps the base_link-frame diagonal to the provided floors. With an
// identity pose the base_link and map frames coincide, so the result is simply the clamped input.
TEST(EstimateCovariance, AdjustDiagonalCovarianceClampsFloor)  // NOLINT
{
  const Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  Eigen::Matrix2d cov;
  cov << 1.0, 0.0, 0.0, 1.0;

  const Eigen::Matrix2d adjusted = pclomp::adjust_diagonal_covariance(cov, pose, 4.0, 9.0);
  EXPECT_NEAR(adjusted(0, 0), 4.0, 1e-12);
  EXPECT_NEAR(adjusted(1, 1), 9.0, 1e-12);
  EXPECT_NEAR(adjusted(0, 1), 0.0, 1e-12);
  EXPECT_NEAR(adjusted(1, 0), 0.0, 1e-12);
}

// When the input diagonal already exceeds the floors, adjust_diagonal_covariance leaves it intact.
TEST(EstimateCovariance, AdjustDiagonalCovarianceKeepsLargerValues)  // NOLINT
{
  const Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  Eigen::Matrix2d cov;
  cov << 10.0, 0.0, 0.0, 20.0;

  const Eigen::Matrix2d adjusted = pclomp::adjust_diagonal_covariance(cov, pose, 4.0, 9.0);
  EXPECT_NEAR(adjusted(0, 0), 10.0, 1e-12);
  EXPECT_NEAR(adjusted(1, 1), 20.0, 1e-12);
}
