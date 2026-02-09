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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cmath>

#include <gtest/gtest.h>

namespace autoware::ndt_scan_matcher
{
std::array<double, 36> rotate_position_covariance(
  const std::array<double, 36> & src_covariance, const Eigen::Matrix3d & rotation);
}  // namespace autoware::ndt_scan_matcher

namespace
{
void expect_arrays_close(const std::array<double, 36> & lhs, const std::array<double, 36> & rhs)
{
  constexpr double kEpsilon = 1e-12;
  for (size_t i = 0; i < lhs.size(); ++i) {
    EXPECT_NEAR(lhs[i], rhs[i], kEpsilon) << "Mismatch at index " << i;
  }
}
}  // namespace

TEST(RotatePositionCovarianceTest, KeepsCovarianceWithIdentityRotation)
{
  std::array<double, 36> src_covariance{};
  for (size_t i = 0; i < src_covariance.size(); ++i) {
    src_covariance[i] = static_cast<double>(i);
  }

  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();

  const auto rotated =
    autoware::ndt_scan_matcher::rotate_position_covariance(src_covariance, identity);

  expect_arrays_close(src_covariance, rotated);
}

TEST(RotatePositionCovarianceTest, MatchesLegacyImplementation)
{
  std::array<double, 36> src_covariance{};
  for (size_t i = 0; i < src_covariance.size(); ++i) {
    src_covariance[i] = static_cast<double>(i + 1);
  }

  const Eigen::Matrix3d rotation =
    Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  // Expected outcome: treat the 6x6 pose covariance as row-major [x y z roll pitch yaw];
  // apply the rotation to the position block (top-left 3x3) and leave orientation/cross terms
  // untouched.
  std::array<double, 36> expected = src_covariance;
  using Covariance6d = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;
  const Eigen::Map<const Covariance6d> src_cov_matrix(src_covariance.data());
  Eigen::Map<Covariance6d> expected_cov_matrix(expected.data());
  expected_cov_matrix.topLeftCorner<3, 3>() =
    rotation * src_cov_matrix.topLeftCorner<3, 3>() * rotation.transpose();

  const auto rotated =
    autoware::ndt_scan_matcher::rotate_position_covariance(src_covariance, rotation);

  expect_arrays_close(expected, rotated);
}
