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

#include "../src/ndt_scan_matcher_helper.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

namespace
{
geometry_msgs::msg::Pose make_pose_xyz(const double x, const double y, const double z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;
  return pose;
}
}  // namespace

// A 90-degree rotation about z swaps the x and y variances of a diagonal position covariance
// block and leaves all other (non-position) entries untouched.
TEST(NdtScanMatcherHelper, RotateCovarianceSwapsDiagonalUnderQuarterTurn)  // NOLINT
{
  std::array<double, 36> src{};
  src.fill(0.0);
  src[0 + 6 * 0] = 4.0;   // var x
  src[1 + 6 * 1] = 9.0;   // var y
  src[2 + 6 * 2] = 16.0;  // var z
  // Put a sentinel into an angular entry to confirm it is preserved.
  src[3 + 6 * 3] = 42.0;

  // 90 degrees CCW about z: R = [[0,-1,0],[1,0,0],[0,0,1]].
  Eigen::Matrix3d rotation;
  rotation << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  const std::array<double, 36> out = autoware::ndt_scan_matcher::rotate_covariance(src, rotation);

  EXPECT_NEAR(out[0 + 6 * 0], 9.0, 1e-9);   // x variance becomes old y variance
  EXPECT_NEAR(out[1 + 6 * 1], 4.0, 1e-9);   // y variance becomes old x variance
  EXPECT_NEAR(out[2 + 6 * 2], 16.0, 1e-9);  // z variance unchanged
  EXPECT_NEAR(out[0 + 6 * 1], 0.0, 1e-9);   // off-diagonals stay zero
  EXPECT_NEAR(out[1 + 6 * 0], 0.0, 1e-9);
  EXPECT_NEAR(out[3 + 6 * 3], 42.0, 1e-9);  // angular block untouched
}

// The identity rotation must return the source covariance unchanged.
TEST(NdtScanMatcherHelper, RotateCovarianceIdentityIsNoOp)  // NOLINT
{
  std::array<double, 36> src{};
  for (std::size_t i = 0; i < src.size(); ++i) {
    src[i] = static_cast<double>(i) + 1.0;
  }

  const std::array<double, 36> out =
    autoware::ndt_scan_matcher::rotate_covariance(src, Eigen::Matrix3d::Identity());

  for (std::size_t i = 0; i < src.size(); ++i) {
    EXPECT_NEAR(out[i], src[i], 1e-9) << "index " << i;
  }
}

// A perfect back-and-forth zig-zag along x produces a consecutive-inversion count equal to the
// number of direction reversals.
TEST(NdtScanMatcherHelper, CountOscillationCountsConsecutiveInversions)  // NOLINT
{
  std::vector<geometry_msgs::msg::Pose> poses;
  for (const double x : {0.0, 1.0, 0.0, 1.0, 0.0}) {
    poses.push_back(make_pose_xyz(x, 0.0, 0.0));
  }
  // Reversals occur at i = 2, 3, 4 and are all consecutive -> max count 3.
  EXPECT_EQ(autoware::ndt_scan_matcher::count_oscillation(poses), 3);
}

// A monotonic straight-line trajectory contains no inversions.
TEST(NdtScanMatcherHelper, CountOscillationStraightLineIsZero)  // NOLINT
{
  std::vector<geometry_msgs::msg::Pose> poses;
  for (const double x : {0.0, 1.0, 2.0, 3.0, 4.0}) {
    poses.push_back(make_pose_xyz(x, 0.0, 0.0));
  }
  EXPECT_EQ(autoware::ndt_scan_matcher::count_oscillation(poses), 0);
}

// The maximum is over consecutive runs: a reversal that is later broken resets the running count.
TEST(NdtScanMatcherHelper, CountOscillationResetsAfterNonInversion)  // NOLINT
{
  // x: 0 -> 1 -> 0 (reversal) -> 1 (reversal) -> 2 (straight, resets) -> 1 (reversal)
  std::vector<geometry_msgs::msg::Pose> poses;
  for (const double x : {0.0, 1.0, 0.0, 1.0, 2.0, 1.0}) {
    poses.push_back(make_pose_xyz(x, 0.0, 0.0));
  }
  // i=2: reversal (cnt 1), i=3: reversal (cnt 2, max 2), i=4: straight (cnt 0),
  // i=5: reversal (cnt 1). Max consecutive run = 2.
  EXPECT_EQ(autoware::ndt_scan_matcher::count_oscillation(poses), 2);
}

// Fewer than three poses cannot form an inversion.
TEST(NdtScanMatcherHelper, CountOscillationTooFewPosesIsZero)  // NOLINT
{
  std::vector<geometry_msgs::msg::Pose> poses;
  poses.push_back(make_pose_xyz(0.0, 0.0, 0.0));
  poses.push_back(make_pose_xyz(1.0, 0.0, 0.0));
  EXPECT_EQ(autoware::ndt_scan_matcher::count_oscillation(poses), 0);
}

// Repeated (identical) poses produce zero-length motion vectors. Normalizing those would yield
// NaNs and an unreliable cosine; the guard must treat such steps as non-oscillations and return a
// finite count of 0 rather than reacting to NaN comparisons.
TEST(NdtScanMatcherHelper, CountOscillationZeroLengthStepsAreNotOscillations)  // NOLINT
{
  std::vector<geometry_msgs::msg::Pose> poses;
  // All identical -> every step has zero length.
  for (int i = 0; i < 5; ++i) {
    poses.push_back(make_pose_xyz(1.0, 2.0, 3.0));
  }
  EXPECT_EQ(autoware::ndt_scan_matcher::count_oscillation(poses), 0);
}

// A zero-length step interleaved with real motion must not be counted as an oscillation and must
// reset the running consecutive-inversion count.
TEST(NdtScanMatcherHelper, CountOscillationResetsOnZeroLengthStep)  // NOLINT
{
  // x: 0 -> 1 -> 0 (reversal, cnt 1) -> 0 (zero-length step, reset) -> 1 (no prior direction yet)
  std::vector<geometry_msgs::msg::Pose> poses;
  for (const double x : {0.0, 1.0, 0.0, 0.0, 1.0}) {
    poses.push_back(make_pose_xyz(x, 0.0, 0.0));
  }
  EXPECT_EQ(autoware::ndt_scan_matcher::count_oscillation(poses), 1);
}
