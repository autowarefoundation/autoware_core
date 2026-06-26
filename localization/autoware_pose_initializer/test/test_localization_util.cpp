// Copyright 2025 The Autoware Contributors
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

#include "../src/localization_util.hpp"

#include <gmock/gmock.h>

namespace autoware::pose_initializer
{
// The reference "now" used across the staleness tests (100.0 seconds).
constexpr double kNowSeconds = 100.0;
constexpr double kTimeout = 3.0;

TEST(IsPoseStale, FreshPoseIsNotStale)
{
  // A pose stamped 1 second in the past, with a 3 second timeout, is fresh.
  const rclcpp::Time now(static_cast<int64_t>(kNowSeconds * 1e9));
  const rclcpp::Time stamp(static_cast<int64_t>((kNowSeconds - 1.0) * 1e9));
  EXPECT_FALSE(is_pose_stale(stamp, now, kTimeout));
}

TEST(IsPoseStale, StalePoseIsRejected)
{
  // A pose stamped 5 seconds in the past, with a 3 second timeout, is stale.
  // This is the regression guard for the previously inverted sign: with the old
  // expression `stamp - now`, the elapsed time was negative and this branch was
  // never taken, so stale GNSS poses were never rejected.
  const rclcpp::Time now(static_cast<int64_t>(kNowSeconds * 1e9));
  const rclcpp::Time stamp(static_cast<int64_t>((kNowSeconds - 5.0) * 1e9));
  EXPECT_TRUE(is_pose_stale(stamp, now, kTimeout));
}

TEST(IsPoseStale, ExactlyAtTimeoutIsNotStale)
{
  // Elapsed time equal to the timeout is not considered stale (strict greater-than).
  const rclcpp::Time now(static_cast<int64_t>(kNowSeconds * 1e9));
  const rclcpp::Time stamp(static_cast<int64_t>((kNowSeconds - kTimeout) * 1e9));
  EXPECT_FALSE(is_pose_stale(stamp, now, kTimeout));
}

namespace
{
geometry_msgs::msg::Pose make_pose(const double x, const double y)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  return pose;
}
}  // namespace

TEST(CheckPoseError, SmallErrorReturnsTrue)
{
  const auto reference = make_pose(0.0, 0.0);
  const auto result = make_pose(0.5, 0.0);
  double error_2d = -1.0;
  EXPECT_TRUE(check_pose_error(reference, result, 1.0, error_2d));
  EXPECT_DOUBLE_EQ(error_2d, 0.5);
}

TEST(CheckPoseError, LargeErrorReturnsFalse)
{
  const auto reference = make_pose(0.0, 0.0);
  const auto result = make_pose(3.0, 4.0);
  double error_2d = -1.0;
  EXPECT_FALSE(check_pose_error(reference, result, 1.0, error_2d));
  EXPECT_DOUBLE_EQ(error_2d, 5.0);
}

TEST(CheckPoseError, CoincidentPosesHaveZeroError)
{
  const auto reference = make_pose(1.0, 2.0);
  const auto result = make_pose(1.0, 2.0);
  double error_2d = -1.0;
  EXPECT_TRUE(check_pose_error(reference, result, 1.0, error_2d));
  EXPECT_DOUBLE_EQ(error_2d, 0.0);
}

TEST(CheckPoseError, ErrorEqualToThresholdReturnsFalse)
{
  // The threshold comparison is `error < threshold`, so an error exactly equal to
  // the threshold is treated as a large error (returns false), matching the
  // original `pose_error_threshold_ <= error_2d` logic.
  const auto reference = make_pose(0.0, 0.0);
  const auto result = make_pose(1.0, 0.0);
  double error_2d = -1.0;
  EXPECT_FALSE(check_pose_error(reference, result, 1.0, error_2d));
  EXPECT_DOUBLE_EQ(error_2d, 1.0);
}
}  // namespace autoware::pose_initializer
