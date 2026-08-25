// Copyright 2026 Tier IV, Inc.
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

#include "autoware/velocity_smoother/trajectory_utils.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <tuple>
#include <vector>

namespace
{
using autoware::velocity_smoother::trajectory_utils::TrajectoryPoints;
using autoware_planning_msgs::msg::TrajectoryPoint;

namespace utils = autoware::velocity_smoother::trajectory_utils;

// Build a straight trajectory along +x with the given spacing. Every point gets the same
// velocity / acceleration so callers can override individual points afterwards.
TrajectoryPoints makeStraightTrajectory(
  const size_t size, const double spacing = 1.0, const double velocity = 0.0,
  const double acceleration = 0.0)
{
  geometry_msgs::msg::Quaternion orientation;
  orientation.x = 0.0;
  orientation.y = 0.0;
  orientation.z = 0.0;
  orientation.w = 1.0;

  TrajectoryPoints trajectory;
  double x = 0.0;
  for (size_t i = 0; i < size; ++i) {
    TrajectoryPoint p;
    p.pose.position.x = x;
    p.pose.position.y = 0.0;
    p.pose.position.z = 0.0;
    p.pose.orientation = orientation;
    p.longitudinal_velocity_mps = static_cast<float>(velocity);
    p.acceleration_mps2 = static_cast<float>(acceleration);
    trajectory.push_back(p);
    x += spacing;
  }
  return trajectory;
}

// ---------------------------------------------------------------------------
// updateStateWithJerkConstraint
// ---------------------------------------------------------------------------

// A single-segment profile integrated exactly to its full duration must reproduce the
// closed-form constant-jerk kinematics x = j t^3 / 6, v = j t^2 / 2, a = j t.
TEST(TrajectoryUtils, UpdateStateSingleSegmentFullDuration)
{
  std::map<double, double> jerk_profile{{1.0, 2.0}};
  const auto state = utils::updateStateWithJerkConstraint(0.0, 0.0, jerk_profile, 2.0);
  ASSERT_TRUE(state.has_value());
  const auto [x, v, a, j] = *state;
  EXPECT_NEAR(x, 8.0 / 6.0, 1e-12);
  EXPECT_NEAR(v, 2.0, 1e-12);
  EXPECT_NEAR(a, 2.0, 1e-12);
  EXPECT_DOUBLE_EQ(j, 1.0);
}

// The profile is keyed by jerk value, so iteration follows ascending jerk regardless of
// insertion order. With t falling inside the second (higher-jerk) segment the state is the
// composition of the full first segment plus a partial second segment.
TEST(TrajectoryUtils, UpdateStateTwoSegmentsPartialSecond)
{
  std::map<double, double> jerk_profile{{5.0, 1.0}, {2.0, 1.0}};
  const auto state = utils::updateStateWithJerkConstraint(0.0, 0.0, jerk_profile, 1.5);
  ASSERT_TRUE(state.has_value());
  const auto [x, v, a, j] = *state;
  EXPECT_NEAR(x, 1.1875, 1e-12);
  EXPECT_NEAR(v, 2.625, 1e-12);
  EXPECT_NEAR(a, 4.5, 1e-12);
  EXPECT_DOUBLE_EQ(j, 5.0);
}

// Asking for a time beyond the sum of all segment durations exhausts the profile without ever
// hitting the t <= t_sum branch, so the function reports an invalid jerk profile (nullopt).
TEST(TrajectoryUtils, UpdateStateTimeBeyondProfileReturnsNullopt)
{
  std::map<double, double> jerk_profile{{1.0, 1.0}};
  const auto state = utils::updateStateWithJerkConstraint(0.0, 0.0, jerk_profile, 2.0);
  EXPECT_FALSE(state.has_value());
}

// ---------------------------------------------------------------------------
// isValidStopDist
// ---------------------------------------------------------------------------

TEST(TrajectoryUtils, IsValidStopDistInsideMargins)
{
  EXPECT_TRUE(utils::isValidStopDist(0.5, 0.0, 0.5, 0.0, 0.3, 0.1));
  // Exactly on the boundary is still valid (the check rejects strictly-outside values only).
  EXPECT_TRUE(utils::isValidStopDist(0.8, 0.1, 0.5, 0.0, 0.3, 0.1));
  EXPECT_TRUE(utils::isValidStopDist(0.2, -0.1, 0.5, 0.0, 0.3, 0.1));
}

TEST(TrajectoryUtils, IsValidStopDistVelocityOutOfRange)
{
  // v_end above v_target + |v_margin|.
  EXPECT_FALSE(utils::isValidStopDist(0.9, 0.0, 0.5, 0.0, 0.3, 0.1));
  // v_end below v_target - |v_margin|.
  EXPECT_FALSE(utils::isValidStopDist(0.1, 0.0, 0.5, 0.0, 0.3, 0.1));
}

TEST(TrajectoryUtils, IsValidStopDistAccelerationOutOfRange)
{
  // a_end outside [a_target - |a_margin|, a_target + |a_margin|].
  EXPECT_FALSE(utils::isValidStopDist(0.5, 0.2, 0.5, 0.0, 0.3, 0.1));
  EXPECT_FALSE(utils::isValidStopDist(0.5, -0.2, 0.5, 0.0, 0.3, 0.1));
}

TEST(TrajectoryUtils, IsValidStopDistUsesAbsoluteMargins)
{
  // Negative margins are taken in absolute value, so a negative margin behaves like a positive one.
  EXPECT_TRUE(utils::isValidStopDist(0.5, 0.0, 0.5, 0.0, -0.3, -0.1));
  EXPECT_FALSE(utils::isValidStopDist(0.9, 0.0, 0.5, 0.0, -0.3, -0.1));
}

// ---------------------------------------------------------------------------
// calcStopDistWithJerkConstraints (state machine over the three acceleration profiles)
// ---------------------------------------------------------------------------

// Large velocity error forces the deceleration to plateau at min_acc -> TRAPEZOID profile with
// three segments (jerk_dec ramp, constant min_acc hold, jerk_acc ramp back to zero).
TEST(TrajectoryUtils, CalcStopDistTrapezoid)
{
  std::map<double, double> jerk_profile;
  double stop_dist = 0.0;
  const bool ok = utils::calcStopDistWithJerkConstraints(
    5.0 /*v0*/, 0.0 /*a0*/, 1.0 /*jerk_acc*/, -1.0 /*jerk_dec*/, -1.0 /*min_acc*/,
    0.5 /*target_vel*/, jerk_profile, stop_dist);

  ASSERT_TRUE(ok);
  EXPECT_NEAR(stop_dist, 15.125, 1e-9);
  ASSERT_EQ(jerk_profile.size(), 3u);
  EXPECT_NEAR(jerk_profile.at(-1.0), 1.0, 1e-12);  // jerk_dec ramp
  EXPECT_NEAR(jerk_profile.at(0.0), 3.5, 1e-12);   // constant min_acc hold
  EXPECT_NEAR(jerk_profile.at(1.0), 1.0, 1e-12);   // jerk_acc ramp
}

// Moderate velocity error with deep min_acc never reaches the plateau -> TRIANGLE profile (two
// segments, symmetric here because |jerk_dec| == |jerk_acc|).
TEST(TrajectoryUtils, CalcStopDistTriangle)
{
  std::map<double, double> jerk_profile;
  double stop_dist = 0.0;
  const bool ok = utils::calcStopDistWithJerkConstraints(
    1.0 /*v0*/, 0.0 /*a0*/, 0.5 /*jerk_acc*/, -0.5 /*jerk_dec*/, -5.0 /*min_acc*/,
    0.5 /*target_vel*/, jerk_profile, stop_dist);

  ASSERT_TRUE(ok);
  EXPECT_NEAR(stop_dist, 1.5, 1e-9);
  ASSERT_EQ(jerk_profile.size(), 2u);
  EXPECT_NEAR(jerk_profile.at(-0.5), 1.0, 1e-9);
  EXPECT_NEAR(jerk_profile.at(0.5), 1.0, 1e-9);
}

// Already decelerating (a0 < 0) with a small velocity error -> LINEAR profile (single constant
// jerk segment driving acceleration back to zero).
TEST(TrajectoryUtils, CalcStopDistLinear)
{
  std::map<double, double> jerk_profile;
  double stop_dist = 0.0;
  const bool ok = utils::calcStopDistWithJerkConstraints(
    1.0 /*v0*/, -1.0 /*a0*/, 1.0 /*jerk_acc*/, -1.0 /*jerk_dec*/, -5.0 /*min_acc*/,
    0.5 /*target_vel*/, jerk_profile, stop_dist);

  ASSERT_TRUE(ok);
  EXPECT_NEAR(stop_dist, 2.0 / 3.0, 1e-9);
  ASSERT_EQ(jerk_profile.size(), 1u);
  EXPECT_NEAR(jerk_profile.at(1.0), 1.0, 1e-9);  // single segment, duration t1 = 1.0
}

// In the LINEAR branch a negative computed t1 is an unexpected condition and the function bails
// out with false (here: decelerating but the target velocity is above the current velocity).
TEST(TrajectoryUtils, CalcStopDistLinearNegativeTimeFails)
{
  std::map<double, double> jerk_profile;
  double stop_dist = 0.0;
  const bool ok = utils::calcStopDistWithJerkConstraints(
    1.0 /*v0*/, -1.0 /*a0*/, 1.0 /*jerk_acc*/, -1.0 /*jerk_dec*/, -5.0 /*min_acc*/,
    1.5 /*target_vel*/, jerk_profile, stop_dist);

  EXPECT_FALSE(ok);
}

// ---------------------------------------------------------------------------
// calcVelocityProfileWithConstantJerkAndAccelerationLimit (TrajectoryPoints overload)
// ---------------------------------------------------------------------------

TEST(TrajectoryUtils, VelocityProfileEmptyTrajectory)
{
  const TrajectoryPoints empty;
  const auto profile =
    utils::calcVelocityProfileWithConstantJerkAndAccelerationLimit(empty, 1.0, 0.0, 0.0, 1.0, -1.0);
  EXPECT_TRUE(profile.empty());
}

TEST(TrajectoryUtils, VelocityProfileConstantWithZeroJerkAndAccel)
{
  const auto trajectory = makeStraightTrajectory(3, 1.0);
  const auto profile = utils::calcVelocityProfileWithConstantJerkAndAccelerationLimit(
    trajectory, 1.0 /*v0*/, 0.0 /*a0*/, 0.0 /*jerk*/, 10.0 /*acc_max*/, -10.0 /*acc_min*/);

  ASSERT_EQ(profile.size(), 3u);
  for (const double v : profile) {
    EXPECT_NEAR(v, 1.0, 1e-12);
  }
}

TEST(TrajectoryUtils, VelocityProfileConstantPositiveAccel)
{
  // v0 = 1, a0 = 1, jerk = 0. With unit spacing and unit initial velocity the first time step is
  // dt = 1 / 1 = 1, so v[1] = v0 + a0 * dt = 2. The next step uses dt = 1 / 2 = 0.5, giving
  // v[2] = 2 + 1 * 0.5 = 2.5.
  const auto trajectory = makeStraightTrajectory(3, 1.0);
  const auto profile = utils::calcVelocityProfileWithConstantJerkAndAccelerationLimit(
    trajectory, 1.0 /*v0*/, 1.0 /*a0*/, 0.0 /*jerk*/, 10.0 /*acc_max*/, -10.0 /*acc_min*/);

  ASSERT_EQ(profile.size(), 3u);
  EXPECT_NEAR(profile.at(0), 1.0, 1e-12);
  EXPECT_NEAR(profile.at(1), 2.0, 1e-12);
  EXPECT_NEAR(profile.at(2), 2.5, 1e-12);
}

// ---------------------------------------------------------------------------
// extractPathAroundIndex
// ---------------------------------------------------------------------------

TEST(TrajectoryUtils, ExtractPathAroundIndexNominal)
{
  const auto trajectory = makeStraightTrajectory(10, 1.0);
  const auto extracted = utils::extractPathAroundIndex(trajectory, 5, 2.0, 2.0);

  // behind stops at index 2 (cumulative 3 m > 2 m), ahead stops at index 8 (cumulative 3 m > 2 m).
  ASSERT_EQ(extracted.size(), 7u);
  EXPECT_NEAR(extracted.front().pose.position.x, 2.0, 1e-12);
  EXPECT_NEAR(extracted.back().pose.position.x, 8.0, 1e-12);
}

TEST(TrajectoryUtils, ExtractPathAroundIndexEmptyAndOutOfRange)
{
  const TrajectoryPoints empty;
  EXPECT_TRUE(utils::extractPathAroundIndex(empty, 0, 1.0, 1.0).empty());

  const auto trajectory = makeStraightTrajectory(5, 1.0);
  // index beyond the last element -> empty result.
  EXPECT_TRUE(utils::extractPathAroundIndex(trajectory, 5, 1.0, 1.0).empty());
}

// ---------------------------------------------------------------------------
// calcArclengthArray / calcTrajectoryIntervalDistance
// ---------------------------------------------------------------------------

TEST(TrajectoryUtils, ArclengthArrayAndIntervals)
{
  const auto trajectory = makeStraightTrajectory(4, 2.0);

  const auto arclength = utils::calcArclengthArray(trajectory);
  ASSERT_EQ(arclength.size(), 4u);
  EXPECT_NEAR(arclength.at(0), 0.0, 1e-12);
  EXPECT_NEAR(arclength.at(1), 2.0, 1e-12);
  EXPECT_NEAR(arclength.at(2), 4.0, 1e-12);
  EXPECT_NEAR(arclength.at(3), 6.0, 1e-12);

  const auto intervals = utils::calcTrajectoryIntervalDistance(trajectory);
  ASSERT_EQ(intervals.size(), 3u);
  for (const double d : intervals) {
    EXPECT_NEAR(d, 2.0, 1e-12);
  }
}

TEST(TrajectoryUtils, ArclengthArrayEmpty)
{
  const TrajectoryPoints empty;
  EXPECT_TRUE(utils::calcArclengthArray(empty).empty());
}

// ---------------------------------------------------------------------------
// applyMaximumVelocityLimit (TrajectoryPoints overload)
// ---------------------------------------------------------------------------

TEST(TrajectoryUtils, ApplyMaximumVelocityLimitClampsRangeOnly)
{
  auto trajectory = makeStraightTrajectory(5, 1.0, 10.0 /*velocity*/);
  utils::applyMaximumVelocityLimit(1, 4, 5.0, trajectory);

  // Only indices [1, 4) are clamped to 5.0; the endpoints keep their original 10.0.
  EXPECT_NEAR(trajectory.at(0).longitudinal_velocity_mps, 10.0, 1e-5);
  EXPECT_NEAR(trajectory.at(1).longitudinal_velocity_mps, 5.0, 1e-5);
  EXPECT_NEAR(trajectory.at(2).longitudinal_velocity_mps, 5.0, 1e-5);
  EXPECT_NEAR(trajectory.at(3).longitudinal_velocity_mps, 5.0, 1e-5);
  EXPECT_NEAR(trajectory.at(4).longitudinal_velocity_mps, 10.0, 1e-5);
}

TEST(TrajectoryUtils, ApplyMaximumVelocityLimitLeavesSlowerPointsUntouched)
{
  auto trajectory = makeStraightTrajectory(3, 1.0, 2.0 /*velocity*/);
  utils::applyMaximumVelocityLimit(0, 3, 5.0, trajectory);
  for (const auto & p : trajectory) {
    EXPECT_NEAR(p.longitudinal_velocity_mps, 2.0, 1e-5);
  }
}

// ---------------------------------------------------------------------------
// calcStopDistance (TrajectoryPoints overload)
// ---------------------------------------------------------------------------

TEST(TrajectoryUtils, CalcStopDistanceFindsZeroVelocityPoint)
{
  auto trajectory = makeStraightTrajectory(6, 1.0, 3.0 /*velocity*/);
  // Insert a zero-velocity point at index 4; the closest index is 1, so the stop distance is the
  // straight-line distance between x = 4 and x = 1, i.e. 3 m.
  trajectory.at(4).longitudinal_velocity_mps = 0.0f;
  const double stop_dist = utils::calcStopDistance(trajectory, 1);
  EXPECT_NEAR(stop_dist, 3.0, 1e-9);
}

TEST(TrajectoryUtils, CalcStopDistanceNoZeroVelocityReturnsMax)
{
  const auto trajectory = makeStraightTrajectory(6, 1.0, 3.0 /*velocity*/);
  const double stop_dist = utils::calcStopDistance(trajectory, 0);
  EXPECT_DOUBLE_EQ(stop_dist, std::numeric_limits<double>::max());
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
