// Copyright 2024 Tier IV, Inc.
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

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/trajectory/trajectory_point.hpp"
#include "autoware/velocity_smoother/smoother/analytical_jerk_constrained_smoother/velocity_planning_utils.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <stdexcept>
#include <vector>

using autoware::velocity_smoother::analytical_velocity_planning_utils::
  calcStopDistWithJerkAndAccConstraints;
using autoware::velocity_smoother::analytical_velocity_planning_utils::
  calcStopVelocityWithConstantJerkAccLimit;
using autoware_planning_msgs::msg::TrajectoryPoint;
using Trajectory =
  autoware::experimental::trajectory::Trajectory<autoware_planning_msgs::msg::TrajectoryPoint>;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

TrajectoryPoints createStraightTrajectory(const size_t num_points, const double step_size = 1.0)
{
  TrajectoryPoints points;
  double x = 0.0;

  geometry_msgs::msg::Quaternion quat;
  quat.x = 0.0;
  quat.y = 0.0;
  quat.z = 0.0;
  quat.w = 1.0;

  for (size_t i = 0; i < num_points; ++i) {
    TrajectoryPoint pt;
    pt.pose.position.x = x;
    pt.pose.position.y = 0.0;
    pt.pose.position.z = 0.0;
    pt.pose.orientation = quat;
    pt.longitudinal_velocity_mps = 0.0;
    pt.acceleration_mps2 = 0.0;
    points.push_back(pt);
    x += step_size;
  }

  return points;
}

Trajectory createStraightTrajectoryContinuous(const size_t num_points, const double step_size = 1.0)
{
  const auto points = createStraightTrajectory(num_points, step_size);
  auto result = Trajectory::Builder().build(points);
  if (!result) {
    throw std::runtime_error("Failed to build trajectory");
  }
  return result.value();
}

void expectDiscreteAndContinuousMatch(
  TrajectoryPoints discrete_input, Trajectory & continuous, const double v0, const double a0,
  const double jerk_acc, const double jerk_dec, const double min_acc, const double decel_target_vel,
  const int type, const std::vector<double> & times, const std::optional<double> tolerance = 5e-1)
{
  TrajectoryPoints discrete_result = std::move(discrete_input);
  ASSERT_TRUE(calcStopVelocityWithConstantJerkAccLimit(
    v0, a0, jerk_acc, jerk_dec, min_acc, decel_target_vel, type, times, 0, discrete_result));
  ASSERT_TRUE(calcStopVelocityWithConstantJerkAccLimit(
    v0, a0, jerk_acc, jerk_dec, min_acc, decel_target_vel, type, times, 0.0, continuous));

  double cumulative_s = 0.0;
  for (size_t i = 0; i < discrete_result.size(); ++i) {
    if (i > 0) {
      cumulative_s += std::hypot(
        discrete_result[i].pose.position.x - discrete_result[i - 1].pose.position.x,
        discrete_result[i].pose.position.y - discrete_result[i - 1].pose.position.y);
    }

    const auto interp_point = continuous.compute(cumulative_s);
    if (tolerance) {
      EXPECT_NEAR(
        discrete_result[i].longitudinal_velocity_mps, interp_point.longitudinal_velocity_mps,
        *tolerance);
      EXPECT_NEAR(discrete_result[i].acceleration_mps2, interp_point.acceleration_mps2, *tolerance);
    }
  }
}

class StopVelocityCalculationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    v0 = 5.0;
    a0 = 0.0;
    jerk_acc = 0.5;
    jerk_dec = -0.8;
    min_acc = -1.5;
    decel_target_vel = 0.5;
  }

  double v0;
  double a0;
  double jerk_acc;
  double jerk_dec;
  double min_acc;
  double decel_target_vel;
};

TEST_F(StopVelocityCalculationTest, CalcStopDistWithJerkAndAccConstraints)
{
  int type = 0;
  std::vector<double> times;
  double stop_dist = 0.0;

  const bool result = calcStopDistWithJerkAndAccConstraints(
    v0, a0, jerk_acc, jerk_dec, min_acc, decel_target_vel, type, times, stop_dist);

  EXPECT_TRUE(result);
  EXPECT_GT(type, 0);
  EXPECT_GT(stop_dist, 0.0);
  EXPECT_GT(times.size(), 0);
}

TEST_F(StopVelocityCalculationTest, NewImplementationBasic)
{
  auto trajectory =
    createStraightTrajectoryContinuous(100, 0.5);  // 100 points, 0.5m spacing = 49.5m total
  double orig_length = trajectory.length();
  EXPECT_GT(orig_length, 0.0);

  int type = 0;
  std::vector<double> times;
  double stop_dist = 0.0;

  const bool calc_dist_result = calcStopDistWithJerkAndAccConstraints(
    v0, a0, jerk_acc, jerk_dec, min_acc, decel_target_vel, type, times, stop_dist);
  EXPECT_TRUE(calc_dist_result);

  auto discrete_points = createStraightTrajectory(100, 0.5);
  expectDiscreteAndContinuousMatch(
    discrete_points, trajectory, v0, a0, jerk_acc, jerk_dec, min_acc, decel_target_vel, type,
    times);

  const auto s_values = trajectory.base_arange(0.1);
  EXPECT_GT(s_values.size(), 0);

  for (double s : {0.0, orig_length * 0.5, orig_length * 0.99}) {
    if (s <= orig_length) {
      const auto pt = trajectory.compute(s);
      const auto vel = pt.longitudinal_velocity_mps;
      const auto acc = pt.acceleration_mps2;

      // Velocity should be non-negative
      EXPECT_GE(vel, -0.1);
      // Acceleration should be reasonable (not positive, around deceleration range)
      EXPECT_LE(acc, 0.1);
    }
  }
}

TEST_F(StopVelocityCalculationTest, DiscreteAndContinuousHelpersMatch)
{
  int type = 0;
  std::vector<double> times;
  double stop_dist = 0.0;

  ASSERT_TRUE(calcStopDistWithJerkAndAccConstraints(
    v0, a0, jerk_acc, jerk_dec, min_acc, decel_target_vel, type, times, stop_dist));

  constexpr double step_size = 0.5;
  auto discrete_points = createStraightTrajectory(60, step_size);
  auto continuous_trajectory = createStraightTrajectoryContinuous(60, step_size);
  expectDiscreteAndContinuousMatch(
    discrete_points, continuous_trajectory, v0, a0, jerk_acc, jerk_dec, min_acc, decel_target_vel,
    type, times);
}

TEST_F(StopVelocityCalculationTest, EmptyTrajectoryCase)
{
  TrajectoryPoints points;
  points.push_back(TrajectoryPoint());
  auto maybe_trajectory = Trajectory::Builder().build(points);
  // New trajectory cannot build empty trajectory
  ASSERT_FALSE(maybe_trajectory);
}

TEST_F(StopVelocityCalculationTest, VelocityMonotonicity)
{
  auto trajectory = createStraightTrajectoryContinuous(100, 0.5);
  const double orig_length = trajectory.length();

  int type = 0;
  std::vector<double> times;
  double stop_dist = 0.0;

  EXPECT_TRUE(calcStopDistWithJerkAndAccConstraints(
    v0, a0, jerk_acc, jerk_dec, min_acc, decel_target_vel, type, times, stop_dist));

  auto discrete_points = createStraightTrajectory(100, 0.5);
  expectDiscreteAndContinuousMatch(
    discrete_points, trajectory, v0, a0, jerk_acc, jerk_dec, min_acc, decel_target_vel, type,
    times);

  std::vector<double> velocities;
  for (double s = 0.0; s <= orig_length; s += 0.5) {
    velocities.push_back(trajectory.compute(s).longitudinal_velocity_mps);
  }

  for (size_t i = 1; i < velocities.size(); ++i) {
    EXPECT_LE(velocities[i], velocities[i - 1] + 0.01)
      << "Velocity increased at s=" << (i * 0.5) << ": " << velocities[i - 1] << " -> "
      << velocities[i];
  }
}

TEST_F(StopVelocityCalculationTest, FinalVelocityTarget)
{
  auto trajectory = createStraightTrajectoryContinuous(150, 0.5);
  const double orig_length = trajectory.length();

  int type = 0;
  std::vector<double> times;
  double stop_dist = 0.0;

  EXPECT_TRUE(calcStopDistWithJerkAndAccConstraints(
    v0, a0, jerk_acc, jerk_dec, min_acc, decel_target_vel, type, times, stop_dist));

  auto discrete_points = createStraightTrajectory(150, 0.5);
  expectDiscreteAndContinuousMatch(
    discrete_points, trajectory, v0, a0, jerk_acc, jerk_dec, min_acc, decel_target_vel, type,
    times);

  const auto vel_at_end = trajectory.compute(orig_length - 0.1).longitudinal_velocity_mps;
  EXPECT_NEAR(vel_at_end, decel_target_vel, 0.1);
}

TEST_F(StopVelocityCalculationTest, DifferentParameters)
{
  auto trajectory = createStraightTrajectoryContinuous(100, 0.5);

  const double test_jerk_acc = 0.3;
  const double test_jerk_dec = -0.5;
  const double test_min_acc = -2.0;

  int type = 0;
  std::vector<double> times;
  double stop_dist = 0.0;

  EXPECT_TRUE(calcStopDistWithJerkAndAccConstraints(
    v0, a0, test_jerk_acc, test_jerk_dec, test_min_acc, decel_target_vel, type, times, stop_dist));

  auto discrete_points = createStraightTrajectory(100, 0.5);
  expectDiscreteAndContinuousMatch(
    discrete_points, trajectory, v0, a0, test_jerk_acc, test_jerk_dec, test_min_acc,
    decel_target_vel, type, times);

  const auto vel_at_start = trajectory.compute(0.0).longitudinal_velocity_mps;
  const auto vel_at_end = trajectory.compute(trajectory.length() - 0.1).longitudinal_velocity_mps;

  EXPECT_GE(vel_at_start, decel_target_vel - 0.1);
  EXPECT_LE(vel_at_end, vel_at_start);
}

TEST_F(StopVelocityCalculationTest, ShortTrajectory)
{
  auto trajectory = createStraightTrajectoryContinuous(10, 0.1);
  EXPECT_LT(trajectory.length(), 1.0);

  int type = 0;
  std::vector<double> times;
  double stop_dist = 0.0;

  EXPECT_TRUE(calcStopDistWithJerkAndAccConstraints(
    v0, a0, jerk_acc, jerk_dec, min_acc, decel_target_vel, type, times, stop_dist));

  auto discrete_points = createStraightTrajectory(10, 0.1);
  expectDiscreteAndContinuousMatch(
    discrete_points, trajectory, v0, a0, jerk_acc, jerk_dec, min_acc, decel_target_vel, type,
    times);

  const auto vel_at_start = trajectory.compute(0.0).longitudinal_velocity_mps;
  EXPECT_GE(vel_at_start, 0.0);
}

TEST_F(StopVelocityCalculationTest, AccelerationBounds)
{
  auto trajectory = createStraightTrajectoryContinuous(100, 0.5);
  const double orig_length = trajectory.length();

  int type = 0;
  std::vector<double> times;
  double stop_dist = 0.0;

  EXPECT_TRUE(calcStopDistWithJerkAndAccConstraints(
    v0, a0, jerk_acc, jerk_dec, min_acc, decel_target_vel, type, times, stop_dist));

  auto discrete_points = createStraightTrajectory(100, 0.5);
  expectDiscreteAndContinuousMatch(
    discrete_points, trajectory, v0, a0, jerk_acc, jerk_dec, min_acc, decel_target_vel, type,
    times);

  for (double s = 0.0; s <= orig_length; s += 0.5) {
    const auto acc = trajectory.compute(s).acceleration_mps2;
    EXPECT_LE(acc, 0.2) << "Acceleration too positive at s=" << s;
    EXPECT_GE(acc, min_acc - 0.2) << "Acceleration too negative at s=" << s;
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
