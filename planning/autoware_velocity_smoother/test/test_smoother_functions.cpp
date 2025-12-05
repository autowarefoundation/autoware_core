// Copyright 2022 Tier IV, Inc.
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
#include "autoware/velocity_smoother/trajectory_utils.hpp"

#include <gtest/gtest.h>

#include <vector>

using autoware::velocity_smoother::trajectory_utils::TrajectoryPoints;
using autoware_planning_msgs::msg::TrajectoryPoint;
using Trajectory =
  autoware::experimental::trajectory::Trajectory<autoware_planning_msgs::msg::TrajectoryPoint>;

TrajectoryPoints genStraightTrajectory(const size_t size)
{
  double x = 0.0;
  double dx = 1.0;
  geometry_msgs::msg::Quaternion o;
  o.x = 0.0;
  o.y = 0.0;
  o.z = 0.0;
  o.w = 1.0;

  TrajectoryPoints tps;
  TrajectoryPoint p;
  for (size_t i = 0; i < size; ++i) {
    p.pose.position.x = x;
    p.pose.orientation = o;
    x += dx;
    tps.push_back(p);
  }
  return tps;
}

Trajectory genStraightTrajectoryContinuous(const size_t size)
{
  double x = 0.0;
  double dx = 1.0;
  geometry_msgs::msg::Quaternion o;
  o.x = 0.0;
  o.y = 0.0;
  o.z = 0.0;
  o.w = 1.0;

  TrajectoryPoints tps;
  TrajectoryPoint p;
  for (size_t i = 0; i < size; ++i) {
    p.pose.position.x = x;
    p.pose.orientation = o;
    p.longitudinal_velocity_mps = 0.0;
    p.acceleration_mps2 = 0.0;
    x += dx;
    tps.push_back(p);
  }

  auto result = Trajectory::Builder().build(tps);
  if (!result) {
    throw std::runtime_error("Failed to build continuous trajectory");
  }
  return result.value();
}

TEST(TestTrajectoryUtils, CalcTrajectoryCurvatureFrom3Points)
{
  // the output curvature vector should have the same size of the input trajectory.
  const auto checkOutputSize = [](const size_t trajectory_size, const size_t idx_dist) {
    const auto trajectory_points = genStraightTrajectory(trajectory_size);
    const auto curvatures =
      autoware::velocity_smoother::trajectory_utils::calcTrajectoryCurvatureFrom3Points(
        trajectory_points, idx_dist);
    EXPECT_EQ(curvatures.size(), trajectory_size) << ", idx_dist = " << idx_dist;
  };

  const auto trajectory_size_arr = {0, 1, 2, 3, 4, 5, 10, 100};
  const auto idx_dist_arr = {0, 1, 2, 3, 4, 5, 10, 100};
  for (const auto trajectory_size : trajectory_size_arr) {
    for (const auto idx_dist : idx_dist_arr) {
      checkOutputSize(trajectory_size, idx_dist);
    }
  }
}

TEST(TestTrajectoryUtils, CalcTrajectoryCurvatureFrom3PointsContinuous)
{
  // test with different trajectory sizes and interval distances
  const auto checkContinuousOutput =
    [](const size_t trajectory_size, const double interval_distance) {
      const auto trajectory = genStraightTrajectoryContinuous(trajectory_size);
      const auto curvatures =
        autoware::velocity_smoother::trajectory_utils::calcTrajectoryCurvatureFrom3PointsContinuous(
          trajectory, interval_distance);

      // for a straight trajectory, all curvatures should be close to zero
      for (const auto & curvature : curvatures) {
        EXPECT_NEAR(curvature, 0.0, 0.01) << "Straight trajectory should have near-zero curvature";
      }
    };

  const auto trajectory_size_arr = {10, 20, 50, 100};
  const auto interval_distance_arr = {0.5, 1.0, 2.0, 5.0};
  for (const auto trajectory_size : trajectory_size_arr) {
    for (const auto interval_distance : interval_distance_arr) {
      checkContinuousOutput(trajectory_size, interval_distance);
    }
  }
}
