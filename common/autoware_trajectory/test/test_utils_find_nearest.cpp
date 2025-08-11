// Copyright 2025 TIER IV, Inc.
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

#include "autoware/trajectory/detail/types.hpp"
#include "autoware/trajectory/pose.hpp"
#include "autoware/trajectory/threshold.hpp"
#include "autoware/trajectory/utils/find_nearest.hpp"

#include <gtest/gtest.h>

#include <fstream>
#include <limits>
#include <string>
#include <vector>

namespace
{

using autoware::experimental::trajectory::find_nearest_index;
using autoware::experimental::trajectory::k_points_minimum_dist_threshold;
using autoware::experimental::trajectory::Trajectory;
using autoware_utils_geometry::create_point;
using autoware_utils_geometry::create_quaternion_from_rpy;

static geometry_msgs::msg::Pose make_pose(double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::Pose p;
  p.position = create_point(x, y, 0.0);
  p.orientation = create_quaternion_from_rpy(0.0, 0.0, yaw);
  return p;
}

static Trajectory<geometry_msgs::msg::Pose> build_curved_trajectory(
  const size_t num_points, const double interval, const double delta_theta)
{
  std::vector<geometry_msgs::msg::Pose> raw_poses;
  raw_poses.reserve(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    double theta = i * delta_theta;
    double x = static_cast<double>(i) * interval * std::cos(theta);
    double y = static_cast<double>(i) * interval * std::sin(theta);
    geometry_msgs::msg::Pose p;
    p.position = create_point(x, y, 0.0);
    p.orientation = create_quaternion_from_rpy(0.0, 0.0, theta);
    raw_poses.push_back(p);
  }

  auto traj = Trajectory<geometry_msgs::msg::Pose>::Builder{}.build(raw_poses);
  return traj.value();
}

static Trajectory<geometry_msgs::msg::Pose> build_parabolic_trajectory(
  const size_t num_points, const double interval)
{
  std::vector<geometry_msgs::msg::Pose> raw_poses;
  raw_poses.reserve(num_points);

  double half = static_cast<double>(num_points - 1) / 2.0;
  for (size_t i = 0; i < num_points; ++i) {
    double x = (static_cast<double>(i) - half) * interval;
    double y = x * x;                       // Parabola: y = x^2
    double yaw = std::atan2(2.0 * x, 1.0);  // Tangent direction
    geometry_msgs::msg::Pose p;
    p.position = create_point(x, y, 0.0);
    p.orientation = create_quaternion_from_rpy(0.0, 0.0, yaw);
    raw_poses.push_back(p);
  }

  auto traj = Trajectory<geometry_msgs::msg::Pose>::Builder{}.build(raw_poses);
  return traj.value();
}

// Test 1: find_nearest_index on a curved trajectory (no thresholds)
TEST(trajectory, find_nearest_index_CurvedTrajectory)
{
  auto traj = build_curved_trajectory(10, 1.0, 0.1);

  {
    double true_theta = 5 * 0.1;
    double tx = 5.0 * std::cos(true_theta);
    double ty = 5.0 * std::sin(true_theta);
    double qx = tx + 0.2 * std::cos(true_theta + M_PI / 2.0);
    double qy = ty + 0.2 * std::sin(true_theta + M_PI / 2.0);

    auto query = make_pose(qx, qy, true_theta);
    auto s_opt = find_nearest_index(traj, query);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 5.2850123, k_points_minimum_dist_threshold);
  }

  {
    double theta3 = 3 * 0.1;
    double x3 = 3.0 * std::cos(theta3);
    double y3 = 3.0 * std::sin(theta3);
    double theta4 = 4 * 0.1;
    double x4 = 4.0 * std::cos(theta4);
    double y4 = 4.0 * std::sin(theta4);
    double mx = 0.5 * (x3 + x4);
    double my = 0.5 * (y3 + y4);
    double avg_theta = 0.5 * (theta3 + theta4);
    double qx = mx + 0.1 * std::cos(avg_theta + M_PI / 2.0);
    double qy = my + 0.1 * std::sin(avg_theta + M_PI / 2.0);

    auto query = make_pose(qx, qy, avg_theta);
    auto s_opt = find_nearest_index(traj, query);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 3.60253801, k_points_minimum_dist_threshold);
  }
}

// Test 2: Pose-based queries on curved trajectory with no threshold
TEST(trajectory, find_nearest_index_Pose_NoThreshold)
{
  auto traj = build_curved_trajectory(10, 1.0, 0.1);

  // Start point
  {
    auto query = make_pose(0.0, 0.0);
    auto s_opt = find_nearest_index(traj, query);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 4.4552394e-05, k_points_minimum_dist_threshold);
  }

  // End point
  {
    auto query = make_pose(9.0, 9.0);
    auto s_opt = find_nearest_index(traj, query);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 10.0846927, k_points_minimum_dist_threshold);
  }

  // Boundary just below 0.5
  {
    auto query = make_pose(0.5, 0.5);
    auto s_opt = find_nearest_index(traj, query);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 0.54431653, k_points_minimum_dist_threshold);
  }

  // Boundary just above 0.5
  {
    auto query = make_pose(0.51, 0.51);
    auto s_opt = find_nearest_index(traj, query);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 0.55598476, k_points_minimum_dist_threshold);
  }

  // Point before start
  {
    auto query = make_pose(-4.0, 5.0);
    auto s_opt = find_nearest_index(traj, query);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 4.4552394e-05, k_points_minimum_dist_threshold);
  }

  // Point after end
  {
    auto query = make_pose(100.0, -3.0);
    auto s_opt = find_nearest_index(traj, query);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 8.72879934, k_points_minimum_dist_threshold);
  }
}

// Test 3: Pose-based queries on curved trajectory with distance threshold
TEST(trajectory, find_nearest_index_Pose_DistThreshold)
{
  auto traj = build_curved_trajectory(10, 1.0, 0.1);

  // Out of threshold
  {
    auto query = make_pose(3.0, 0.6);
    auto s_opt = find_nearest_index(traj, query, 0.2);
    EXPECT_FALSE(s_opt.has_value());
  }

  // Within threshold
  {
    auto query = make_pose(3.0, 0.9);
    auto s_opt = find_nearest_index(traj, query, 2.0);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 3.1567543, k_points_minimum_dist_threshold);
  }
}

// Pose-based queries on curved trajectory with yaw threshold
TEST(trajectory, find_nearest_index_Pose_YawThreshold)
{
  auto traj = build_curved_trajectory(10, 1.0, 0.1);
  const double max_d = std::numeric_limits<double>::max();

  // Out of yaw threshold
  {
    auto query = make_pose(3.0, 0.0, 2);
    auto s_opt = find_nearest_index(traj, query, max_d, 0.2);
    EXPECT_FALSE(s_opt.has_value());
  }

  // Within yaw threshold
  {
    auto query = make_pose(3.0, 0.0, 0.9);
    auto s_opt = find_nearest_index(traj, query, max_d, 2.0);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 2.70806359, k_points_minimum_dist_threshold);
  }
}

// Test 4: Pose-based queries on curved trajectory with both distance & yaw thresholds
TEST(trajectory, find_nearest_index_Pose_DistAndYawThreshold)
{
  auto traj = build_curved_trajectory(10, 1.0, 0.1);

  // Within both thresholds
  {
    auto query = make_pose(3.0, 0.9, 1.2678071089);
    auto s_opt = find_nearest_index(traj, query, 2.0);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 3.1567543, k_points_minimum_dist_threshold);
  }

  // Out of distance and yaw threshold
  {
    auto query = make_pose(3.0, 0.6, 2);
    auto s_opt = find_nearest_index(traj, query, 0.6, 0.2);
    EXPECT_FALSE(s_opt.has_value());
  }
}

// Test 5: Pose-based queries on curved trajectory with both distance & yaw thresholds
TEST(trajectory, find_nearest_index_Pose_TwoMinimaDistAndYawThreshold)
{
  auto traj = build_curved_trajectory(10, 1.0, 0.1);

  // Within both thresholds
  {
    auto query = make_pose(3.0, 0.9, 1.2678071089);
    auto s_opt = find_nearest_index(traj, query, 2.0);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 3.1567543, k_points_minimum_dist_threshold);
  }

  // Out of distance and yaw threshold
  {
    auto query = make_pose(3.0, 0.6, 2);
    auto s_opt = find_nearest_index(traj, query, 0.6, 0.2);
    EXPECT_FALSE(s_opt.has_value());
  }
}

// Test 6: Two equal distance nearest points on a parabolic trajectory
TEST(trajectory, find_nearest_index_ParabolicTrajectory_AboveMinima)
{
  auto traj = build_parabolic_trajectory(11, 1.0);
  {
    auto query = make_pose(0.0, 4.0, 1.5707963267948966);
    auto s_opt =
      find_nearest_index(traj, query, std::numeric_limits<double>::max(), 1.2707963267948966);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 29.944142653, k_points_minimum_dist_threshold);
  }
}

}  // namespace
