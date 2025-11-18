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

#include "autoware/motion_utils/trajectory/trajectory.hpp"
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

using autoware::experimental::trajectory::find_first_nearest_index;
using autoware::experimental::trajectory::find_nearest_index;
using autoware::experimental::trajectory::k_points_minimum_dist_threshold;
using autoware::experimental::trajectory::Trajectory;
using autoware_utils_geometry::calc_squared_distance2d;
using autoware_utils_geometry::create_point;
using autoware_utils_geometry::create_quaternion_from_rpy;
using autoware_utils_geometry::get_rpy;

// ============================== Helper Function ======================================== //

static geometry_msgs::msg::Pose make_pose(double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::Pose p;
  p.position = create_point(x, y, 0.0);
  p.orientation = create_quaternion_from_rpy(0.0, 0.0, yaw);
  return p;
}

static bool is_within_distance_constraint(
  Trajectory<geometry_msgs::msg::Pose> traj, double s, geometry_msgs::msg::Pose query,
  double criteria)
{
  auto target_pose = traj.compute(s);
  auto target_pos = target_pose.position;
  auto query_pos = query.position;

  if (
    autoware_utils_geometry::calc_squared_distance2d(query_pos, target_pos) <=
    criteria * criteria) {
    return true;
  } else {
    return false;
  }
}

static bool is_within_yaw_constraint(
  Trajectory<geometry_msgs::msg::Pose> traj, double s, geometry_msgs::msg::Pose query,
  double criteria)
{
  auto target_pose = traj.compute(s);
  auto target_yaw = get_rpy(target_pose.orientation).z;
  auto query_yaw = get_rpy(query.orientation).z;

  if (std::fabs(autoware_utils_math::normalize_radian(query_yaw - target_yaw)) <= criteria) {
    return true;
  } else {
    return false;
  }
}

static bool is_within_yaw_and_distance_constraint(
  Trajectory<geometry_msgs::msg::Pose> traj, double s, geometry_msgs::msg::Pose query,
  double distance_criteria = std::numeric_limits<double>::max(),
  double yaw_criteria = std::numeric_limits<double>::max())
{
  if (
    is_within_yaw_constraint(traj, s, query, yaw_criteria) &&
    is_within_distance_constraint(traj, s, query, distance_criteria)) {
    return true;
  } else {
    return false;
  }
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

static Trajectory<geometry_msgs::msg::Pose> build_bow_trajectory(
  const size_t num_points, const double size, const double total_angle)
{
  std::vector<geometry_msgs::msg::Pose> raw_poses;
  raw_poses.reserve(num_points);

  auto delta_theta = total_angle / num_points;

  for (size_t i = 0; i < num_points; ++i) {
    double theta = M_PI / 4 + i * delta_theta;

    double x = size * cos(theta);
    double y = size * sin(theta) * cos(theta);
    double yaw = std::atan2(cos(2 * theta), -1 * sin(theta));
    raw_poses.push_back(make_pose(x, y, yaw));
  }
  auto traj = Trajectory<geometry_msgs::msg::Pose>::Builder{}.build(raw_poses);
  return traj.value();
}

static double calculate_bow_trajectory_yaw(const double theta)
{
  return std::atan2(cos(2 * theta), -1 * sin(theta));
}

static double calculate_bow_trajectory_yaw_from_x(const double x, double size = 3)
{
  auto theta = std::acos(x / size);
  return calculate_bow_trajectory_yaw(theta);
}

static Trajectory<geometry_msgs::msg::Pose> build_vertical_loop_trajectory(
  const size_t num_points, const double radius, const double start_x, const double offset)
{
  std::vector<geometry_msgs::msg::Pose> raw_poses;
  raw_poses.reserve(num_points);

  size_t first_part = static_cast<size_t>(num_points / 3);
  size_t last_part = static_cast<size_t>(num_points / 3);
  size_t second_part = static_cast<size_t>(num_points - first_part - last_part);

  // First part: going in (from right to left)
  double rate = (start_x) / (first_part);
  for (size_t i = 0u; i < first_part; ++i) {
    double x = start_x - rate * i;
    double y = offset;
    raw_poses.push_back(make_pose(x, y, M_PI));
  }

  // Second part: go in the loop
  double start_theta = M_PI * 3 / 2;
  double loop_rate = 2 * M_PI / (second_part - 1);
  for (size_t i = 0u; i < second_part; ++i) {
    // from 3/2 pi to -1/2pi
    double theta = start_theta - i * loop_rate;
    double x = radius * cos(theta);
    double y = (offset + radius) + radius * sin(theta);
    raw_poses.push_back(make_pose(x, y, theta - M_PI / 2));
  }

  // Last part: go out to the left
  double out_rate = (start_x) / (first_part);
  for (size_t i = 1u; i <= last_part; ++i) {
    double x = 0 - out_rate * i;
    double y = offset;
    raw_poses.push_back(make_pose(x, y, M_PI));
  }

  auto traj = Trajectory<geometry_msgs::msg::Pose>::Builder{}.build(raw_poses);
  return traj.value();
}

static Trajectory<geometry_msgs::msg::Pose> build_lollipop_trajectory(
  const size_t num_points, const double radius, const double phase_dif = M_PI / 6,
  const double start_x = 3, const double offset = 0)
{
  std::vector<geometry_msgs::msg::Pose> raw_poses;
  raw_poses.reserve(num_points);

  size_t first_part = static_cast<size_t>(num_points / 3);
  size_t last_part = static_cast<size_t>(num_points / 3);
  size_t second_part = static_cast<size_t>(num_points - first_part - last_part);

  // First part: go into bottom
  double rate = (start_x) / (first_part);
  for (size_t i = 0u; i < first_part; ++i) {
    double x = start_x - rate * i;
    double y = offset - radius * sin(phase_dif / 2);
    raw_poses.push_back(make_pose(x, y, M_PI));
  }

  // Second part: go in the loop
  double start_theta = 2 * M_PI - phase_dif / 2;
  double end_theta = phase_dif / 2;
  double loop_rate = (end_theta - start_theta) / (second_part - 1);
  for (size_t i = 0u; i < second_part; ++i) {
    // from -phase_dif/2 to +phase_dif/2
    double theta = start_theta + i * loop_rate;
    double x = -radius * cos(phase_dif / 2) + radius * cos(theta);
    double y = offset + radius * sin(theta);
    raw_poses.push_back(make_pose(x, y, theta - M_PI / 2));
  }

  // Last part: go out from top
  double out_rate = (start_x) / (last_part);
  for (size_t i = 1u; i <= last_part; ++i) {
    double x = 0 + out_rate * i;
    double y = offset + radius * sin(phase_dif / 2);
    raw_poses.push_back(make_pose(x, y, 0));
  }

  auto traj = Trajectory<geometry_msgs::msg::Pose>::Builder{}.build(raw_poses);
  return traj.value();
}
// ======================================== TEST ============================================//

// Test 1: Point-based queries find_nearest_index on a curved trajectory (no threshold)
TEST(trajectory, find_nearest_index_Point_CurvedTrajectory)
{
  auto traj = build_curved_trajectory(10, 1.0, 0.1);
  {
    double true_theta = 5 * 0.1;
    double tx = 5.0 * std::cos(true_theta);
    double ty = 5.0 * std::sin(true_theta);
    double qx = tx + 0.2 * std::cos(true_theta + M_PI / 2.0);
    double qy = ty + 0.2 * std::sin(true_theta + M_PI / 2.0);

    auto query = create_point(qx, qy, 0);
    auto s_opt = find_nearest_index(traj, query);
    EXPECT_NEAR(s_opt, 5.2850123, k_points_minimum_dist_threshold);
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

    auto query = create_point(qx, qy, 0);
    auto s_opt = find_nearest_index(traj, query);
    EXPECT_NEAR(s_opt, 3.60253801, k_points_minimum_dist_threshold);
  }
}

// Test 2: find_first_nearest_index on a curved trajectory (no thresholds)
TEST(trajectory, find_first_nearest_index_CurvedTrajectory)
{
  auto traj = build_curved_trajectory(10, 1.0, 0.1);

  {
    double true_theta = 5 * 0.1;
    double tx = 5.0 * std::cos(true_theta);
    double ty = 5.0 * std::sin(true_theta);
    double qx = tx + 0.2 * std::cos(true_theta + M_PI / 2.0);
    double qy = ty + 0.2 * std::sin(true_theta + M_PI / 2.0);

    auto query = make_pose(qx, qy, true_theta);
    auto s_opt = find_first_nearest_index(traj, query);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 5.2850123, k_points_minimum_dist_threshold);
    EXPECT_TRUE(is_within_yaw_and_distance_constraint(traj, *s_opt, query));
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
    auto s_opt = find_first_nearest_index(traj, query);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 3.60253801, k_points_minimum_dist_threshold);
    EXPECT_TRUE(is_within_yaw_and_distance_constraint(traj, *s_opt, query));
  }
}

// Test 3: Pose-based queries on curved trajectory with no threshold
TEST(trajectory, find_first_nearest_index_Pose_NoThreshold)
{
  auto traj = build_curved_trajectory(10, 1.0, 0.1);

  // Start point
  {
    auto query = make_pose(0.0, 0.0);
    auto s_opt = find_first_nearest_index(traj, query);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 4.4552394e-05, k_points_minimum_dist_threshold);
    EXPECT_TRUE(is_within_yaw_and_distance_constraint(traj, *s_opt, query));
  }

  // End point
  {
    auto query = make_pose(9.0, 9.0);
    auto s_opt = find_first_nearest_index(traj, query);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 10.0846927, k_points_minimum_dist_threshold);
    EXPECT_TRUE(is_within_yaw_and_distance_constraint(traj, *s_opt, query));
  }

  // Boundary just below 0.5
  {
    auto query = make_pose(0.5, 0.5);
    auto s_opt = find_first_nearest_index(traj, query);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 0.54431653, k_points_minimum_dist_threshold);
    EXPECT_TRUE(is_within_yaw_and_distance_constraint(traj, *s_opt, query));
  }

  // Boundary just above 0.5
  {
    auto query = make_pose(0.51, 0.51);
    auto s_opt = find_first_nearest_index(traj, query);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 0.55598476, k_points_minimum_dist_threshold);
    EXPECT_TRUE(is_within_yaw_and_distance_constraint(traj, *s_opt, query));
  }

  // Point before start
  {
    auto query = make_pose(-4.0, 5.0);
    auto s_opt = find_first_nearest_index(traj, query);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 4.4552394e-05, k_points_minimum_dist_threshold);
    EXPECT_TRUE(is_within_yaw_and_distance_constraint(traj, *s_opt, query));
  }

  // Point after end
  {
    auto query = make_pose(100.0, -3.0);
    auto s_opt = find_first_nearest_index(traj, query);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 8.72879934, k_points_minimum_dist_threshold);
    EXPECT_TRUE(is_within_yaw_and_distance_constraint(traj, *s_opt, query));
  }
}

// Test 4: Pose-based queries on curved trajectory with distance threshold
TEST(trajectory, find_first_nearest_index_Pose_DistThreshold)
{
  auto traj = build_curved_trajectory(10, 1.0, 0.1);

  // Out of threshold
  {
    auto query = make_pose(3.0, 0.6);
    auto s_opt = find_first_nearest_index(traj, query, 0.2);
    EXPECT_FALSE(s_opt.has_value());
  }

  // Within threshold
  {
    auto query = make_pose(3.0, 0.9);
    auto s_opt = find_first_nearest_index(traj, query, 2.0);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 3.1567543, k_points_minimum_dist_threshold);
    EXPECT_TRUE(is_within_yaw_and_distance_constraint(traj, *s_opt, query, 2.0));
  }
}

// Pose-based queries on curved trajectory with yaw threshold
TEST(trajectory, find_first_nearest_index_Pose_YawThreshold)
{
  auto traj = build_curved_trajectory(10, 1.0, 0.1);
  const double max_d = std::numeric_limits<double>::max();

  // Out of yaw threshold
  {
    auto query = make_pose(3.0, 0.0, 2);
    auto s_opt = find_first_nearest_index(traj, query, max_d, 0.2);
    EXPECT_FALSE(s_opt.has_value());
  }

  // Within yaw threshold
  {
    auto query = make_pose(3.0, 0.0, 0.9);
    auto s_opt = find_first_nearest_index(traj, query, max_d, 2.0);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 2.70806359, k_points_minimum_dist_threshold);
    EXPECT_TRUE(is_within_yaw_and_distance_constraint(traj, *s_opt, query, 2.0));
  }
}

// Test 5: Pose-based queries on curved trajectory with both distance & yaw thresholds
TEST(trajectory, find_first_nearest_index_Pose_DistAndYawThreshold)
{
  auto traj = build_curved_trajectory(10, 1.0, 0.1);

  // Within both thresholds
  {
    auto query = make_pose(3.0, 0.9, 1.2678071089);
    auto s_opt = find_first_nearest_index(traj, query, 2.0);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 3.1567543, k_points_minimum_dist_threshold);
    EXPECT_TRUE(is_within_yaw_and_distance_constraint(traj, *s_opt, query, 2.0));
  }

  // Out of distance and yaw threshold
  {
    auto query = make_pose(3.0, 0.6, 2);
    auto s_opt = find_first_nearest_index(traj, query, 0.6, 0.2);
    EXPECT_FALSE(s_opt.has_value());
  }
}

// Test 6: Pose-based queries on curved trajectory with both distance & yaw thresholds
TEST(trajectory, find_first_nearest_index_Pose_TwoMinimaDistAndYawThreshold)
{
  auto traj = build_curved_trajectory(10, 1.0, 0.1);

  // Within both thresholds
  {
    auto query = make_pose(3.0, 0.9, 1.2678071089);
    auto s_opt = find_first_nearest_index(traj, query, 2.0);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 3.1567543, k_points_minimum_dist_threshold);
    EXPECT_TRUE(is_within_yaw_and_distance_constraint(traj, *s_opt, query, 2.0));
  }

  // Out of distance and yaw threshold
  {
    auto query = make_pose(3.0, 0.6, 2);
    auto s_opt = find_first_nearest_index(traj, query, 0.6, 0.2);
    EXPECT_FALSE(s_opt.has_value());
  }
}

// Test 7: Two equal distance nearest points on a parabolic trajectory
TEST(trajectory, find_first_nearest_index_ParabolicTrajectory_AboveMinima)
{
  auto traj = build_parabolic_trajectory(11, 1.0);
  {
    auto query = make_pose(0.0, 4.0, 1.5707963267948966);
    auto s_opt =
      find_first_nearest_index(traj, query, std::numeric_limits<double>::max(), 1.2707963267948966);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 29.944142653, k_points_minimum_dist_threshold);
    EXPECT_TRUE(is_within_yaw_and_distance_constraint(
      traj, *s_opt, query, std::numeric_limits<double>::max(), 1.2707963267948966));
  }
}

// Test 8: Two equal distance nearest points at the self-intersecting trajectory (less bases)
TEST(trajectory, find_first_nearest_index_BowTrajectoryLessBases)
{
  // ALL FAILED WITH SMALL INACCURATE
  auto traj = build_bow_trajectory(10, 3, 1.5 * M_PI);
  {
    auto yaw = calculate_bow_trajectory_yaw_from_x(1);
    auto query = make_pose(1, 1, yaw);
    auto s_opt = find_first_nearest_index(traj, query);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 1.2181577509092187, k_points_minimum_dist_threshold);
    EXPECT_TRUE(is_within_yaw_and_distance_constraint(traj, *s_opt, query));
  }
  // At intersection, first round
  {
    auto yaw = calculate_bow_trajectory_yaw(M_PI / 2);
    auto query = make_pose(-0.05, 0, yaw);
    auto s_opt = find_first_nearest_index(traj, query, std::numeric_limits<double>::max(), 1.046);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 2.6770152278921948, k_points_minimum_dist_threshold);
    EXPECT_TRUE(is_within_yaw_and_distance_constraint(
      traj, *s_opt, query, std::numeric_limits<double>::max(), 1.046));
  }

  // At intersection, second round
  {
    auto yaw = calculate_bow_trajectory_yaw(M_PI * 1.5);
    auto query = make_pose(-0.05, 0, yaw);
    auto s_opt = find_first_nearest_index(traj, query, std::numeric_limits<double>::max(), 1.046);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 11.501379660920705, k_points_minimum_dist_threshold);
    EXPECT_TRUE(is_within_yaw_and_distance_constraint(
      traj, *s_opt, query, std::numeric_limits<double>::max(), 1.046));
  }
}

// Test 9: Two equal distance nearest points at the self-intersecting trajectory (many bases)
TEST(trajectory, find_first_nearest_index_BowTrajectoryManyBases)
{
  auto traj = build_bow_trajectory(1000, 3, 1.5 * M_PI);
  {
    auto yaw = calculate_bow_trajectory_yaw_from_x(1);
    auto query = make_pose(1, 1, yaw);
    auto s_opt = find_first_nearest_index(traj, query);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 1.2381368903716181, k_points_minimum_dist_threshold);
    EXPECT_TRUE(is_within_yaw_and_distance_constraint(traj, *s_opt, query));
  }
  // At intersection, first round
  {
    auto yaw = calculate_bow_trajectory_yaw(M_PI / 2);
    auto query = make_pose(-0.05, 0, yaw);
    auto s_opt = find_first_nearest_index(traj, query, std::numeric_limits<double>::max(), 1.046);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 2.6851636870536018, k_points_minimum_dist_threshold);
    EXPECT_TRUE(is_within_yaw_and_distance_constraint(
      traj, *s_opt, query, std::numeric_limits<double>::max(), 1.046));
  }

  // At intersection, second round
  {
    auto yaw = calculate_bow_trajectory_yaw(M_PI * 1.5);
    auto query = make_pose(-0.05, 0, yaw);
    auto s_opt = find_first_nearest_index(traj, query, std::numeric_limits<double>::max(), 1.046);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 11.760225108845566, k_points_minimum_dist_threshold);
    EXPECT_TRUE(is_within_yaw_and_distance_constraint(
      traj, *s_opt, query, std::numeric_limits<double>::max(), 1.046));
  }
}

// Test 10: find_first_nearest_index on vertical loop trajectory
TEST(trajectory, find_first_nearest_index_VerticalLoop)
{
  {
    auto traj = build_vertical_loop_trajectory(10, 3, 3, 0);
    {
      auto query = make_pose(1, 1, -M_PI);
      auto s_opt = find_first_nearest_index(traj, query, 2.0, 1.046);
      ASSERT_TRUE(s_opt.has_value());
      EXPECT_NEAR(*s_opt, 1.96171, k_points_minimum_dist_threshold);
      EXPECT_TRUE(is_within_yaw_and_distance_constraint(traj, *s_opt, query, 2, 1.046));
    }
    {
      auto query = make_pose(0, 0.05, -M_PI);
      auto s_opt = find_first_nearest_index(traj, query, 2.0, 1.046);
      ASSERT_TRUE(s_opt.has_value());
      EXPECT_NEAR(*s_opt, 3.00769, k_points_minimum_dist_threshold);
      EXPECT_TRUE(is_within_yaw_and_distance_constraint(traj, *s_opt, query, 2, 1.046));
    }
    {
      auto query = make_pose(0, -0.05, -M_PI);
      auto s_opt = find_first_nearest_index(traj, query, 2.0, 1.046);
      ASSERT_TRUE(s_opt.has_value());
      EXPECT_NEAR(*s_opt, 2.99234, k_points_minimum_dist_threshold);
      EXPECT_TRUE(is_within_yaw_and_distance_constraint(traj, *s_opt, query, 2, 1.046));
    }
  }
  {
    auto traj = build_vertical_loop_trajectory(100, 3, 3, 0);
    {
      auto query = make_pose(1, 1, -M_PI);
      auto s_opt = find_first_nearest_index(traj, query, 2.0, 1.046);
      ASSERT_TRUE(s_opt.has_value());
      EXPECT_NEAR(*s_opt, 1.99964, k_points_minimum_dist_threshold);
      EXPECT_TRUE(is_within_yaw_and_distance_constraint(traj, *s_opt, query, 2, 1.046));
    }
    {
      auto query = make_pose(0, 0.05, -M_PI);
      auto s_opt = find_first_nearest_index(traj, query, 2.0, 1.046);
      ASSERT_TRUE(s_opt.has_value());
      EXPECT_NEAR(*s_opt, 3.00024, k_points_minimum_dist_threshold);
      EXPECT_TRUE(is_within_yaw_and_distance_constraint(traj, *s_opt, query, 2, 1.046));
    }
    {
      auto query = make_pose(0, -0.05, -M_PI);
      auto s_opt = find_first_nearest_index(traj, query, 2.0, 1.046);
      ASSERT_TRUE(s_opt.has_value());
      EXPECT_NEAR(*s_opt, 2.99981, k_points_minimum_dist_threshold);
      EXPECT_TRUE(is_within_yaw_and_distance_constraint(traj, *s_opt, query, 2, 1.046));
    }
  }
  {
    auto traj = build_vertical_loop_trajectory(1000, 3, 3, 0);
    {
      auto query = make_pose(1, 1, -M_PI);
      auto s_opt = find_first_nearest_index(traj, query, 2.0, 1.046);
      ASSERT_TRUE(s_opt.has_value());
      EXPECT_NEAR(*s_opt, 1.99967, k_points_minimum_dist_threshold);
      EXPECT_TRUE(is_within_yaw_and_distance_constraint(traj, *s_opt, query, 2, 1.046));
    }
    {
      auto query = make_pose(0, 0.05, -M_PI);
      auto s_opt = find_first_nearest_index(traj, query, 2.0, 1.046);
      ASSERT_TRUE(s_opt.has_value());
      EXPECT_NEAR(*s_opt, 3.00011, k_points_minimum_dist_threshold);
      EXPECT_TRUE(is_within_yaw_and_distance_constraint(traj, *s_opt, query, 2, 1.046));
    }
    {
      auto query = make_pose(0, -0.05, -M_PI);
      auto s_opt = find_first_nearest_index(traj, query, 2.0, 1.046);
      ASSERT_TRUE(s_opt.has_value());
      EXPECT_NEAR(*s_opt, 3.00011, k_points_minimum_dist_threshold);
      EXPECT_TRUE(is_within_yaw_and_distance_constraint(traj, *s_opt, query, 2, 1.046));
    }
  }
}

// TEST 11: Test Lollipop trajectory find_first_nearest_index
TEST(trajectory, find_first_nearest_index_LollipopTrajectory)
{
  auto traj = build_lollipop_trajectory(1000, 3);
  {
    auto query = make_pose(1.5, 0, M_PI);
    auto s_opt =
      find_first_nearest_index(traj, query, std::numeric_limits<double>::max(), M_PI / 3);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 1.50006, k_points_minimum_dist_threshold);
    EXPECT_TRUE(is_within_yaw_and_distance_constraint(
      traj, *s_opt, query, std::numeric_limits<double>::max(), M_PI / 3));
  }
  {
    auto query = make_pose(1.5, 0, 0);
    auto s_opt =
      find_first_nearest_index(traj, query, std::numeric_limits<double>::max(), M_PI / 3);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 19.4914, k_points_minimum_dist_threshold);
    EXPECT_TRUE(is_within_yaw_and_distance_constraint(
      traj, *s_opt, query, std::numeric_limits<double>::max(), M_PI / 3));
  }
  {
    auto query = make_pose(-(3 * cos(M_PI / 12)), 0, M_PI);
    auto s_opt =
      find_first_nearest_index(traj, query, std::numeric_limits<double>::max(), M_PI / 3);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 2.99733, k_points_minimum_dist_threshold);
    EXPECT_TRUE(is_within_yaw_and_distance_constraint(
      traj, *s_opt, query, std::numeric_limits<double>::max(), M_PI / 3));
  }
  {
    auto query = make_pose(-(3 * cos(M_PI / 12)), 0, M_PI / 2);
    auto s_opt =
      find_first_nearest_index(traj, query, std::numeric_limits<double>::max(), M_PI / 3);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 9.14898, k_points_minimum_dist_threshold);
    EXPECT_TRUE(is_within_yaw_and_distance_constraint(
      traj, *s_opt, query, std::numeric_limits<double>::max(), M_PI / 3));
  }
  {
    auto query = make_pose(-(3 * cos(M_PI / 12)), 0, -M_PI / 4);
    auto s_opt =
      find_first_nearest_index(traj, query, std::numeric_limits<double>::max(), M_PI / 3);
    ASSERT_TRUE(s_opt.has_value());
    EXPECT_NEAR(*s_opt, 16.9315, k_points_minimum_dist_threshold);
    EXPECT_TRUE(is_within_yaw_and_distance_constraint(
      traj, *s_opt, query, std::numeric_limits<double>::max(), M_PI / 3));
  }
}

}  // namespace
