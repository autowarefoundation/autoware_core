// Copyright 2025 Autonomous Systems sp. z o.o.
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

// Unit tests for VelocitySmootherNode::applyExternalVelocityLimit().
// Terminology used below:
// - A "trajectory point" is one discrete sample in the trajectory array (p0, p1, p2...).
// - A "segment" is the straight line between two consecutive points (p[i] -> p[i+1]).
//   Many geometry operations (nearest search / projection) work on segments because ego can be
//   between samples.
//
//   points:   p0        p1        p2
//            *---------*---------*
//             segment0   segment1
//
#include "autoware/velocity_smoother/node.hpp"
#include "velocity_smoother_test_accessor.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <optional>
#include <string>

namespace
{
using autoware::velocity_smoother::TrajectoryPoints;
using autoware::velocity_smoother::VelocitySmootherNode;
using autoware::velocity_smoother::test::VelocitySmootherTestAccessor;
using autoware_planning_msgs::msg::TrajectoryPoint;

TrajectoryPoint make_point(const double x, const double y, const double yaw, const double vel)
{
  TrajectoryPoint p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.position.z = 0.0;
  p.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
  p.longitudinal_velocity_mps = static_cast<float>(vel);
  p.acceleration_mps2 = 0.0;
  return p;
}

nav_msgs::msg::Odometry::ConstSharedPtr make_odom_ptr(
  const double x, const double y, const double yaw, const double vx)
{
  auto odom = std::make_shared<nav_msgs::msg::Odometry>();
  odom->header.frame_id = "map";
  odom->child_frame_id = "base_link";
  odom->pose.pose.position.x = x;
  odom->pose.pose.position.y = y;
  odom->pose.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
  odom->twist.twist.linear.x = vx;
  return odom;
}

std::shared_ptr<VelocitySmootherNode> create_node()
{
  auto node_options = rclcpp::NodeOptions{};
  node_options.append_parameter_override("algorithm_type", "JerkFiltered");
  node_options.append_parameter_override("publish_debug_trajs", false);
  node_options.append_parameter_override("plan_from_ego_speed_on_manual_mode", false);
  node_options.append_parameter_override("margin_to_insert_external_velocity_limit", 0.0);

  const auto autoware_test_utils_dir =
    ament_index_cpp::get_package_share_directory("autoware_test_utils");
  const auto velocity_smoother_dir =
    ament_index_cpp::get_package_share_directory("autoware_velocity_smoother");

  node_options.arguments({
    "--ros-args",
    "--params-file",
    autoware_test_utils_dir + "/config/test_common.param.yaml",
    "--params-file",
    autoware_test_utils_dir + "/config/test_nearest_search.param.yaml",
    "--params-file",
    autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml",
    "--params-file",
    velocity_smoother_dir + "/config/default_velocity_smoother.param.yaml",
    "--params-file",
    velocity_smoother_dir + "/config/default_common.param.yaml",
    "--params-file",
    velocity_smoother_dir + "/config/JerkFiltered.param.yaml",
  });

  return std::make_shared<VelocitySmootherNode>(node_options);
}

std::optional<size_t> find_point_index(
  const TrajectoryPoints & traj, const double x, const double y, const double eps)
{
  for (size_t i = 0; i < traj.size(); ++i) {
    const auto & p = traj.at(i).pose.position;
    if (std::fabs(p.x - x) < eps && std::fabs(p.y - y) < eps) {
      return i;
    }
  }
  return std::nullopt;
}
}  // namespace

class VelocitySmootherApplyExternalVelocityLimitTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(VelocitySmootherApplyExternalVelocityLimitTest, DoesNothingForEmptyTrajectory)
{
  // Purpose: guard against crashes and unintended mutation on empty input.
  auto node = create_node();
  VelocitySmootherTestAccessor accessor{*node};
  accessor.setCurrentOdometry(make_odom_ptr(0.0, 0.0, 0.0, 0.0));
  accessor.setMaxVelocityWithDeceleration(100.0);
  accessor.setExternalVelocityLimitDistance(1.0);
  accessor.setExternalVelocityLimitVelocity(1.0);

  TrajectoryPoints traj;
  accessor.applyExternalVelocityLimit(traj);

  EXPECT_TRUE(traj.empty());
}

TEST_F(VelocitySmootherApplyExternalVelocityLimitTest, ClampsVelocityForSinglePointTrajectory)
{
  // Purpose: Verify that single-point trajectories are handled gracefully without crashes
  // and that the velocity limit is applied to the single point.
  // This is a critical edge case because line 943 in node.cpp computes traj.size() - 2,
  // which would underflow when traj.size() == 1. The early return at line 937 prevents this.
  auto node = create_node();
  VelocitySmootherTestAccessor accessor{*node};
  accessor.setCurrentOdometry(make_odom_ptr(0.0, 0.0, 0.0, 10.0));
  accessor.setMaxVelocityWithDeceleration(100.0);
  accessor.setExternalVelocityLimitDistance(5.0);
  accessor.setExternalVelocityLimitVelocity(2.0);

  TrajectoryPoints traj;
  traj.push_back(make_point(0.0, 0.0, 0.0, 10.0));  // Single point with velocity 10.0

  ASSERT_EQ(traj.size(), 1u);
  accessor.applyExternalVelocityLimit(traj);

  // Should still have exactly one point (no insertion possible)
  ASSERT_EQ(traj.size(), 1u);

  // The velocity should be clamped to the external limit
  constexpr double vel_eps = 1.0e-3;
  EXPECT_LE(traj.at(0).longitudinal_velocity_mps, 2.0F + vel_eps);
}

TEST_F(VelocitySmootherApplyExternalVelocityLimitTest, ClampsLastPointWhenInsertionIsOutOfRange)
{
  // Purpose: if insertion cannot be done (distance exceeds trajectory), the function still
  // clamps the tail velocity (fallback behavior).
  auto node = create_node();
  VelocitySmootherTestAccessor accessor{*node};
  accessor.setCurrentOdometry(make_odom_ptr(0.0, 0.0, 0.0, 10.0));
  accessor.setMaxVelocityWithDeceleration(100.0);
  accessor.setExternalVelocityLimitDistance(1.0e6);
  accessor.setExternalVelocityLimitVelocity(1.0);

  TrajectoryPoints traj;
  traj.push_back(make_point(0.0, 0.0, 0.0, 10.0));
  traj.push_back(make_point(10.0, 0.0, 0.0, 10.0));
  traj.push_back(make_point(20.0, 0.0, 0.0, 10.0));

  accessor.applyExternalVelocityLimit(traj);

  ASSERT_EQ(traj.size(), 3u);
  EXPECT_FLOAT_EQ(traj.front().longitudinal_velocity_mps, 10.0F);
  EXPECT_LE(traj.back().longitudinal_velocity_mps, 1.0F + 1.0e-3F);
}

// Test: Insertion within the same segment where ego is located.
//
//   p0        E    X(insert)    p1            p2
//   *---------|-------*---------*-------------*
//   0        2.0     5.0       10            20
//
// Ego at x=2, limit distance=3m -> insert at x=5 (still in segment 0).
TEST_F(VelocitySmootherApplyExternalVelocityLimitTest, InsertsPointInSameSegmentAsEgo)
{
  auto node = create_node();
  VelocitySmootherTestAccessor accessor{*node};

  constexpr double ego_x = 2.0;
  constexpr double limit_dist = 3.0;
  accessor.setCurrentOdometry(make_odom_ptr(ego_x, 0.0, 0.0, 10.0));
  accessor.setMaxVelocityWithDeceleration(100.0);
  accessor.setExternalVelocityLimitDistance(limit_dist);
  accessor.setExternalVelocityLimitVelocity(2.0);

  TrajectoryPoints traj;
  traj.push_back(make_point(0.0, 0.0, 0.0, 10.0));
  traj.push_back(make_point(10.0, 0.0, 0.0, 10.0));
  traj.push_back(make_point(20.0, 0.0, 0.0, 10.0));

  const auto original_size = traj.size();
  accessor.applyExternalVelocityLimit(traj);
  ASSERT_EQ(traj.size(), original_size + 1u);

  constexpr double pos_eps = 0.1;
  constexpr double expected_x = ego_x + limit_dist;  // 5.0
  const auto inserted_idx = find_point_index(traj, expected_x, 0.0, pos_eps);
  ASSERT_TRUE(inserted_idx) << "Expected inserted point at x=" << expected_x;

  // Verify velocity limit is applied from the inserted point onwards.
  constexpr double vel_eps = 1.0e-3;
  for (size_t i = 0; i < *inserted_idx; ++i) {
    EXPECT_GT(traj.at(i).longitudinal_velocity_mps, 2.0F + vel_eps)
      << "Points before insertion should not be limited to external velocity";
  }
  for (size_t i = *inserted_idx; i < traj.size(); ++i) {
    EXPECT_LE(traj.at(i).longitudinal_velocity_mps, 2.0F + vel_eps)
      << "Points from insertion onwards should be limited";
  }
}

// Test: Insertion in the first segment when ego is exactly at the trajectory start.
//
//   E/p0      X(insert)        p1            p2
//   *-------------*-------------*-------------*
//   0            5.0           10            20
//
// Ego at x=0 (coincident with p0), limit distance=5m -> insert at x=5.
TEST_F(VelocitySmootherApplyExternalVelocityLimitTest, InsertsPointInFirstSegmentWhenEgoAtStart)
{
  auto node = create_node();
  VelocitySmootherTestAccessor accessor{*node};

  constexpr double ego_x = 0.0;
  constexpr double limit_dist = 5.0;
  accessor.setCurrentOdometry(make_odom_ptr(ego_x, 0.0, 0.0, 10.0));
  accessor.setMaxVelocityWithDeceleration(100.0);
  accessor.setExternalVelocityLimitDistance(limit_dist);
  accessor.setExternalVelocityLimitVelocity(1.5);

  TrajectoryPoints traj;
  traj.push_back(make_point(0.0, 0.0, 0.0, 10.0));
  traj.push_back(make_point(10.0, 0.0, 0.0, 10.0));
  traj.push_back(make_point(20.0, 0.0, 0.0, 10.0));

  const auto original_size = traj.size();
  accessor.applyExternalVelocityLimit(traj);
  ASSERT_EQ(traj.size(), original_size + 1u);

  constexpr double pos_eps = 0.1;
  constexpr double expected_x = ego_x + limit_dist;  // 5.0
  const auto inserted_idx = find_point_index(traj, expected_x, 0.0, pos_eps);
  ASSERT_TRUE(inserted_idx) << "Expected inserted point at x=" << expected_x;

  // First point should NOT be clamped to external limit (it's before the insertion).
  constexpr double vel_eps = 1.0e-3;
  EXPECT_GT(traj.at(0).longitudinal_velocity_mps, 1.5F + vel_eps);

  // From insertion onwards, velocity should be limited.
  for (size_t i = *inserted_idx; i < traj.size(); ++i) {
    EXPECT_LE(traj.at(i).longitudinal_velocity_mps, 1.5F + vel_eps);
  }
}

// Test: Insertion in the last segment.
//
//   p0            p1        E      X(insert)  p2
//   *-------------*---------|----------*-------*
//   0            10        15         18      20
//
// Ego at x=15, limit distance=3m -> insert at x=18 (in segment p1->p2).
TEST_F(VelocitySmootherApplyExternalVelocityLimitTest, InsertsPointInLastSegment)
{
  auto node = create_node();
  VelocitySmootherTestAccessor accessor{*node};

  constexpr double ego_x = 15.0;
  constexpr double limit_dist = 3.0;
  accessor.setCurrentOdometry(make_odom_ptr(ego_x, 0.0, 0.0, 10.0));
  accessor.setMaxVelocityWithDeceleration(100.0);
  accessor.setExternalVelocityLimitDistance(limit_dist);
  accessor.setExternalVelocityLimitVelocity(0.5);

  TrajectoryPoints traj;
  traj.push_back(make_point(0.0, 0.0, 0.0, 10.0));
  traj.push_back(make_point(10.0, 0.0, 0.0, 10.0));
  traj.push_back(make_point(20.0, 0.0, 0.0, 10.0));

  const auto original_size = traj.size();
  accessor.applyExternalVelocityLimit(traj);
  ASSERT_EQ(traj.size(), original_size + 1u);

  constexpr double pos_eps = 0.1;
  constexpr double expected_x = ego_x + limit_dist;  // 18.0
  const auto inserted_idx = find_point_index(traj, expected_x, 0.0, pos_eps);
  ASSERT_TRUE(inserted_idx) << "Expected inserted point at x=" << expected_x;

  constexpr double vel_eps = 1.0e-3;
  for (size_t i = *inserted_idx; i < traj.size(); ++i) {
    EXPECT_LE(traj.at(i).longitudinal_velocity_mps, 0.5F + vel_eps);
  }
}

// Test: Ego exactly at a trajectory point (not between points).
//
//   p0          E/p1          X(insert)       p2
//   *-------------*---------------*------------*
//   0            10              17           20
//
// Ego coincident with p1 at x=10, limit distance=7m -> insert at x=17.
TEST_F(VelocitySmootherApplyExternalVelocityLimitTest, InsertsPointWhenEgoExactlyOnTrajectoryPoint)
{
  auto node = create_node();
  VelocitySmootherTestAccessor accessor{*node};

  constexpr double ego_x = 10.0;
  constexpr double limit_dist = 7.0;
  accessor.setCurrentOdometry(make_odom_ptr(ego_x, 0.0, 0.0, 10.0));
  accessor.setMaxVelocityWithDeceleration(100.0);
  accessor.setExternalVelocityLimitDistance(limit_dist);
  accessor.setExternalVelocityLimitVelocity(2.5);

  TrajectoryPoints traj;
  traj.push_back(make_point(0.0, 0.0, 0.0, 10.0));
  traj.push_back(make_point(10.0, 0.0, 0.0, 10.0));
  traj.push_back(make_point(20.0, 0.0, 0.0, 10.0));

  const auto original_size = traj.size();
  accessor.applyExternalVelocityLimit(traj);
  ASSERT_EQ(traj.size(), original_size + 1u);

  constexpr double pos_eps = 0.1;
  constexpr double expected_x = ego_x + limit_dist;  // 17.0
  const auto inserted_idx = find_point_index(traj, expected_x, 0.0, pos_eps);
  ASSERT_TRUE(inserted_idx) << "Expected inserted point at x=" << expected_x;

  constexpr double vel_eps = 1.0e-3;
  for (size_t i = *inserted_idx; i < traj.size(); ++i) {
    EXPECT_LE(traj.at(i).longitudinal_velocity_mps, 2.5F + vel_eps);
  }
}

// Test: Curved trajectory (quarter circle arc).
// This verifies that arc-length distance is used correctly, not just Euclidean distance.
//
//   p3 *
//      |
//      |
//   p2 *        Curved path (quarter circle, radius=10)
//      .
//       .
//   p1   *
//         .
//          .
//   p0  E   *-----> x
//       |
//       y=0
//
// Trajectory follows a quarter circle from (10,0) to (0,10) with center at origin.
// Ego at (10,0), yaw=90deg (pointing along curve).
// Arc length of quarter circle = pi*R/2 ≈ 15.7m for R=10.
// Limit distance=5m -> should insert at ~5m arc length from ego.
TEST_F(VelocitySmootherApplyExternalVelocityLimitTest, InsertsPointOnCurvedTrajectory)
{
  auto node = create_node();
  VelocitySmootherTestAccessor accessor{*node};

  constexpr double radius = 10.0;
  constexpr double half_pi = boost::math::constants::half_pi<double>();

  // Ego at start of curve
  accessor.setCurrentOdometry(make_odom_ptr(radius, 0.0, half_pi, 10.0));
  accessor.setMaxVelocityWithDeceleration(100.0);
  constexpr double limit_dist = 5.0;
  accessor.setExternalVelocityLimitDistance(limit_dist);
  accessor.setExternalVelocityLimitVelocity(1.0);

  // Create quarter circle trajectory with multiple points.
  // Points at angles 0, 22.5, 45, 67.5, 90 degrees from positive x-axis.
  TrajectoryPoints traj;
  constexpr int num_points = 9;
  for (int i = 0; i < num_points; ++i) {
    const double angle = half_pi * static_cast<double>(i) / (num_points - 1);
    const double x = radius * std::cos(angle);
    const double y = radius * std::sin(angle);
    // Tangent direction (perpendicular to radius, pointing along arc)
    const double yaw = angle + half_pi;
    traj.push_back(make_point(x, y, yaw, 10.0));
  }

  const auto original_size = traj.size();
  accessor.applyExternalVelocityLimit(traj);

  // A point should be inserted
  ASSERT_EQ(traj.size(), original_size + 1u) << "Expected one point to be inserted";

  // Calculate expected insertion point by arc length.
  // Arc length from angle=0: s = R * theta
  // For s=5m on R=10: theta = 0.5 rad ≈ 28.6 degrees
  const double expected_angle = limit_dist / radius;
  const double expected_x = radius * std::cos(expected_angle);
  const double expected_y = radius * std::sin(expected_angle);

  constexpr double pos_eps = 0.5;  // Allow some tolerance due to linear interpolation on arc
  const auto inserted_idx = find_point_index(traj, expected_x, expected_y, pos_eps);
  ASSERT_TRUE(inserted_idx) << "Expected inserted point near (" << expected_x << ", " << expected_y
                            << ")";

  constexpr double vel_eps = 1.0e-3;
  for (size_t i = *inserted_idx; i < traj.size(); ++i) {
    EXPECT_LE(traj.at(i).longitudinal_velocity_mps, 1.0F + vel_eps)
      << "Velocity limit should apply from inserted point onwards";
  }
}

// Test: S-curve trajectory to verify correct handling of direction changes.
//
//       p2 *-------* p3
//         .
//        .
//   p1  *
//      .
//     .
//   E/p0 *
//
// S-shaped path: first curves left, then curves right.
TEST_F(VelocitySmootherApplyExternalVelocityLimitTest, InsertsPointOnSCurveTrajectory)
{
  auto node = create_node();
  VelocitySmootherTestAccessor accessor{*node};

  constexpr double half_pi = boost::math::constants::half_pi<double>();
  constexpr double quarter_pi = boost::math::constants::quarter_pi<double>();

  // Ego at start
  accessor.setCurrentOdometry(make_odom_ptr(0.0, 0.0, quarter_pi, 10.0));
  accessor.setMaxVelocityWithDeceleration(100.0);
  constexpr double limit_dist = 12.0;
  accessor.setExternalVelocityLimitDistance(limit_dist);
  accessor.setExternalVelocityLimitVelocity(2.0);

  // S-curve trajectory
  TrajectoryPoints traj;
  traj.push_back(make_point(0.0, 0.0, quarter_pi, 10.0));
  traj.push_back(make_point(5.0, 5.0, half_pi, 10.0));
  traj.push_back(make_point(5.0, 15.0, half_pi, 10.0));
  traj.push_back(make_point(10.0, 20.0, quarter_pi, 10.0));
  traj.push_back(make_point(20.0, 25.0, 0.1, 10.0));

  const auto original_size = traj.size();
  accessor.applyExternalVelocityLimit(traj);

  // A point should be inserted
  ASSERT_EQ(traj.size(), original_size + 1u) << "Expected one point to be inserted";

  // Verify velocity clamping is applied after insertion
  constexpr double vel_eps = 1.0e-3;
  bool found_limited_point = false;
  for (size_t i = 0; i < traj.size(); ++i) {
    if (traj.at(i).longitudinal_velocity_mps <= 2.0F + vel_eps) {
      found_limited_point = true;
      // All subsequent points should also be limited
      for (size_t j = i; j < traj.size(); ++j) {
        EXPECT_LE(traj.at(j).longitudinal_velocity_mps, 2.0F + vel_eps)
          << "All points after insertion should be velocity-limited";
      }
      break;
    }
  }
  EXPECT_TRUE(found_limited_point) << "Expected at least one velocity-limited point";
}

// Test: Single segment trajectory (only two points).
//
//   E/p0          X(insert)           p1
//   *-----------------*----------------*
//   0                5.0              10
//
// Minimal trajectory with just one segment.
TEST_F(VelocitySmootherApplyExternalVelocityLimitTest, InsertsPointOnSingleSegmentTrajectory)
{
  auto node = create_node();
  VelocitySmootherTestAccessor accessor{*node};

  constexpr double ego_x = 0.0;
  constexpr double limit_dist = 5.0;
  accessor.setCurrentOdometry(make_odom_ptr(ego_x, 0.0, 0.0, 10.0));
  accessor.setMaxVelocityWithDeceleration(100.0);
  accessor.setExternalVelocityLimitDistance(limit_dist);
  accessor.setExternalVelocityLimitVelocity(3.0);

  TrajectoryPoints traj;
  traj.push_back(make_point(0.0, 0.0, 0.0, 10.0));
  traj.push_back(make_point(10.0, 0.0, 0.0, 10.0));

  const auto original_size = traj.size();
  accessor.applyExternalVelocityLimit(traj);
  ASSERT_EQ(traj.size(), original_size + 1u);

  constexpr double pos_eps = 0.1;
  constexpr double expected_x = ego_x + limit_dist;  // 5.0
  const auto inserted_idx = find_point_index(traj, expected_x, 0.0, pos_eps);
  ASSERT_TRUE(inserted_idx) << "Expected inserted point at x=" << expected_x;

  constexpr double vel_eps = 1.0e-3;
  EXPECT_GT(traj.at(0).longitudinal_velocity_mps, 3.0F + vel_eps);
  for (size_t i = *inserted_idx; i < traj.size(); ++i) {
    EXPECT_LE(traj.at(i).longitudinal_velocity_mps, 3.0F + vel_eps);
  }
}

// Test: Very small limit distance (insertion very close to ego).
//
//   E    X(insert)                    p1
//   *----*----------------------------*
//   0   0.1                          10
//
// Edge case: limit distance is tiny (0.1m).
TEST_F(VelocitySmootherApplyExternalVelocityLimitTest, InsertsPointWithVerySmallLimitDistance)
{
  auto node = create_node();
  VelocitySmootherTestAccessor accessor{*node};

  constexpr double ego_x = 0.0;
  constexpr double limit_dist = 0.1;
  accessor.setCurrentOdometry(make_odom_ptr(ego_x, 0.0, 0.0, 10.0));
  accessor.setMaxVelocityWithDeceleration(100.0);
  accessor.setExternalVelocityLimitDistance(limit_dist);
  accessor.setExternalVelocityLimitVelocity(0.0);  // Complete stop

  TrajectoryPoints traj;
  traj.push_back(make_point(0.0, 0.0, 0.0, 10.0));
  traj.push_back(make_point(10.0, 0.0, 0.0, 10.0));

  const auto original_size = traj.size();
  accessor.applyExternalVelocityLimit(traj);
  ASSERT_EQ(traj.size(), original_size + 1u);

  constexpr double pos_eps = 0.05;
  constexpr double expected_x = ego_x + limit_dist;
  const auto inserted_idx = find_point_index(traj, expected_x, 0.0, pos_eps);
  ASSERT_TRUE(inserted_idx) << "Expected inserted point at x=" << expected_x;

  constexpr double vel_eps = 1.0e-3;
  for (size_t i = *inserted_idx; i < traj.size(); ++i) {
    EXPECT_LE(traj.at(i).longitudinal_velocity_mps, vel_eps);
  }
}

// Test: Zero limit distance (immediate velocity limit at ego position).
//
//   E/X(insert)                       p1
//   *---------------------------------*
//   0                                10
//
// Edge case: limit distance is 0 - limit should apply immediately.
TEST_F(VelocitySmootherApplyExternalVelocityLimitTest, AppliesLimitImmediatelyWithZeroDistance)
{
  auto node = create_node();
  VelocitySmootherTestAccessor accessor{*node};

  constexpr double ego_x = 5.0;
  constexpr double limit_dist = 0.0;
  accessor.setCurrentOdometry(make_odom_ptr(ego_x, 0.0, 0.0, 10.0));
  accessor.setMaxVelocityWithDeceleration(100.0);
  accessor.setExternalVelocityLimitDistance(limit_dist);
  accessor.setExternalVelocityLimitVelocity(1.0);

  TrajectoryPoints traj;
  traj.push_back(make_point(0.0, 0.0, 0.0, 10.0));
  traj.push_back(make_point(10.0, 0.0, 0.0, 10.0));
  traj.push_back(make_point(20.0, 0.0, 0.0, 10.0));

  const auto original_size = traj.size();
  accessor.applyExternalVelocityLimit(traj);

  // With zero distance, insertion happens at ego position (x=5)
  ASSERT_EQ(traj.size(), original_size + 1u);

  constexpr double pos_eps = 0.1;
  const auto inserted_idx = find_point_index(traj, ego_x, 0.0, pos_eps);
  ASSERT_TRUE(inserted_idx) << "Expected inserted point at ego position x=" << ego_x;

  constexpr double vel_eps = 1.0e-3;
  for (size_t i = *inserted_idx; i < traj.size(); ++i) {
    EXPECT_LE(traj.at(i).longitudinal_velocity_mps, 1.0F + vel_eps);
  }
}

// Test: Insertion exactly at an existing trajectory point position.
// When the calculated insertion point coincides with an existing point,
// no new point should be inserted (or it may insert a duplicate - implementation dependent).
//
//   p0        E           p1(insert?)       p2
//   *---------|------------*----------------*
//   0        2.0          10               20
//
// Ego at x=2, limit distance=8m -> insertion at x=10 (coincides with p1).
TEST_F(
  VelocitySmootherApplyExternalVelocityLimitTest, HandlesInsertionAtExistingTrajectoryPointPosition)
{
  auto node = create_node();
  VelocitySmootherTestAccessor accessor{*node};

  constexpr double ego_x = 2.0;
  constexpr double limit_dist = 8.0;  // 2 + 8 = 10, exactly at p1
  accessor.setCurrentOdometry(make_odom_ptr(ego_x, 0.0, 0.0, 10.0));
  accessor.setMaxVelocityWithDeceleration(100.0);
  accessor.setExternalVelocityLimitDistance(limit_dist);
  accessor.setExternalVelocityLimitVelocity(1.0);

  TrajectoryPoints traj;
  traj.push_back(make_point(0.0, 0.0, 0.0, 10.0));
  traj.push_back(make_point(10.0, 0.0, 0.0, 10.0));
  traj.push_back(make_point(20.0, 0.0, 0.0, 10.0));

  accessor.applyExternalVelocityLimit(traj);

  // Velocity limit should be applied from x=10 onwards regardless of whether a point was inserted.
  constexpr double vel_eps = 1.0e-3;
  bool found_limited = false;
  for (const auto & pt : traj) {
    if (pt.pose.position.x >= 10.0 - 0.1) {
      EXPECT_LE(pt.longitudinal_velocity_mps, 1.0F + vel_eps)
        << "Velocity should be limited from x=10 onwards";
      found_limited = true;
    }
  }
  EXPECT_TRUE(found_limited);
}

// Regression test for autoware_universe#3494 + ego-offset correctness.
// https://github.com/autowarefoundation/autoware_universe/pull/3494
// Behavior difference:
// - Buggy behavior (distance measured from segment start): inserts at x=dist.
// - Correct behavior (distance measured from ego projection): inserts at x=ego_x+dist.

// The external velocity limit is specified by a *distance ahead of ego*.
// If ego is not exactly on a trajectory point, we must insert a new point at the exact
// requested distance from ego (not from the segment start).
//
//   x-axis (straight trajectory)
//
//   p0        E              X(insert)        p1            p2
//   *---------|--------------*---------------*-------------*
//   0       ego_x        ego_x+dist         10            20
//
// Example (actual test values):
// - Trajectory points at x=0, 10, 20 (3 points)
// - Ego at x=1.5
// - External velocity limit distance = 10.0m ahead of ego
// - Expected insertion at x=11.5 (ego_x + dist)
//
// Ego is in segment 0 (p0->p1), offset 1.5m from p0.
// Previously, the code measured the insertion distance from P0 (segment start).
// This would insert the new point at X=dist (wrong).
// closest_seg_idx = 0
// insertTargetPoint(seg=0, 10.0)  // 10.0m from P0
// → Point inserted at x=10.0, only 8.5m ahead of ego!
//
// Currently, the code measures the insertion distance from ego projection.
// anchor_seg_idx = 0
// offset = 1.5m  (ego is 1.5m into segment 0)
// insertTargetPoint(seg=0, dist + offset) = insertTargetPoint(seg=0, 11.5)
// → Point inserted at x=11.5, exactly 10.0m AHEAD of ego ✓
//
// Diagram with correct values:
// P0────[ego]─────────────────────────[LIMIT]──P1─────────P2
// |←1.5m→|←────────── 10.0m ──────────→|
// |←──────────── 11.5m from P0 ─────────→|
//
// Without offset (buggy):
// P0────[ego]────────────[LIMIT]──P1─────────P2
// |←───────── 10.0m from P0 ─────→|
//       |←──── only 8.5m! ────→|
TEST_F(
  VelocitySmootherApplyExternalVelocityLimitTest,
  InsertsPointAtDistanceAheadOfEgoAndClampsFromThere)
{
  auto node = create_node();
  VelocitySmootherTestAccessor accessor{*node};

  constexpr double ego_x = 1.5;
  constexpr double external_limit_distance_ahead_of_ego = 10.0;
  accessor.setCurrentOdometry(make_odom_ptr(ego_x, 0.0, 0.0, 10.0));
  accessor.setMaxVelocityWithDeceleration(3.0);
  accessor.setExternalVelocityLimitDistance(external_limit_distance_ahead_of_ego);
  accessor.setExternalVelocityLimitVelocity(1.0);

  TrajectoryPoints traj;
  traj.push_back(make_point(0.0, 0.0, 0.0, 10.0));
  traj.push_back(make_point(10.0, 0.0, 0.0, 10.0));
  traj.push_back(make_point(20.0, 0.0, 0.0, 10.0));

  const auto original_size = traj.size();
  accessor.applyExternalVelocityLimit(traj);
  ASSERT_EQ(traj.size(), original_size + 1u);

  constexpr double pos_eps = 0.1;
  constexpr double expected_x = ego_x + external_limit_distance_ahead_of_ego;
  const auto inserted_idx = find_point_index(traj, expected_x, 0.0, pos_eps);
  ASSERT_TRUE(inserted_idx) << "Expected an inserted point at x=" << expected_x;

  constexpr double vel_eps = 1.0e-3;
  for (size_t i = 0; i < *inserted_idx; ++i) {
    EXPECT_LE(traj.at(i).longitudinal_velocity_mps, 3.0F + vel_eps) << "Pre-clamp must apply";
    EXPECT_GT(traj.at(i).longitudinal_velocity_mps, 1.0F + vel_eps) << "No ext limit before insert";
  }
  for (size_t i = *inserted_idx; i < traj.size(); ++i) {
    EXPECT_LE(traj.at(i).longitudinal_velocity_mps, 1.0F + vel_eps) << "Ext limit from insert";
  }
}

// Regression test for: insertion anchor selection in applyExternalVelocityLimit().
// What went wrong historically:
// - The code could pick a nearest *segment* as the anchor, and then treat the anchor as if it
//   were "in front of" ego. Under yaw constraints, the segment-based choice can fall back to a
//   different segment (even behind/earlier along the path), so the insertion/clamp starts on the
//   wrong part of the trajectory.
// - The fix uses ego-nearest *index* (point) as the anchor, which is stable with respect to
//   "progress along the path".
//
// The fix switches the insertion anchor from a nearest *segment* search to the
// ego-nearest *index* (point) search. In a corner case where yaw constraints reject
// the geometrically-nearest segment, the segment-based anchor can drift behind ego,
// producing an insertion on a different part of the path.
//
//   y
//   ^2
//   |
//   |                p2 (yaw=+90deg)
//   |                *
//   |                |
//   |                |  (vertical leg)
//   |                |
//   |                * p1 (corner, yaw=0deg)
//   |               E  (ego at corner, yaw=0deg)
//   |
//   *----------------*---------------------------> x
//  p0
//
// Behavior difference:
// - Before fix (segment-based fallback under yaw constraints): anchor can be p0->p1,
//   so the first clamped point tends to appear on/near y≈0 (horizontal leg).
// - After fix (ego-nearest index anchor): anchor is p1, so clamping starts on the
//   vertical leg (x≈10, y>0).
TEST_F(VelocitySmootherApplyExternalVelocityLimitTest, AnchorsInsertionToEgoNearestIndexAtCorner)
{
  auto node = create_node();
  VelocitySmootherTestAccessor accessor{*node};

  constexpr double half_pi = boost::math::constants::half_pi<double>();
  accessor.setCurrentOdometry(make_odom_ptr(10.0, 0.0, 0.0, 10.0));
  accessor.setMaxVelocityWithDeceleration(100.0);
  accessor.setExternalVelocityLimitDistance(2.0);
  accessor.setExternalVelocityLimitVelocity(1.0);

  TrajectoryPoints traj;
  traj.push_back(make_point(0.0, 0.0, 0.0, 10.0));
  traj.push_back(make_point(10.0, 0.0, 0.0, 10.0));
  traj.push_back(make_point(10.0, 100.0, half_pi, 10.0));

  const auto original_size = traj.size();
  accessor.applyExternalVelocityLimit(traj);
  ASSERT_EQ(traj.size(), original_size + 1u);

  // Expected insertion along the vertical leg, x≈10 and y>0.
  constexpr double pos_eps = 1.0e-3;
  const auto inserted_idx = find_point_index(traj, 10.0, 2.0, pos_eps);
  ASSERT_TRUE(inserted_idx) << "Expected inserted point near (10, 2) on the vertical segment";
  EXPECT_LE(traj.at(*inserted_idx).longitudinal_velocity_mps, 1.0F + 1.0e-3F);
}
