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

#include "autoware/motion_velocity_planner_common/polygon_utils.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using autoware::vehicle_info_utils::VehicleInfo;
using autoware_planning_msgs::msg::TrajectoryPoint;

// Tests the implementation with an alternative method using elementary geometry (e.g., Pythagorean
// theorem). This method is not used in the production code due to a division-by-zero issue on
// straight paths.
class OffTrackTest : public ::testing::Test
{
protected:
  VehicleInfo vehicle_info_;
  std::vector<TrajectoryPoint> trajectory_points_;
  const double step_length_ = 2.0;

  void SetUp() override
  {
    vehicle_info_.wheel_base_m = 2.7;
    vehicle_info_.vehicle_width_m = 1.8;
  }

  void generate_arc_trajectory(
    const double radius, const int rotation_sign, const bool is_driving_forward, size_t num_points)
  {
    trajectory_points_.clear();
    trajectory_points_.reserve(num_points);

    const double step_angle = rotation_sign * step_length_ / radius;
    for (size_t i = 0; i < num_points; ++i) {
      TrajectoryPoint point;
      point.pose.position.x = radius * std::cos(i * step_angle);
      point.pose.position.y = radius * std::sin(i * step_angle);
      point.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(
        is_driving_forward ? i * step_angle + rotation_sign * M_PI / 2.0
                           : i * step_angle - rotation_sign * M_PI / 2.0);
      trajectory_points_.push_back(point);
    }
  }
  void generate_straight_trajectory(const bool is_driving_forward, size_t num_points)
  {
    trajectory_points_.clear();
    trajectory_points_.reserve(num_points);

    for (size_t i = 0; i < num_points; ++i) {
      TrajectoryPoint point;
      point.pose.position.x = i * step_length_;
      point.pose.position.y = 0.0;
      point.pose.orientation =
        autoware_utils_geometry::create_quaternion_from_yaw(is_driving_forward ? 0.0 : M_PI);
      trajectory_points_.push_back(point);
    }
  }
};

TEST_F(OffTrackTest, ForwardDriving)
{
  for (size_t num_points = 0; num_points < 10; ++num_points) {
    generate_straight_trajectory(true, num_points);
    const auto result =
      autoware::motion_velocity_planner::polygon_utils::calc_front_outer_wheel_off_tracking(
        trajectory_points_, vehicle_info_);
    ASSERT_EQ(result.size(), num_points);
    for (const auto & value : result) {
      EXPECT_NEAR(value, 0.0, 1e-6);
    }
  }

  std::vector<double> rotation_signs{1.0, -1.0};
  std::vector<double> radius_vec{10.0, 1e3, 1e6};

  const size_t num_points = 10;  // Set a fixed number of points for the arc trajectory
  for (const auto & rotation_sign : rotation_signs) {
    for (const auto & radius : radius_vec) {
      generate_arc_trajectory(radius, rotation_sign, true, num_points);
      const auto result =
        autoware::motion_velocity_planner::polygon_utils::calc_front_outer_wheel_off_tracking(
          trajectory_points_, vehicle_info_);
      ASSERT_EQ(result.size(), num_points);

      const auto expected_value = [&]() {
        const double dist =
          std::hypot(radius + vehicle_info_.vehicle_width_m / 2.0, vehicle_info_.wheel_base_m) -
          (radius + vehicle_info_.vehicle_width_m / 2.0);
        return -rotation_sign * dist;
      }();
      for (size_t i = 1; i < result.size() - 1; ++i) {
        EXPECT_NEAR(result[i], expected_value, 1e-6);
      }
    }
  }
}

TEST_F(OffTrackTest, BackwardDriving)
{
  for (size_t num_points = 0; num_points < 10; ++num_points) {
    generate_straight_trajectory(false, num_points);
    const auto result =
      autoware::motion_velocity_planner::polygon_utils::calc_front_outer_wheel_off_tracking(
        trajectory_points_, vehicle_info_);
    ASSERT_EQ(result.size(), num_points);
    for (const auto & value : result) {
      EXPECT_NEAR(value, 0.0, 1e-6);
    }
  }

  std::vector<double> rotation_signs{1.0, -1.0};
  std::vector<double> radius_vec{10.0, 1e3, 1e6};

  const size_t num_points = 10;  // Set a fixed number of points for the arc trajectory
  for (const auto & rotation_sign : rotation_signs) {
    for (const auto & radius : radius_vec) {
      generate_arc_trajectory(radius, rotation_sign, false, num_points);
      const auto result =
        autoware::motion_velocity_planner::polygon_utils::calc_front_outer_wheel_off_tracking(
          trajectory_points_, vehicle_info_);
      ASSERT_EQ(result.size(), num_points);

      const auto expected_value = [&]() {
        const double dist =
          std::hypot(radius + vehicle_info_.vehicle_width_m / 2.0, vehicle_info_.wheel_base_m) -
          (radius + vehicle_info_.vehicle_width_m / 2.0);
        return rotation_sign * dist;
      }();
      for (size_t i = 1; i < result.size() - 1; ++i) {
        EXPECT_NEAR(result[i], expected_value, 1e-6);
      }
    }
  }
}
