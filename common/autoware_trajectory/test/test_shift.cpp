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

#include "autoware/motion_utils/trajectory/path_shift.hpp"
#include "autoware/trajectory/point.hpp"
#include "autoware/trajectory/utils/shift.hpp"

#include "geometry_msgs/msg/detail/point__struct.hpp"

#include <gtest/gtest.h>

#include <vector>

geometry_msgs::msg::Point point(double x, double y)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  return p;
}

namespace autoware::experimental::trajectory
{

TEST(ShiftInvalid, error_shift_start_is_less_than_end)
{
  std::vector<geometry_msgs::msg::Point> points = {point(0.0, 0.0),  point(3.0, 0.0),
                                                   point(6.0, 0.0),  point(9.0, 0.0),
                                                   point(12.0, 0.0), point(18.0, 0.0)};
  auto trajectory = Trajectory<geometry_msgs::msg::Point>::Builder{}.build(points);

  const double longitudinal_velocity = 2.77;
  const double lateral_shift = 2.5;
  const double lateral_acc_limit = 5.0;

  const ShiftInterval shift_interval{-1.0, 10.0, lateral_shift};
  const ShiftParameters shift_parameter{
    longitudinal_velocity,
    lateral_acc_limit,
  };

  auto shifted_trajectory = shift(*trajectory, shift_interval, shift_parameter);
  ASSERT_TRUE(shifted_trajectory);

  geometry_msgs::msg::Point start_point = shifted_trajectory->compute(0.0);
  geometry_msgs::msg::Point end_point = shifted_trajectory->compute(shifted_trajectory->length());

  EXPECT_LT(start_point.y, 0.0);
  EXPECT_NEAR(end_point.y, -lateral_shift, 1e-3);
}

TEST(ShiftInvalid, error_shift_end_is_less_than_end)
{
  std::vector<geometry_msgs::msg::Point> points = {point(0.0, 0.0),  point(3.0, 0.0),
                                                   point(6.0, 0.0),  point(9.0, 0.0),
                                                   point(12.0, 0.0), point(18.0, 0.0)};
  auto trajectory = Trajectory<geometry_msgs::msg::Point>::Builder{}.build(points);

  const double longitudinal_velocity = 2.77;
  const double lateral_shift = 2.5;
  const double lateral_acc_limit = 5.0;

  const ShiftInterval shift_interval{5.0, 25.0, lateral_shift};
  const ShiftParameters shift_parameter{
    longitudinal_velocity,
    lateral_acc_limit,
  };

  auto shifted_trajectory = shift(*trajectory, shift_interval, shift_parameter);
  ASSERT_TRUE(shifted_trajectory);

  geometry_msgs::msg::Point start_point = shifted_trajectory->compute(0.0);
  geometry_msgs::msg::Point end_point = shifted_trajectory->compute(shifted_trajectory->length());

  EXPECT_NEAR(start_point.y, 0.0, 1e-3);
  EXPECT_GT(end_point.y, -lateral_shift);
}

TEST(ShiftInvalid, error_interval_is_backward)
{
  std::vector<geometry_msgs::msg::Point> points = {point(0.0, 0.0),  point(3.0, 0.0),
                                                   point(6.0, 0.0),  point(9.0, 0.0),
                                                   point(12.0, 0.0), point(18.0, 0.0)};
  auto trajectory = Trajectory<geometry_msgs::msg::Point>::Builder{}.build(points);

  const double longitudinal_velocity = 2.77;
  const double lateral_shift = 2.5;
  const double lateral_acc_limit = 5.0;

  const ShiftInterval shift_interval{1.0, 0.0, lateral_shift};
  const ShiftParameters shift_parameter{
    longitudinal_velocity,
    lateral_acc_limit,
  };

  auto shifted_trajectory = shift(*trajectory, shift_interval, shift_parameter);
  ASSERT_TRUE(!shifted_trajectory);
}

TEST(ShiftInvalid, error_longitudinal_velocity_is_negative)
{
  std::vector<geometry_msgs::msg::Point> points = {point(0.0, 0.0),  point(3.0, 0.0),
                                                   point(6.0, 0.0),  point(9.0, 0.0),
                                                   point(12.0, 0.0), point(18.0, 0.0)};
  auto trajectory = Trajectory<geometry_msgs::msg::Point>::Builder{}.build(points);

  const double longitudinal_velocity = -2.77;
  const double lateral_shift = 2.5;
  const double lateral_acc_limit = 5.0;

  const ShiftInterval shift_interval{5.0, 25.0, lateral_shift};
  const ShiftParameters shift_parameter{
    longitudinal_velocity,
    lateral_acc_limit,
  };

  auto shifted_trajectory_info = shift(*trajectory, shift_interval, shift_parameter);
  ASSERT_TRUE(!shifted_trajectory_info);
}

TEST(ShiftSuccess, shift_end_meets_given_lateral_longitudinal_distance_4points)
{
  std::vector<geometry_msgs::msg::Point> points = {point(0.0, 0.0),  point(3.0, 0.0),
                                                   point(6.0, 0.0),  point(9.0, 0.0),
                                                   point(12.0, 0.0), point(18.0, 0.0)};
  auto trajectory = Trajectory<geometry_msgs::msg::Point>::Builder{}.build(points);

  const double longitudinal_velocity = 2.77;
  const double lateral_jerk = 1.0;
  const double lateral_shift = 2.5;
  const double lateral_acc_limit = 5.0;
  const double longitudinal_dist = autoware::motion_utils::calc_longitudinal_dist_from_jerk(
    lateral_shift, lateral_jerk, lateral_shift);

  const auto start_s = 3.0;
  const ShiftInterval shift_interval{start_s, start_s + longitudinal_dist, lateral_shift};
  const ShiftParameters shift_parameter{
    longitudinal_velocity,
    lateral_acc_limit,
  };

  auto shifted_trajectory_info = shift(*trajectory, shift_interval, shift_parameter);
  ASSERT_TRUE(shifted_trajectory_info);

  const auto & shifted_trajectory = shifted_trajectory_info.value();

  EXPECT_FLOAT_EQ(shifted_trajectory.compute(0.0).y, 0.0);
  EXPECT_FLOAT_EQ(shifted_trajectory.compute(shifted_trajectory.length()).y, -lateral_shift);
}

TEST(ShiftSuccess, shift_end_meets_given_lateral_longitudinal_distance_6points)
{
  std::vector<geometry_msgs::msg::Point> points = {point(0.0, 0.0),  point(3.0, 0.0),
                                                   point(6.0, 0.0),  point(9.0, 0.0),
                                                   point(12.0, 0.0), point(18.0, 0.0)};
  auto trajectory = Trajectory<geometry_msgs::msg::Point>::Builder{}.build(points);

  const double longitudinal_velocity = 2.77;
  const double lateral_jerk = 1.5;
  const double lateral_shift = 5.0;
  const double lateral_acc_limit = 1.5;
  const double longitudinal_dist = autoware::motion_utils::calc_longitudinal_dist_from_jerk(
    lateral_shift, lateral_jerk, longitudinal_velocity);

  const auto start_s = 3.0;
  const ShiftInterval shift_interval{start_s, start_s + longitudinal_dist, lateral_shift};
  const ShiftParameters shift_parameter{
    longitudinal_velocity,
    lateral_acc_limit,
  };

  auto shifted_trajectory_info = shift(*trajectory, shift_interval, shift_parameter);
  ASSERT_TRUE(shifted_trajectory_info);
}

TEST(ShiftInvalid, multiple_shift)
{
  std::vector<geometry_msgs::msg::Point> points = {point(0.0, 0.0),  point(3.0, 0.0),
                                                   point(6.0, 0.0),  point(9.0, 0.0),
                                                   point(12.0, 0.0), point(18.0, 0.0)};
  auto trajectory = Trajectory<geometry_msgs::msg::Point>::Builder{}.build(points);

  const double longitudinal_velocity = 2.77;
  const double lateral_shift = 2.5;
  const double lateral_acc_limit = 5.0;

  const ShiftInterval shift_interval1{1.0, 9.0, lateral_shift};
  const ShiftInterval shift_interval2{9.0, 17.0, -lateral_shift};
  const ShiftParameters shift_parameter{
    longitudinal_velocity,
    lateral_acc_limit,
  };

  auto shifted_trajectory = shift(*trajectory, {shift_interval1, shift_interval2}, shift_parameter);
  ASSERT_TRUE(shifted_trajectory);

  geometry_msgs::msg::Point start_point = shifted_trajectory->compute(0.0);
  geometry_msgs::msg::Point middle_point =
    shifted_trajectory->compute(shifted_trajectory->length() / 2.0);
  geometry_msgs::msg::Point end_point = shifted_trajectory->compute(shifted_trajectory->length());

  EXPECT_NEAR(start_point.y, 0.0, 1e-3);
  EXPECT_NEAR(middle_point.y, -lateral_shift, 1e-3);
  EXPECT_NEAR(end_point.y, 0.0, 1e-3);
}

}  // namespace autoware::experimental::trajectory
