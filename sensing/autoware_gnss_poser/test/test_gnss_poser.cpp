// Copyright 2026 TIER IV, Inc.
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

// Unit tests of the rclcpp-free helpers in gnss_poser.hpp. They run without a ROS context; the
// node-level behavior is covered by test_gnss_poser_node.cpp and the characterization suite.

#include "gnss_poser.hpp"

#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/circular_buffer.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace
{
using autoware::gnss_poser::get_average_position;
using autoware::gnss_poser::get_median_position;
using autoware::gnss_poser::get_quaternion_by_position_difference;

geometry_msgs::msg::Point make_point(double x, double y, double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

boost::circular_buffer<geometry_msgs::msg::Point> make_position_buffer(
  const std::vector<geometry_msgs::msg::Point> & points)
{
  boost::circular_buffer<geometry_msgs::msg::Point> buffer(points.size());
  for (const auto & point : points) {
    buffer.push_back(point);
  }
  return buffer;
}

double yaw_of(const geometry_msgs::msg::Quaternion & quaternion)
{
  tf2::Quaternion tf_quaternion;
  tf2::fromMsg(quaternion, tf_quaternion);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);
  return yaw;
}
}  // namespace

// Odd-sized buffer: median is the middle element of each coordinate.
TEST(GnssPoserHelpers, MedianPositionOddSize)
{
  const auto buffer = make_position_buffer({
    make_point(3.0, 30.0, 300.0),
    make_point(1.0, 10.0, 100.0),
    make_point(2.0, 20.0, 200.0),
  });

  const auto median = get_median_position(buffer);
  EXPECT_DOUBLE_EQ(median.x, 2.0);
  EXPECT_DOUBLE_EQ(median.y, 20.0);
  EXPECT_DOUBLE_EQ(median.z, 200.0);
}

// Even-sized buffer: median averages the two central elements (previously uncovered branch).
TEST(GnssPoserHelpers, MedianPositionEvenSize)
{
  const auto buffer = make_position_buffer({
    make_point(4.0, 40.0, 400.0),
    make_point(1.0, 10.0, 100.0),
    make_point(3.0, 30.0, 300.0),
    make_point(2.0, 20.0, 200.0),
  });

  // Sorted x: {1,2,3,4} -> median = (2+3)/2 = 2.5; same scaling applies to y and z.
  const auto median = get_median_position(buffer);
  EXPECT_DOUBLE_EQ(median.x, 2.5);
  EXPECT_DOUBLE_EQ(median.y, 25.0);
  EXPECT_DOUBLE_EQ(median.z, 250.0);
}

// Average asserts the actual mean values (previously only existence was checked).
TEST(GnssPoserHelpers, AveragePositionValues)
{
  const auto buffer = make_position_buffer({
    make_point(1.0, 10.0, 100.0),
    make_point(2.0, 20.0, 200.0),
    make_point(6.0, 60.0, 600.0),
  });

  const auto average = get_average_position(buffer);
  EXPECT_DOUBLE_EQ(average.x, 3.0);
  EXPECT_DOUBLE_EQ(average.y, 30.0);
  EXPECT_DOUBLE_EQ(average.z, 300.0);
}

// Orientation-from-motion across the cardinal directions, plus the identical-points edge case.
TEST(GnssPoserHelpers, QuaternionByPositionDifferenceHeadings)
{
  const auto origin = make_point(0.0, 0.0, 0.0);

  // East: dx>0, dy=0 -> yaw 0.
  EXPECT_NEAR(
    yaw_of(get_quaternion_by_position_difference(make_point(1.0, 0.0, 0.0), origin)), 0.0, 1e-9);

  // North: dy>0, dx=0 -> yaw +PI/2.
  EXPECT_NEAR(
    yaw_of(get_quaternion_by_position_difference(make_point(0.0, 1.0, 0.0), origin)), M_PI / 2.0,
    1e-9);

  // West: dx<0, dy=0 -> yaw +-PI.
  EXPECT_NEAR(
    std::abs(yaw_of(get_quaternion_by_position_difference(make_point(-1.0, 0.0, 0.0), origin))),
    M_PI, 1e-9);

  // South: dy<0, dx=0 -> yaw -PI/2.
  EXPECT_NEAR(
    yaw_of(get_quaternion_by_position_difference(make_point(0.0, -1.0, 0.0), origin)), -M_PI / 2.0,
    1e-9);

  // Identical points: atan2(0,0) -> yaw 0 (identity quaternion).
  const auto identity = get_quaternion_by_position_difference(origin, origin);
  EXPECT_NEAR(yaw_of(identity), 0.0, 1e-9);
  EXPECT_DOUBLE_EQ(identity.w, 1.0);
}
