// Copyright 2023 TIER IV, Inc.
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

#include <autoware/planning_topic_converter/conversion.hpp>

#include <gtest/gtest.h>

#include <vector>

namespace autoware::planning_topic_converter
{
namespace
{
PathPoint make_path_point(
  const double x, const double y, const double z, const float longitudinal_velocity_mps,
  const float lateral_velocity_mps, const float heading_rate_rps, const bool is_final)
{
  PathPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.position.z = z;
  point.pose.orientation.x = 0.1;
  point.pose.orientation.y = 0.2;
  point.pose.orientation.z = 0.3;
  point.pose.orientation.w = 0.9;
  point.longitudinal_velocity_mps = longitudinal_velocity_mps;
  point.lateral_velocity_mps = lateral_velocity_mps;
  point.heading_rate_rps = heading_rate_rps;
  point.is_final = is_final;
  return point;
}
}  // namespace

// Pins the PathPoint -> TrajectoryPoint mapping contract: which fields are copied from the
// source and which TrajectoryPoint-only fields are left default-constructed. The pose is
// copied as a whole (a representative position and orientation component is checked rather
// than restating every sub-field).
TEST(ConversionTest, ConvertSinglePointMapsContract)
{
  const auto path_point = make_path_point(1.0, 2.0, 3.0, 4.0f, 5.0f, 6.0f, true);

  const auto traj_point = convertToTrajectoryPoint(path_point);

  // Mapped fields are copied from the source point.
  EXPECT_DOUBLE_EQ(traj_point.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(traj_point.pose.orientation.w, 0.9);
  EXPECT_FLOAT_EQ(traj_point.longitudinal_velocity_mps, 4.0f);
  EXPECT_FLOAT_EQ(traj_point.lateral_velocity_mps, 5.0f);
  EXPECT_FLOAT_EQ(traj_point.heading_rate_rps, 6.0f);

  // TrajectoryPoint fields with no PathPoint counterpart stay default-constructed.
  EXPECT_FLOAT_EQ(traj_point.acceleration_mps2, 0.0f);
  EXPECT_FLOAT_EQ(traj_point.front_wheel_angle_rad, 0.0f);
  EXPECT_FLOAT_EQ(traj_point.rear_wheel_angle_rad, 0.0f);
  EXPECT_EQ(traj_point.time_from_start.sec, 0);
  EXPECT_EQ(traj_point.time_from_start.nanosec, 0u);
}

// Empty input maps to empty output.
TEST(ConversionTest, ConvertEmptyVectorReturnsEmpty)
{
  const std::vector<PathPoint> points;

  const auto traj_points = convertToTrajectoryPoints(points);

  EXPECT_TRUE(traj_points.empty());
}

// Multi-point input preserves order and per-element mapping.
TEST(ConversionTest, ConvertMultiPointPreservesOrderAndMapping)
{
  std::vector<PathPoint> points;
  points.push_back(make_path_point(0.0, 0.0, 0.0, 1.0f, 0.1f, 0.01f, false));
  points.push_back(make_path_point(10.0, 20.0, 30.0, 2.0f, 0.2f, 0.02f, false));
  points.push_back(make_path_point(-5.0, -6.0, -7.0, 3.0f, 0.3f, 0.03f, true));

  const auto traj_points = convertToTrajectoryPoints(points);

  ASSERT_EQ(traj_points.size(), points.size());

  EXPECT_DOUBLE_EQ(traj_points[0].pose.position.x, 0.0);
  EXPECT_FLOAT_EQ(traj_points[0].longitudinal_velocity_mps, 1.0f);
  EXPECT_FLOAT_EQ(traj_points[0].lateral_velocity_mps, 0.1f);
  EXPECT_FLOAT_EQ(traj_points[0].heading_rate_rps, 0.01f);

  EXPECT_DOUBLE_EQ(traj_points[1].pose.position.x, 10.0);
  EXPECT_DOUBLE_EQ(traj_points[1].pose.position.y, 20.0);
  EXPECT_DOUBLE_EQ(traj_points[1].pose.position.z, 30.0);
  EXPECT_FLOAT_EQ(traj_points[1].longitudinal_velocity_mps, 2.0f);

  EXPECT_DOUBLE_EQ(traj_points[2].pose.position.x, -5.0);
  EXPECT_FLOAT_EQ(traj_points[2].longitudinal_velocity_mps, 3.0f);
  EXPECT_FLOAT_EQ(traj_points[2].lateral_velocity_mps, 0.3f);
  EXPECT_FLOAT_EQ(traj_points[2].heading_rate_rps, 0.03f);
}

}  // namespace autoware::planning_topic_converter

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
