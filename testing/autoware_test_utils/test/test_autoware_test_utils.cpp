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

#include "autoware_test_utils/autoware_test_utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace autoware::test_utils
{

TEST(AutowareTestUtils, CreateLaneletSegment)
{
  const auto segment = createLaneletSegment(42);
  EXPECT_EQ(segment.preferred_primitive.id, 42);
  ASSERT_EQ(segment.primitives.size(), static_cast<size_t>(1));
  EXPECT_EQ(segment.primitives.front().id, 42);
  EXPECT_EQ(segment.primitives.front().primitive_type, "lane");
}

TEST(AutowareTestUtils, MakeCostMapMsg)
{
  const size_t width = 5;
  const size_t height = 3;
  const double resolution = 0.5;
  const auto costmap = makeCostMapMsg(width, height, resolution);

  EXPECT_EQ(costmap.header.frame_id, "map");
  EXPECT_EQ(costmap.info.width, width);
  EXPECT_EQ(costmap.info.height, height);
  EXPECT_FLOAT_EQ(costmap.info.resolution, static_cast<float>(resolution));

  // assign() must produce exactly width * height zero-initialized cells
  ASSERT_EQ(costmap.data.size(), width * height);
  for (const auto cell : costmap.data) {
    EXPECT_EQ(cell, 0);
  }
}

TEST(AutowareTestUtils, MakeCostMapMsgZeroSize)
{
  const auto costmap = makeCostMapMsg(0, 0, 1.0);
  EXPECT_TRUE(costmap.data.empty());
}

TEST(AutowareTestUtils, MakeOdometry)
{
  const double shift = 2.5;
  const auto odometry = makeOdometry(shift);

  EXPECT_EQ(odometry.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(odometry.pose.pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(odometry.pose.pose.position.y, shift);
  EXPECT_DOUBLE_EQ(odometry.pose.pose.position.z, 0.0);
  // yaw == 0 -> identity orientation
  EXPECT_DOUBLE_EQ(odometry.pose.pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(odometry.pose.pose.orientation.w, 1.0);
}

TEST(AutowareTestUtils, MakeInitialPose)
{
  const double shift = 4.0;
  const auto odometry = makeInitialPose(shift);

  constexpr double yaw = 0.9724497591854532;
  const double expected_x = 3722.16015625 + shift * std::sin(yaw);
  const double expected_y = 73723.515625 + shift * std::cos(yaw);

  EXPECT_EQ(odometry.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(odometry.pose.pose.position.x, expected_x);
  EXPECT_DOUBLE_EQ(odometry.pose.pose.position.y, expected_y);
  EXPECT_DOUBLE_EQ(odometry.pose.pose.position.z, 0.233112560494183);
  // Orientation is a yaw-only quaternion: z = sin(yaw/2), w = cos(yaw/2)
  EXPECT_NEAR(odometry.pose.pose.orientation.z, std::sin(yaw / 2.0), 1e-12);
  EXPECT_NEAR(odometry.pose.pose.orientation.w, std::cos(yaw / 2.0), 1e-12);
}

TEST(AutowareTestUtils, CombineConsecutiveRouteSectionsBothNonEmpty)
{
  RouteSections first;
  first.push_back(createLaneletSegment(1));
  first.push_back(createLaneletSegment(2));
  first.push_back(createLaneletSegment(3));

  RouteSections second;
  second.push_back(createLaneletSegment(4));
  second.push_back(createLaneletSegment(5));

  const auto combined = combineConsecutiveRouteSections(first, second);

  // The last element of the first sections is dropped (overlap removal),
  // then all of the second sections are appended.
  ASSERT_EQ(combined.size(), static_cast<size_t>(4));
  EXPECT_EQ(combined[0].preferred_primitive.id, 1);
  EXPECT_EQ(combined[1].preferred_primitive.id, 2);
  EXPECT_EQ(combined[2].preferred_primitive.id, 4);
  EXPECT_EQ(combined[3].preferred_primitive.id, 5);
}

TEST(AutowareTestUtils, CombineConsecutiveRouteSectionsFirstEmpty)
{
  RouteSections first;
  RouteSections second;
  second.push_back(createLaneletSegment(7));
  second.push_back(createLaneletSegment(8));

  const auto combined = combineConsecutiveRouteSections(first, second);

  ASSERT_EQ(combined.size(), static_cast<size_t>(2));
  EXPECT_EQ(combined[0].preferred_primitive.id, 7);
  EXPECT_EQ(combined[1].preferred_primitive.id, 8);
}

TEST(AutowareTestUtils, CombineConsecutiveRouteSectionsSecondEmpty)
{
  RouteSections first;
  first.push_back(createLaneletSegment(1));
  first.push_back(createLaneletSegment(2));
  RouteSections second;

  const auto combined = combineConsecutiveRouteSections(first, second);

  // Only the first sections minus the dropped overlap remain.
  ASSERT_EQ(combined.size(), static_cast<size_t>(1));
  EXPECT_EQ(combined[0].preferred_primitive.id, 1);
}

TEST(AutowareTestUtils, GenerateTrajectoryNoOverlap)
{
  const size_t num_points = 5;
  const double point_interval = 2.0;
  const double velocity = 3.0;
  const auto traj = generateTrajectory<Trajectory>(num_points, point_interval, velocity);

  ASSERT_EQ(traj.points.size(), num_points);
  for (size_t i = 0; i < num_points; ++i) {
    EXPECT_DOUBLE_EQ(traj.points[i].pose.position.x, static_cast<double>(i) * point_interval);
    EXPECT_DOUBLE_EQ(traj.points[i].pose.position.y, 0.0);
    EXPECT_DOUBLE_EQ(traj.points[i].longitudinal_velocity_mps, velocity);
  }
}

TEST(AutowareTestUtils, GenerateTrajectoryOverlapBranch)
{
  const size_t num_points = 5;
  const double point_interval = 1.0;
  const size_t overlapping_point_index = 2;
  const auto traj = generateTrajectory<Trajectory>(
    num_points, point_interval, 0.0, 0.0, 0.0, overlapping_point_index);

  // The overlapping branch inserts a duplicate of the point at the given index,
  // so the resulting size is num_points + 1.
  ASSERT_EQ(traj.points.size(), num_points + 1);
  // Points at overlapping_point_index and overlapping_point_index + 1 are equal.
  EXPECT_DOUBLE_EQ(
    traj.points[overlapping_point_index].pose.position.x,
    traj.points[overlapping_point_index + 1].pose.position.x);
  EXPECT_DOUBLE_EQ(
    traj.points[overlapping_point_index].pose.position.x,
    static_cast<double>(overlapping_point_index) * point_interval);
}

TEST(AutowareTestUtils, GenerateTrajectoryPathWithLaneIdOverlapBranch)
{
  const size_t num_points = 4;
  const double point_interval = 1.0;
  const size_t overlapping_point_index = 1;
  const auto traj = generateTrajectory<PathWithLaneId>(
    num_points, point_interval, 0.0, 0.0, 0.0, overlapping_point_index);

  ASSERT_EQ(traj.points.size(), num_points + 1);
  EXPECT_EQ(
    traj.points[overlapping_point_index].lane_ids,
    traj.points[overlapping_point_index + 1].lane_ids);
  EXPECT_DOUBLE_EQ(
    traj.points[overlapping_point_index].point.pose.position.x,
    traj.points[overlapping_point_index + 1].point.pose.position.x);
}

TEST(AutowareTestUtils, ResolvePkgShareUriNoMatch)
{
  // A URI that does not match the package:// pattern returns nullopt.
  const auto result = resolve_pkg_share_uri("not_a_package_uri/foo/bar");
  EXPECT_FALSE(result.has_value());
}

TEST(AutowareTestUtils, ResolvePkgShareUriMatchExists)
{
  // A well-formed package:// URI pointing at an installed resource resolves to
  // an absolute path that exists on disk.
  const auto result =
    resolve_pkg_share_uri("package://autoware_test_utils/config/path_with_lane_id_data.yaml");
  ASSERT_TRUE(result.has_value());
  const auto expected = ament_index_cpp::get_package_share_directory("autoware_test_utils") +
                        "/config/path_with_lane_id_data.yaml";
  EXPECT_EQ(*result, expected);
}

TEST(AutowareTestUtils, ResolvePkgShareUriMatchMissing)
{
  // A well-formed package:// URI whose resource does not exist returns nullopt.
  const auto result =
    resolve_pkg_share_uri("package://autoware_test_utils/this/resource/does_not_exist.yaml");
  EXPECT_FALSE(result.has_value());
}

}  // namespace autoware::test_utils
