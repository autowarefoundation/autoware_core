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

#include "autoware/planning_test_manager/autoware_planning_test_manager_utils.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <cmath>
#include <utility>
#include <vector>

namespace autoware_planning_test_manager::utils
{
namespace
{
// IDs of connected lanelets in the bundled autoware_test_utils sample map
// (lanelet2_map.osm). The directed path 9102 -> ... -> 112 is the one exercised
// by autoware_test_utils::makeBehaviorGoalOnLeftSideRoute().
constexpr lanelet::Id g_start_lane_id = 9102;
constexpr lanelet::Id g_goal_lane_id = 112;

// Build an in-process centerline (no map / RouteHandler) from (x, y) pairs so the
// poseFromCenterline branch logic can be exercised directly.
lanelet::ConstLineString3d make_centerline(const std::vector<std::pair<double, double>> & xy)
{
  std::vector<lanelet::Point3d> points;
  points.reserve(xy.size());
  for (const auto & [x, y] : xy) {
    points.emplace_back(lanelet::utils::getId(), x, y, 0.0);
  }
  return lanelet::ConstLineString3d(lanelet::utils::getId(), points);
}
}  // namespace

TEST(PlanningTestManagerUtils, CreatePoseFromLaneIdReturnsPoseOnCenterline)
{
  const auto pose = createPoseFromLaneID(g_start_lane_id);

  // The pose must be populated (non-zero position on the map) and carry a
  // normalized orientation derived from the centerline heading.
  EXPECT_TRUE(std::isfinite(pose.position.x));
  EXPECT_TRUE(std::isfinite(pose.position.y));
  EXPECT_NE(pose.position.x, 0.0);
  EXPECT_NE(pose.position.y, 0.0);

  const double norm = std::sqrt(
    pose.orientation.x * pose.orientation.x + pose.orientation.y * pose.orientation.y +
    pose.orientation.z * pose.orientation.z + pose.orientation.w * pose.orientation.w);
  EXPECT_NEAR(norm, 1.0, 1e-9);
}

TEST(PlanningTestManagerUtils, CreatePoseFromLaneIdSharedRouteHandlerMatchesFileOverload)
{
  // The injectable-RouteHandler overload must produce exactly the same pose as
  // the file-loading convenience overload (the latter just loads the map once
  // and delegates to the former).
  const auto map_bin_msg = autoware::test_utils::makeMapBinMsg();
  autoware::route_handler::RouteHandler route_handler;
  route_handler.setMap(map_bin_msg);

  const auto pose_via_handler = createPoseFromLaneID(route_handler, g_start_lane_id);
  const auto pose_via_file = createPoseFromLaneID(g_start_lane_id);

  EXPECT_DOUBLE_EQ(pose_via_handler.position.x, pose_via_file.position.x);
  EXPECT_DOUBLE_EQ(pose_via_handler.position.y, pose_via_file.position.y);
  EXPECT_DOUBLE_EQ(pose_via_handler.orientation.z, pose_via_file.orientation.z);
  EXPECT_DOUBLE_EQ(pose_via_handler.orientation.w, pose_via_file.orientation.w);
}

TEST(PlanningTestManagerUtils, MakeBehaviorRouteFromLaneIdHappyPath)
{
  const auto route = makeBehaviorRouteFromLaneId(g_start_lane_id, g_goal_lane_id);

  EXPECT_EQ(route.header.frame_id, "map");
  EXPECT_FALSE(route.allow_modification);
  // a connected start/goal pair must yield a non-empty set of route segments
  EXPECT_FALSE(route.segments.empty());

  // start/goal poses are filled from the lanelet centerlines
  EXPECT_NE(route.start_pose.position.x, 0.0);
  EXPECT_NE(route.goal_pose.position.x, 0.0);
}

TEST(PlanningTestManagerUtils, MakeBehaviorRouteFromLaneIdPlanningFailureReturnsEmptyRoute)
{
  // Reversing the directed path (goal lanelet used as start) has no forward
  // route, so planPathLaneletsBetweenCheckpoints fails and an empty LaneletRoute
  // is returned (default frame_id, no segments).
  const auto route = makeBehaviorRouteFromLaneId(g_goal_lane_id, g_start_lane_id);

  EXPECT_TRUE(route.segments.empty());
  EXPECT_TRUE(route.header.frame_id.empty());
}

TEST(PlanningTestManagerUtils, MakeInitialPoseFromLaneIdSetsMapFrame)
{
  const auto odometry = makeInitialPoseFromLaneId(g_start_lane_id);

  EXPECT_EQ(odometry.header.frame_id, "map");
  EXPECT_NE(odometry.pose.pose.position.x, 0.0);
  EXPECT_NE(odometry.pose.pose.position.y, 0.0);
}

// --- poseFromCenterline branch coverage (ROS/map-free degenerate-centerline cases) ---

TEST(PlanningTestManagerUtils, PoseFromCenterlineEmptyReturnsIdentity)
{
  // empty centerline -> zero position with identity orientation (w == 1)
  const auto pose = poseFromCenterline(make_centerline({}));

  EXPECT_DOUBLE_EQ(pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(pose.position.z, 0.0);
  EXPECT_DOUBLE_EQ(pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(pose.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(pose.orientation.w, 1.0);
}

TEST(PlanningTestManagerUtils, PoseFromCenterlineSinglePointKeepsIdentityOrientation)
{
  // single point -> that point's position, identity orientation (a point has no heading)
  const auto pose = poseFromCenterline(make_centerline({{5.0, 7.0}}));

  EXPECT_DOUBLE_EQ(pose.position.x, 5.0);
  EXPECT_DOUBLE_EQ(pose.position.y, 7.0);
  EXPECT_DOUBLE_EQ(pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(pose.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(pose.orientation.w, 1.0);
}

TEST(PlanningTestManagerUtils, PoseFromCenterlineTwoPointsUsesPrecedingSegmentHeading)
{
  // two points -> middle idx is 1 (the last point), so the heading falls back to the
  // preceding segment (0 -> 1). Segment (0,0) -> (2,2) has yaw = +pi/4.
  const auto pose = poseFromCenterline(make_centerline({{0.0, 0.0}, {2.0, 2.0}}));

  EXPECT_DOUBLE_EQ(pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(pose.position.y, 2.0);

  constexpr double expected_yaw = M_PI_4;
  EXPECT_NEAR(pose.orientation.z, std::sin(expected_yaw / 2.0), 1e-9);
  EXPECT_NEAR(pose.orientation.w, std::cos(expected_yaw / 2.0), 1e-9);
}

TEST(PlanningTestManagerUtils, PoseFromCenterlineMultiPointUsesForwardSegmentHeading)
{
  // three points -> middle idx is 1, heading from the forward segment (1 -> 2).
  // Segment (1,1) -> (1,2) points straight up: yaw = +pi/2.
  const auto pose = poseFromCenterline(make_centerline({{0.0, 0.0}, {1.0, 1.0}, {1.0, 2.0}}));

  EXPECT_DOUBLE_EQ(pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(pose.position.y, 1.0);

  constexpr double expected_yaw = M_PI_2;
  EXPECT_NEAR(pose.orientation.z, std::sin(expected_yaw / 2.0), 1e-9);
  EXPECT_NEAR(pose.orientation.w, std::cos(expected_yaw / 2.0), 1e-9);
}

}  // namespace autoware_planning_test_manager::utils
