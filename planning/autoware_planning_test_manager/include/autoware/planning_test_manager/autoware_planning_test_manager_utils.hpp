// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__PLANNING_TEST_MANAGER__AUTOWARE_PLANNING_TEST_MANAGER_UTILS_HPP_
#define AUTOWARE__PLANNING_TEST_MANAGER__AUTOWARE_PLANNING_TEST_MANAGER_UTILS_HPP_
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace autoware_planning_test_manager::utils
{
using autoware::route_handler::RouteHandler;
using autoware_planning_msgs::msg::LaneletRoute;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::Odometry;
using RouteSections = std::vector<autoware_planning_msgs::msg::LaneletSegment>;

// Compute the mid-centerline pose for a lanelet centerline. This is the ROS/map-free core of
// createPoseFromLaneID: it operates purely on the centerline points so every branch (including
// the degenerate empty / single-point / two-point cases) is unit-testable without a RouteHandler.
//
// Postconditions:
//   - empty centerline      -> zero position with identity orientation (w == 1).
//   - single-point          -> that point's position with identity orientation (no heading).
//   - two-point             -> middle point's position; heading from the preceding segment.
//   - three-or-more points  -> middle point's position; heading from the forward segment.
inline Pose poseFromCenterline(const lanelet::ConstLineString3d & center_line)
{
  geometry_msgs::msg::Pose middle_pose;
  // Pose default-constructs its quaternion to (0,0,0,0), which is invalid; start from identity
  // so degenerate centerlines (empty / single point) still return a valid orientation.
  middle_pose.orientation.w = 1.0;
  if (center_line.empty()) {
    return middle_pose;
  }

  // get middle idx of the lanelet
  const size_t middle_point_idx = static_cast<size_t>(std::floor(center_line.size() / 2.0));

  // get middle position of the lanelet
  geometry_msgs::msg::Point middle_pos;
  middle_pos.x = center_line[middle_point_idx].x();
  middle_pos.y = center_line[middle_point_idx].y();
  middle_pose.position = middle_pos;

  // Derive the heading from a centerline segment adjacent to the middle point: normally the
  // forward segment (middle, middle+1), but when the middle point is the last one (a two-point
  // centerline, where middle_point_idx == 1) fall back to the preceding segment so the heading
  // is still derivable. A single point cannot define a heading, so keep the identity orientation.
  size_t from_idx = middle_point_idx;
  size_t to_idx = middle_point_idx + 1;
  if (to_idx >= center_line.size()) {
    if (middle_point_idx == 0) {
      return middle_pose;
    }
    from_idx = middle_point_idx - 1;
    to_idx = middle_point_idx;
  }

  geometry_msgs::msg::Point from_pos;
  from_pos.x = center_line[from_idx].x();
  from_pos.y = center_line[from_idx].y();
  geometry_msgs::msg::Point to_pos;
  to_pos.x = center_line[to_idx].x();
  to_pos.y = center_line[to_idx].y();

  // calculate middle pose orientation
  const double yaw = autoware_utils_geometry::calc_azimuth_angle(from_pos, to_pos);
  middle_pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw);

  return middle_pose;
}

// Compute the mid-lanelet pose for the given lanelet id using an already-loaded RouteHandler.
// The orientation is derived from the heading between the two central centerline points.
inline Pose createPoseFromLaneID(const RouteHandler & route_handler, const lanelet::Id & lane_id)
{
  const auto lanelet = route_handler.getLaneletsFromId(lane_id);
  return poseFromCenterline(lanelet.centerline());
}

inline Pose createPoseFromLaneID(
  const lanelet::Id & lane_id, const std::string & package_name = "autoware_test_utils",
  const std::string & map_filename = "lanelet2_map.osm")
{
  const auto map_bin_msg = autoware::test_utils::makeMapBinMsg(package_name, map_filename);
  RouteHandler route_handler;
  route_handler.setMap(map_bin_msg);
  return createPoseFromLaneID(route_handler, lane_id);
}

// Function to create a route from given start and goal lanelet ids
// start pose and goal pose are set to the middle of the lanelet
inline LaneletRoute makeBehaviorRouteFromLaneId(
  const int & start_lane_id, const int & goal_lane_id,
  const std::string & package_name = "autoware_test_utils",
  const std::string & map_filename = "lanelet2_map.osm")
{
  // load the lanelet map / routing graph once and reuse it for both pose
  // extraction and route planning
  const auto map_bin_msg = autoware::test_utils::makeMapBinMsg(package_name, map_filename);
  auto route_handler = std::make_shared<RouteHandler>();
  route_handler->setMap(map_bin_msg);

  LaneletRoute route;
  route.header.frame_id = "map";
  const auto start_pose = createPoseFromLaneID(*route_handler, start_lane_id);
  const auto goal_pose = createPoseFromLaneID(*route_handler, goal_lane_id);
  route.start_pose = start_pose;
  route.goal_pose = goal_pose;

  LaneletRoute route_msg;
  RouteSections route_sections;
  lanelet::ConstLanelets all_route_lanelets;

  // Plan the path between checkpoints (start and goal poses)
  lanelet::ConstLanelets path_lanelets;
  if (!route_handler->planPathLaneletsBetweenCheckpoints(start_pose, goal_pose, &path_lanelets)) {
    return route_msg;
  }

  // Add all path_lanelets to all_route_lanelets
  for (const auto & lane : path_lanelets) {
    all_route_lanelets.push_back(lane);
  }
  // create local route sections
  route_handler->setRouteLanelets(path_lanelets);
  const auto local_route_sections = route_handler->createMapSegments(path_lanelets);
  route_sections =
    autoware::test_utils::combineConsecutiveRouteSections(route_sections, local_route_sections);
  route_handler->setRouteLanelets(all_route_lanelets);
  route.segments = route_sections;

  route.allow_modification = false;
  return route;
}

inline Odometry makeInitialPoseFromLaneId(const lanelet::Id & lane_id)
{
  Odometry current_odometry;
  current_odometry.pose.pose = createPoseFromLaneID(lane_id);
  current_odometry.header.frame_id = "map";

  return current_odometry;
}

}  // namespace autoware_planning_test_manager::utils
#endif  // AUTOWARE__PLANNING_TEST_MANAGER__AUTOWARE_PLANNING_TEST_MANAGER_UTILS_HPP_
