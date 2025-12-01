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

#ifndef AUTOWARE__LANELET2_UTILS__CONVERSION_HPP_
#define AUTOWARE__LANELET2_UTILS__CONVERSION_HPP_

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_routing/Forward.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::experimental::lanelet2_utils
{

/**
 * @brief load a map file from the given path and return LaneletMap object
 */
lanelet::LaneletMapConstPtr load_mgrs_coordinate_map(
  const std::string & path, const double centerline_resolution = 5.0);

/**
 * @brief instantiate RoutingGraph from given LaneletMap only from "road" lanes
 * @param location [in, opt, lanelet::Locations::Germany] location value
 * @param participant [in, opt, lanelet::Participants::Vehicle] participant value
 * @return RoutingGraph object without road_shoulder and bicycle_lane, and traffic rule object
 */
std::pair<lanelet::routing::RoutingGraphConstPtr, lanelet::traffic_rules::TrafficRulesPtr>
instantiate_routing_graph_and_traffic_rules(
  lanelet::LaneletMapConstPtr lanelet_map, const char * location = lanelet::Locations::Germany,
  const char * participant = lanelet::Participants::Vehicle);
/**
 * @brief convert BasicPoint3d, ConstPoint3d, BasicPoint2d, and ConstPoint2d to ROS point
 * (geometry_msgs::msg::Point)
 * @param src source point BasicPoint3d, ConstPoint3d, BasicPoint2d, and ConstPoint2d
 * @param z (for 2d) z component of point
 * @return ROS Point (geometry_msgs::msg::Point)
 */
geometry_msgs::msg::Point to_ros(const lanelet::BasicPoint3d & src);
geometry_msgs::msg::Point to_ros(const lanelet::ConstPoint3d & src);
geometry_msgs::msg::Point to_ros(const lanelet::BasicPoint2d & src, const double & z = 0.0);
geometry_msgs::msg::Point to_ros(const lanelet::ConstPoint2d & src, const double & z = 0.0);

/**
 * @brief convert ROS Point or Pose to lanelet::ConstPoint3d
 * @param src source point/pose
 * @return lanelet::ConstPoint3d
 */
lanelet::ConstPoint3d from_ros(const geometry_msgs::msg::Point & src);
lanelet::ConstPoint3d from_ros(const geometry_msgs::msg::Pose & src);

/**
 * @brief serialize lanelet map message to binary ROS message
 */
autoware_map_msgs::msg::LaneletMapBin to_autoware_map_msgs(const lanelet::LaneletMapConstPtr & map);

/**
 * @brief deserialize lanelet map object from binary ROS message
 */
lanelet::LaneletMapConstPtr from_autoware_map_msgs(
  const autoware_map_msgs::msg::LaneletMapBin & msg);

/**
 * @brief construct BasicLineString3d from vector of BasicPoint3d
 */
std::optional<lanelet::BasicLineString3d> create_safe_linestring(
  const std::vector<lanelet::BasicPoint3d> & points);

/**
 * @brief construct ConstLineString3d from vector of ConstPoint3d
 */
std::optional<lanelet::ConstLineString3d> create_safe_linestring(
  const std::vector<lanelet::ConstPoint3d> & points);

/**
 * @brief construct ConstLanelet from BasicPoint3d or ConstPoint3d
 * @param left_points vector of points (for left side)
 * @param right_points vector of points (for right side)
 * @return ConstLanelet
 */
std::optional<lanelet::ConstLanelet> create_safe_lanelet(
  const std::vector<lanelet::BasicPoint3d> & left_points,
  const std::vector<lanelet::BasicPoint3d> & right_points);

std::optional<lanelet::ConstLanelet> create_safe_lanelet(
  const std::vector<lanelet::ConstPoint3d> & left_points,
  const std::vector<lanelet::ConstPoint3d> & right_points);

}  // namespace autoware::experimental::lanelet2_utils
#endif  // AUTOWARE__LANELET2_UTILS__CONVERSION_HPP_
