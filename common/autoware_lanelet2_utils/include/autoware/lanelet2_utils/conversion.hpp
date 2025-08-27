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
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_routing/Forward.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <string>
#include <utility>

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
 * @brief converts lanelet2 map to ROS message. Similar implementation to
 * lanelet::io_handlers::BinHandler::write()
 * @param map lanelet map data
 * @param msg converted ROS message. Only "data" field is filled
 */
void to_bin_msg(const lanelet::LaneletMapPtr & map, autoware_map_msgs::msg::LaneletMapBin * msg);

/**
 * @brief converts ROS message into lanelet2 data. Similar implementation to
 * lanelet::io_handlers::BinHandler::parse()
 * @param msg ROS message for lanelet map
 * @param map converted lanelet2 data
 */
void from_bin_msg(
  const autoware_map_msgs::msg::LaneletMapBin & msg, const lanelet::LaneletMapPtr map);
void from_bin_msg(
  const autoware_map_msgs::msg::LaneletMapBin & msg, const lanelet::LaneletMapPtr map,
  lanelet::traffic_rules::TrafficRulesPtr * traffic_rules,
  lanelet::routing::RoutingGraphPtr * routing_graph);

/**
 * @brief converts various point types to geometry_msgs point
 * @param src input point(geometry_msgs::msg::Point32,
 * Eigen::Vector3d=lanelet::BasicPoint3d, lanelet::Point3d, lanelet::Point2d)
 * @param dst converted geometry_msgs point
 */
void to_geom_msg_pt(const geometry_msgs::msg::Point32 & src, geometry_msgs::msg::Point * dst);
void to_geom_msg_pt(const Eigen::Vector3d & src, geometry_msgs::msg::Point * dst);
void to_geom_msg_pt(const lanelet::ConstPoint3d & src, geometry_msgs::msg::Point * dst);
void to_geom_msg_pt(const lanelet::ConstPoint2d & src, geometry_msgs::msg::Point * dst);

/**
 * @brief converts Eigen::Vector3d(lanelet:BasicPoint3d to
 * geometry_msgs::msg::Point32)
 * @param src input point
 * @param dst converted point
 */
void to_geom_msg_pt32(const Eigen::Vector3d & src, geometry_msgs::msg::Point32 * dst);

/**
 * @brief converts various point types to geometry_msgs point]
 * @param src input point(geometry_msgs::msg::Point32,
 * Eigen::Vector3d=lanelet::BasicPoint3d, lanelet::Point3d, lanelet::Point2d)
 * @return converted geometry_msgs point
 */
geometry_msgs::msg::Point to_geom_msg_pt(const geometry_msgs::msg::Point32 & src);
geometry_msgs::msg::Point to_geom_msg_pt(const Eigen::Vector3d & src);
geometry_msgs::msg::Point to_geom_msg_pt(const lanelet::ConstPoint3d & src);
geometry_msgs::msg::Point to_geom_msg_pt(const lanelet::ConstPoint2d & src);

/**
 * @brief converts point to ConstPoint3d
 * @param src input point(geometry_msgs::msg::Point)
 * @return converted ConstPoint3d
 */
lanelet::ConstPoint3d to_lanelet_point(const geometry_msgs::msg::Point & src);
void to_lanelet_point(const geometry_msgs::msg::Point & src, lanelet::ConstPoint3d * dst);

/**
 * @brief converts lanelet polygon to geometry_msgs polygon]
 * @param ll_poly   input polygon
 * @param geom_poly converted geometry_msgs point
 */
void to_geom_msg_poly(
  const lanelet::ConstPolygon3d & ll_poly, geometry_msgs::msg::Polygon * geom_poly);

}  // namespace autoware::experimental::lanelet2_utils
#endif  // AUTOWARE__LANELET2_UTILS__CONVERSION_HPP_
