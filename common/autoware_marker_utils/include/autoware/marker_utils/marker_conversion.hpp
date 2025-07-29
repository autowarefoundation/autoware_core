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

#ifndef AUTOWARE__MARKER_UTILS__MARKER_CONVERSION_HPP_
#define AUTOWARE__MARKER_UTILS__MARKER_CONVERSION_HPP_

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/time.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/Forward.h>

#include <string>
#include <vector>

namespace autoware::experimental::marker_utils
{

/**
 * @brief check if the input type is LINE_LIST or LINE_STRIP
 * @details it will throw runtime error if it's not LINE_LIST or LINE_STRIP
 */
void check_marker_type_line(uint32_t marker_type);

/**
 * @brief create marker from Autoware Polygon2d
 * @param [in] polygon Autoware Polygon2d
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] marker_type type of the marker (LINE_LIST or LINE_STRIP)
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @param [in] z z position of the marker
 * @return marker of the Autoware Polygon2d
 */
visualization_msgs::msg::Marker create_autoware_geometry_marker(
  const autoware_utils_geometry::Polygon2d & polygon, const rclcpp::Time & stamp,
  const std::string & ns, int32_t id, uint32_t marker_type,
  const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color,
  double z = 0.0);

/**
 * @brief Convert Eigen::Vector3d to geometry_msgs::msg::Point32
 * @param [in] src input point
 */
geometry_msgs::msg::Point32 to_geom_msg_pt32(const Eigen::Vector3d & src);

/**
 * @brief Convert lanelet polygon (ConstPolygon3d) to geometry_msgs polygon
 * @param [in] ll_poly input ConstPolygon3d polygon
 */
geometry_msgs::msg::Polygon to_geom_msg_poly(const lanelet::ConstPolygon3d & ll_poly);

/**
 * @brief Make a marker of LineString2d
 * @param [in] ls LineString2d
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * return marker of LineString2d
 */
visualization_msgs::msg::Marker create_linestring_marker(
  const autoware_utils::LineString2d & ls, const rclcpp::Time & stamp, const std::string & ns,
  int32_t id, const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color,
  const double z);

/**
 * @brief create marker array from geometry polygon based on the marker type
 * @details if marker_type is LINE_LIST, the polygon is drawn as a line list
 *          if marker_type is LINE_STRIP, the polygon is drawn as a line strip
 * @param [in] polygon geometry polygon
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] marker_type type of the marker (LINE_LIST or LINE_STRIP)
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @return marker array of the geometry polygon
 */
visualization_msgs::msg::MarkerArray create_autoware_geometry_marker_array(
  const geometry_msgs::msg::Polygon & polygon, const rclcpp::Time & stamp, const std::string & ns,
  int32_t id, uint32_t marker_type, const geometry_msgs::msg::Vector3 & scale,
  const std_msgs::msg::ColorRGBA & color);

/**
 * @brief create footprint from LinearRing2d (used mainly for goal_footprint)
 * @param [in] ring LinearRing2d  to convert into marker
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] marker_type type of the marker (LINE_LIST or LINE_STRIP)
 * @param [in] scale scale of the marker
 * @return marker array of the boost LinearRing2d
 */
visualization_msgs::msg::MarkerArray create_autoware_geometry_marker_array(
  const autoware_utils_geometry::LinearRing2d & ring, const rclcpp::Time & stamp,
  const std::string & ns, int32_t id, uint32_t marker_type,
  const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color);

/**
 * @brief create marker array from boost MultiPolygon2d (Pull over area)
 * @param [in] area_polygon boost MultiPolygon2d
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] marker_type type of the marker (LINE_LIST or LINE_STRIP)
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @param [in] z z position of the marker
 * @return marker array of the boost MultiPolygon2d (Pull over area)
 */
visualization_msgs::msg::MarkerArray create_autoware_geometry_marker_array(
  const autoware_utils_geometry::MultiPolygon2d & area_polygons, const rclcpp::Time & stamp,
  const std::string & ns, int32_t & id, uint32_t marker_type,
  const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color, double z = 0.0,
  bool running_id = false);

/**
 * @brief return marker array from centroid of MultiPolygon2d and trajectory point
 * @param [in] multi_polygon boost MultiPolygon2d
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] marker_type type of the marker (LINE_LIST or LINE_STRIP)
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @return marker array of the boost MultiPolygon2d (Pull over area)
 */
visualization_msgs::msg::MarkerArray create_autoware_geometry_marker_array(
  const autoware_utils_geometry::MultiPolygon2d & multi_polygon, const size_t & trajectory_index,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const rclcpp::Time & stamp, const std::string & ns, int32_t id, uint32_t marker_type,
  const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color);

/**
 * @brief create marker array from stop obstacle point
 * @param [in] stop_obstacle_point point of the stop obstacle
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] marker_type type of the marker (LINE_LIST or LINE_STRIP)
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @return marker array of the stop obstacle point
 */
visualization_msgs::msg::MarkerArray create_autoware_geometry_marker_array(
  const geometry_msgs::msg::Point & stop_obstacle_point, const rclcpp::Time & stamp,
  const std::string & ns, int32_t id, uint32_t marker_type,
  const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color);

/**
 * @brief create marker array from the vector of points
 * @param [in] points vector of points
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @param [in] separate require to separate point into several markers or not
 * @return marker array of the stop obstacle point
 */
visualization_msgs::msg::MarkerArray create_autoware_geometry_marker_array(
  const std::vector<geometry_msgs::msg::Point> & points, const rclcpp::Time & stamp,
  const std::string & ns, int32_t id, const geometry_msgs::msg::Vector3 & scale,
  const std_msgs::msg::ColorRGBA & color, const bool & separate);

/**
 * @brief create marker array with marker type ARROW from two points
 * @param [in] point_start start point of the arrow
 * @param [in] point_end end point of the arrow
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] color color of the marker
 * @return marker array of ARROW marker
 */
visualization_msgs::msg::MarkerArray create_autoware_geometry_marker_array(
  const geometry_msgs::msg::Point & point_start, const geometry_msgs::msg::Point & point_end,
  const rclcpp::Time & stamp, const std::string & ns, const int64_t id,
  const std_msgs::msg::ColorRGBA & color);

/**
 * @brief create marker array from pose (drawing yaw line)
 * @param [in] pose center of yaw line (pose)
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @return marker array of yaw line marker
 */
visualization_msgs::msg::MarkerArray create_autoware_geometry_marker_array(
  const geometry_msgs::msg::Pose & pose, const rclcpp::Time & stamp, const std::string & ns,
  const int64_t id, const geometry_msgs::msg::Vector3 & scale,
  const std_msgs::msg::ColorRGBA & color);

/**
 * @brief Make a marker of BasicLineString2d
 * @param [in] ls BasicLineString2d
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * return marker of BasicLineString2d
 */
visualization_msgs::msg::Marker create_lanelet_linestring_marker(
  const lanelet::BasicLineString2d & ls, const rclcpp::Time & stamp, const std::string & ns,
  int32_t id, const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color,
  const double z);

/**
 * @brief return marker array from lanelets
 * @details This function creates a marker array from lanelets, draw the lanelets either as triangle
 * marker or boundary as marker.
 * @param [in] lanelets lanelets to create markers from
 * @param [in] color color of the marker
 * @param [in] ns namespace of the marker
 * @return marker array of the lanelets
 */
visualization_msgs::msg::MarkerArray create_lanelets_marker_array(
  const lanelet::ConstLanelets & lanelets, const std::string & ns,
  const geometry_msgs::msg::Vector3 scale, const std_msgs::msg::ColorRGBA & color,
  const double z = 0.0, const bool planning = false);

/**
 * @brief create marker array from MultiLineString2d (group of LineString2d)
 * @param [in] mls MultiLineString2d (group of LineString2d)
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @return marker array of the MultiLineString2d
 */
visualization_msgs::msg::MarkerArray create_lanelet_linestring_marker_array(
  const autoware_utils::MultiLineString2d & mls, const rclcpp::Time & stamp, const std::string & ns,
  int32_t id, const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color,
  const double z);

/**
 * @brief create marker array from lanelet polygon (CompoundPolygon3d)
 * @param [in] polygon lanelet polygon
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] color color of the marker
 * @return marker array of the lanelet polygon
 */
visualization_msgs::msg::MarkerArray create_lanelet_polygon_marker_array(
  const lanelet::CompoundPolygon3d & polygon, const rclcpp::Time & stamp, const std::string & ns,
  int32_t id, const std_msgs::msg::ColorRGBA & color);

/**
 * @brief create marker array from lanelet BasicPolygon2d
 * @param [in] polygons lanelet BasicPolygon2d
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @return marker array of the lanelet BasicPolygon2d
 */
visualization_msgs::msg::MarkerArray create_lanelet_polygon_marker_array(
  const lanelet::BasicPolygon2d & polygon, const rclcpp::Time & stamp, const std::string & ns,
  int32_t id, const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color,
  double z);

/**
 * @brief create marker array from lanelet BasicPolygons2d (vector of BasicPolygon2d)
 * @param [in] polygons lanelet BasicPolygons2d (vector of BasicPolygon2d)
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] marker_type type of the marker (LINE_LIST or LINE_STRIP)
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @return marker array of the lanelet BasicPolygons2d (vector of BasicPolygon2d)
 */
visualization_msgs::msg::MarkerArray create_lanelet_polygon_marker_array(
  const lanelet::BasicPolygons2d & polygons, const rclcpp::Time & stamp, const std::string & ns,
  int32_t id, uint32_t marker_type, const geometry_msgs::msg::Vector3 & scale,
  const std_msgs::msg::ColorRGBA & color, double z = 0.0);

/**
 * @brief create marker array for polygon info from lanelet ConstPolygons3d (from Regulatory
 * Element) for Optional StopLine
 * @param [in] reg_elem_areas lanelet ConstPolygons3d (from Regulatory Element)
 * @param [in] stop_line stop line (from Regulatory Element)
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @return marker array of the lanelet ConstPolygons3d (from Regulatory Element)
 */
visualization_msgs::msg::MarkerArray create_lanelet_polygon_info_marker_array(
  const lanelet::ConstPolygons3d & reg_elem_areas,
  const boost::optional<lanelet::ConstLineString3d> & stop_line, const rclcpp::Time & stamp,
  const std::string & ns_prefix, int32_t id, const geometry_msgs::msg::Vector3 & scale,
  const std_msgs::msg::ColorRGBA & color);

/**
 * @brief create marker array for polygon info from lanelet ConstPolygons3d (from Regulatory
 * Element) for Mandatory Stopline
 * @param [in] reg_elem_areas lanelet ConstPolygons3d (from Regulatory Element)
 * @param [in] stop_line stop line (from Regulatory Element)
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @return marker array of the lanelet ConstPolygons3d (from Regulatory Element)
 */
visualization_msgs::msg::MarkerArray create_lanelet_polygon_info_marker_array(
  const lanelet::ConstPolygons3d & reg_elem_areas, const lanelet::ConstLineString3d & stop_line,
  const rclcpp::Time & stamp, const std::string & ns_prefix, int32_t id,
  const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color);

/**
 * @brief create marker array from predicted object
 * @details This function creates a marker array from a PredictedObjects object
 * (initial_pose_with_covariance), draw the vehicle based on initial pose.
 * @param [in] objects PredictedObjects object
 * @param [in] stamp current time
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] color color of the marker
 * @return marker array of the boost MultiPolygon2d (Pull over area)
 */
visualization_msgs::msg::MarkerArray create_predicted_objects_marker_array(
  const autoware_perception_msgs::msg::PredictedObjects & objects, const rclcpp::Time & stamp,
  const std::string & ns, const int64_t id, const std_msgs::msg::ColorRGBA & color);

/**
 * @brief create predicted path marker array from PredictedPath
 * @param [in] predicted_path PredictedPath object
 * @param [in] vehicle_info vehicle information to calculate footprint
 * @param [in] ns namespace for the marker
 * @param [in] id id of the marker
 * @param [in] color color of the marker
 * @return marker array of the predicted path
 */
visualization_msgs::msg::MarkerArray create_predicted_path_marker_array(
  const autoware_perception_msgs::msg::PredictedPath & predicted_path,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const std::string & ns,
  const int32_t & id, const std_msgs::msg::ColorRGBA & color);

/**
 * @brief create marker array from predicted object (tier4 msg or internal planning msg)
 * @param [in] path PathWithLaneId object
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] now current time
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @param [in] with_text if true, add text to the marker
 * @return marker array of the boost MultiPolygon2d (Pull over area)
 */
visualization_msgs::msg::MarkerArray create_path_with_lane_id_marker_array(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path, const std::string & ns,
  const int64_t id, const rclcpp::Time & now, const geometry_msgs::msg::Vector3 scale,
  const std_msgs::msg::ColorRGBA & color, const bool with_text);

/**
 * @brief create a vehicle trajectory point marker array object
 * @param [in] mpt_traj trajectory points to create markers from
 * @param [in] vehicle_info vehicle information to calculate footprint
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @return visualization_msgs::msg::MarkerArray
 */
visualization_msgs::msg::MarkerArray create_vehicle_trajectory_point_marker_array(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & mpt_traj,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const std::string & ns,
  const int32_t id);

}  // namespace autoware::experimental::marker_utils

#endif  // AUTOWARE__MARKER_UTILS__MARKER_CONVERSION_HPP_
