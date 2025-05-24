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

#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <rclcpp/time.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/Forward.h>

#include <string>

namespace autoware::marker_utils
{
/**
 * @brief create marker array from geometry polygon based on the marker type
 * @details if marker_type is LINE_LIST, the polygon is drawn as a line list
 *          if marker_type is LINE_STRIP, the polygon is drawn as a line strip
 * @param [in] polygon geometry polygon
 * @param [in] frame_id frame id of the marker
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] marker_type type of the marker (LINE_LIST or LINE_STRIP)
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @return marker array of the geometry polygon
 */
visualization_msgs::msg::MarkerArray create_polygon_marker_array(
  const geometry_msgs::msg::Polygon & polygon, const std::string & frame_id,
  const rclcpp::Time & stamp, const std::string & ns, int32_t id, uint32_t marker_type,
  const geometry_msgs::msg::Vector3 scale, const std_msgs::msg::ColorRGBA & color);

/**
 * @brief create marker array from boost Polygon2d
 * @param [in] area_polygon boost Polygon2d
 * @param [in] header header of the marker
 * @param [in] color color of the marker
 * @param [in] z z position of the marker
 * @return marker array of the boost Polygon2d
 */
visualization_msgs::msg::Marker create_boost_polygon_marker(
  const autoware_utils::Polygon2d & polygon, const std_msgs::msg::Header & header,
  const std::string & ns, int32_t id, uint32_t marker_type, const geometry_msgs::msg::Vector3 scale,
  const std_msgs::msg::ColorRGBA & color, double z);

/**
 * @brief create marker array from boost MultiPolygon2d (Pull over area)
 * @param [in] area_polygon boost MultiPolygon2d
 * @param [in] header header of the marker
 * @param [in] color color of the marker
 * @param [in] z z position of the marker
 * @return marker array of the boost MultiPolygon2d (Pull over area)
 */
visualization_msgs::msg::MarkerArray create_pull_over_area_marker_array(
  const autoware_utils::MultiPolygon2d & area_polygons, const std_msgs::msg::Header & header,
  const std_msgs::msg::ColorRGBA & color, double z);

/**
 * @brief create marker array from predicted object
 * @param [in] objects PredictedObjects object
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] now current time
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @return marker array of the boost MultiPolygon2d (Pull over area)
 */
visualization_msgs::msg::MarkerArray create_objects_marker_array(
  const autoware_perception_msgs::msg::PredictedObjects & objects, const std::string & ns,
  const int64_t id, const rclcpp::Time & now, const std_msgs::msg::ColorRGBA & color);

/**
 * @brief create debug footprint from goal_footprint (LinearRing2d)
 * @param [in] goal_footprint LinearRing2d goal_footprint to convert into marker
 * @return marker array of the boost LinearRing2d (goal_footprint)
 */
visualization_msgs::msg::MarkerArray visualize_debug_footprint(
  autoware_utils::LinearRing2d goal_footprint);

/**
 * @brief create marker array from predicted object
 * @param [in] path PathWithLaneId object
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] now current time
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @param [in] with_text if true, add text to the marker
 * @return marker array of the boost MultiPolygon2d (Pull over area)
 */
visualization_msgs::msg::MarkerArray create_path_marker_array(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path, const std::string & ns,
  const int64_t id, const rclcpp::Time & now, const geometry_msgs::msg::Vector3 scale,
  const std_msgs::msg::ColorRGBA & color, const bool with_text = false);

/**
 * @brief create marker array from lanelet polygon (CompoundPolygon3d or ConstPolygon3d)
 * @param [in] polygon lanelet polygon
 * @param [in] header header of the marker
 * @param [in] ns namespace
 * @param [in] color color of the marker
 * @return marker array of the lanelet polygon
 */
template <typename PolygonT>
visualization_msgs::msg::MarkerArray create_lanelet_polygon_marker_array(
  const PolygonT & polygon, const std_msgs::msg::Header & header, const std::string & ns,
  const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::MarkerArray marker_array;
  auto marker = autoware_utils::create_default_marker(
    header.frame_id, header.stamp, ns, 0, visualization_msgs::msg::Marker::LINE_STRIP,
    autoware_utils::create_marker_scale(0.1, 0.0, 0.0), color);
  for (const auto & p : polygon) {
    geometry_msgs::msg::Point pt;
    pt.x = p.x();
    pt.y = p.y();
    pt.z = p.z();
    marker.points.push_back(pt);
  }
  marker_array.markers.push_back(marker);
  return marker_array;
}
}  // namespace autoware::marker_utils

#endif  // AUTOWARE__MARKER_UTILS__MARKER_CONVERSION_HPP_
