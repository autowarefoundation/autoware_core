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

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/primitives/CompoundPolygon.h>

#include <string>

namespace autoware::marker_utils
{
visualization_msgs::msg::MarkerArray make_marker_from_polygon(
  const geometry_msgs::msg::Polygon & polygon, const std::string & frame_id,
  const rclcpp::Time & stamp, const std::string & ns, int32_t id, uint32_t marker_type,
  double scale_x, double scale_y, double scale_z, const std_msgs::msg::ColorRGBA & color);

visualization_msgs::msg::MarkerArray create_lanelet_polygon_marker_array(
  const lanelet::CompoundPolygon3d & polygon, const std_msgs::msg::Header & header,
  const std::string & ns, const std_msgs::msg::ColorRGBA & color);

visualization_msgs::msg::MarkerArray create_pull_over_area_marker_array(
  const autoware_utils::MultiPolygon2d & area_polygons, const std_msgs::msg::Header & header,
  const std_msgs::msg::ColorRGBA & color, double z);

visualization_msgs::msg::MarkerArray create_objects_marker_array(
  const autoware_perception_msgs::msg::PredictedObjects & objects, const std::string & ns,
  const int64_t module_id, const rclcpp::Time & now, const double r, const double g,
  const double b);

visualization_msgs::msg::MarkerArray create_footprint_marker_array(
  autoware_utils::LinearRing2d goal_footprint);

}  // namespace autoware::marker_utils
#endif  // AUTOWARE__MARKER_UTILS__MARKER_CONVERSION_HPP_
