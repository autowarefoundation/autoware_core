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

#include <autoware/marker_utils/marker_conversion.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <lanelet2_core/primitives/CompoundPolygon.h>

#include <string>

namespace autoware::marker_utils
{
using autoware_utils::create_default_marker;
using autoware_utils::create_marker_color;
using autoware_utils::create_marker_scale;

visualization_msgs::msg::MarkerArray create_polygon_marker_array(
  const geometry_msgs::msg::Polygon & polygon, const std::string & frame_id,
  const rclcpp::Time & stamp, const std::string & ns, int32_t id, uint32_t marker_type,
  const geometry_msgs::msg::Vector3 scale, const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::MarkerArray marker_array;
  auto marker = create_default_marker(frame_id, stamp, ns, id, marker_type, scale, color);

  marker.lifetime = rclcpp::Duration::from_seconds(0.3);

  const auto & points_ = polygon.points;
  const size_t N = points_.size();

  if (marker_type == visualization_msgs::msg::Marker::LINE_LIST) {
    for (size_t i = 0; i < N; ++i) {
      const auto & cur = points_[i];
      const auto & nxt = points_[(i + 1) % N];
      geometry_msgs::msg::Point p1, p2;
      p1.x = cur.x;
      p1.y = cur.y;
      p1.z = cur.z;
      p2.x = nxt.x;
      p2.y = nxt.y;
      p2.z = nxt.z;
      marker.points.push_back(p1);
      marker.points.push_back(p2);
    }
  } else {
    for (const auto & p : points_) {
      geometry_msgs::msg::Point point;
      point.x = p.x;
      point.y = p.y;
      point.z = p.z;
      marker.points.push_back(point);
    }
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
    }
  }

  marker_array.markers.push_back(marker);
  return marker_array;
}

visualization_msgs::msg::Marker create_boost_polygon_marker(
  const autoware_utils::Polygon2d & polygon, const std_msgs::msg::Header & header,
  const std::string & ns, int32_t id, uint32_t marker_type, const geometry_msgs::msg::Vector3 scale,
  const std_msgs::msg::ColorRGBA & color, double z)
{
  visualization_msgs::msg::Marker marker =
    create_default_marker(header.frame_id, header.stamp, ns, id, marker_type, scale, color);

  if (marker_type == visualization_msgs::msg::Marker::LINE_LIST) {
    for (size_t i = 0; i < polygon.outer().size(); ++i) {
      const auto & cur = polygon.outer().at(i);
      const auto & nxt = polygon.outer().at((i + 1) % polygon.outer().size());
      geometry_msgs::msg::Point p1, p2;
      p1.x = cur.x();
      p1.y = cur.y();
      p1.z = z;
      p2.x = nxt.x();
      p2.y = nxt.y();
      p2.z = z;
      marker.points.push_back(p1);
      marker.points.push_back(p2);
    }
  } else {
    for (const auto & p : polygon.outer()) {
      geometry_msgs::msg::Point pt;
      pt.x = p.x();
      pt.y = p.y();
      pt.z = z;
      marker.points.push_back(pt);
    }
  }

  return marker;
}

visualization_msgs::msg::MarkerArray create_pull_over_area_marker_array(
  const autoware_utils::MultiPolygon2d & area_polygons, const std_msgs::msg::Header & header,
  const std_msgs::msg::ColorRGBA & color, double z)
{
  visualization_msgs::msg::MarkerArray marker_array;

  for (size_t i = 0; i < area_polygons.size(); ++i) {
    const auto marker = create_boost_polygon_marker(
      area_polygons[i], header, "pull_over_area_" + std::to_string(i), static_cast<int32_t>(i),
      visualization_msgs::msg::Marker::LINE_STRIP, create_marker_scale(0.1f, 0.0f, 0.0f), color, z);

    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

visualization_msgs::msg::MarkerArray create_objects_marker_array(
  const autoware_perception_msgs::msg::PredictedObjects & objects, const std::string & ns,
  const int64_t id, const rclcpp::Time & now, const double r, const double g, const double b)
{
  visualization_msgs::msg::MarkerArray marker_array;

  auto marker = create_default_marker(
    "map", now, ns, 0, visualization_msgs::msg::Marker::CUBE, create_marker_scale(3.0, 1.0, 1.0),
    create_marker_color(static_cast<float>(r), static_cast<float>(g), static_cast<float>(b), 0.8f));
  marker.lifetime = rclcpp::Duration::from_seconds(1.0);

  for (size_t i = 0; i < objects.objects.size(); ++i) {
    const auto & object = objects.objects.at(i);
    marker.id = static_cast<int>((id << (sizeof(int32_t) * 8 / 2)) + static_cast<int32_t>(i));
    marker.pose = object.kinematics.initial_pose_with_covariance.pose;
    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

visualization_msgs::msg::MarkerArray visualize_debug_footprint(
  autoware_utils::LinearRing2d goal_footprint)
{
  visualization_msgs::msg::MarkerArray msg;
  auto marker = autoware_utils::create_default_marker(
    "map", rclcpp::Clock().now(), "goal_footprint", 0, visualization_msgs::msg::Marker::LINE_STRIP,
    autoware_utils::create_marker_scale(0.05, 0.0, 0.0),
    autoware_utils::create_marker_color(0.99, 0.99, 0.2, 1.0));
  marker.lifetime = rclcpp::Duration::from_seconds(2.5);

  for (size_t i = 0; i < goal_footprint.size(); ++i) {
    geometry_msgs::msg::Point pt;
    pt.x = goal_footprint[i][0];
    pt.y = goal_footprint[i][1];
    pt.z = 0.0;
    marker.points.push_back(pt);
  }

  if (!marker.points.empty()) {
    marker.points.push_back(marker.points.front());
  }

  msg.markers.push_back(marker);

  return msg;
}
}  // namespace autoware::marker_utils
