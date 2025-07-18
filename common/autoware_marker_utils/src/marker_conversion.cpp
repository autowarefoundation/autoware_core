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
#include <autoware_lanelet2_extension/visualization/visualization.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_path.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <boost/geometry/algorithms/centroid.hpp>
#include <boost/geometry/strategies/cartesian/centroid_bashein_detmer.hpp>

#include <lanelet2_core/primitives/CompoundPolygon.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

namespace autoware::experimental::marker_utils
{
// shorten type
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Vector3;
using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

using autoware_utils_visualization::create_default_marker;
using autoware_utils_visualization::create_marker_color;
using autoware_utils_visualization::create_marker_position;
using autoware_utils_visualization::create_marker_scale;

// ===================== Helper Function ==================================

lanelet::BasicPoint3d get_centroid_point(const lanelet::BasicPolygon3d & poly)
{
  lanelet::BasicPoint3d p_sum{0.0, 0.0, 0.0};
  for (const auto & p : poly) {
    p_sum += p;
  }
  return p_sum / poly.size();
}

Point to_msg(const lanelet::BasicPoint3d & point)
{
  Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
  return msg;
}

geometry_msgs::msg::Point32 to_geom_msg_pt32(const Eigen::Vector3d & src)
{
  geometry_msgs::msg::Point32 dst;

  dst.x = static_cast<float>(src.x());
  dst.y = static_cast<float>(src.y());
  dst.z = static_cast<float>(src.z());

  return dst;
}

geometry_msgs::msg::Polygon to_geom_msg_poly(const lanelet::ConstPolygon3d & ll_poly)
{
  geometry_msgs::msg::Polygon geom_poly;
  geom_poly.points.clear();
  geom_poly.points.reserve(ll_poly.size());
  for (const auto & ll_pt : ll_poly) {
    geometry_msgs::msg::Point32 geom_pt32;
    geom_pt32 = to_geom_msg_pt32(ll_pt.basicPoint());
    geom_poly.points.push_back(geom_pt32);
  }
  return geom_poly;
}

void check_marker_type_line(uint32_t marker_type)
{
  if (marker_type != Marker::LINE_STRIP && marker_type != Marker::LINE_LIST) {
    throw std::runtime_error(
      "Unsupported marker type: only LINE_STRIP and LINE_LIST are supported.");
  }
}

inline int64_t bitShift(int64_t original_id)
{
  return original_id << (sizeof(int32_t) * 8 / 2);
}

Marker create_autoware_geometry_marker(
  const autoware_utils_geometry::Polygon2d & polygon, const rclcpp::Time & stamp,
  const std::string & ns, int32_t id, uint32_t marker_type, const Vector3 & scale,
  const ColorRGBA & color, double z)
{
  check_marker_type_line(marker_type);
  Marker marker = create_default_marker("map", stamp, ns, id, marker_type, scale, color);

  if (marker_type == Marker::LINE_LIST) {
    for (size_t i = 0; i < polygon.outer().size(); ++i) {
      const auto & cur = polygon.outer().at(i);
      const auto & nxt = polygon.outer().at((i + 1) % polygon.outer().size());
      Point p1, p2;
      p1.x = cur.x();
      p1.y = cur.y();
      p1.z = z;
      p2.x = nxt.x();
      p2.y = nxt.y();
      p2.z = z;
      marker.points.push_back(p1);
      marker.points.push_back(p2);
    }
  } else if (marker_type == Marker::LINE_STRIP) {
    marker.pose.orientation = autoware_utils_visualization::create_marker_orientation(0, 0, 0, 1.0);
    for (const auto & p : polygon.outer()) {
      Point pt;
      pt.x = p.x();
      pt.y = p.y();
      pt.z = z;
      marker.points.push_back(pt);
    }
  }
  return marker;
}

static void create_vehicle_footprint_marker(
  Marker & marker, const geometry_msgs::msg::Pose & pose, const double & base_to_right,
  const double & base_to_left, const double & base_to_front, const double & base_to_rear)
{
  marker.points.push_back(
    autoware_utils_geometry::calc_offset_pose(pose, base_to_front, base_to_left, 0.0).position);
  marker.points.push_back(
    autoware_utils_geometry::calc_offset_pose(pose, base_to_front, -base_to_right, 0.0).position);
  marker.points.push_back(
    autoware_utils_geometry::calc_offset_pose(pose, -base_to_rear, -base_to_right, 0.0).position);
  marker.points.push_back(
    autoware_utils_geometry::calc_offset_pose(pose, -base_to_rear, base_to_left, 0.0).position);
  marker.points.push_back(marker.points.front());
}

static std::vector<double> calc_path_arc_length_array(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  std::vector<double> out;
  if (path.points.empty()) return out;

  out.reserve(path.points.size());
  double sum = 0.0;
  out.push_back(sum);

  for (size_t i = 1; i < path.points.size(); ++i) {
    sum += autoware_utils_geometry::calc_distance2d(
      path.points.at(i).point, path.points.at(i - 1).point);
    out.push_back(sum);
  }
  return out;
}

Marker create_linestring_marker(
  const autoware_utils::LineString2d & ls, const rclcpp::Time & stamp, const std::string & ns,
  int32_t id, const Vector3 & scale, const ColorRGBA & color, const double z)
{
  Marker marker = create_default_marker("map", stamp, ns, id, Marker::LINE_STRIP, scale, color);
  for (const auto & point : ls) {
    Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = z;
    marker.points.push_back(p);
  }
  return marker;
}

Marker create_basiclinestring_marker(
  const lanelet::BasicLineString2d & ls, const rclcpp::Time & stamp, const std::string & ns,
  int32_t id, const Vector3 & scale, const ColorRGBA & color, const double z)
{
  Marker marker = create_default_marker("map", stamp, ns, id, Marker::LINE_LIST, scale, color);
  Point p1, p2;
  p1.z = p2.z = z;
  for (auto i = 0UL; i + 1 < ls.size(); ++i) {
    p1.x = ls[i].x();
    p1.y = ls[i].y();
    p2.x = ls[i + 1].x();
    p2.y = ls[i + 1].y();
    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }
  return marker;
}

// ====================== Main Function ===================================

MarkerArray create_autoware_geometry_marker_array(
  const geometry_msgs::msg::Polygon & polygon, const rclcpp::Time & stamp, const std::string & ns,
  int32_t id, uint32_t marker_type, const Vector3 & scale, const ColorRGBA & color)
{
  check_marker_type_line(marker_type);

  MarkerArray marker_array;
  auto marker = create_default_marker("map", stamp, ns, id, marker_type, scale, color);

  // previous set marker_lifetime value = 0.3

  const auto & points = polygon.points;
  const size_t N = points.size();

  if (marker_type == Marker::LINE_LIST) {
    marker.points.reserve(2 * N);
    for (size_t i = 0; i < N; ++i) {
      const auto & cur = points[i];
      const auto & nxt = points[(i + 1) % N];
      Point p1, p2;
      p1.x = cur.x;
      p1.y = cur.y;
      p1.z = cur.z;
      p2.x = nxt.x;
      p2.y = nxt.y;
      p2.z = nxt.z;
      marker.points.push_back(p1);
      marker.points.push_back(p2);
    }
  } else if (marker_type == Marker::LINE_STRIP) {
    marker.points.reserve(N + 1);
    for (const auto & p : points) {
      Point point;
      point.x = p.x;
      point.y = p.y;
      point.z = p.z;
      marker.points.push_back(point);
    }
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
    }
  } else {
    // return null marker array, but should not have this case
    return marker_array;
  }

  marker_array.markers.push_back(marker);
  return marker_array;
}

MarkerArray create_autoware_geometry_marker_array(
  const autoware_utils_geometry::MultiPolygon2d & area_polygons, const rclcpp::Time & stamp,
  const std::string & ns, int32_t & id, uint32_t marker_type, const Vector3 & scale,
  const ColorRGBA & color, double z, bool running_id)
{
  MarkerArray marker_array;

  // pass reference to input id
  int32_t & uid = id;

  for (size_t i = 0; i < area_polygons.size(); ++i) {
    const auto marker = create_autoware_geometry_marker(
      area_polygons[i], stamp, ns, uid, marker_type, scale, color, z);
    marker_array.markers.push_back(marker);

    if (running_id) ++uid;
  }
  return marker_array;
}

MarkerArray create_autoware_geometry_marker_array(
  const autoware_utils_geometry::MultiPolygon2d & multi_polygon, const size_t & trajectory_index,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const rclcpp::Time & stamp, const std::string & ns, int32_t id, uint32_t marker_type,
  const Vector3 & scale, const ColorRGBA & color)
{
  MarkerArray marker_array;
  auto marker = create_default_marker("map", stamp, ns, id, marker_type, scale, color);

  for (const auto & polygon : multi_polygon) {
    marker.points.push_back(trajectory[trajectory_index].pose.position);
    const auto centroid =
      boost::geometry::return_centroid<autoware_utils_geometry::Point2d>(polygon);
    marker.points.push_back(Point().set__x(centroid.x()).set__y(centroid.y()));
  }
  marker_array.markers.push_back(marker);
  return marker_array;
}

MarkerArray create_autoware_geometry_marker_array(
  const Point & stop_obstacle_point, const rclcpp::Time & stamp, const std::string & ns, int32_t id,
  uint32_t marker_type, const Vector3 & scale, const ColorRGBA & color)
{
  MarkerArray marker_array;
  auto marker = create_default_marker("map", stamp, ns, id, marker_type, scale, color);

  marker.pose.position = stop_obstacle_point;
  marker.pose.position.z += 2.0;

  marker.text = "!";
  marker_array.markers.push_back(marker);
  return marker_array;
}

MarkerArray create_autoware_geometry_marker_array(
  const std::vector<Point> & points, const rclcpp::Time & stamp, const std::string & ns, int32_t id,
  const Vector3 & scale, const ColorRGBA & color, const bool & separate)
{
  MarkerArray marker_array;
  auto marker = create_default_marker("map", stamp, ns, id, Marker::SPHERE, scale, color);

  // previous set marker_lifetime value = 0.3

  if (separate) {
    // Put each point to each marker
    for (size_t i = 0; i < points.size(); ++i) {
      marker.id = static_cast<int32_t>(i + bitShift(id));
      marker.pose.position = points.at(i);
      marker_array.markers.push_back(marker);
    }
  } else {
    // Put all points in one marker
    for (const auto & point : points) {
      marker.points.push_back(point);
    }
    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

MarkerArray create_autoware_geometry_marker_array(
  const autoware_utils_geometry::LinearRing2d & ring, const rclcpp::Time & stamp,
  const std::string & ns, int32_t id, uint32_t marker_type, const Vector3 & scale,
  const ColorRGBA & color)
{
  MarkerArray marker_array;
  auto marker = create_default_marker("map", stamp, ns, id, marker_type, scale, color);
  // previous set marker_lifetime value = 2.5

  for (size_t i = 0; i < ring.size(); ++i) {
    Point pt;
    pt.x = ring[i][0];
    pt.y = ring[i][1];
    pt.z = 0.0;
    marker.points.push_back(pt);
  }

  if (!marker.points.empty()) {
    marker.points.push_back(marker.points.front());
  }

  marker_array.markers.push_back(marker);

  return marker_array;
}

MarkerArray create_autoware_geometry_marker_array(
  const Point & point_start, const Point & point_end, const rclcpp::Time & stamp,
  const std::string & ns, const int64_t id, const ColorRGBA & color)
{
  MarkerArray marker_array;
  Vector3 scale;
  scale.x = 1.0;
  scale.y = 1.0;
  scale.z = 1.0;
  Marker marker =
    create_default_marker("map", stamp, ns + "_line", id, Marker::ARROW, scale, color);

  marker.points.push_back(point_start);
  marker.points.push_back(point_end);

  marker_array.markers.push_back(marker);
  return marker_array;
}

MarkerArray create_autoware_geometry_marker_array(
  const geometry_msgs::msg::Pose & pose, const rclcpp::Time & stamp, const std::string & ns,
  const int64_t id, const Vector3 & scale, const ColorRGBA & color)
{
  MarkerArray marker_array;

  Marker marker_line =
    create_default_marker("map", stamp, ns + "_line", id, Marker::LINE_STRIP, scale, color);

  const double yaw = tf2::getYaw(pose.orientation);

  const double a = 3.0;
  Point p0;
  p0.x = pose.position.x - a * std::sin(yaw);
  p0.y = pose.position.y + a * std::cos(yaw);
  p0.z = pose.position.z;
  marker_line.points.push_back(p0);

  Point p1;
  p1.x = pose.position.x + a * std::sin(yaw);
  p1.y = pose.position.y - a * std::cos(yaw);
  p1.z = pose.position.z;
  marker_line.points.push_back(p1);

  marker_array.markers.push_back(marker_line);

  return marker_array;
}

MarkerArray create_lanelets_marker_array(
  const lanelet::ConstLanelets & lanelets, const std::string & ns, const Vector3 scale,
  const ColorRGBA & color, const double z, const bool planning)
{
  if (lanelets.empty()) {
    return MarkerArray{};
  }

  MarkerArray marker_array;

  if (planning) {
    if (ns.empty()) {
      return lanelet::visualization::laneletsBoundaryAsMarkerArray(lanelets, color, false);
    } else {
      return lanelet::visualization::laneletsAsTriangleMarkerArray(ns, lanelets, color);
    }
  }

  auto marker =
    create_default_marker("map", rclcpp::Time(0), ns, 0, Marker::LINE_LIST, scale, color);

  for (const auto & ll : lanelets) {
    marker.points.clear();
    for (const auto & p : ll.polygon2d().basicPolygon()) {
      marker.points.push_back(create_marker_position(p.x(), p.y(), z + 0.5));
    }
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
    }
    marker_array.markers.push_back(marker);
    ++marker.id;
  }
  return marker_array;
}

MarkerArray create_lanelet_polygon_marker_array(
  const lanelet::CompoundPolygon3d & polygon, const rclcpp::Time & stamp, const std::string & ns,
  int32_t id, const ColorRGBA & color)
{
  MarkerArray marker_array;
  auto marker = create_default_marker(
    "map", stamp, ns, id, Marker::LINE_STRIP, create_marker_scale(0.1, 0.0, 0.0), color);
  for (const auto & p : polygon) {
    Point pt;
    pt.x = p.x();
    pt.y = p.y();
    pt.z = p.z();
    marker.points.push_back(pt);
  }
  marker_array.markers.push_back(marker);
  return marker_array;
}

MarkerArray create_lanelet_polygon_marker_array(
  const lanelet::BasicPolygon2d & polygon, const rclcpp::Time & stamp, const std::string & ns,
  int32_t id, const Vector3 & scale, const ColorRGBA & color, double z)
{
  MarkerArray marker_array;
  auto marker = create_default_marker("map", stamp, ns, id, Marker::LINE_STRIP, scale, color);

  marker.points.clear();
  for (const auto & p : polygon) {
    Point point = create_marker_position(p.x(), p.y(), z + 0.5);
    marker.points.push_back(point);
  }
  if (!marker.points.empty()) marker.points.push_back(marker.points.front());
  // Add to marker array
  marker_array.markers.push_back(marker);
  marker.id++;  // Not necessary (?)

  return marker_array;
}

MarkerArray create_lanelet_polygon_marker_array(
  const lanelet::BasicPolygons2d & polygons, const rclcpp::Time & stamp, const std::string & ns,
  int32_t id, uint32_t marker_type, const Vector3 & scale, const ColorRGBA & color, double z)
{
  check_marker_type_line(marker_type);

  MarkerArray marker_array;
  auto marker = create_default_marker("map", stamp, ns, id, marker_type, scale, color);

  if (polygons.empty()) return MarkerArray{};

  if (marker_type == Marker::LINE_LIST) {
    for (const auto & poly : polygons) {
      for (size_t i = 0; i < poly.size(); ++i) {
        const auto & cur = poly.at(i);
        const auto & nxt = poly.at((i + 1) % poly.size());
        Point p1, p2;
        p1.x = cur.x();
        p1.y = cur.y();
        p2.x = nxt.x();
        p2.y = nxt.y();
        marker.points.push_back(create_marker_position(p1.x, p1.y, 0.0));
        marker.points.push_back(create_marker_position(p2.x, p2.y, 0.0));
      }
    }
    // put all Polygons in one marker
    marker_array.markers.push_back(marker);
  } else if (marker_type == Marker::LINE_STRIP) {
    for (const auto & poly : polygons) {
      for (const auto & p : poly) {
        Point point = create_marker_position(p.x(), p.y(), z + 0.5);
        marker.points.push_back(point);
      }
      // Separate Polygon in each marker in marker array
      marker_array.markers.push_back(marker);
      marker.id++;
      marker.points.clear();
    }
  } else {
    // return null marker array, but should not have this case
    return marker_array;
  }

  return marker_array;
}

MarkerArray create_lanelet_polygon_info_marker_array(
  const lanelet::ConstPolygons3d & reg_elem_areas,
  const boost::optional<lanelet::ConstLineString3d> & stop_line, const rclcpp::Time & stamp,
  const std::string & ns_prefix, int32_t id, const Vector3 & scale, const ColorRGBA & color)
{
  MarkerArray marker_array;

  // ID
  {
    Marker marker_text = create_default_marker(
      "map", stamp, ns_prefix + "_id", static_cast<int32_t>(id), Marker::TEXT_VIEW_FACING, scale,
      color);

    for (const auto & area : reg_elem_areas) {
      const auto poly = area.basicPolygon();

      marker_text.pose.position = to_msg(poly.front());
      marker_text.pose.position.z += 2.0;
      marker_text.text = std::to_string(id);

      marker_array.markers.push_back(marker_text);
    }
  }

  // Polygon
  {
    auto marker = create_default_marker(
      "map", stamp, ns_prefix + "_polygon", static_cast<int32_t>(id), Marker::LINE_LIST, scale,
      color);

    for (const auto & area : reg_elem_areas) {
      const auto poly = area.basicPolygon();

      for (size_t i = 0; i < poly.size(); ++i) {
        const auto idx_front = i;
        const auto idx_back = (i == poly.size() - 1) ? 0 : i + 1;

        const auto & p_front = poly.at(idx_front);
        const auto & p_back = poly.at(idx_back);

        marker.points.push_back(to_msg(p_front));
        marker.points.push_back(to_msg(p_back));
      }

      marker_array.markers.push_back(marker);
    }
  }

  // Polygon to Stopline
  {
    if (stop_line) {
      const auto stop_line_center_point =
        (stop_line->front().basicPoint() + stop_line->back().basicPoint()) / 2;

      // TODO: Now Set to LINE_LIST as default,
      // but NoStoppingArea uses LINE_STRIP, consider the reason.
      auto marker = create_default_marker(
        "map", stamp, ns_prefix + "_correspondence", static_cast<int32_t>(id), Marker::LINE_LIST,
        scale, color);

      for (auto & area : reg_elem_areas) {
        const auto poly = area.basicPolygon();
        const auto centroid_point = get_centroid_point(poly);
        for (size_t i = 0; i < poly.size(); ++i) {
          marker.points.push_back(to_msg(centroid_point));
          marker.points.push_back(to_msg(stop_line_center_point));
        }
      }
      // put all centroid point of all areas msg in one marker
      marker_array.markers.push_back(marker);
    }
  }

  return marker_array;
}

MarkerArray create_lanelet_polygon_info_marker_array(
  const lanelet::ConstPolygons3d & reg_elem_areas, const lanelet::ConstLineString3d & stop_line,
  const rclcpp::Time & stamp, const std::string & ns_prefix, int32_t id, const Vector3 & scale,
  const ColorRGBA & color)
{
  // wrap stop_line with boost::optional
  return create_lanelet_polygon_info_marker_array(
    reg_elem_areas, boost::optional<lanelet::ConstLineString3d>{stop_line}, stamp, ns_prefix, id,
    scale, color);
}

MarkerArray create_predicted_objects_marker_array(
  const autoware_perception_msgs::msg::PredictedObjects & objects, const rclcpp::Time & stamp,
  const std::string & ns, const int64_t id, const ColorRGBA & color)
{
  MarkerArray marker_array;

  auto marker = create_default_marker(
    "map", stamp, ns, 0, Marker::CUBE, create_marker_scale(3.0, 1.0, 1.0), color);
  // previous set marker_lifetime value = 1.0
  int32_t uid = bitShift(id);

  for (size_t i = 0; i < objects.objects.size(); ++i) {
    const auto & object = objects.objects.at(i);
    marker.id = static_cast<int>(uid + static_cast<int32_t>(i));
    marker.pose = object.kinematics.initial_pose_with_covariance.pose;
    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

MarkerArray create_vehicle_trajectory_point_marker_array(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & mpt_traj,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const std::string & ns,
  const int32_t id)
{
  auto marker = create_default_marker(
    "map", rclcpp::Clock().now(), ns, id, Marker::LINE_STRIP, create_marker_scale(0.05, 0.0, 0.0),
    create_marker_color(0.99, 0.99, 0.2, 0.99));
  // previous set marker_lifetime value = 1.5

  const double base_to_right = (vehicle_info.wheel_tread_m / 2.0) + vehicle_info.right_overhang_m;
  const double base_to_left = (vehicle_info.wheel_tread_m / 2.0) + vehicle_info.left_overhang_m;
  const double base_to_front = vehicle_info.vehicle_length_m - vehicle_info.rear_overhang_m;
  const double base_to_rear = vehicle_info.rear_overhang_m;

  MarkerArray marker_array;
  for (size_t i = 0; i < mpt_traj.size(); ++i) {
    marker.id = i;
    marker.points.clear();

    const auto & traj_point = mpt_traj.at(i);
    create_vehicle_footprint_marker(
      marker, traj_point.pose, base_to_right, base_to_left, base_to_front, base_to_rear);
    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

MarkerArray create_predicted_path_marker_array(
  const autoware_perception_msgs::msg::PredictedPath & predicted_path,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const std::string & ns,
  const int32_t & id, const ColorRGBA & color)
{
  if (predicted_path.path.empty()) {
    return MarkerArray{};
  }

  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  const auto & path = predicted_path.path;

  Marker marker = create_default_marker(
    "map", current_time, ns, id, Marker::LINE_STRIP, create_marker_scale(0.1, 0.1, 0.1), color);
  // previous set marker_lifetime value = 1.5

  MarkerArray marker_array;
  const double half_width = -vehicle_info.vehicle_width_m / 2.0;
  const double base_to_front = vehicle_info.vehicle_length_m - vehicle_info.rear_overhang_m;
  const double base_to_rear = vehicle_info.rear_overhang_m;

  for (size_t i = 0; i < path.size(); ++i) {
    marker.id = i + id;
    marker.points.clear();

    const auto & predicted_path_pose = path.at(i);
    create_vehicle_footprint_marker(
      marker, predicted_path_pose, half_width, half_width, base_to_front, base_to_rear);
    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

MarkerArray create_path_with_lane_id_marker_array(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path, const std::string & ns,
  const int64_t id, const rclcpp::Time & now, const Vector3 scale, const ColorRGBA & color,
  const bool with_text)
{
  int32_t uid = bitShift(id);
  int32_t idx = 0;
  int32_t i = 0;
  MarkerArray msg;

  Marker marker =
    create_default_marker("map", now, ns, static_cast<int32_t>(uid), Marker::ARROW, scale, color);

  for (const auto & p : path.points) {
    marker.id = uid + i++;
    // previous set vmarker_lifetime alue = 0.3

    marker.pose = p.point.pose;

    if (std::find(p.lane_ids.begin(), p.lane_ids.end(), id) == p.lane_ids.end() && !with_text) {
      marker.color = create_marker_color(0.5, 0.5, 0.5, 0.999);
    }
    msg.markers.push_back(marker);
    if (i % 10 == 0 && with_text) {
      const auto arclength = calc_path_arc_length_array(path);
      Marker marker_text = create_default_marker(
        "map", now, ns, 0L, Marker::TEXT_VIEW_FACING, create_marker_scale(0.2, 0.1, 0.3),
        create_marker_color(1, 1, 1, 0.999));
      marker_text.id = uid + i++;
      std::stringstream ss;
      ss << std::fixed << std::setprecision(1) << "i=" << idx << "\ns=" << arclength.at(idx);
      marker_text.text = ss.str();
      msg.markers.push_back(marker_text);
    }
    ++idx;
  }
  return msg;
}

MarkerArray create_multistring_marker_array(
  const autoware_utils::MultiLineString2d & mls, const rclcpp::Time & stamp, const std::string & ns,
  int32_t id, const Vector3 & scale, const ColorRGBA & color, const double z)
{
  MarkerArray marker_array;
  int32_t uid;
  for (auto j = 0ul; j < mls.size(); ++j) {
    uid = id + j;
    auto marker = create_linestring_marker(mls[j], stamp, ns, uid, scale, color, z);
    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

}  // namespace autoware::experimental::marker_utils
