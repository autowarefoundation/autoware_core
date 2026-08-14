// Copyright 2026 Autoware Foundation
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

#ifndef HARNESS__STIMULUS_HPP_
#define HARNESS__STIMULUS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <string>

namespace ndt_test
{

inline constexpr const char * map_frame = "map";
inline constexpr const char * base_link_frame = "base_link";
inline constexpr const char * sensor_frame = "sensor_frame";
/// The child frame the node broadcasts its result on. Matches `frame.ndt_base_frame`, which the
/// converged cases pin explicitly.
inline constexpr const char * ndt_base_link_frame = "ndt_base_link";

/// @brief Diagnostic status names published by the node under test.
inline constexpr const char * scan_matching_status = "ndt_scan_matcher: scan_matching_status";
inline constexpr const char * initial_pose_status =
  "ndt_scan_matcher: initial_pose_subscriber_status";
inline constexpr const char * map_update_status = "ndt_scan_matcher: map_update_status";
inline constexpr const char * ndt_align_status = "ndt_scan_matcher: ndt_align_service_status";

/// @brief The map is served by `StubMapLoader` centered here, so all valid poses sit at (100, 100).
inline constexpr double map_center_x = 100.0;
inline constexpr double map_center_y = 100.0;

// ------------------------------------------------------------------------------- world geometry

/// @brief Edge length [m] of the box whose corner is the only shape in this world.
inline constexpr double box_edge_length = 20.0;

/// @brief Point spacing [m] of the map served by `StubMapLoader`.
inline constexpr double map_spacing = 0.2;

/// @brief Point spacing [m] of a scan. An exact multiple of `map_spacing`, so every scan point
/// coincides with a map point once the scan is placed at the map center.
inline constexpr double scan_spacing = 1.0;

/// @brief Three faces of a box meeting at one corner — the single shape this suite is built from.
///
/// Both the scan and the map are this cloud: the scan at the origin of `base_link`, the map
/// translated to (`map_center_x`, `map_center_y`). One generator for both is what makes them lie
/// on the same surfaces; two hand-written clouds would drift apart without any test noticing.
///
///        z                    xy face:  z = 0,  x, y in [0, edge]
///        |  yz                yz face:  x = 0,  y, z in [0, edge]
///        | /|                 zx face:  y = 0,  x, z in [0, edge]
///     zx |/ |
///        +--+---- y           Nothing sits in a negative octant, so the farthest point is a
///       /  xy                 face corner at ~28.3 m — well above the 10 m near-field gate.
///      x
///
/// The faces are emitted one after another and share their three edges, so the edge points are
/// duplicated. That is harmless for NDT and matches what the map has always contained.
inline pcl::PointCloud<pcl::PointXYZ> make_corner_cloud(
  const double spacing, const double offset_x = 0.0, const double offset_y = 0.0)
{
  const auto interval = static_cast<float>(spacing);
  const auto points_per_line = static_cast<int>(std::lround(box_edge_length / spacing)) + 1;
  const auto base_x = static_cast<float>(offset_x);
  const auto base_y = static_cast<float>(offset_y);

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.points.reserve(static_cast<size_t>(3 * points_per_line * points_per_line));
  for (int face = 0; face < 3; ++face) {
    for (int i = 0; i < points_per_line; ++i) {
      for (int j = 0; j < points_per_line; ++j) {
        // `u` and `v` sweep the face currently being emitted, each from 0 to `box_edge_length`.
        // Which two world axes they land on is the only thing that differs between the faces; the
        // remaining axis stays at the cloud's own corner, which is what keeps all three planar and
        // meeting at that corner.
        const float u = interval * static_cast<float>(j);
        const float v = interval * static_cast<float>(i);
        if (face == 0) {
          cloud.points.emplace_back(base_x + u, base_y + v, 0.0F);  // xy: u->x, v->y, z fixed
        } else if (face == 1) {
          cloud.points.emplace_back(base_x, base_y + u, v);  // yz: u->y, v->z, x fixed
        } else {
          cloud.points.emplace_back(base_x + u, base_y, v);  // zx: u->x, v->z, y fixed
        }
      }
    }
  }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  return cloud;
}

// -------------------------------------------------------------------------------------- stimulus

/// @brief A normal scan stamped at `stamp`: the corner cloud at 1 m spacing, 1,323 points.
inline sensor_msgs::msg::PointCloud2 make_scan_at(
  const builtin_interfaces::msg::Time & stamp, const std::string & frame_id = sensor_frame)
{
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(make_corner_cloud(scan_spacing), cloud);
  cloud.header.stamp = stamp;
  cloud.header.frame_id = frame_id;
  return cloud;
}

/// @brief A well-formed but point-less cloud (`width == 0`).
///
/// The node reads `msg->width` and returns before `pcl::fromROSMsg`, so the payload is never
/// parsed; only the header and `width` matter.
inline sensor_msgs::msg::PointCloud2 make_empty_scan(
  const builtin_interfaces::msg::Time & stamp, const std::string & frame_id = sensor_frame)
{
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(pcl::PointCloud<pcl::PointXYZ>{}, cloud);
  cloud.width = 0;
  cloud.height = 1;
  cloud.row_step = 0;
  cloud.data.clear();
  cloud.header.stamp = stamp;
  cloud.header.frame_id = frame_id;
  return cloud;
}

/// @brief Eight points on the corners of a 1 m cube, so the farthest is ~0.866 m.
///
/// That is below `sensor_points.required_distance` (10 m), which is what the near-field gate
/// rejects.
inline sensor_msgs::msg::PointCloud2 make_near_field_scan(
  const builtin_interfaces::msg::Time & stamp, const std::string & frame_id = sensor_frame)
{
  pcl::PointCloud<pcl::PointXYZ> pcd;
  for (const float x : {-0.5F, 0.5F}) {
    for (const float y : {-0.5F, 0.5F}) {
      for (const float z : {-0.5F, 0.5F}) {
        pcd.points.emplace_back(x, y, z);
      }
    }
  }
  pcd.width = pcd.points.size();
  pcd.height = 1;
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(pcd, cloud);
  cloud.header.stamp = stamp;
  cloud.header.frame_id = frame_id;
  return cloud;
}

/// @brief An EKF-like initial pose, with an isotropic xy variance.
///
/// `variance_xy` reaches two different consumers. `SmartPoseBuffer::interpolate` copies the *older*
/// pose's covariance into its result verbatim, so a pair with differing values makes that choice
/// observable; and `align_pose` takes the square roots of the covariance diagonal as the standard
/// deviations of its initial-pose search, so the value decides how widely `ndt_align_srv` searches.
inline geometry_msgs::msg::PoseWithCovarianceStamped make_pose_at(
  const builtin_interfaces::msg::Time & stamp, const double x, const double y,
  const std::string & frame_id = map_frame, const double variance_xy = 0.25)
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose{};
  pose.header.stamp = stamp;
  pose.header.frame_id = frame_id;
  pose.pose.pose.position.x = x;
  pose.pose.pose.position.y = y;
  pose.pose.pose.position.z = 0.0;
  pose.pose.pose.orientation.w = 1.0;
  pose.pose.covariance[0] = variance_xy;
  pose.pose.covariance[7] = variance_xy;
  pose.pose.covariance[14] = 0.0025;
  pose.pose.covariance[21] = 0.0006853891909122467;
  pose.pose.covariance[28] = 0.0006853891909122467;
  pose.pose.covariance[35] = 0.06853891909122467;
  return pose;
}

}  // namespace ndt_test

#endif  // HARNESS__STIMULUS_HPP_
