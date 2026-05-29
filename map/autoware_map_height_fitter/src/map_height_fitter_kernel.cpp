// Copyright 2026 The Autoware Contributors
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

#include "autoware/map_height_fitter/map_height_fitter_kernel.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Point.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

namespace autoware::map_height_fitter
{

pcl::KdTreeFLANN<pcl::PointXYZ> build_pointcloud_xy_kdtree(
  const pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  if (cloud.points.empty()) {
    return kdtree;
  }

  // Project onto the z = 0 plane so the KdTree uses a purely horizontal (x, y) distance metric,
  // matching the original 2D nearest-point search. Point indices are preserved one-to-one with the
  // source cloud so the original z values can be recovered through them.
  auto projected = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  projected->points.reserve(cloud.points.size());
  for (const auto & p : cloud.points) {
    projected->points.emplace_back(p.x, p.y, 0.0F);
  }
  projected->width = static_cast<std::uint32_t>(projected->points.size());
  projected->height = 1;
  kdtree.setInputCloud(projected);
  return kdtree;
}

double get_ground_height_from_pointcloud(
  const pcl::PointCloud<pcl::PointXYZ> & cloud, const pcl::KdTreeFLANN<pcl::PointXYZ> & kdtree,
  double x, double y, double fallback_z)
{
  if (cloud.points.empty()) {
    return fallback_z;
  }

  pcl::PointXYZ query;
  query.x = static_cast<float>(x);
  query.y = static_cast<float>(y);
  query.z = 0.0F;

  // Find the closest point's squared 2D distance, then recompute it in double precision against the
  // original point so the radius matches the original implementation bit-for-bit.
  std::vector<int> nn_indices;
  std::vector<float> nn_sqr_dists;
  if (kdtree.nearestKSearch(query, 1, nn_indices, nn_sqr_dists) < 1) {
    return fallback_z;
  }
  const auto & nearest = cloud.points[nn_indices.front()];
  const double ndx = x - nearest.x;
  const double ndy = y - nearest.y;
  const double min_dist2 = (ndx * ndx) + (ndy * ndy);

  // Lowest height within radius (sqrt(min_dist2) + 1.0).
  const double radius = std::sqrt(min_dist2) + 1.0;
  const double radius2 = radius * radius;

  // Query a slightly enlarged radius so every point the double-precision strict test would accept
  // is guaranteed to be in the candidate set despite the KdTree's internal float arithmetic; the
  // double-precision filter below then reproduces the original membership exactly.
  const double search_radius = radius + (1e-3 + (1e-6 * radius));
  std::vector<int> radius_indices;
  std::vector<float> radius_sqr_dists;
  kdtree.radiusSearch(query, search_radius, radius_indices, radius_sqr_dists);

  double height = std::numeric_limits<double>::infinity();
  for (const int index : radius_indices) {
    const auto & p = cloud.points[index];
    const double dx = x - p.x;
    const double dy = y - p.y;
    const double sd = (dx * dx) + (dy * dy);
    if (sd < radius2) {
      height = std::min(height, static_cast<double>(p.z));
    }
  }

  return std::isfinite(height) ? height : fallback_z;
}

double get_ground_height_from_pointcloud(
  const pcl::PointCloud<pcl::PointXYZ> & cloud, double x, double y, double fallback_z)
{
  const auto kdtree = build_pointcloud_xy_kdtree(cloud);
  return get_ground_height_from_pointcloud(cloud, kdtree, x, y, fallback_z);
}

std::optional<double> get_ground_height_from_vector_map(
  const lanelet::LaneletMap & map, double x, double y, double fallback_z)
{
  // Run the nearest-point search exactly once. The empty case is reported as std::nullopt so the
  // caller can emit the "failed to get closest lanelet" warning without a second lookup.
  const auto closest_points = map.pointLayer.nearest(lanelet::BasicPoint2d{x, y}, 1);
  if (closest_points.empty()) {
    return std::nullopt;
  }
  const double height = closest_points.front().z();
  return std::isfinite(height) ? height : fallback_z;
}

}  // namespace autoware::map_height_fitter
