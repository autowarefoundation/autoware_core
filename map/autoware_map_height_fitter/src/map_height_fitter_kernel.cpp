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

// cspell:ignore dists

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

  // A non-finite query makes every squared distance non-finite, so the original double-precision
  // scan rejected every point and fell back. Guard here as well: pcl::KdTreeFLANN asserts on a
  // NaN/Inf query point, so we must not hand one to the tree.
  if (!std::isfinite(x) || !std::isfinite(y)) {
    return fallback_z;
  }

  pcl::PointXYZ query;
  query.x = static_cast<float>(x);
  query.y = static_cast<float>(y);
  query.z = 0.0F;

  // The original computed min_dist2 as the true double-precision minimum over all points. The
  // KdTree, however, searches in float: both the query and the indexed coordinates are rounded to
  // float, so nearestKSearch can return a point that is *not* the double-precision nearest when two
  // points are near-tied or the coordinates are large (where the float grid is coarse). Trusting a
  // single returned index would therefore over- or under-estimate min_dist2 and, through the
  // radius, change the selected height. To stay bit-for-bit faithful to the original, we use the
  // KdTree only to localize the search and then recompute everything in double precision.
  //
  // Float rounding of a coordinate of magnitude up to coord_max introduces an absolute error of at
  // most coord_max * 2^-24 (round-to-nearest, half an ULP) per coordinate; the float distance
  // evaluation adds further rounding of the same order. coord_margin bounds the resulting error in
  // the (linear) distance domain with margin to spare, so any float-radius search enlarged by
  // coord_margin is a guaranteed superset of the corresponding double-precision predicate.
  std::vector<int> nn_indices;
  std::vector<float> nn_sqr_dists;
  if (kdtree.nearestKSearch(query, 1, nn_indices, nn_sqr_dists) < 1) {
    return fallback_z;
  }
  const auto & nn = cloud.points[nn_indices.front()];
  const double nn_dx = x - nn.x;
  const double nn_dy = y - nn.y;
  // Double-precision distance to the float-nearest point: an upper bound on the true min_dist2.
  const double nn_dist = std::sqrt((nn_dx * nn_dx) + (nn_dy * nn_dy));

  const double coord_max =
    std::max(std::fabs(x), std::fabs(y)) +
    std::max(std::fabs(static_cast<double>(nn.x)), std::fabs(static_cast<double>(nn.y))) + 1.0;
  const double coord_margin = (2.0 * coord_max * std::ldexp(1.0, -23)) + 1e-3;

  // Recompute the true double-precision min_dist2 over a candidate set guaranteed to contain the
  // double-nearest point: every point whose float distance is within coord_margin of the
  // float-nearest distance. This reproduces the original min_dist2 (and thus radius) exactly.
  const double nearest_search_radius = nn_dist + coord_margin;
  std::vector<int> nearest_indices;
  std::vector<float> nearest_sqr_dists;
  kdtree.radiusSearch(query, nearest_search_radius, nearest_indices, nearest_sqr_dists);

  double min_dist2 = (nn_dx * nn_dx) + (nn_dy * nn_dy);
  for (const int index : nearest_indices) {
    const auto & p = cloud.points[index];
    const double dx = x - p.x;
    const double dy = y - p.y;
    min_dist2 = std::min(min_dist2, (dx * dx) + (dy * dy));
  }

  // Lowest height within radius (sqrt(min_dist2) + 1.0).
  const double radius = std::sqrt(min_dist2) + 1.0;
  const double radius2 = radius * radius;

  // Query a margin-enlarged radius so every point the double-precision strict test would accept is
  // guaranteed to be in the candidate set despite the KdTree's internal float arithmetic; the
  // double-precision filter below then reproduces the original membership exactly.
  const double search_radius = radius + coord_margin;
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
