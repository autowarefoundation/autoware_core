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

#ifndef MAP_HEIGHT_FITTER_KERNEL_HPP_
#define MAP_HEIGHT_FITTER_KERNEL_HPP_

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <optional>

namespace lanelet
{
class LaneletMap;
}  // namespace lanelet

namespace autoware::map_height_fitter
{

/// \brief Build a 2D (x, y) KdTree over the horizontal projection of a point cloud.
///
/// The map_height_fitter ground-height search uses a purely horizontal (2D) distance metric, so
/// the cloud is projected onto the z = 0 plane before indexing. The returned tree is queried with
/// the same projection and its point indices map one-to-one onto \p cloud, so the original (un
/// projected) z values are recovered through those indices.
///
/// \param cloud the source point cloud (its x, y are used for indexing, z is ignored).
/// \return a KdTree built over the (x, y, 0) projection of \p cloud.
pcl::KdTreeFLANN<pcl::PointXYZ> build_pointcloud_xy_kdtree(
  const pcl::PointCloud<pcl::PointXYZ> & cloud);

/// \brief Find the ground height at (x, y) from a point-cloud map, using a prebuilt 2D KdTree.
///
/// Behavior matches the original two-pass linear scan exactly: it finds the minimum squared 2D
/// distance d2 to any cloud point, then returns the lowest z among all points whose squared 2D
/// distance is strictly less than (sqrt(d2) + 1.0)^2. The squared-distance comparison is performed
/// in double precision against the original cloud points, so the selected set is identical to the
/// original implementation. When no point qualifies (e.g. an empty cloud), \p fallback_z is
/// returned.
///
/// \param cloud the source point cloud (the z values used for the lowest-height selection).
/// \param kdtree a 2D KdTree built from the same \p cloud via build_pointcloud_xy_kdtree().
/// \param x query x coordinate (map frame).
/// \param y query y coordinate (map frame).
/// \param fallback_z the value returned when no finite height is found.
/// \return the fitted ground height, or \p fallback_z when none is found.
double get_ground_height_from_pointcloud(
  const pcl::PointCloud<pcl::PointXYZ> & cloud, const pcl::KdTreeFLANN<pcl::PointXYZ> & kdtree,
  double x, double y, double fallback_z);

/// \brief Find the ground height at (x, y) from a point-cloud map.
///
/// Convenience overload that builds the 2D KdTree from \p cloud and delegates to the prebuilt-tree
/// overload. Prefer the prebuilt-tree overload on hot paths to avoid rebuilding the tree per call.
///
/// \param cloud the source point cloud.
/// \param x query x coordinate (map frame).
/// \param y query y coordinate (map frame).
/// \param fallback_z the value returned when no finite height is found.
/// \return the fitted ground height, or \p fallback_z when none is found.
double get_ground_height_from_pointcloud(
  const pcl::PointCloud<pcl::PointXYZ> & cloud, double x, double y, double fallback_z);

/// \brief Find the ground height at (x, y) from a vector (lanelet) map.
///
/// Runs the nearest-point search on the map's point layer exactly once and returns the z value of
/// that nearest point. Returns std::nullopt when the point layer has no nearest point (e.g. an
/// empty map), letting the caller emit the "failed to get closest lanelet" warning and fall back.
/// When a nearest point exists but its z is non-finite, \p fallback_z is returned (matching the
/// original implementation, which silently fell back in that case).
///
/// \param map the lanelet map whose point layer is queried.
/// \param x query x coordinate (map frame).
/// \param y query y coordinate (map frame).
/// \param fallback_z the value returned when the nearest point's height is non-finite.
/// \return the fitted ground height, or std::nullopt when there is no nearest point.
std::optional<double> get_ground_height_from_vector_map(
  const lanelet::LaneletMap & map, double x, double y, double fallback_z);

}  // namespace autoware::map_height_fitter

#endif  // MAP_HEIGHT_FITTER_KERNEL_HPP_
