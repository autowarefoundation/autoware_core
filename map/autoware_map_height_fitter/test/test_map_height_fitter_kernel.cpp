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

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Point.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <random>
#include <vector>

namespace
{
using autoware::map_height_fitter::get_ground_height_from_pointcloud;
using autoware::map_height_fitter::get_ground_height_from_vector_map;

/// \brief Faithful re-implementation of the original (pre-KdTree) two-pass linear scan.
///
/// This is the characterization oracle: the new kernel must reproduce its output exactly. It
/// mirrors the historical src/map_height_fitter.cpp get_ground_height pointcloud branch line for
/// line.
double reference_ground_height_from_pointcloud(
  const pcl::PointCloud<pcl::PointXYZ> & cloud, double x, double y, double fallback_z)
{
  double height = std::numeric_limits<double>::infinity();

  double min_dist2 = std::numeric_limits<double>::infinity();
  for (const auto & p : cloud.points) {
    const double dx = x - p.x;
    const double dy = y - p.y;
    const double sd = (dx * dx) + (dy * dy);
    min_dist2 = std::min(min_dist2, sd);
  }

  const double radius2 = std::pow(std::sqrt(min_dist2) + 1.0, 2.0);

  for (const auto & p : cloud.points) {
    const double dx = x - p.x;
    const double dy = y - p.y;
    const double sd = (dx * dx) + (dy * dy);
    if (sd < radius2) {
      height = std::min(height, static_cast<double>(p.z));
    }
  }

  return std::isfinite(height) ? height : fallback_z;
}

pcl::PointCloud<pcl::PointXYZ> make_cloud(const std::vector<std::array<float, 3>> & pts)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.points.reserve(pts.size());
  for (const auto & p : pts) {
    cloud.points.emplace_back(p[0], p[1], p[2]);
  }
  cloud.width = static_cast<std::uint32_t>(cloud.points.size());
  cloud.height = 1;
  return cloud;
}
}  // namespace

// --- Empty-cloud fallback -----------------------------------------------------------------------

TEST(MapHeightFitterKernel, EmptyCloudReturnsFallback)
{
  const pcl::PointCloud<pcl::PointXYZ> cloud;  // empty
  EXPECT_DOUBLE_EQ(get_ground_height_from_pointcloud(cloud, 1.0, 2.0, 42.0), 42.0);
}

// --- Single-point cloud -------------------------------------------------------------------------

TEST(MapHeightFitterKernel, SinglePointReturnsThatPointHeight)
{
  // The only point is the closest; radius = sqrt(min_dist2) + 1.0 always includes it, so its z is
  // returned regardless of the query offset.
  const auto cloud = make_cloud({{{10.0F, 20.0F, 7.5F}}});
  EXPECT_DOUBLE_EQ(get_ground_height_from_pointcloud(cloud, 10.0, 20.0, -100.0), 7.5);
  EXPECT_DOUBLE_EQ(get_ground_height_from_pointcloud(cloud, 13.0, 24.0, -100.0), 7.5);
}

// --- Lowest-z-within-radius selection -----------------------------------------------------------

TEST(MapHeightFitterKernel, SelectsLowestZWithinRadius)
{
  // Query at origin. Closest point is at distance 1 (min_dist2 = 1), so radius = 2, radius2 = 4.
  //  - A: (1, 0, 5.0)   sd = 1   < 4  -> candidate, z = 5.0
  //  - B: (0, 1.5, 2.0) sd = 2.25 < 4 -> candidate, z = 2.0  (lowest within radius)
  //  - C: (3, 0, -9.0)  sd = 9   >= 4 -> excluded even though its z is far lower
  const auto cloud = make_cloud({
    {{1.0F, 0.0F, 5.0F}},
    {{0.0F, 1.5F, 2.0F}},
    {{3.0F, 0.0F, -9.0F}},
  });
  EXPECT_DOUBLE_EQ(get_ground_height_from_pointcloud(cloud, 0.0, 0.0, 100.0), 2.0);
}

TEST(MapHeightFitterKernel, FarPointOutsideRadiusIsIgnored)
{
  // Closest point at distance 10 (min_dist2 = 100), radius = 11, radius2 = 121.
  //  - near: (10, 0, 3.0)  sd = 100 < 121 -> candidate
  //  - far : (0, 12, -50)  sd = 144 >= 121 -> excluded
  const auto cloud = make_cloud({
    {{10.0F, 0.0F, 3.0F}},
    {{0.0F, 12.0F, -50.0F}},
  });
  EXPECT_DOUBLE_EQ(get_ground_height_from_pointcloud(cloud, 0.0, 0.0, 0.0), 3.0);
}

TEST(MapHeightFitterKernel, DistanceMetricIsHorizontalOnly)
{
  // z must NOT influence the nearest-point / radius computation. Point A is horizontally closest
  // (dx = 1) but high up; B is horizontally farther (dx = 1.8) but its z is lower and still inside
  // the radius. min_dist2 = 1 -> radius2 = 4; B sd = 3.24 < 4 -> included, z = -1.0 wins.
  const auto cloud = make_cloud({
    {{1.0F, 0.0F, 100.0F}},
    {{1.8F, 0.0F, -1.0F}},
  });
  EXPECT_DOUBLE_EQ(get_ground_height_from_pointcloud(cloud, 0.0, 0.0, 0.0), -1.0);
}

// --- Prebuilt-KdTree overload matches the convenience overload ----------------------------------

TEST(MapHeightFitterKernel, PrebuiltKdtreeMatchesConvenienceOverload)
{
  const auto cloud = make_cloud({
    {{1.0F, 0.0F, 5.0F}},
    {{0.0F, 1.5F, 2.0F}},
    {{3.0F, 0.0F, -9.0F}},
  });
  const auto kdtree = autoware::map_height_fitter::build_pointcloud_xy_kdtree(cloud);
  EXPECT_DOUBLE_EQ(
    get_ground_height_from_pointcloud(cloud, kdtree, 0.0, 0.0, 100.0),
    get_ground_height_from_pointcloud(cloud, 0.0, 0.0, 100.0));
}

// --- Characterization: kernel == original two-pass scan over randomized inputs -------------------

TEST(MapHeightFitterKernel, MatchesReferenceOverRandomClouds)
{
  std::mt19937 rng(20260529);
  std::uniform_real_distribution<float> coord(-50.0F, 50.0F);
  std::uniform_real_distribution<float> zdist(-20.0F, 20.0F);
  std::uniform_real_distribution<double> qdist(-60.0, 60.0);
  std::uniform_int_distribution<int> npts(1, 200);

  for (int trial = 0; trial < 300; ++trial) {
    const int n = npts(rng);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.points.reserve(n);
    for (int i = 0; i < n; ++i) {
      cloud.points.emplace_back(coord(rng), coord(rng), zdist(rng));
    }
    cloud.width = static_cast<std::uint32_t>(cloud.points.size());
    cloud.height = 1;

    const double qx = qdist(rng);
    const double qy = qdist(rng);
    const double fallback = 12345.0;

    const double expected = reference_ground_height_from_pointcloud(cloud, qx, qy, fallback);

    // Exercise the production path: Impl::get_ground_height calls the prebuilt-KdTree overload, so
    // build the tree once (as the node does) and query that overload.
    const auto kdtree = autoware::map_height_fitter::build_pointcloud_xy_kdtree(cloud);
    const double actual_prebuilt =
      get_ground_height_from_pointcloud(cloud, kdtree, qx, qy, fallback);
    EXPECT_DOUBLE_EQ(actual_prebuilt, expected)
      << "prebuilt overload, trial " << trial << " n=" << n << " q=(" << qx << "," << qy << ")";

    // The convenience overload must agree as well.
    const double actual_convenience = get_ground_height_from_pointcloud(cloud, qx, qy, fallback);
    EXPECT_DOUBLE_EQ(actual_convenience, expected)
      << "convenience overload, trial " << trial << " n=" << n << " q=(" << qx << "," << qy << ")";
  }
}

// --- Characterization at large coordinates / near-ties (float-vs-double divergence regime) -------

TEST(MapHeightFitterKernel, MatchesReferenceAtLargeCoordinates)
{
  // Autoware map frames can place points at large coordinates (MGRS/UTM-derived, up to ~1e6) where
  // the float grid spacing the KdTree searches on is coarse (sub-decimeter). Combined with
  // near-tied candidate points this is exactly where a naive float nearest/radius search diverges
  // from the original double-precision two-pass scan. The kernel must still match bit-for-bit.
  std::mt19937 rng(20260602);
  std::uniform_real_distribution<double> center(-2.0e6, 2.0e6);
  std::uniform_real_distribution<double> offset(-30.0, 30.0);
  std::uniform_real_distribution<float> zdist(-50.0F, 50.0F);
  std::uniform_int_distribution<int> npts(1, 24);

  for (int trial = 0; trial < 5000; ++trial) {
    const double cx = center(rng);
    const double cy = center(rng);
    const int n = npts(rng);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.points.reserve(n);
    for (int i = 0; i < n; ++i) {
      cloud.points.emplace_back(
        static_cast<float>(cx + offset(rng)), static_cast<float>(cy + offset(rng)), zdist(rng));
    }
    cloud.width = static_cast<std::uint32_t>(cloud.points.size());
    cloud.height = 1;

    const double qx = cx + offset(rng);
    const double qy = cy + offset(rng);
    const double fallback = 12345.0;

    const double expected = reference_ground_height_from_pointcloud(cloud, qx, qy, fallback);
    const auto kdtree = autoware::map_height_fitter::build_pointcloud_xy_kdtree(cloud);
    const double actual = get_ground_height_from_pointcloud(cloud, kdtree, qx, qy, fallback);
    EXPECT_DOUBLE_EQ(actual, expected) << "large-coord trial " << trial << " n=" << n << " center=("
                                       << cx << "," << cy << ") q=(" << qx << "," << qy << ")";
  }
}

// --- Non-finite degenerate inputs (must still match the original double-precision scan) ----------

TEST(MapHeightFitterKernel, NonFiniteCloudCoordinatesAreIgnored)
{
  // A realistic degenerate map cloud: some points carry NaN/Inf x or y (e.g. from a corrupt sensor
  // frame). The original two-pass scan tolerated such points: a NaN squared distance is dropped by
  // std::min (which keeps its first argument) and fails the strict sd < radius2 test, so the
  // non-finite points never participate in the nearest-point or lowest-z selection. The KdTree path
  // must stay bit-for-bit faithful to that behavior and still return the lowest z among the finite
  // points within the radius (here z = -3.0 at the finite point closest to the origin).
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const auto cloud = make_cloud({
    {{static_cast<float>(nan), 0.0F, 99.0F}},   // NaN x: must be ignored
    {{0.0F, static_cast<float>(inf), 88.0F}},   // Inf y: must be ignored
    {{static_cast<float>(-inf), 0.0F, 77.0F}},  // -Inf x: must be ignored
    {{1.0F, 0.0F, -3.0F}},                      // finite, nearest to origin
    {{0.0F, 1.5F, 4.0F}},                       // finite, within radius
  });
  const double fallback = 12345.0;
  const double expected = reference_ground_height_from_pointcloud(cloud, 0.0, 0.0, fallback);
  const auto kdtree = autoware::map_height_fitter::build_pointcloud_xy_kdtree(cloud);
  EXPECT_DOUBLE_EQ(get_ground_height_from_pointcloud(cloud, kdtree, 0.0, 0.0, fallback), expected);
  EXPECT_DOUBLE_EQ(get_ground_height_from_pointcloud(cloud, 0.0, 0.0, fallback), expected);
}

TEST(MapHeightFitterKernel, AllNonFiniteCloudCoordinatesReturnFallback)
{
  // When EVERY point has a non-finite coordinate, no point participates in the scan: min_dist2
  // stays +inf, radius2 is +inf, and the strict sd < radius2 test still rejects every NaN/Inf sd,
  // so the lowest-z stays +inf and the fallback is returned. The KdTree path must agree.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const auto cloud = make_cloud({
    {{static_cast<float>(nan), 0.0F, 1.0F}},
    {{0.0F, static_cast<float>(inf), 2.0F}},
  });
  const double fallback = 55.5;
  const double expected = reference_ground_height_from_pointcloud(cloud, 0.0, 0.0, fallback);
  EXPECT_DOUBLE_EQ(expected, fallback);  // pin the oracle's fallback decision
  const auto kdtree = autoware::map_height_fitter::build_pointcloud_xy_kdtree(cloud);
  EXPECT_DOUBLE_EQ(get_ground_height_from_pointcloud(cloud, kdtree, 0.0, 0.0, fallback), expected);
  EXPECT_DOUBLE_EQ(get_ground_height_from_pointcloud(cloud, 0.0, 0.0, fallback), expected);
}

TEST(MapHeightFitterKernel, NonFiniteQueryReturnsFallback)
{
  // A non-finite QUERY (x or y is NaN/Inf) makes every squared distance non-finite, so the original
  // scan rejects every point and falls back. The KdTree path (whose FLANN search over a NaN/Inf
  // query is otherwise unspecified) must reproduce that fallback.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const auto cloud = make_cloud({
    {{1.0F, 0.0F, 5.0F}},
    {{0.0F, 1.5F, 2.0F}},
    {{3.0F, 0.0F, -9.0F}},
  });
  const double fallback = 678.0;
  for (const auto & q : std::vector<std::array<double, 2>>{
         {nan, 0.0}, {0.0, nan}, {inf, 0.0}, {0.0, -inf}, {nan, nan}}) {
    const double expected = reference_ground_height_from_pointcloud(cloud, q[0], q[1], fallback);
    EXPECT_DOUBLE_EQ(expected, fallback);  // pin the oracle's fallback decision
    const auto kdtree = autoware::map_height_fitter::build_pointcloud_xy_kdtree(cloud);
    EXPECT_DOUBLE_EQ(
      get_ground_height_from_pointcloud(cloud, kdtree, q[0], q[1], fallback), expected)
      << "query=(" << q[0] << "," << q[1] << ")";
    EXPECT_DOUBLE_EQ(get_ground_height_from_pointcloud(cloud, q[0], q[1], fallback), expected)
      << "query=(" << q[0] << "," << q[1] << ")";
  }
}

TEST(MapHeightFitterKernel, FiniteCoordinateNonFiniteHeightFallsBack)
{
  // Mirrors VectorMapNonFiniteHeightFallsBack for the pointcloud branch: the nearest point has
  // finite (x, y) but a non-finite z. The original scan sets height = min(+inf, -inf) = -inf inside
  // the radius, then the final std::isfinite(height) guard returns fallback_z. The KdTree path must
  // reproduce that guard return for both -Inf and NaN z.
  const double inf = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const float bad_z : {static_cast<float>(-inf), static_cast<float>(nan)}) {
    const auto cloud = make_cloud({{{0.0F, 0.0F, bad_z}}});
    const double fallback = -100.0;
    const double expected = reference_ground_height_from_pointcloud(cloud, 0.0, 0.0, fallback);
    EXPECT_DOUBLE_EQ(expected, fallback);  // pin the oracle's guard return
    const auto kdtree = autoware::map_height_fitter::build_pointcloud_xy_kdtree(cloud);
    EXPECT_DOUBLE_EQ(
      get_ground_height_from_pointcloud(cloud, kdtree, 0.0, 0.0, fallback), expected);
    EXPECT_DOUBLE_EQ(get_ground_height_from_pointcloud(cloud, 0.0, 0.0, fallback), expected);
  }
}

// --- Vector-map branch --------------------------------------------------------------------------

TEST(MapHeightFitterKernel, VectorMapReturnsNearestPointHeight)
{
  lanelet::LaneletMap map;
  map.add(lanelet::Point3d(1, lanelet::BasicPoint3d(0.0, 0.0, 1.0)));
  map.add(lanelet::Point3d(2, lanelet::BasicPoint3d(10.0, 0.0, 5.0)));
  map.add(lanelet::Point3d(3, lanelet::BasicPoint3d(0.0, 10.0, 9.0)));

  // Closest to (1, 0) is point 1 (z = 1.0).
  const auto h1 = get_ground_height_from_vector_map(map, 1.0, 0.0, -1.0);
  ASSERT_TRUE(h1.has_value());
  EXPECT_DOUBLE_EQ(*h1, 1.0);
  // Closest to (9, 0) is point 2 (z = 5.0).
  const auto h2 = get_ground_height_from_vector_map(map, 9.0, 0.0, -1.0);
  ASSERT_TRUE(h2.has_value());
  EXPECT_DOUBLE_EQ(*h2, 5.0);
  // Closest to (0, 9) is point 3 (z = 9.0).
  const auto h3 = get_ground_height_from_vector_map(map, 0.0, 9.0, -1.0);
  ASSERT_TRUE(h3.has_value());
  EXPECT_DOUBLE_EQ(*h3, 9.0);
}

TEST(MapHeightFitterKernel, VectorMapEmptyReturnsNullopt)
{
  const lanelet::LaneletMap map;  // no points
  // No nearest point: the kernel reports std::nullopt so the caller can warn and fall back.
  EXPECT_FALSE(get_ground_height_from_vector_map(map, 1.0, 2.0, 77.0).has_value());
}

TEST(MapHeightFitterKernel, VectorMapNonFiniteHeightFallsBack)
{
  lanelet::LaneletMap map;
  const double inf = std::numeric_limits<double>::infinity();
  map.add(lanelet::Point3d(1, lanelet::BasicPoint3d(0.0, 0.0, inf)));
  // A nearest point exists but its z is non-finite, so the original fallback (query z) is returned
  // as a value (not std::nullopt, so the caller does not warn).
  const auto height = get_ground_height_from_vector_map(map, 0.0, 0.0, 3.0);
  ASSERT_TRUE(height.has_value());
  EXPECT_DOUBLE_EQ(*height, 3.0);
}
