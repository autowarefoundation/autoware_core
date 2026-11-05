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

// radiusSearchDirect() must return exactly the same set of leaves as the kd-tree
// radiusSearch() it replaces. These tests pin that equivalence, including the cases
// that motivated the implementation: multiple map segments, queries outside the map,
// and segments being removed and re-added while the node runs.

#include <autoware/ndt_scan_matcher/ndt_omp/multi_voxel_grid_covariance_omp.h>
#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace
{
using PointT = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointT>;
using Grid = pclomp::MultiVoxelGridCovariance<PointT>;
using LeafConstPtr = const Grid::Leaf *;

constexpr float resolution = 2.0F;

// A structured scene: ground plane, two facades and a row of poles, so the voxel
// occupancy resembles a real map rather than uniform noise.
Cloud::Ptr make_segment(double x0, double x1, double half_w, std::mt19937 & gen)
{
  auto cloud = std::make_shared<Cloud>();
  std::uniform_real_distribution<double> jitter(-0.02, 0.02);
  for (double x = x0; x < x1; x += 0.35) {
    for (double y = -half_w; y <= half_w; y += 0.35) {
      cloud->push_back(
        {static_cast<float>(x + jitter(gen)), static_cast<float>(y + jitter(gen)),
         static_cast<float>(jitter(gen))});
    }
    for (double z = 0.0; z < 6.0; z += 0.35) {
      cloud->push_back(
        {static_cast<float>(x + jitter(gen)), static_cast<float>(half_w + jitter(gen)),
         static_cast<float>(z + jitter(gen))});
      cloud->push_back(
        {static_cast<float>(x + jitter(gen)), static_cast<float>(-half_w + jitter(gen)),
         static_cast<float>(z + jitter(gen))});
    }
  }
  cloud->width = cloud->size();
  cloud->height = 1;
  return cloud;
}

// Sorted comparison: the two methods may enumerate in different orders, but the sets
// they return must be identical.
bool same_leaves(std::vector<LeafConstPtr> a, std::vector<LeafConstPtr> b)
{
  std::sort(a.begin(), a.end());
  std::sort(b.begin(), b.end());
  return a == b;
}

class VoxelNeighbourSearch : public ::testing::Test
{
protected:
  void SetUp() override
  {
    grid_.setLeafSize(resolution, resolution, resolution);
    std::mt19937 gen(1234);
    for (int i = 0; i < 3; ++i) {
      const std::string id = "seg" + std::to_string(i);
      grid_.setInputCloudAndFilter(make_segment(i * 40.0, (i + 1) * 40.0, 10.0, gen), id);
      ids_.push_back(id);
    }
    grid_.createKdtree();
  }

  // Compare both methods over n query points drawn near real surfaces, which is where
  // scan points actually land.
  size_t count_mismatches(size_t n, std::mt19937 & gen)
  {
    const Cloud voxel_pcd = grid_.getVoxelPCD();
    if (voxel_pcd.empty()) {
      return 0;
    }
    std::uniform_int_distribution<size_t> pick(0, voxel_pcd.size() - 1);
    std::normal_distribution<float> jitter(0.0F, 1.0F);
    std::vector<LeafConstPtr> from_kdtree;
    std::vector<LeafConstPtr> from_direct;
    size_t mismatches = 0;
    for (size_t i = 0; i < n; ++i) {
      const PointT & seed = voxel_pcd[pick(gen)];
      const PointT query{seed.x + jitter(gen), seed.y + jitter(gen), seed.z + jitter(gen)};
      grid_.radiusSearch(query, resolution, from_kdtree);
      grid_.radiusSearchDirect(query, resolution, from_direct);
      if (!same_leaves(from_kdtree, from_direct)) {
        ++mismatches;
      }
    }
    return mismatches;
  }

  Grid grid_;
  std::vector<std::string> ids_;
};

// The ordinary case: many query points near occupied voxels, several segments loaded.
TEST_F(VoxelNeighbourSearch, MatchesKdtreeNearSurfaces)  // NOLINT
{
  std::mt19937 gen(7);
  EXPECT_EQ(count_mismatches(20000, gen), 0U);
}

// A query far outside every segment's bounds must return nothing, from both methods.
TEST_F(VoxelNeighbourSearch, MatchesKdtreeOutsideTheMap)  // NOLINT
{
  std::vector<LeafConstPtr> from_kdtree;
  std::vector<LeafConstPtr> from_direct;
  for (const float d : {-500.0F, 500.0F, 5000.0F}) {
    const PointT query{d, d, d};
    grid_.radiusSearch(query, resolution, from_kdtree);
    grid_.radiusSearchDirect(query, resolution, from_direct);
    EXPECT_TRUE(from_kdtree.empty());
    EXPECT_TRUE(from_direct.empty());
  }
}

// The dynamic map path: the index is rebuilt by createKdtree(), so it has to stay
// correct as segments are unloaded and loaded again around the vehicle.
TEST_F(VoxelNeighbourSearch, MatchesKdtreeAcrossMapUpdates)  // NOLINT
{
  std::mt19937 gen(11);
  ASSERT_EQ(count_mismatches(5000, gen), 0U);

  grid_.removeCloud(ids_[1]);
  grid_.createKdtree();
  EXPECT_EQ(count_mismatches(5000, gen), 0U);

  std::mt19937 rebuild(4321);
  grid_.setInputCloudAndFilter(make_segment(120.0, 160.0, 10.0, rebuild), "seg3");
  grid_.createKdtree();
  EXPECT_EQ(count_mismatches(5000, gen), 0U);
}

// A voxel exactly on the query radius, and one just beyond it, must be classified the
// same way by both methods -- this is the boundary the cell enumeration has to respect.
TEST_F(VoxelNeighbourSearch, AgreesOnRadiusBoundary)  // NOLINT
{
  const Cloud voxel_pcd = grid_.getVoxelPCD();
  ASSERT_FALSE(voxel_pcd.empty());
  std::vector<LeafConstPtr> from_kdtree;
  std::vector<LeafConstPtr> from_direct;
  for (size_t i = 0; i < voxel_pcd.size(); i += 97) {
    const PointT & centre = voxel_pcd[i];
    for (const float offset : {0.0F, resolution * 0.999F, resolution * 1.001F}) {
      const PointT query{centre.x + offset, centre.y, centre.z};
      grid_.radiusSearch(query, resolution, from_kdtree);
      grid_.radiusSearchDirect(query, resolution, from_direct);
      EXPECT_TRUE(same_leaves(from_kdtree, from_direct));
    }
  }
}
}  // namespace
