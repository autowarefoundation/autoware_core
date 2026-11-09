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

// The OMP backend must give exactly the results of ndt_omp, and its copies must be independent.

#include "omp_ndt_backend.hpp"

#include <Eigen/Geometry>
#include <autoware/ndt_scan_matcher_common/ndt_omp/multigrid_ndt_omp.h>
#include <gtest/gtest.h>
#include <pcl/common/transforms.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::ndt_scan_matcher
{
namespace
{

pclomp::NdtParams make_params()
{
  pclomp::NdtParams params{};
  params.trans_epsilon = 0.01;
  params.step_size = 0.1;
  params.resolution = 2.0F;
  params.max_iterations = 30;
  params.num_threads = 1;
  params.regularization_scale_factor = 0.01F;
  return params;
}

// Three orthogonal planes of 20 m x 20 m, with a point every 0.2 m.
pcl::PointCloud<pcl::PointXYZ> make_half_cube()
{
  constexpr float length = 20.0F;
  constexpr float interval = 0.2F;
  constexpr int num_points_per_line = static_cast<int>(length / interval) + 1;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int plane = 0; plane < 3; ++plane) {
    for (int i = 0; i < num_points_per_line; ++i) {
      for (int j = 0; j < num_points_per_line; ++j) {
        const float u = interval * static_cast<float>(j);
        const float v = interval * static_cast<float>(i);
        if (plane == 0) {
          cloud.push_back(pcl::PointXYZ(u, v, 0.0F));
        } else if (plane == 1) {
          cloud.push_back(pcl::PointXYZ(0.0F, u, v));
        } else {
          cloud.push_back(pcl::PointXYZ(u, 0.0F, v));
        }
      }
    }
  }
  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr make_map()
{
  return std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(make_half_cube());
}

// Every 10th point of the map, as the scan.
pcl::PointCloud<pcl::PointXYZ>::Ptr make_scan()
{
  const auto map = make_half_cube();
  auto scan = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (std::size_t i = 0; i < map.size(); i += 10) {
    scan->push_back(map.points[i]);
  }
  return scan;
}

Eigen::Matrix4f make_guess()
{
  Eigen::Affine3f guess = Eigen::Affine3f::Identity();
  guess.translate(Eigen::Vector3f(0.3F, -0.2F, 0.1F));
  guess.rotate(Eigen::AngleAxisf(0.02F, Eigen::Vector3f::UnitZ()));
  return guess.matrix();
}

}  // namespace

TEST(OmpNdtBackend, AlignGivesTheResultOfNdtOmp)
{
  pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.set_params(make_params());
  ndt.add_target(make_map(), "a");
  ndt.create_voxel_kdtree();
  ndt.set_regularization_pose(Eigen::Matrix4f::Identity());
  pcl::PointCloud<pcl::PointXYZ> output;
  ndt.align(output, make_guess(), make_scan());
  const pclomp::NdtResult expected = ndt.get_result();

  OmpNdtBackend backend;
  backend.initialize(make_params());
  backend.add_target(make_map(), "a");
  backend.commit_target();
  backend.set_regularization_pose(Eigen::Matrix4f::Identity());
  const pclomp::NdtResult result = backend.align(make_guess(), make_scan());

  EXPECT_EQ(result.pose, expected.pose);
  EXPECT_EQ(result.hessian, expected.hessian);
  EXPECT_EQ(result.iteration_num, expected.iteration_num);
  EXPECT_EQ(result.transform_probability, expected.transform_probability);
  EXPECT_EQ(
    result.nearest_voxel_transformation_likelihood,
    expected.nearest_voxel_transformation_likelihood);
  EXPECT_EQ(result.transformation_array.size(), expected.transformation_array.size());

  pcl::PointCloud<pcl::PointXYZ> scan_in_map;
  pcl::transformPointCloud(*make_scan(), scan_in_map, expected.pose);
  EXPECT_EQ(
    backend.calculate_nearest_voxel_transformation_likelihood(scan_in_map),
    ndt.calculate_nearest_voxel_transformation_likelihood(scan_in_map));
  EXPECT_EQ(
    backend.calculate_transformation_probability(scan_in_map),
    ndt.calculate_transformation_probability(scan_in_map));
}

TEST(OmpNdtBackend, CloneIsIndependent)
{
  OmpNdtBackend backend;
  backend.initialize(make_params());
  backend.add_target(make_map(), "a");
  backend.commit_target();

  const NdtBackend::SharedPtr clone = backend.clone();
  EXPECT_EQ(clone->get_current_map_ids(), std::vector<std::string>{"a"});
  EXPECT_EQ(clone->get_params().max_iterations, make_params().max_iterations);

  clone->remove_target("a");
  clone->commit_target();
  EXPECT_FALSE(clone->has_target());
  EXPECT_TRUE(backend.has_target());
}

TEST(OmpNdtBackend, CloneAlignsLikeTheOriginal)
{
  OmpNdtBackend backend;
  backend.initialize(make_params());
  backend.add_target(make_map(), "a");
  backend.commit_target();
  const NdtBackend::SharedPtr clone = backend.clone();

  EXPECT_EQ(
    clone->align(make_guess(), make_scan()).pose, backend.align(make_guess(), make_scan()).pose);
}

TEST(OmpNdtBackend, CloneEmptyHasTheParametersAndNoMap)
{
  OmpNdtBackend backend;
  pclomp::NdtParams params = make_params();
  params.max_iterations = 17;
  backend.initialize(params);
  backend.add_target(make_map(), "a");
  backend.commit_target();

  const NdtBackend::SharedPtr empty = backend.clone_empty();
  EXPECT_FALSE(empty->has_target());
  EXPECT_EQ(empty->get_params().max_iterations, 17);
  EXPECT_EQ(empty->name(), "omp");
}

}  // namespace autoware::ndt_scan_matcher
