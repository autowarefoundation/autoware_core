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

#include "omp_ndt_backend.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::ndt_scan_matcher
{
namespace
{

pclomp::NdtParams to_omp_params(const NdtParams & params)
{
  // The other fields of pclomp::NdtParams keep their default values: the KD-tree search and no line
  // search.
  pclomp::NdtParams omp_params{};
  omp_params.trans_epsilon = params.trans_epsilon;
  omp_params.step_size = params.step_size;
  omp_params.resolution = params.resolution;
  omp_params.max_iterations = params.max_iterations;
  omp_params.num_threads = params.num_threads;
  omp_params.regularization_scale_factor = params.regularization_scale_factor;
  return omp_params;
}

NdtParams from_omp_params(const pclomp::NdtParams & omp_params)
{
  NdtParams params;
  params.trans_epsilon = omp_params.trans_epsilon;
  params.step_size = omp_params.step_size;
  params.resolution = omp_params.resolution;
  params.max_iterations = omp_params.max_iterations;
  params.num_threads = omp_params.num_threads;
  params.regularization_scale_factor = omp_params.regularization_scale_factor;
  return params;
}

NdtResult from_omp_result(pclomp::NdtResult && omp_result)
{
  NdtResult result;
  result.pose = omp_result.pose;
  result.transform_probability = omp_result.transform_probability;
  result.nearest_voxel_transformation_likelihood =
    omp_result.nearest_voxel_transformation_likelihood;
  result.iteration_num = omp_result.iteration_num;
  result.transformation_array = std::move(omp_result.transformation_array);
  result.hessian = omp_result.hessian;
  result.transform_probability_array = std::move(omp_result.transform_probability_array);
  result.nearest_voxel_transformation_likelihood_array =
    std::move(omp_result.nearest_voxel_transformation_likelihood_array);
  return result;
}

}  // namespace

void OmpNdtBackend::initialize(const NdtParams & params)
{
  ndt_.set_params(to_omp_params(params));
}

std::string OmpNdtBackend::name() const
{
  return "ndt_omp";
}

NdtParams OmpNdtBackend::get_params() const
{
  return from_omp_params(ndt_.get_params());
}

void OmpNdtBackend::add_target(
  const pcl::PointCloud<PointTarget>::ConstPtr & cloud, const std::string & map_id)
{
  ndt_.add_target(cloud, map_id);
}

void OmpNdtBackend::remove_target(const std::string & map_id)
{
  ndt_.remove_target(map_id);
}

void OmpNdtBackend::commit_target()
{
  ndt_.create_voxel_kdtree();
}

std::vector<std::string> OmpNdtBackend::get_current_map_ids() const
{
  return ndt_.get_current_map_ids();
}

bool OmpNdtBackend::has_target() const
{
  return ndt_.has_target();
}

NdtBackend::SharedPtr OmpNdtBackend::clone() const
{
  // The copy constructor of ndt_omp. It copies the map, and it shares the immutable voxel grids.
  return std::make_shared<OmpNdtBackend>(*this);
}

NdtBackend::SharedPtr OmpNdtBackend::clone_empty() const
{
  // Copy the parameters of ndt_omp directly, so that the fields that only ndt_omp has stay too.
  auto empty = std::make_shared<OmpNdtBackend>();
  empty->ndt_.set_params(ndt_.get_params());
  return empty;
}

void OmpNdtBackend::set_regularization_pose(const Eigen::Matrix4f & pose)
{
  ndt_.set_regularization_pose(pose);
}

void OmpNdtBackend::unset_regularization_pose()
{
  ndt_.unset_regularization_pose();
}

NdtResult OmpNdtBackend::align(
  const Eigen::Matrix4f & guess, const pcl::PointCloud<PointSource>::ConstPtr & source)
{
  // ndt_omp writes the aligned cloud here. The node does not use it.
  pcl::PointCloud<PointSource> output;
  ndt_.align(output, guess, source);
  // A known difference from NdtResult::hessian: if a step of ndt_omp has the length 0 or is not a
  // number, ndt_omp returns before it stores the hessian. Then the result holds the hessian of the
  // alignment before. This is the behavior of ndt_omp before the backend interface.
  return from_omp_result(ndt_.get_result());
}

double OmpNdtBackend::calculate_transformation_probability(
  const pcl::PointCloud<PointSource> & cloud_in_map) const
{
  return ndt_.calculate_transformation_probability(cloud_in_map);
}

double OmpNdtBackend::calculate_nearest_voxel_transformation_likelihood(
  const pcl::PointCloud<PointSource> & cloud_in_map) const
{
  return ndt_.calculate_nearest_voxel_transformation_likelihood(cloud_in_map);
}

pcl::PointCloud<pcl::PointXYZI> OmpNdtBackend::calculate_nearest_voxel_score_each_point(
  const pcl::PointCloud<PointSource> & cloud_in_map) const
{
  return ndt_.calculate_nearest_voxel_score_each_point(cloud_in_map);
}

}  // namespace autoware::ndt_scan_matcher

PLUGINLIB_EXPORT_CLASS(
  autoware::ndt_scan_matcher::OmpNdtBackend, autoware::ndt_scan_matcher::NdtBackend)
