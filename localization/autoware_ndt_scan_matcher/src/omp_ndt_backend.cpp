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

#include "autoware/ndt_scan_matcher/omp_ndt_backend.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::ndt_scan_matcher
{

void OmpNdtBackend::initialize(const pclomp::NdtParams & params)
{
  ndt_.set_params(params);
}

std::string OmpNdtBackend::name() const
{
  return "omp";
}

void OmpNdtBackend::set_params(const pclomp::NdtParams & params)
{
  ndt_.set_params(params);
}

pclomp::NdtParams OmpNdtBackend::get_params() const
{
  return ndt_.get_params();
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
  // A deep copy, the same as the copy of the NDT object before the backend interface.
  return std::make_shared<OmpNdtBackend>(*this);
}

NdtBackend::SharedPtr OmpNdtBackend::clone_empty() const
{
  auto empty = std::make_shared<OmpNdtBackend>();
  empty->initialize(get_params());
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

pclomp::NdtResult OmpNdtBackend::align(
  const Eigen::Matrix4f & guess, const pcl::PointCloud<PointSource>::ConstPtr & source)
{
  // ndt_omp writes the aligned cloud here. The node does not use it.
  pcl::PointCloud<PointSource> output;
  ndt_.align(output, guess, source);
  return ndt_.get_result();
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
