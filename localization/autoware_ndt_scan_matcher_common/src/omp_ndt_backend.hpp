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

#ifndef OMP_NDT_BACKEND_HPP_
#define OMP_NDT_BACKEND_HPP_

#include "autoware/ndt_scan_matcher_common/ndt_backend.hpp"
#include "autoware/ndt_scan_matcher_common/ndt_omp/multigrid_ndt_omp.h"

#include <string>
#include <vector>

namespace autoware::ndt_scan_matcher
{

/// The CPU backend: pclomp::MultiGridNormalDistributionsTransform (ndt_omp), without a change.
class OmpNdtBackend : public NdtBackend
{
public:
  void initialize(const pclomp::NdtParams & params) override;
  std::string name() const override;

  void set_params(const pclomp::NdtParams & params) override;
  pclomp::NdtParams get_params() const override;

  void add_target(
    const pcl::PointCloud<PointTarget>::ConstPtr & cloud, const std::string & map_id) override;
  void remove_target(const std::string & map_id) override;
  void commit_target() override;
  std::vector<std::string> get_current_map_ids() const override;
  bool has_target() const override;

  SharedPtr clone() const override;
  SharedPtr clone_empty() const override;

  void set_regularization_pose(const Eigen::Matrix4f & pose) override;
  void unset_regularization_pose() override;

  pclomp::NdtResult align(
    const Eigen::Matrix4f & guess, const pcl::PointCloud<PointSource>::ConstPtr & source) override;

  double calculate_transformation_probability(
    const pcl::PointCloud<PointSource> & cloud_in_map) const override;
  double calculate_nearest_voxel_transformation_likelihood(
    const pcl::PointCloud<PointSource> & cloud_in_map) const override;
  pcl::PointCloud<pcl::PointXYZI> calculate_nearest_voxel_score_each_point(
    const pcl::PointCloud<PointSource> & cloud_in_map) const override;

private:
  pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget> ndt_;
};

}  // namespace autoware::ndt_scan_matcher

#endif  // OMP_NDT_BACKEND_HPP_
