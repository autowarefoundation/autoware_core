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

#ifndef AUTOWARE__NDT_SCAN_MATCHER__COVARIANCE_ESTIMATION_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__COVARIANCE_ESTIMATION_HPP_

#include "hyper_parameters.hpp"

#include <autoware/ndt_scan_matcher/ndt_omp/multigrid_ndt_omp.h>
#include <autoware_utils_geometry/geometry.hpp>

#include <geometry_msgs/msg/pose_array.hpp>

#include <Eigen/Core>

namespace autoware::ndt_scan_matcher
{

using PointSource = pcl::PointXYZ;
using PointTarget = pcl::PointXYZ;
using NdtType = pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>;

struct CovarianceComputationResult
{
  Eigen::Matrix2d covariance{Eigen::Matrix2d::Zero()};
  geometry_msgs::msg::PoseArray ndt_result_poses;
  geometry_msgs::msg::PoseArray ndt_initial_poses;
};

/// Compute covariance and the auxiliary pose arrays without publishing/logging.
CovarianceComputationResult compute_covariance_estimate(
  const pclomp::NdtResult & ndt_result, const Eigen::Matrix4f & initial_pose_matrix,
  const HyperParameters::Covariance & param, const std::shared_ptr<NdtType> & ndt_ptr,
  const rclcpp::Time & stamp, const std::string & map_frame);

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__COVARIANCE_ESTIMATION_HPP_
