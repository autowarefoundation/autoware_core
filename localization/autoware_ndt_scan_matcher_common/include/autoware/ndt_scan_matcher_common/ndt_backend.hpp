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

#ifndef AUTOWARE__NDT_SCAN_MATCHER_COMMON__NDT_BACKEND_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER_COMMON__NDT_BACKEND_HPP_

#include "ndt_omp/ndt_struct.hpp"

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::ndt_scan_matcher
{

/// The NDT implementation that the node uses. The node loads it with pluginlib, by the name in the
/// `ndt.backend` parameter. The default backend is "omp" (OmpNdtBackend).
///
/// Thread safety: one thread at a time uses one instance. The node makes sure of this with
/// Guarded<>. Two different instances (the active one and the one that the map update module
/// builds) run at the same time in two threads, so they must not share mutable state.
class NdtBackend
{
public:
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;
  using SharedPtr = std::shared_ptr<NdtBackend>;

  virtual ~NdtBackend() = default;

  /// Called once, after pluginlib makes the instance. pluginlib needs a default constructor, so a
  /// backend that needs a resource (for example, a GPU) gets it here. Throws std::runtime_error if
  /// the backend cannot run on this machine.
  virtual void initialize(const pclomp::NdtParams & params) = 0;

  /// The name of the backend, for logs.
  virtual std::string name() const = 0;

  virtual void set_params(const pclomp::NdtParams & params) = 0;
  virtual pclomp::NdtParams get_params() const = 0;

  /// add_target() and remove_target() change a pending map. commit_target() makes the pending map
  /// active. All heavy map work happens in commit_target(), which the map update module calls in
  /// its own thread. align() must not do deferred map work.
  virtual void add_target(
    const pcl::PointCloud<PointTarget>::ConstPtr & cloud, const std::string & map_id) = 0;
  virtual void remove_target(const std::string & map_id) = 0;
  virtual void commit_target() = 0;
  virtual std::vector<std::string> get_current_map_ids() const = 0;
  virtual bool has_target() const = 0;

  /// An independent instance with the same parameters, map and state. Later changes to one instance
  /// do not change the other. It can share immutable data with the original.
  virtual SharedPtr clone() const = 0;
  /// A new instance of the same backend, initialized with the same parameters, without a map.
  virtual SharedPtr clone_empty() const = 0;

  virtual void set_regularization_pose(const Eigen::Matrix4f & pose) = 0;
  virtual void unset_regularization_pose() = 0;

  /// Aligns source (in the base_link frame) to the map, from guess (base_link to map).
  /// The result follows the conventions of ndt_omp: `hessian` is the hessian of the score at the
  /// final pose, as estimate_xy_covariance_by_laplace_approximation() expects.
  virtual pclomp::NdtResult align(
    const Eigen::Matrix4f & guess, const pcl::PointCloud<PointSource>::ConstPtr & source) = 0;

  /// Scores of a cloud that is already in the map frame.
  virtual double calculate_transformation_probability(
    const pcl::PointCloud<PointSource> & cloud_in_map) const = 0;
  virtual double calculate_nearest_voxel_transformation_likelihood(
    const pcl::PointCloud<PointSource> & cloud_in_map) const = 0;
  virtual pcl::PointCloud<pcl::PointXYZI> calculate_nearest_voxel_score_each_point(
    const pcl::PointCloud<PointSource> & cloud_in_map) const = 0;
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER_COMMON__NDT_BACKEND_HPP_
